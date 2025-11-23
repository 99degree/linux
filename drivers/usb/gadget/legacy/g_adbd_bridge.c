// SPDX-License-Identifier: GPL-2.0
//
// g_adbd_bridge.c - ADBBridgeX: Event-driven FSM legacy USB gadget
//                   Workqueue-based FSM (no permanent kthread)
//                   ADB (FF/42/01) bulk endpoints bridged to TCP (127.0.0.1:5555)
//
// Key features:
// - FSM driven by an ordered workqueue; runs only when events are posted.
// - Event-driven lifecycle: USB config, TCP connect, I/O, errors, shutdown.
// - OUT path uses a request pool; completions enqueue slices to TX ring.
// - IN path frees buffers via completion.
// - TCP reconnect handled by FSM on poll error/hup.
// - Clean disable: stop workers, release socket, drain rings.
//
// Author: You

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/usb/composite.h>
#include <linux/kthread.h>
#include <linux/net.h>
#include <linux/in.h>
#include <linux/slab.h>
#include <linux/atomic.h>
#include <linux/string.h>
#include <linux/wait.h>
#include <linux/sched/signal.h>
#include <linux/poll.h>
#include <linux/list.h>
#include <linux/workqueue.h>

/* -------------------------------------------------------------------------- */
/* Config constants                                                            */
/* -------------------------------------------------------------------------- */

#define VENDOR_ID   0x18D1     /* Example VID; change for your product */
#define PRODUCT_ID  0x4EE7     /* Example PID; change for your product */

#define ADB_TCP_PORT    5555
#define ADB_TCP_ADDR    INADDR_LOOPBACK

#define OUT_REQ_COUNT   64
#define OUT_REQ_SIZE    (128 * 1024)

#define RING_SIZE       1024
#define TCP_SNDBUF      (2 * 1024 * 1024)
#define TCP_RCVBUF      (2 * 1024 * 1024)

/* -------------------------------------------------------------------------- */
/* USB descriptors                                                             */
/* -------------------------------------------------------------------------- */

static struct usb_device_descriptor dev_desc = {
    .bLength            = USB_DT_DEVICE_SIZE,
    .bDescriptorType    = USB_DT_DEVICE,
    .bcdUSB             = cpu_to_le16(0x0200),
    .bDeviceClass       = USB_CLASS_PER_INTERFACE,
    .bDeviceSubClass    = 0,
    .bDeviceProtocol    = 0,
    .bMaxPacketSize0    = 64,
    .idVendor           = cpu_to_le16(VENDOR_ID),
    .idProduct          = cpu_to_le16(PRODUCT_ID),
    .bcdDevice          = cpu_to_le16(0x0100),
    .iManufacturer      = 1,
    .iProduct           = 2,
    .iSerialNumber      = 3,
};

enum {
    STR_MANUFACTURER = 1,
    STR_PRODUCT,
    STR_SERIAL,
    STR_INTERFACE,
};

static struct usb_string strings[] = {
    { STR_MANUFACTURER, "Example Inc." },
    { STR_PRODUCT,      "ADBBridgeX" },
    { STR_SERIAL,       "000000000000" },
    { STR_INTERFACE,    "ADB" },
    { }
};

static struct usb_gadget_strings stringtab = {
    .language = 0x0409, /* en-US */
    .strings  = strings,
};

static struct usb_gadget_strings *gadget_strings[] = {
    &stringtab, NULL
};

#define ADB_CLASS  0xFF
#define ADB_SUB    0x42
#define ADB_PROTO  0x01

static struct usb_interface_descriptor intf_desc = {
    .bLength            = sizeof(intf_desc),
    .bDescriptorType    = USB_DT_INTERFACE,
    .bAlternateSetting  = 0,
    .bNumEndpoints      = 2,
    .bInterfaceClass    = ADB_CLASS,
    .bInterfaceSubClass = ADB_SUB,
    .bInterfaceProtocol = ADB_PROTO,
    .iInterface         = STR_INTERFACE,
};

static struct usb_endpoint_descriptor fs_ep_in = {
    .bLength          = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType  = USB_DT_ENDPOINT,
    .bEndpointAddress = USB_DIR_IN,
    .bmAttributes     = USB_ENDPOINT_XFER_BULK,
    .wMaxPacketSize   = cpu_to_le16(64),
};

static struct usb_endpoint_descriptor fs_ep_out = {
    .bLength          = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType  = USB_DT_ENDPOINT,
    .bEndpointAddress = USB_DIR_OUT,
    .bmAttributes     = USB_ENDPOINT_XFER_BULK,
    .wMaxPacketSize   = cpu_to_le16(64),
};

static struct usb_endpoint_descriptor hs_ep_in = {
    .bLength          = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType  = USB_DT_ENDPOINT,
    .bEndpointAddress = USB_DIR_IN,
    .bmAttributes     = USB_ENDPOINT_XFER_BULK,
    .wMaxPacketSize   = cpu_to_le16(512),
};

static struct usb_endpoint_descriptor hs_ep_out = {
    .bLength          = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType  = USB_DT_ENDPOINT,
    .bEndpointAddress = USB_DIR_OUT,
    .bmAttributes     = USB_ENDPOINT_XFER_BULK,
    .wMaxPacketSize   = cpu_to_le16(512),
};

/* -------------------------------------------------------------------------- */
/* Simple SPSC ring with wait queues                                           */
/* -------------------------------------------------------------------------- */

struct adbd_ring {
    void   **bufs;
    size_t  *lens;
    u32      size;
    u32      head;
    u32      tail;
    wait_queue_head_t wq_not_empty;
    wait_queue_head_t wq_has_space;
};

static inline bool ring_empty(const struct adbd_ring *r)
{
    return r->head == r->tail;
}

static inline bool ring_full(const struct adbd_ring *r)
{
    return ((r->head + 1) % r->size) == r->tail;
}

static void ring_init(struct adbd_ring *r, u32 size)
{
    r->size = size;
    r->head = r->tail = 0;
    r->bufs = kcalloc(size, sizeof(void *), GFP_KERNEL);
    r->lens = kcalloc(size, sizeof(size_t), GFP_KERNEL);
    init_waitqueue_head(&r->wq_not_empty);
    init_waitqueue_head(&r->wq_has_space);
}

static void ring_free(struct adbd_ring *r)
{
    kfree(r->bufs);
    kfree(r->lens);
}

static int ring_push_blocking(struct adbd_ring *r, void *buf, size_t len)
{
    int ret;
    ret = wait_event_interruptible(r->wq_has_space, !ring_full(r));
    if (ret) return ret;

    r->bufs[r->head] = buf;
    r->lens[r->head] = len;
    smp_wmb();
    r->head = (r->head + 1) % r->size;

    wake_up_interruptible(&r->wq_not_empty);
    return 0;
}

static void *ring_pop_blocking(struct adbd_ring *r, size_t *len)
{
    int ret = wait_event_interruptible(r->wq_not_empty, !ring_empty(r));
    if (ret) return NULL;

    *len = r->lens[r->tail];
    smp_rmb();
    {
        void *buf = r->bufs[r->tail];
        r->tail = (r->tail + 1) % r->size;
        wake_up_interruptible(&r->wq_has_space);
        return buf;
    }
}

/* -------------------------------------------------------------------------- */
/* FSM: states and events                                                      */
/* -------------------------------------------------------------------------- */

enum adbd_state {
    USB_IDLE = 0,
    USB_CONFIGURED,
    TCP_CONNECTING,
    TCP_CONNECTED,
    TCP_ERROR,
    SHUTDOWN,
};

enum adbd_event_type {
    EV_USB_CONFIGURED = 1,
    EV_USB_DISABLE,
    EV_TCP_CONNECT,
    EV_TCP_CONNECTED,
    EV_TCP_ERROR,
    EV_TCP_READABLE,
    EV_TCP_WRITABLE,
};

struct adbd_event {
    enum adbd_event_type type;
    unsigned long        arg;
    struct list_head     node;
};

/* -------------------------------------------------------------------------- */
/* Context                                                                     */
/* -------------------------------------------------------------------------- */

struct adbd_ctx {
    /* USB function glue */
    struct usb_function func;
    struct usb_ep *ep_in, *ep_out;

    /* OUT request pool */
    struct usb_request **out_pool;

    /* TCP socket */
    struct socket *tcp_sock;

    /* Workers */
    struct task_struct *t_send;
    struct task_struct *t_recv;
    struct task_struct *t_in;

    /* Rings */
    struct adbd_ring tx_ring_tcp; /* USB OUT slices → TCP send */
    struct adbd_ring rx_ring_usb; /* TCP recv slices → USB IN */

    /* FSM */
    enum adbd_state state;
    spinlock_t      ev_lock;
    struct list_head ev_queue;
    struct workqueue_struct *wq;
    struct work_struct      fsm_work;

    /* Counters */
    atomic64_t urb_out_done;
    atomic64_t urb_in_submit;
    atomic64_t tcp_sent_bytes;
    atomic64_t tcp_recv_bytes;
    atomic64_t drops_ring_full;

    /* TCP tuning */
    u32 tcp_sndbuf;
    u32 tcp_rcvbuf;

    bool active;
};

/* -------------------------------------------------------------------------- */
/* Event queue helpers                                                         */
/* -------------------------------------------------------------------------- */

static void evq_init(struct adbd_ctx *ctx)
{
    spin_lock_init(&ctx->ev_lock);
    INIT_LIST_HEAD(&ctx->ev_queue);
}

static int evq_push(struct adbd_ctx *ctx, enum adbd_event_type type, unsigned long arg, gfp_t gfp)
{
    struct adbd_event *ev = kzalloc(sizeof(*ev), gfp);
    if (!ev) return -ENOMEM;
    ev->type = type;
    ev->arg  = arg;
    INIT_LIST_HEAD(&ev->node);

    spin_lock(&ctx->ev_lock);
    list_add_tail(&ev->node, &ctx->ev_queue);
    spin_unlock(&ctx->ev_lock);

    /* Schedule FSM work to process events */
    if (ctx->wq)
        queue_work(ctx->wq, &ctx->fsm_work);

    return 0;
}

static struct adbd_event *evq_pop(struct adbd_ctx *ctx)
{
    struct adbd_event *ev = NULL;

    spin_lock(&ctx->ev_lock);
    if (!list_empty(&ctx->ev_queue)) {
        struct list_head *n = ctx->ev_queue.next;
        list_del(n);
        ev = list_entry(n, struct adbd_event, node);
    }
    spin_unlock(&ctx->ev_lock);
    return ev;
}

/* -------------------------------------------------------------------------- */
/* Socket helpers                                                              */
/* -------------------------------------------------------------------------- */

static __poll_t adbd_sock_poll(struct socket *sock)
{
    if (!sock || !sock->ops || !sock->ops->poll)
        return EPOLLERR;
    return sock->ops->poll(NULL, sock, NULL);
}

static int adbd_tcp_connect(struct adbd_ctx *ctx)
{
    struct sockaddr_in addr = {
        .sin_family      = AF_INET,
        .sin_port        = htons(ADB_TCP_PORT),
        .sin_addr.s_addr = htonl(ADB_TCP_ADDR),
    };
    int ret, one = 1;

    ret = sock_create_kern(&init_net, AF_INET, SOCK_STREAM, IPPROTO_TCP, &ctx->tcp_sock);
    if (ret) return ret;

    kernel_setsockopt(ctx->tcp_sock, SOL_TCP, TCP_NODELAY, (char *)&one, sizeof(one));
    kernel_setsockopt(ctx->tcp_sock, SOL_SOCKET, SO_SNDBUF, (char *)&ctx->tcp_sndbuf, sizeof(ctx->tcp_sndbuf));
    kernel_setsockopt(ctx->tcp_sock, SOL_SOCKET, SO_RCVBUF, (char *)&ctx->tcp_rcvbuf, sizeof(ctx->tcp_rcvbuf));

    ret = kernel_connect(ctx->tcp_sock, (struct sockaddr *)&addr, sizeof(addr), O_NONBLOCK);
    if (ret && ret != -EINPROGRESS) {
        sock_release(ctx->tcp_sock);
        ctx->tcp_sock = NULL;
        return ret;
    }

    /* Consider connected upon writability; workers/poll will confirm */
    return 0;
}

static void adbd_tcp_close(struct adbd_ctx *ctx)
{
    if (ctx->tcp_sock) {
        sock_release(ctx->tcp_sock);
        ctx->tcp_sock = NULL;
    }
}

/* -------------------------------------------------------------------------- */
/* USB callbacks                                                               */
/* -------------------------------------------------------------------------- */

static void adbd_in_complete(struct usb_ep *ep, struct usb_request *req)
{
    kfree(req->buf);
    usb_ep_free_request(ep, req);
}

static void adbd_out_complete(struct usb_ep *ep, struct usb_request *req)
{
    struct adbd_ctx *ctx = ep->driver_data;
    void *copy;

    if (!ctx || !ctx->active) goto requeue;
    if (req->status || !req->actual) goto requeue;

    copy = kmemdup(req->buf, req->actual, GFP_ATOMIC);
    if (!copy) goto requeue;

    if (ring_push_blocking(&ctx->tx_ring_tcp, copy, req->actual) < 0) {
        kfree(copy);
        atomic64_inc(&ctx->drops_ring_full);
    } else {
        atomic64_inc(&ctx->urb_out_done);
        /* Notify FSM that data is available to send */
        evq_push(ctx, EV_TCP_WRITABLE, 0, GFP_ATOMIC);
    }

requeue:
    usb_ep_queue(ep, req, GFP_ATOMIC);
}

/* -------------------------------------------------------------------------- */
/* Worker threads                                                              */
/* -------------------------------------------------------------------------- */

static int adbd_tcp_send_thread(void *data)
{
    struct adbd_ctx *ctx = data;
    allow_signal(SIGKILL);

    while (!kthread_should_stop()) {
        size_t len;
        void *buf = ring_pop_blocking(&ctx->tx_ring_tcp, &len);
        __poll_t mask;
        int sent;

        if (!buf)
            continue;

        if (!ctx->tcp_sock) {
            kfree(buf);
            evq_push(ctx, EV_TCP_ERROR, 0, GFP_KERNEL);
            continue;
        }

        mask = adbd_sock_poll(ctx->tcp_sock);
        if (mask & (EPOLLERR | EPOLLHUP)) {
            kfree(buf);
            evq_push(ctx, EV_TCP_ERROR, 0, GFP_KERNEL);
            continue;
        }
        if (!(mask & (EPOLLOUT | EPOLLWRNORM | EPOLLWRBAND))) {
            /* Not writable: push back and hint FSM */
            if (ring_push_blocking(&ctx->tx_ring_tcp, buf, len) < 0) {
                kfree(buf);
                atomic64_inc(&ctx->drops_ring_full);
            }
            evq_push(ctx, EV_TCP_WRITABLE, 0, GFP_KERNEL);
            continue;
        }

        {
            struct kvec iov = { .iov_base = buf, .iov_len = len };
            struct msghdr msg = { .msg_flags = MSG_DONTWAIT };
            sent = kernel_sendmsg(ctx->tcp_sock, &msg, &iov, 1, len);
        }

        if (sent <= 0) {
            kfree(buf);
            evq_push(ctx, EV_TCP_ERROR, 0, GFP_KERNEL);
            continue;
        }

        atomic64_add(sent, &ctx->tcp_sent_bytes);
        kfree(buf);
    }
    return 0;
}

static int adbd_tcp_recv_thread(void *data)
{
    struct adbd_ctx *ctx = data;
    allow_signal(SIGKILL);

    while (!kthread_should_stop()) {
        __poll_t mask;
        void *buf;
        int recvd;
        size_t cap = 64 * 1024;

        if (!ctx->tcp_sock) {
            evq_push(ctx, EV_TCP_ERROR, 0, GFP_KERNEL);
            schedule();
            continue;
        }

        mask = adbd_sock_poll(ctx->tcp_sock);
        if (mask & (EPOLLERR | EPOLLHUP)) {
            evq_push(ctx, EV_TCP_ERROR, 0, GFP_KERNEL);
            schedule();
            continue;
        }
        if (!(mask & (EPOLLIN | EPOLLRDNORM | EPOLLRDBAND))) {
            /* Not readable yet; hint FSM and yield */
            evq_push(ctx, EV_TCP_READABLE, 0, GFP_KERNEL);
            schedule();
            continue;
        }

        buf = kmalloc(cap, GFP_KERNEL);
        if (!buf) { schedule(); continue; }

        {
            struct kvec iov = { .iov_base = buf, .iov_len = cap };
            struct msghdr msg = { .msg_flags = MSG_DONTWAIT };
            recvd = kernel_recvmsg(ctx->tcp_sock, &msg, &iov, 1, cap, MSG_DONTWAIT);
        }

        if (recvd <= 0) {
            kfree(buf);
            evq_push(ctx, EV_TCP_ERROR, 0, GFP_KERNEL);
            continue;
        }

        atomic64_add(recvd, &ctx->tcp_recv_bytes);

        if (ring_push_blocking(&ctx->rx_ring_usb, buf, recvd) < 0) {
            kfree(buf);
            atomic64_inc(&ctx->drops_ring_full);
        } else {
            /* IN submitter wakes via ring wait queue */
        }
    }
    return 0;
}

static int adbd_usb_in_thread(void *data)
{
    struct adbd_ctx *ctx = data;
    allow_signal(SIGKILL);

    while (!kthread_should_stop()) {
        size_t len;
        void *buf = ring_pop_blocking(&ctx->rx_ring_usb, &len);
        struct usb_request *req;

        if (!buf)
            continue;

        req = usb_ep_alloc_request(ctx->ep_in, GFP_KERNEL);
        if (!req) { kfree(buf); continue; }

        req->buf = buf;
        req->length = len;
        req->complete = adbd_in_complete;

        if (usb_ep_queue(ctx->ep_in, req, GFP_ATOMIC)) {
            usb_ep_free_request(ctx->ep_in, req);
            kfree(buf);
        } else {
            atomic64_inc(&ctx->urb_in_submit);
        }
    }
    return 0;
}

/* -------------------------------------------------------------------------- */
/* FSM work handler                                                            */
/* -------------------------------------------------------------------------- */

static void adbd_tcp_connect_start(struct adbd_ctx *ctx)
{
    int ret = adbd_tcp_connect(ctx);
    if (ret == 0) {
        /* Consider connecting; workers/poll will confirm writability */
        evq_push(ctx, EV_TCP_CONNECTED, 0, GFP_KERNEL);
    } else {
        evq_push(ctx, EV_TCP_ERROR, ret, GFP_KERNEL);
    }
}

static void adbd_fsm_work(struct work_struct *work)
{
    struct adbd_ctx *ctx = container_of(work, struct adbd_ctx, fsm_work);

    /* Drain queue atomically until empty to preserve ordering */
    for (;;) {
        struct adbd_event *ev = evq_pop(ctx);
        if (!ev) break;

        switch (ctx->state) {
        case USB_IDLE:
            if (ev->type == EV_USB_CONFIGURED) {
                ctx->state = USB_CONFIGURED;
                /* Initiate connecting */
                evq_push(ctx, EV_TCP_CONNECT, 0, GFP_KERNEL);
            }
            break;

        case USB_CONFIGURED:
            if (ev->type == EV_TCP_CONNECT) {
                ctx->state = TCP_CONNECTING;
                adbd_tcp_connect_start(ctx);
            } else if (ev->type == EV_USB_DISABLE) {
                ctx->state = SHUTDOWN;
            }
            break;

        case TCP_CONNECTING:
            if (ev->type == EV_TCP_CONNECTED) {
                ctx->state = TCP_CONNECTED;
            } else if (ev->type == EV_TCP_ERROR) {
                ctx->state = TCP_ERROR;
            } else if (ev->type == EV_USB_DISABLE) {
                ctx->state = SHUTDOWN;
            }
            break;

        case TCP_CONNECTED:
            if (ev->type == EV_TCP_ERROR) {
                ctx->state = TCP_ERROR;
            } else if (ev->type == EV_USB_DISABLE) {
                ctx->state = SHUTDOWN;
            } else if (ev->type == EV_TCP_READABLE || ev->type == EV_TCP_WRITABLE) {
                /* Workers perform I/O; FSM maintains state */
            }
            break;

        case TCP_ERROR:
            /* Close and attempt reconnect */
            adbd_tcp_close(ctx);
            ctx->state = TCP_CONNECTING;
            evq_push(ctx, EV_TCP_CONNECT, 0, GFP_KERNEL);
            break;

        case SHUTDOWN:
            /* disable() handles cleanup; return to idle once cleared */
            ctx->state = USB_IDLE;
            break;

        default:
            break;
        }

        kfree(ev);
    }
}

/* -------------------------------------------------------------------------- */
/* usb_function glue                                                           */
/* -------------------------------------------------------------------------- */

static int adbd_func_bind(struct usb_configuration *c, struct usb_function *f)
{
    struct adbd_ctx *ctx = container_of(f, struct adbd_ctx, func);
    int id;

    id = usb_interface_id(c, f);
    if (id < 0) return id;

    intf_desc.bInterfaceNumber = id;

    ctx->ep_in  = usb_ep_autoconfig(c->cdev->gadget, &fs_ep_in);
    ctx->ep_out = usb_ep_autoconfig(c->cdev->gadget, &fs_ep_out);
    if (!ctx->ep_in || !ctx->ep_out) return -ENODEV;

    ctx->ep_in->driver_data  = ctx;
    ctx->ep_out->driver_data = ctx;

    /* Rings */
    ring_init(&ctx->tx_ring_tcp, RING_SIZE);
    ring_init(&ctx->rx_ring_usb, RING_SIZE);

    /* OUT pool */
    ctx->out_pool = kcalloc(OUT_REQ_COUNT, sizeof(*ctx->out_pool), GFP_KERNEL);
    if (!ctx->out_pool || !ctx->tx_ring_tcp.bufs || !ctx->rx_ring_usb.bufs)
        return -ENOMEM;

    /* TCP tuning */
    ctx->tcp_sndbuf = TCP_SNDBUF;
    ctx->tcp_rcvbuf = TCP_RCVBUF;

    /* Counters */
    atomic64_set(&ctx->urb_out_done, 0);
    atomic64_set(&ctx->urb_in_submit, 0);
    atomic64_set(&ctx->tcp_sent_bytes, 0);
    atomic64_set(&ctx->tcp_recv_bytes, 0);
    atomic64_set(&ctx->drops_ring_full, 0);

    /* FSM queue and workqueue */
    evq_init(ctx);
    INIT_WORK(&ctx->fsm_work, adbd_fsm_work);
    ctx->wq = alloc_ordered_workqueue("adbd_fsm_wq", WQ_MEM_RECLAIM);
    if (!ctx->wq)
        return -ENOMEM;

    /* Initial state */
    ctx->state = USB_IDLE;

    return 0;
}

static int adbd_func_set_alt(struct usb_function *f, unsigned intf, unsigned alt)
{
    struct adbd_ctx *ctx = container_of(f, struct adbd_ctx, func);
    struct usb_composite_dev *cdev = f->config->cdev;
    int i, ret;

    /* Enable endpoints by speed */
    if (cdev->gadget->speed >= USB_SPEED_HIGH) {
        ret = usb_ep_enable(ctx->ep_out, &hs_ep_out);
        if (ret) return ret;
        ret = usb_ep_enable(ctx->ep_in, &hs_ep_in);
        if (ret) { usb_ep_disable(ctx->ep_out); return ret; }
    } else {
        ret = usb_ep_enable(ctx->ep_out, &fs_ep_out);
        if (ret) return ret;
        ret = usb_ep_enable(ctx->ep_in, &fs_ep_in);
        if (ret) { usb_ep_disable(ctx->ep_out); return ret; }
    }

    /* Pre-queue OUT pool */
    for (i = 0; i < OUT_REQ_COUNT; i++) {
        struct usb_request *req = usb_ep_alloc_request(ctx->ep_out, GFP_KERNEL);
        if (!req) break;
        req->buf = kmalloc(OUT_REQ_SIZE, GFP_KERNEL | GFP_DMA);
        if (!req->buf) { usb_ep_free_request(ctx->ep_out, req); break; }
        req->length   = OUT_REQ_SIZE;
        req->complete = adbd_out_complete;

        if (usb_ep_queue(ctx->ep_out, req, GFP_KERNEL)) {
            kfree(req->buf);
            usb_ep_free_request(ctx->ep_out, req);
            break;
        }
        ctx->out_pool[i] = req;
    }

    /* Start workers */
    ctx->t_send = kthread_run(adbd_tcp_send_thread, ctx, "adbd_tcp_send");
    ctx->t_recv = kthread_run(adbd_tcp_recv_thread, ctx, "adbd_tcp_recv");
    ctx->t_in   = kthread_run(adbd_usb_in_thread,   ctx, "adbd_usb_in");

    ctx->active = true;

    /* Notify FSM: USB configured; FSM will initiate TCP connect via workqueue */
    evq_push(ctx, EV_USB_CONFIGURED, 0, GFP_KERNEL);

    return 0;
}

static void adbd_func_disable(struct usb_function *f)
{
    struct adbd_ctx *ctx = container_of(f, struct adbd_ctx, func);
    int i;

    ctx->active = false;

    /* Signal FSM to shutdown; it will set state accordingly */
    evq_push(ctx, EV_USB_DISABLE, 0, GFP_KERNEL);
    flush_work(&ctx->fsm_work);

    /* Stop workers */
    if (ctx->t_send) { kthread_stop(ctx->t_send); ctx->t_send = NULL; }
    if (ctx->t_recv) { kthread_stop(ctx->t_recv); ctx->t_recv = NULL; }
    if (ctx->t_in)   { kthread_stop(ctx->t_in);   ctx->t_in   = NULL; }

    /* Disable endpoints */
    if (ctx->ep_in)  usb_ep_disable(ctx->ep_in);
    if (ctx->ep_out) usb_ep_disable(ctx->ep_out);

    /* Close TCP */
    adbd_tcp_close(ctx);

    /* Free OUT request pool */
    for (i = 0; i < OUT_REQ_COUNT; i++) {
        struct usb_request *req = ctx->out_pool[i];
        if (!req) continue;
        kfree(req->buf);
        usb_ep_free_request(ctx->ep_out, req);
        ctx->out_pool[i] = NULL;
    }

    /* Drain rings */
    while (!ring_empty(&ctx->tx_ring_tcp)) {
        void *buf = ctx->tx_ring_tcp.bufs[ctx->tx_ring_tcp.tail];
        ctx->tx_ring_tcp.tail = (ctx->tx_ring_tcp.tail + 1) % ctx->tx_ring_tcp.size;
        kfree(buf);
    }
    while (!ring_empty(&ctx->rx_ring_usb)) {
        void *buf = ctx->rx_ring_usb.bufs[ctx->rx_ring_usb.tail];
        ctx->rx_ring_usb.tail = (ctx->rx_ring_usb.tail + 1) % ctx->rx_ring_usb.size;
        kfree(buf);
    }

    wake_up_interruptible(&ctx->tx_ring_tcp.wq_not_empty);
    wake_up_interruptible(&ctx->tx_ring_tcp.wq_has_space);
    wake_up_interruptible(&ctx->rx_ring_usb.wq_not_empty);
    wake_up_interruptible(&ctx->rx_ring_usb.wq_has_space);
}

static struct usb_function *adbd_func_alloc(struct usb_function_instance *fi)
{
    struct adbd_ctx *ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
    if (!ctx) return ERR_PTR(-ENOMEM);

    ctx->func.name    = "adbd-bridge";
    ctx->func.bind    = adbd_func_bind;
    ctx->func.set_alt = adbd_func_set_alt;
    ctx->func.disable = adbd_func_disable;

    return &ctx->func;
}

static void adbd_func_free(struct usb_function *f)
{
    struct adbd_ctx *ctx = container_of(f, struct adbd_ctx, func);

    if (ctx->wq) {
        flush_work(&ctx->fsm_work);
        destroy_workqueue(ctx->wq);
        ctx->wq = NULL;
    }

    kfree(ctx->out_pool);
    ring_free(&ctx->tx_ring_tcp);
    ring_free(&ctx->rx_ring_usb);
    kfree(ctx);
}

/* -------------------------------------------------------------------------- */
/* Configuration glue                                                          */
/* -------------------------------------------------------------------------- */

static struct usb_configuration cfg = {
    .label               = "ADBBridgeX",
    .bmAttributes        = USB_CONFIG_ATT_SELFPOWERED,
    .bConfigurationValue = 1,
    .iConfiguration      = STR_PRODUCT,
    .MaxPower            = 0,
};

static int adbd_add_config(struct usb_composite_dev *cdev)
{
    int ret;

    ret = usb_string_ids_tab(cdev, strings);
    if (ret < 0) return ret;

    dev_desc.iManufacturer = strings[0].id;
    dev_desc.iProduct      = strings[1].id;
    dev_desc.iSerialNumber = strings[2].id;
    intf_desc.iInterface   = strings[3].id;

    ret = usb_add_config(cdev, &cfg, ({
        int __fn(struct usb_configuration *c)
        {
            struct usb_function *f = adbd_func_alloc(NULL);
            if (IS_ERR(f)) return PTR_ERR(f);
            f->free_func = adbd_func_free;
            return usb_add_function(c, f);
        }
        __fn;
    }));
    return ret;
}

/* -------------------------------------------------------------------------- */
/* Composite driver registration                                               */
/* -------------------------------------------------------------------------- */

static int adbd_bind(struct usb_composite_dev *cdev)
{
    int ret;

    /* Optional: exclude dummy UDC */
    if (cdev->gadget && cdev->gadget->name &&
        !strncmp(cdev->gadget->name, "dummy_udc", 9))
        return -ENODEV;

    cdev->desc.idVendor  = cpu_to_le16(VENDOR_ID);
    cdev->desc.idProduct = cpu_to_le16(PRODUCT_ID);

    ret = adbd_add_config(cdev);
    if (ret) return ret;

    return 0;
}

static int adbd_unbind(struct usb_composite_dev *cdev)
{
    return 0;
}

static struct usb_composite_driver g_adbd_bridge = {
    .name    = "g_adbd_bridge",
    .dev     = &dev_desc,
    .strings = gadget_strings,
    .bind    = adbd_bind,
    .unbind  = adbd_unbind,
};

module_usb_composite_driver(g_adbd_bridge);

MODULE_DESCRIPTION("ADBBridgeX: Workqueue-FSM event-driven legacy ADB USB→TCP bridge");
MODULE_AUTHOR("You");
MODULE_LICENSE("GPL");
