/* drivers/usb/gadget/function/f_adb.c
 *
 * Project E: ADB function (FF/42/01) with bridge tcpip:5555 (ported from Project D)
 * - Single interface FF/42/01
 * - Alt0: 0 endpoints (idle)
 * - Alt1: 2 endpoints (bulk OUT + bulk IN)
 * - When enabled, a kernel TCP listener on port 5555 forwards traffic:
 *     USB BULK OUT -> TCP socket send
 *     TCP socket recv -> USB BULK IN
 *
 * SPDX-License-Identifier: GPL-2.0
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/usb/composite.h>
#include <linux/kthread.h>
#include <linux/net.h>
#include <linux/in.h>
#include <net/sock.h>

/* ---------------- Bridge control (module params) ---------------- */
static bool bridge_enable = true;
module_param(bridge_enable, bool, 0644);
MODULE_PARM_DESC(bridge_enable, "Enable ADB TCP bridge on tcpip:5555");

static unsigned short bridge_port = 5555;
module_param(bridge_port, ushort, 0644);
MODULE_PARM_DESC(bridge_port, "ADB TCP bridge port (default 5555)");

/* ---------------- Function context ---------------- */
struct f_adb {
    struct usb_function func;
    struct usb_ep *ep_in, *ep_out;
    u8 intf;

    /* Bridge state */
    struct task_struct *listen_task;
    struct socket *listen_sock;
    struct socket *session_sock;
    struct task_struct *rx_task;
    struct task_struct *tx_task;
    bool bridge_running;
    spinlock_t bridge_lock;
};

/* ---------------- Descriptors ---------------- */
static struct usb_interface_descriptor adb_intf_alt0 = {
    .bLength            = sizeof(adb_intf_alt0),
    .bDescriptorType    = USB_DT_INTERFACE,
    .bInterfaceNumber   = 0,
    .bAlternateSetting  = 0,
    .bNumEndpoints      = 0,
    .bInterfaceClass    = 0xFF,
    .bInterfaceSubClass = 0x42,
    .bInterfaceProtocol = 0x01,
    .iInterface         = 0,
};

static struct usb_interface_descriptor adb_intf_alt1 = {
    .bLength            = sizeof(adb_intf_alt1),
    .bDescriptorType    = USB_DT_INTERFACE,
    .bInterfaceNumber   = 0,
    .bAlternateSetting  = 1,
    .bNumEndpoints      = 2,
    .bInterfaceClass    = 0xFF,
    .bInterfaceSubClass = 0x42,
    .bInterfaceProtocol = 0x01,
    .iInterface         = 0,
};

/* FS endpoints */
static struct usb_endpoint_descriptor fs_out = {
    .bLength          = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType  = USB_DT_ENDPOINT,
    .bEndpointAddress = USB_DIR_OUT,
    .bmAttributes     = USB_ENDPOINT_XFER_BULK,
    .wMaxPacketSize   = cpu_to_le16(64),
};
static struct usb_endpoint_descriptor fs_in = {
    .bLength          = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType  = USB_DT_ENDPOINT,
    .bEndpointAddress = USB_DIR_IN,
    .bmAttributes     = USB_ENDPOINT_XFER_BULK,
    .wMaxPacketSize   = cpu_to_le16(64),
};

/* HS endpoints (mirror FS addresses) */
static struct usb_endpoint_descriptor hs_out = {
    .bLength          = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType  = USB_DT_ENDPOINT,
    .bEndpointAddress = USB_DIR_OUT,
    .bmAttributes     = USB_ENDPOINT_XFER_BULK,
    .wMaxPacketSize   = cpu_to_le16(512),
};
static struct usb_endpoint_descriptor hs_in = {
    .bLength          = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType  = USB_DT_ENDPOINT,
    .bEndpointAddress = USB_DIR_IN,
    .bmAttributes     = USB_ENDPOINT_XFER_BULK,
    .wMaxPacketSize   = cpu_to_le16(512),
};

static struct usb_descriptor_header *adb_fs_descs[] = {
    (struct usb_descriptor_header *)&adb_intf_alt0,
    (struct usb_descriptor_header *)&adb_intf_alt1,
    (struct usb_descriptor_header *)&fs_out,
    (struct usb_descriptor_header *)&fs_in,
    NULL,
};
static struct usb_descriptor_header *adb_hs_descs[] = {
    (struct usb_descriptor_header *)&adb_intf_alt0,
    (struct usb_descriptor_header *)&adb_intf_alt1,
    (struct usb_descriptor_header *)&hs_out,
    (struct usb_descriptor_header *)&hs_in,
    NULL,
};

/* ---------------- USB helpers ---------------- */
static inline struct usb_request *adb_alloc_req(struct usb_ep *ep, size_t len)
{
    struct usb_request *req = usb_ep_alloc_request(ep, GFP_KERNEL);
    if (!req) return NULL;
    req->buf = kzalloc(len, GFP_KERNEL);
    if (!req->buf) { usb_ep_free_request(ep, req); return NULL; }
    req->length = len;
    return req;
}

static void adb_free_req(struct usb_ep *ep, struct usb_request *req)
{
    if (!req) return;
    kfree(req->buf);
    usb_ep_free_request(ep, req);
}

/* ---------------- TCP bridge threads ---------------- */

/* RX: TCP -> USB IN */
static int adb_bridge_rx(void *arg)
{
    struct f_adb *ctx = arg;
    struct socket *sk = ctx->session_sock;
    struct usb_ep *ep_in = ctx->ep_in;
    int ret;

    allow_signal(SIGKILL);

    while (!kthread_should_stop()) {
        struct usb_request *req = adb_alloc_req(ep_in, 512);
        struct kvec iov = { .iov_base = req ? req->buf : NULL, .iov_len = req ? req->length : 0 };
        struct msghdr msg = { .msg_flags = MSG_DONTWAIT };
        size_t got = 0;

        if (!req) { ret = -ENOMEM; break; }

        ret = kernel_recvmsg(sk, &msg, &iov, 1, iov.iov_len, msg.msg_flags);
        if (ret <= 0) { adb_free_req(ep_in, req); if (ret == -EAGAIN) { schedule_timeout_interruptible(HZ/50); continue; } break; }

        got = ret;
        req->length = got;
        req->complete = NULL; /* fire-and-forget */
        ret = usb_ep_queue(ep_in, req, GFP_ATOMIC);
        if (ret) { adb_free_req(ep_in, req); schedule_timeout_interruptible(HZ/100); }
    }
    return 0;
}

/* TX: USB OUT -> TCP */
static void adb_out_complete(struct usb_ep *ep, struct usb_request *req)
{
    complete((struct completion *)req->context);
}

static int adb_bridge_tx(void *arg)
{
    struct f_adb *ctx = arg;
    struct socket *sk = ctx->session_sock;
    struct usb_ep *ep_out = ctx->ep_out;
    int ret;

    allow_signal(SIGKILL);

    while (!kthread_should_stop()) {
        struct usb_request *req = adb_alloc_req(ep_out, 512);
        struct completion done;

        if (!req) { ret = -ENOMEM; break; }

        init_completion(&done);
        req->complete = adb_out_complete;
        req->context  = &done;

        /* queue OUT request to receive data from host */
        ret = usb_ep_queue(ep_out, req, GFP_ATOMIC);
        if (ret) { adb_free_req(ep_out, req); schedule_timeout_interruptible(HZ/50); continue; }

        /* wait for completion */
        wait_for_completion_interruptible(&done);

        /* push to TCP */
        if (ctx->session_sock) {
            struct kvec iov = { .iov_base = req->buf, .iov_len = req->actual };
            struct msghdr msg = { .msg_flags = MSG_DONTWAIT };
            ret = kernel_sendmsg(sk, &msg, &iov, 1, iov.iov_len);
            /* ignore short sends for simplicity; retry semantics can be added */
        }

        adb_free_req(ep_out, req);
    }
    return 0;
}

/* Listener: accept one session and spawn rx/tx workers */
static int adb_bridge_listen(void *arg)
{
    struct f_adb *ctx = arg;
    int ret;
    struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_addr.s_addr = htonl(INADDR_LOOPBACK), /* default: loopback; change to INADDR_ANY if needed */
        .sin_port = htons(bridge_port),
    };

    allow_signal(SIGKILL);

    ret = sock_create_kern(&init_net, AF_INET, SOCK_STREAM, IPPROTO_TCP, &ctx->listen_sock);
    if (ret) return ret;

    ret = kernel_bind(ctx->listen_sock, (struct sockaddr *)&addr, sizeof(addr));
    if (ret) goto out_close;

    ret = kernel_listen(ctx->listen_sock, 1);
    if (ret) goto out_close;

    while (!kthread_should_stop()) {
        ret = kernel_accept(ctx->listen_sock, &ctx->session_sock, 0);
        if (ret) { schedule_timeout_interruptible(HZ/10); continue; }

        /* Spawn RX/TX for this session */
        ctx->rx_task = kthread_run(adb_bridge_rx, ctx, "adb_rx");
        if (IS_ERR(ctx->rx_task)) { ret = PTR_ERR(ctx->rx_task); ctx->rx_task = NULL; goto drop_session; }

        ctx->tx_task = kthread_run(adb_bridge_tx, ctx, "adb_tx");
        if (IS_ERR(ctx->tx_task)) { ret = PTR_ERR(ctx->tx_task); ctx->tx_task = NULL; goto stop_rx; }

        /* wait until either thread finishes, then tear down and accept next */
        while (!kthread_should_stop()) {
            if (!ctx->rx_task || !ctx->tx_task)
                break;
            schedule_timeout_interruptible(HZ/10);
        }

stop_rx:
        if (ctx->rx_task) { kthread_stop(ctx->rx_task); ctx->rx_task = NULL; }
drop_session:
        if (ctx->session_sock) { sock_release(ctx->session_sock); ctx->session_sock = NULL; }
    }

out_close:
    if (ctx->listen_sock) { kernel_sock_shutdown(ctx->listen_sock, SHUT_RDWR); sock_release(ctx->listen_sock); ctx->listen_sock = NULL; }
    return 0;
}

/* ---------------- USB function callbacks ---------------- */

static int adb_bind(struct usb_configuration *c, struct usb_function *f)
{
    struct f_adb *ctx = container_of(f, struct f_adb, func);
    struct usb_gadget *g = c->cdev->gadget;
    int intf;

    spin_lock_init(&ctx->bridge_lock);

    intf = usb_interface_id(c, f);
    if (intf < 0)
        return intf;
    ctx->intf = intf;

    adb_intf_alt0.bInterfaceNumber = intf;
    adb_intf_alt1.bInterfaceNumber = intf;

    ctx->ep_out = usb_ep_autoconfig(g, &fs_out);
    if (!ctx->ep_out)
        return -ENODEV;
    hs_out.bEndpointAddress = fs_out.bEndpointAddress;

    ctx->ep_in = usb_ep_autoconfig(g, &fs_in);
    if (!ctx->ep_in)
        return -ENODEV;
    hs_in.bEndpointAddress = fs_in.bEndpointAddress;

    return usb_assign_descriptors(f, adb_fs_descs, adb_hs_descs, NULL, NULL);
}

static void adb_unbind(struct usb_configuration *c, struct usb_function *f)
{
    struct f_adb *ctx = container_of(f, struct f_adb, func);
    usb_free_all_descriptors(f);
    /* Ensure bridge is stopped */
    if (ctx->listen_task) { kthread_stop(ctx->listen_task); ctx->listen_task = NULL; }
    if (ctx->rx_task) { kthread_stop(ctx->rx_task); ctx->rx_task = NULL; }
    if (ctx->tx_task) { kthread_stop(ctx->tx_task); ctx->tx_task = NULL; }
    if (ctx->session_sock) { sock_release(ctx->session_sock); ctx->session_sock = NULL; }
    if (ctx->listen_sock) { kernel_sock_shutdown(ctx->listen_sock, SHUT_RDWR); sock_release(ctx->listen_sock); ctx->listen_sock = NULL; }
}

static int adb_set_alt(struct usb_function *f, unsigned intf, unsigned alt)
{
    struct f_adb *ctx = container_of(f, struct f_adb, func);
    int ret;

    if (intf != ctx->intf) return -EINVAL;

    if (alt == 0) {
        usb_ep_disable(ctx->ep_out);
        usb_ep_disable(ctx->ep_in);

        /* stop bridge if running */
        if (ctx->bridge_running) {
            if (ctx->listen_task) { kthread_stop(ctx->listen_task); ctx->listen_task = NULL; }
            if (ctx->rx_task) { kthread_stop(ctx->rx_task); ctx->rx_task = NULL; }
            if (ctx->tx_task) { kthread_stop(ctx->tx_task); ctx->tx_task = NULL; }
            if (ctx->session_sock) { sock_release(ctx->session_sock); ctx->session_sock = NULL; }
            if (ctx->listen_sock) { kernel_sock_shutdown(ctx->listen_sock, SHUT_RDWR); sock_release(ctx->listen_sock); ctx->listen_sock = NULL; }
            ctx->bridge_running = false;
        }
        return 0;
    } else if (alt == 1) {
        ret = usb_ep_enable(ctx->ep_out);
        if (ret) return ret;
        ret = usb_ep_enable(ctx->ep_in);
        if (ret) { usb_ep_disable(ctx->ep_out); return ret; }

        /* start bridge on enable if allowed */
        if (bridge_enable && !ctx->bridge_running) {
            ctx->listen_task = kthread_run(adb_bridge_listen, ctx, "adb_listen");
            if (IS_ERR(ctx->listen_task)) {
                ret = PTR_ERR(ctx->listen_task);
                ctx->listen_task = NULL;
                /* endpoints remain enabled even if bridge fails */
            } else {
                ctx->bridge_running = true;
            }
        }
        return 0;
    }
    return -EINVAL;
}

static void adb_disable(struct usb_function *f)
{
    struct f_adb *ctx = container_of(f, struct f_adb, func);
    usb_ep_disable(ctx->ep_out);
    usb_ep_disable(ctx->ep_in);

    /* stop bridge if running */
    if (ctx->listen_task) { kthread_stop(ctx->listen_task); ctx->listen_task = NULL; }
    if (ctx->rx_task) { kthread_stop(ctx->rx_task); ctx->rx_task = NULL; }
    if (ctx->tx_task) { kthread_stop(ctx->tx_task); ctx->tx_task = NULL; }
    if (ctx->session_sock) { sock_release(ctx->session_sock); ctx->session_sock = NULL; }
    if (ctx->listen_sock) { kernel_sock_shutdown(ctx->listen_sock, SHUT_RDWR); sock_release(ctx->listen_sock); ctx->listen_sock = NULL; }
    ctx->bridge_running = false;
}

static void adb_free(struct usb_function *f)
{
    struct f_adb *ctx = container_of(f, struct f_adb, func);
    kfree(ctx);
}

/* ---------------- Function driver plumbing ---------------- */

static void adb_free_inst(struct usb_function_instance *fi)
{
    kfree(fi);
}

static struct usb_function_instance *adb_alloc_inst(void)
{
    struct usb_function_instance *fi;

    fi = kzalloc(sizeof(*fi), GFP_KERNEL);
    if (!fi)
        return ERR_PTR(-ENOMEM);

    fi->free_func_inst = adb_free_inst;
    return fi;
}

static struct usb_function *adb_alloc(struct usb_function_instance *fi)
{
    struct f_adb *ctx;

    ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
    if (!ctx)
        return ERR_PTR(-ENOMEM);

    ctx->func.name      = "adb";
    ctx->func.bind      = adb_bind;
    ctx->func.unbind    = adb_unbind;
    ctx->func.set_alt   = adb_set_alt;
    ctx->func.disable   = adb_disable;
    ctx->func.free_func = adb_free;

    return &ctx->func;
}

static struct usb_function_driver adb_func_driver = {
    .name       = "adb",
    .mod        = THIS_MODULE,
    .alloc_inst = adb_alloc_inst,
    .alloc_func = adb_alloc,
};

static int __init f_adb_init(void)
{
    return usb_function_register(&adb_func_driver);
}

static void __exit f_adb_exit(void)
{
    usb_function_unregister(&adb_func_driver);
}

module_init(f_adb_init);
module_exit(f_adb_exit);

MODULE_DESCRIPTION("Project E: ADB function with tcpip:5555 bridge");
MODULE_AUTHOR("George");
MODULE_LICENSE("GPL");
