/*
 * drivers/usb/gadget/function/f_adb.c
 *
 * ADB gadget function with:
 *  - configfs-backed instance (opts->func_inst.group)
 *  - two 1:1 pools (rx_pool: URB; tx_pool: skb)
 *  - thin OUT completion that only enqueues a small FIFO event
 *  - worker copies URB payload -> skb and performs kernel_sendmsg under RCU
 *  - RX kthread owns socket lifecycle and publishes ctx->adbd_sock via RCU
 *
 * SPDX-License-Identifier: GPL-2.0
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/usb/composite.h>
#include <linux/configfs.h>
#include <linux/workqueue.h>
#include <linux/kthread.h>
#include <linux/skbuff.h>
#include <linux/spinlock.h>
#include <linux/list.h>
#include <linux/net.h>
#include <net/sock.h>
#include <linux/in.h>
#include <linux/socket.h>
#include <linux/jiffies.h>
#include <linux/atomic.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/errno.h>
#include <linux/tcp.h>
#include <linux/rcupdate.h>
#include <linux/mutex.h>

/* Tunables */
#define ADB_OUT_REQS_DEF     16
#define ADB_OUT_BUFSIZE_DEF  512
#define ADB_IN_BUFSIZE_DEF   512
#define EVT_RING_MULT        2    /* event ring capacity = out_req_count * multiplier */
#define MAX_PENDING          64
#define STATE_RETRY_MAX      8
#define STATE_RETRY_DELAY_MS 200
#define CONNECT_BACKOFF_MS   200
#define MAX_CONNECT_RETRIES  6

static unsigned short bridge_port = 5555;
module_param(bridge_port, ushort, 0644);
MODULE_PARM_DESC(bridge_port, "ADB TCP bridge port (default 5555)");

static bool adb_enabled = true;
module_param(adb_enabled, bool, 0644);
MODULE_PARM_DESC(adb_enabled, "Enable ADB bridge runtime when set_alt runs (default true)");

/* ---------------- Configfs-backed options ---------------- */

struct adb_opts {
    struct usb_function_instance func_inst; /* must be first; contains .group */
    struct mutex lock;
    atomic_t refcnt;

    bool adb_enabled;
    unsigned short port;
    unsigned int out_count;
    unsigned int out_size;
    unsigned int in_size;
};

static inline struct adb_opts *fi_to_adb_opts(struct usb_function_instance *fi)
{
    return container_of(fi, struct adb_opts, func_inst);
}

/* configfs attribute helpers (use func_inst.group) */

static ssize_t adb_optsport_show(struct config_item *item, char *page)
{
    struct adb_opts *opts = container_of(to_config_group(item), struct adb_opts, func_inst.group);
    ssize_t len;

    if (!opts)
        return -EINVAL;

    mutex_lock(&opts->lock);
    len = scnprintf(page, PAGE_SIZE, "%u\n", (unsigned int)opts->port);
    mutex_unlock(&opts->lock);

    return len;
}

static ssize_t adb_optsport_store(struct config_item *item, const char *page, size_t count)
{
    struct adb_opts *opts = container_of(to_config_group(item), struct adb_opts, func_inst.group);
    unsigned long v;
    int ret;

    if (!opts)
        return -EINVAL;

    ret = kstrtoul(page, 0, &v);
    if (ret)
        return ret;

    mutex_lock(&opts->lock);
    if (atomic_read(&opts->refcnt)) {
        mutex_unlock(&opts->lock);
        return -EBUSY;
    }
    opts->port = (unsigned short)v;
    mutex_unlock(&opts->lock);

    return count;
}
CONFIGFS_ATTR(adb_opts, port);

static ssize_t adb_optsout_count_show(struct config_item *item, char *page)
{
    struct adb_opts *opts = container_of(to_config_group(item), struct adb_opts, func_inst.group);
    ssize_t len;

    if (!opts)
        return -EINVAL;

    mutex_lock(&opts->lock);
    len = scnprintf(page, PAGE_SIZE, "%u\n", opts->out_count);
    mutex_unlock(&opts->lock);

    return len;
}

static ssize_t adb_optsout_count_store(struct config_item *item, const char *page, size_t count)
{
    struct adb_opts *opts = container_of(to_config_group(item), struct adb_opts, func_inst.group);
    unsigned long v;
    int ret;

    if (!opts)
        return -EINVAL;

    ret = kstrtoul(page, 0, &v);
    if (ret)
        return ret;

    mutex_lock(&opts->lock);
    if (atomic_read(&opts->refcnt)) {
        mutex_unlock(&opts->lock);
        return -EBUSY;
    }
    opts->out_count = (unsigned int)v;
    mutex_unlock(&opts->lock);

    return count;
}
CONFIGFS_ATTR(adb_opts, out_count);

static ssize_t adb_optsout_size_show(struct config_item *item, char *page)
{
    struct adb_opts *opts = container_of(to_config_group(item), struct adb_opts, func_inst.group);
    ssize_t len;

    if (!opts)
        return -EINVAL;

    mutex_lock(&opts->lock);
    len = scnprintf(page, PAGE_SIZE, "%u\n", opts->out_size);
    mutex_unlock(&opts->lock);

    return len;
}

static ssize_t adb_optsout_size_store(struct config_item *item, const char *page, size_t count)
{
    struct adb_opts *opts = container_of(to_config_group(item), struct adb_opts, func_inst.group);
    unsigned long v;
    int ret;

    if (!opts)
        return -EINVAL;

    ret = kstrtoul(page, 0, &v);
    if (ret)
        return ret;

    mutex_lock(&opts->lock);
    if (atomic_read(&opts->refcnt)) {
        mutex_unlock(&opts->lock);
        return -EBUSY;
    }
    opts->out_size = (unsigned int)v;
    mutex_unlock(&opts->lock);

    return count;
}
CONFIGFS_ATTR(adb_opts, out_size);

static ssize_t adb_optsin_size_show(struct config_item *item, char *page)
{
    struct adb_opts *opts = container_of(to_config_group(item), struct adb_opts, func_inst.group);
    ssize_t len;

    if (!opts)
        return -EINVAL;

    mutex_lock(&opts->lock);
    len = scnprintf(page, PAGE_SIZE, "%u\n", opts->in_size);
    mutex_unlock(&opts->lock);

    return len;
}

static ssize_t adb_optsin_size_store(struct config_item *item, const char *page, size_t count)
{
    struct adb_opts *opts = container_of(to_config_group(item), struct adb_opts, func_inst.group);
    unsigned long v;
    int ret;

    if (!opts)
        return -EINVAL;

    ret = kstrtoul(page, 0, &v);
    if (ret)
        return ret;

    mutex_lock(&opts->lock);
    if (atomic_read(&opts->refcnt)) {
        mutex_unlock(&opts->lock);
        return -EBUSY;
    }
    opts->in_size = (unsigned int)v;
    mutex_unlock(&opts->lock);

    return count;
}
CONFIGFS_ATTR(adb_opts, in_size);

static struct configfs_attribute *adb_attrs[] = {
    &adb_optsattr_port,
    &adb_optsattr_out_count,
    &adb_optsattr_out_size,
    &adb_optsattr_in_size,
    NULL,
};

static void adb_attr_release(struct config_item *item)
{
    struct adb_opts *opts = container_of(to_config_group(item), struct adb_opts, func_inst.group);
    usb_put_function_instance(&opts->func_inst);
}

static struct configfs_item_operations adb_item_ops = {
    .release = adb_attr_release,
};

static struct config_item_type adb_func_type = {
    .ct_item_ops = &adb_item_ops,
    .ct_attrs = adb_attrs,
    .ct_owner = THIS_MODULE,
};

/* free_func_inst: actually free instance memory when USB core calls it */
static void adb_free_instance(struct usb_function_instance *fi)
{
    struct adb_opts *opts = fi_to_adb_opts(fi);

    if (atomic_read(&opts->refcnt))
        pr_warn("f_adb: free_instance called with refcnt=%d\n",
            atomic_read(&opts->refcnt));

    mutex_destroy(&opts->lock);
    pr_info("f_adb: free_instance freeing opts=%p\n", opts);
    kfree(opts);
}

/* ---------------- ADB headers, events, and state machine ---------------- */

struct adb_header {
    __le32 command;
    __le32 arg0;
    __le32 arg1;
    __le32 length;
    __le32 checksum;
    __le32 magic;
} __packed;

static bool parse_adb_header(const void *buf, size_t len, struct adb_header *hdr)
{
    if (!buf || len < sizeof(*hdr) || !hdr) return false;
    memcpy(hdr, buf, sizeof(*hdr));
    return true;
}

static void build_adb_clse(const struct adb_header *in, struct adb_header *out)
{
    if (!in || !out) return;
    out->command  = cpu_to_le32((u32)('C' | ('L'<<8) | ('S'<<16) | ('E'<<24))); /* "CLSE" */
    out->arg0     = in->arg1;
    out->arg1     = in->arg0;
    out->length   = cpu_to_le32(0);
    out->checksum = cpu_to_le32(0);
    out->magic    = in->magic;
}

/* ---------------- Pools and context ---------------- */

struct pool_elem {
    struct usb_request *req;    /* rx_pool: bound to OUT ep (URB) */
    struct sk_buff     *skb;    /* tx_pool: skb to send (scratch) */
    struct f_adb       *owner;  /* back-pointer to parent context */
    unsigned int        index;  /* index in the pool array */
    atomic_t            in_use; /* 0 = free, 1 = in use (atomic to avoid races) */
};

enum adb_state {
    ADB_STATE_STOPPED = 0,
    ADB_STATE_STARTING,
    ADB_STATE_RUNNING,
    ADB_STATE_STOPPING,
    ADB_STATE_ERROR,
};

/* FIFO event from OUT completion to worker */
struct out_evt {
    u16 idx;          /* rx_pool index that completed */
    u16 len;          /* req->actual length */
};

struct f_adb {
    struct usb_function port_func;
    struct usb_ep *in;
    struct usb_ep *out;
    struct device *dev;
    u8 data_id;

    /* RCU-published socket; RX thread owns connect/teardown */
    struct socket __rcu *adbd_sock;

    /* bridge queue used as backpressure buffer when socket absent */
    struct sk_buff_head out_queue;
    atomic_t out_skb_count;

    /* pools */
    struct pool_elem *rx_pool;      /* size N = out_req_count */
    struct pool_elem *tx_pool;      /* size N = out_req_count */
    unsigned int pool_size;

    /* FIFO from out_complete to worker */
    struct out_evt *evt_ring;
    unsigned int evt_cap;
    unsigned int evt_head;
    unsigned int evt_tail;
    spinlock_t evt_lock;

    /* state machine */
    enum adb_state current_state;
    enum adb_state requested_state;
    spinlock_t state_lock;
    unsigned int state_retries;
    struct work_struct state_work;

    /* worker hints */
    atomic_t is_sock_connected;
    atomic_t drain_request;
    atomic_t connect_request;

    /* counters */
    atomic_t primed_out_count;
    atomic_t completed_out_count;
    atomic_t dropped_on_error;

    /* sizes */
    unsigned int out_req_count;
    size_t out_req_size;
    size_t in_buf_size;

    /* RX thread */
    struct task_struct *rx_task;
    wait_queue_head_t rx_wq;

    u8   *pending;        /* allocated buffer holding partial message */
    size_t pending_len;   /* bytes currently in pending */
    size_t pending_expect;/* expected payload length from header (payload only) */
    ktime_t pending_ts;   /* timestamp when pending started */
    size_t pending_cap;   /* current allocation size of pending buffer */

    /* add to struct f_adb */
    wait_queue_head_t tx_wq;          /* sender thread waitqueue */
    struct task_struct *sender_task;  /* sender kthread */

    /* opts */
    struct adb_opts *opts;

    /* enable tracking */
    bool enabled_in_by_us;
    bool enabled_out_by_us;
};

#define func_to_adb(f) container_of(f, struct f_adb, port_func)

/* --- descriptors (FS/HS/SS) --- */

static struct usb_interface_descriptor adb_interface_desc = {
    .bLength = sizeof(adb_interface_desc),
    .bDescriptorType = USB_DT_INTERFACE,
    .bInterfaceNumber = 0,
    .bAlternateSetting = 0,
    .bNumEndpoints = 2,
    .bInterfaceClass = USB_CLASS_VENDOR_SPEC,
    .bInterfaceSubClass = 0x42,
    .bInterfaceProtocol = 0x01,
    .iInterface = 0,
};

static struct usb_endpoint_descriptor adb_fs_out_desc = {
    .bLength = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType = USB_DT_ENDPOINT,
    .bEndpointAddress = USB_DIR_OUT,
    .bmAttributes = USB_ENDPOINT_XFER_BULK,
    .wMaxPacketSize = cpu_to_le16(64),
};

static struct usb_endpoint_descriptor adb_fs_in_desc = {
    .bLength = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType = USB_DT_ENDPOINT,
    .bEndpointAddress = USB_DIR_IN,
    .bmAttributes = USB_ENDPOINT_XFER_BULK,
    .wMaxPacketSize = cpu_to_le16(64),
};

/* High-Speed descriptors (512 bytes bulk) */
static struct usb_endpoint_descriptor adb_hs_out_desc = {
    .bLength = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType = USB_DT_ENDPOINT,
    .bEndpointAddress = USB_DIR_OUT,
    .bmAttributes = USB_ENDPOINT_XFER_BULK,
    .wMaxPacketSize = cpu_to_le16(512),
};

static struct usb_endpoint_descriptor adb_hs_in_desc = {
    .bLength = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType = USB_DT_ENDPOINT,
    .bEndpointAddress = USB_DIR_IN,
    .bmAttributes = USB_ENDPOINT_XFER_BULK,
    .wMaxPacketSize = cpu_to_le16(512),
};

/* SuperSpeed descriptors (1024 bytes bulk) and companion */
static struct usb_endpoint_descriptor adb_ss_in_desc = {
    .bLength = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType = USB_DT_ENDPOINT,
    .bEndpointAddress = USB_DIR_IN,
    .bmAttributes = USB_ENDPOINT_XFER_BULK,
    .wMaxPacketSize = cpu_to_le16(1024),
};

static struct usb_endpoint_descriptor adb_ss_out_desc = {
    .bLength = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType = USB_DT_ENDPOINT,
    .bEndpointAddress = USB_DIR_OUT,
    .bmAttributes = USB_ENDPOINT_XFER_BULK,
    .wMaxPacketSize = cpu_to_le16(1024),
};

static struct usb_ss_ep_comp_descriptor adb_ss_bulk_comp_desc = {
    .bLength = sizeof adb_ss_bulk_comp_desc,
    .bDescriptorType = USB_DT_SS_ENDPOINT_COMP,
};

static struct usb_descriptor_header *adb_fs_function[] = {
    (struct usb_descriptor_header *)&adb_interface_desc,
    (struct usb_descriptor_header *)&adb_fs_out_desc,
    (struct usb_descriptor_header *)&adb_fs_in_desc,
    NULL,
};

static struct usb_descriptor_header *adb_hs_function[] = {
    (struct usb_descriptor_header *)&adb_interface_desc,
    (struct usb_descriptor_header *)&adb_hs_out_desc,
    (struct usb_descriptor_header *)&adb_hs_in_desc,
    NULL,
};

static struct usb_descriptor_header *adb_ss_function[] = {
    (struct usb_descriptor_header *)&adb_interface_desc,
    (struct usb_descriptor_header *)&adb_ss_in_desc,
    (struct usb_descriptor_header *)&adb_ss_bulk_comp_desc,
    (struct usb_descriptor_header *)&adb_ss_out_desc,
    (struct usb_descriptor_header *)&adb_ss_bulk_comp_desc,
    NULL,
};

/* --- helpers --- */

static inline struct usb_request *adb_alloc_req(struct usb_ep *ep, size_t len)
{
    struct usb_request *req = usb_ep_alloc_request(ep, GFP_KERNEL);
    if (!req) return NULL;
    req->buf = kzalloc(len, GFP_KERNEL);
    if (!req->buf) { usb_ep_free_request(ep, req); return NULL; }
    req->length = (unsigned int)len;
    return req;
}

static void adb_free_req(struct usb_ep *ep, struct usb_request *req)
{
    if (!req) return;
    kfree(req->buf);
    usb_ep_free_request(ep, req);
}

static int try_connect_tcp(unsigned short port, struct socket **sockp)
{
    struct socket *sock = NULL;
    struct sockaddr_in addr;
    int ret;

    *sockp = NULL;

    ret = sock_create_kern(&init_net, AF_INET, SOCK_STREAM, IPPROTO_TCP, &sock);
    if (ret) {
        pr_err("f_adb: sock_create_kern failed: %d\n", ret);
        return ret;
    }

    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);

    ret = kernel_connect(sock, (struct sockaddr *)&addr, sizeof(addr), 0);
    if (ret) {
        pr_info("f_adb: kernel_connect failed: %d\n", ret);
        sock_release(sock);
        return ret;
    }

    *sockp = sock;
    return 0;
}

/* Forward declarations for functions referenced earlier */
static void adb_in_complete(struct usb_ep *ep, struct usb_request *req);
static void adb_out_complete(struct usb_ep *ep, struct usb_request *req);
static int  adb_rx_kthread(void *arg);
static int  adb_sender_thread(void *arg);
static void adb_process_events(struct f_adb *ctx);
static int  adb_start_runtime_internal(struct f_adb *ctx);
static void adb_stop_runtime_internal(struct f_adb *ctx);
static int  adb_drain_out_queue_to_socket_once(struct f_adb *ctx);

/* --- Thin OUT completion: only produce FIFO event --- */
static void adb_out_complete(struct usb_ep *ep, struct usb_request *req)
{
    struct pool_elem *rx;
    struct f_adb *ctx;
    unsigned long flags;

    if (!req) {
        pr_err("f_adb: out_complete NULL req\n");
        return;
    }

    /* req->context must be a pool_elem pointer set at allocation time */
    rx = req->context;
    if (!rx) {
        /* unknown req, free defensively */
        adb_free_req(ep, req);
        return;
    }

    ctx = rx->owner;
    if (!ctx) {
        adb_free_req(ep, req);
        return;
    }

    atomic_inc(&ctx->completed_out_count);
    dev_info(ctx->dev, "f_adb: out_complete req=%p actual=%u status=%d\n", req, req->actual, req->status);

    if (req->status == 0 && req->actual > 0) {
        /* Protect ring and rx->in_use with evt_lock (or pool_lock) */
        spin_lock_irqsave(&ctx->evt_lock, flags);
        if (((ctx->evt_tail + 1) % ctx->evt_cap) == ctx->evt_head) {
            spin_unlock_irqrestore(&ctx->evt_lock, flags);
            atomic_inc(&ctx->dropped_on_error);
            if (!work_pending(&ctx->state_work))
                schedule_work(&ctx->state_work);
            atomic_set(&ctx->drain_request, 1);
            wake_up_interruptible(&ctx->rx_wq);
        } else {
            ctx->evt_ring[ctx->evt_tail].idx = rx->index; /* store index in pool_elem */
            ctx->evt_ring[ctx->evt_tail].len = req->actual;
            ctx->evt_tail = (ctx->evt_tail + 1) % ctx->evt_cap;
            spin_unlock_irqrestore(&ctx->evt_lock, flags);
            if (!work_pending(&ctx->state_work))
                schedule_work(&ctx->state_work);
        }
    } else {
        /* error or zero-length: free */
        adb_free_req(ep, req);
    }
}


/* IN completion handler (for CLSE or other IN responses) */
static void adb_in_complete(struct usb_ep *ep, struct usb_request *req)
{
    struct f_adb *ctx = req ? req->context : NULL;

    if (!req)
        return;

    if (req->status) {
        if (ctx && ctx->dev)
            dev_dbg(ctx->dev, "f_adb: in_complete status=%d actual=%u\n", req->status, req->actual);
        else
            pr_debug("f_adb: in_complete status=%d actual=%u\n", req->status, req->actual);
    }

    adb_free_req(ep, req);
}

/* Prototype (add to your forward-decls area) */
static void adb_handle_cnxn(struct f_adb *ctx, const struct adb_header *hdr);

/* Build CLSE, queue it to IN endpoint, log, and request RX connect */
static void adb_handle_cnxn(struct f_adb *ctx, const struct adb_header *hdr)
{
    struct usb_request *in_req;
    struct adb_header clse;
    unsigned short port = ctx->opts ? ctx->opts->port : bridge_port;

    if (!ctx || !hdr)
        return;

    /* Log CNXN header fields */
    dev_info(ctx->dev,
         "f_adb: detected CNXN hdr: arg0=0x%08x arg1=0x%08x len=%u magic=0x%08x\n",
         le32_to_cpu(hdr->arg0), le32_to_cpu(hdr->arg1),
         (unsigned)le32_to_cpu(hdr->length), le32_to_cpu(hdr->magic));

    /* Build CLSE reply */
    build_adb_clse(hdr, &clse);

    /* Allocate IN request and queue CLSE */
    in_req = adb_alloc_req(ctx->in, sizeof(clse));
    if (!in_req) {
        dev_err(ctx->dev, "f_adb: failed to alloc in_req for CLSE\n");
    } else {
        memcpy(in_req->buf, &clse, sizeof(clse));
        in_req->length = sizeof(clse);
        in_req->complete = adb_in_complete;
        in_req->context = ctx;

        if (usb_ep_queue(ctx->in, in_req, GFP_ATOMIC)) {
            adb_free_req(ctx->in, in_req);
            dev_err(ctx->dev, "f_adb: failed to queue CLSE for CNXN\n");
        } else {
            dev_info(ctx->dev, "f_adb: queued CLSE in response to CNXN\n");
        }
    }

    /* Request RX thread to connect to TCP bridge and log the request */
    atomic_set(&ctx->connect_request, 1);
    dev_info(ctx->dev, "f_adb: CNXN -> requested RX connect to TCP bridge (port=%u)\n", port);
    wake_up_interruptible(&ctx->rx_wq);
}

/* --- Worker: drain FIFO, bridge, send via kernel_sendmsg under RCU, re-prime URB --- */
/*
 * Drain FIFO events, copy URB payload -> tx skb, attempt kernel_sendmsg under RCU,
 * buffer to out_queue on failure, and re-prime the corresponding OUT URB.
 *
 * This version uses atomic in_use flags on pool elements and an atomic
 * lockless claim (atomic_cmpxchg) for tx slots to avoid races.
 */
static void adb_process_events(struct f_adb *ctx)
{
    unsigned long flags;

    if (!ctx)
        return;

    for (;;) {
        struct out_evt evt;

        /* Pop one event from the ring (protected by evt_lock) */
        spin_lock_irqsave(&ctx->evt_lock, flags);
        if (ctx->evt_head == ctx->evt_tail) {
            spin_unlock_irqrestore(&ctx->evt_lock, flags);
            break;
        }
        evt = ctx->evt_ring[ctx->evt_head];
        ctx->evt_head = (ctx->evt_head + 1) % ctx->evt_cap;
        spin_unlock_irqrestore(&ctx->evt_lock, flags);

        /* Validate index */
        if (evt.idx >= ctx->pool_size) {
            atomic_inc(&ctx->dropped_on_error);
            continue;
        }

        /* Per-event processing block */
        {
            struct pool_elem *rx = &ctx->rx_pool[evt.idx];
            struct pool_elem *tx = NULL;
            unsigned int j;

            /* Defensive checks */
            if (!rx->owner || !rx->req) {
                atomic_inc(&ctx->dropped_on_error);
                continue;
            }

            /* Claim a free tx slot atomically */
            for (j = 0; j < ctx->pool_size; j++) {
                if (atomic_read(&ctx->tx_pool[j].in_use) == 0) {
                    if (atomic_cmpxchg(&ctx->tx_pool[j].in_use, 0, 1) == 0) {
                        tx = &ctx->tx_pool[j];
                        break;
                    }
                }
            }

            if (!tx) {
                /* No tx slot: reinsert evt at head and retry later */
                spin_lock_irqsave(&ctx->evt_lock, flags);
                ctx->evt_head = (ctx->evt_head + ctx->evt_cap - 1) % ctx->evt_cap;
                ctx->evt_ring[ctx->evt_head] = evt;
                spin_unlock_irqrestore(&ctx->evt_lock, flags);
                if (!work_pending(&ctx->state_work))
                    schedule_work(&ctx->state_work);
                break;
            }

            /* Ensure tx->skb exists (allocate in sleepable context) */
            if (!tx->skb) {
                tx->skb = alloc_skb(ctx->out_req_size, GFP_KERNEL);
                if (!tx->skb) {
                    dev_err(ctx->dev, "f_adb: failed to alloc tx skb\n");
                    atomic_set(&tx->in_use, 0);
                    atomic_inc(&ctx->dropped_on_error);
                    continue;
                }
            }

            /* Copy URB payload into tx scratch skb */
            skb_trim(tx->skb, 0);
            if (evt.len > skb_tailroom(tx->skb))
                evt.len = skb_tailroom(tx->skb);
            memcpy(skb_put(tx->skb, evt.len), rx->req->buf, evt.len);

            /* Opportunistic CNXN detection */
            if (!atomic_read(&ctx->is_sock_connected)) {
                struct adb_header hdr;
                if (parse_adb_header(tx->skb->data, tx->skb->len, &hdr)) {
                    if (hdr.command == cpu_to_le32((u32)('C' | ('N'<<8) | ('X'<<16) | ('N'<<24)))) {
                        adb_handle_cnxn(ctx, &hdr);
                    }
                }
            }

            /*
             * Offload actual send to sender thread:
             *  - clone tx->skb into local
             *  - release tx slot immediately
             *  - enqueue local into out_queue
             *  - wake sender thread
             */

            {
                struct sk_buff *local = skb_copy(tx->skb, GFP_KERNEL);
                if (!local) {
                    dev_err(ctx->dev, "f_adb: failed to copy tx skb\n");
                    atomic_inc(&ctx->dropped_on_error);
                    atomic_set(&tx->in_use, 0);
                    continue;
                }

                /* Release tx slot so other events can use it */
                atomic_set(&tx->in_use, 0);

                /* Enqueue for sender thread to drain */
                skb_queue_tail(&ctx->out_queue, local);
                atomic_inc(&ctx->out_skb_count);

                /* Hint sender thread to wake and drain */
                atomic_set(&ctx->drain_request, 1);
                wake_up_interruptible(&ctx->tx_wq);
            }

            /* Re-prime the rx URB for this slot */
            if (ctx->current_state == ADB_STATE_RUNNING && ctx->out && ctx->out->enabled) {
                atomic_set(&rx->in_use, 1);
                rx->req->length = (unsigned int)ctx->out_req_size;
                rx->req->status = 0;
                if (usb_ep_queue(ctx->out, rx->req, GFP_ATOMIC)) {
                    atomic_set(&rx->in_use, 0);
                    atomic_inc(&ctx->dropped_on_error);
                }
            } else {
                atomic_set(&rx->in_use, 0);
            }
        } /* end per-event block */
    } /* for */
}

/* --- State worker and runtime start/stop --- */

static int adb_config_ep_prepare(struct f_adb *ctx, struct usb_ep *ep)
{
    struct usb_function *f = &ctx->port_func;
    struct usb_configuration *c = f->config;
    struct usb_gadget *g = c ? c->cdev->gadget : NULL;
    if (!g) return -ENODEV;
    return config_ep_by_speed(g, f, ep);
}

/*
 * Initialize runtime state for the ADB function and start runtime threads.
 *
 * This implementation:
 *  - initializes the pending coalescing state (ctx->pending, etc.)
 *  - enables endpoints and allocates request pools
 *  - starts RX and worker threads
 *  - unwinds cleanly on error
 *
 * Replace the helper calls (f_adb_enable_endpoints, f_adb_alloc_reqs,
 * adb_start_rx_thread, adb_start_worker_threads) with the concrete
 * functions from your driver if they have different names.
 */
/*
 * Initialize runtime state for the ADB function and start runtime threads.
 *
 * This version inlines endpoint enablement, pool allocation and thread
 * startup using only helpers already present in this file:
 *  - adb_config_ep_prepare()
 *  - adb_alloc_req() / adb_free_req()
 *  - usb_ep_enable() / usb_ep_disable()
 *  - kthread_run() / kthread_stop()
 *
 * It allocates rx_pool and tx_pool, the event ring, primes rx URBs,
 * starts the RX and sender threads, and unwinds cleanly on error.
 */
/*
 * Initialize runtime state for the ADB function and start runtime threads.
 *
 * Inlined startup that:
 *  - initializes pending coalesce state
 *  - prepares and enables endpoints
 *  - allocates pools and event ring
 *  - primes OUT URBs
 *  - starts RX and sender threads
 *
 * Uses only helpers already present in this file (adb_config_ep_prepare,
 * adb_alloc_req, adb_free_req, usb_ep_enable/disable, kthread_run/stop).
 */
static int adb_start_runtime_internal(struct f_adb *ctx)
{
    int ret = 0;
    unsigned int i;
    unsigned int pool_size;
    size_t out_req_size;
    size_t in_buf_size;

    if (!ctx)
        return -EINVAL;

    /* initialize pending coalesce state */
    ctx->pending = NULL;
    ctx->pending_len = 0;
    ctx->pending_expect = 0;
    ctx->pending_ts = 0;
    ctx->pending_cap = 0;

    /* determine pool sizes and buffer sizes (fall back to defaults) */
    pool_size = ctx->out_req_count ? ctx->out_req_count :
            (ctx->opts && ctx->opts->out_count ? ctx->opts->out_count : ADB_OUT_REQS_DEF);
    out_req_size = ctx->out_req_size ? ctx->out_req_size :
               (ctx->opts && ctx->opts->out_size ? ctx->opts->out_size : ADB_OUT_BUFSIZE_DEF);
    in_buf_size = ctx->in_buf_size ? ctx->in_buf_size :
              (ctx->opts && ctx->opts->in_size ? ctx->opts->in_size : ADB_IN_BUFSIZE_DEF);

    ctx->pool_size = pool_size;
    ctx->out_req_size = out_req_size;
    ctx->in_buf_size = in_buf_size;

    /* prepare endpoints for current speed (sets ep->desc) */
    ret = adb_config_ep_prepare(ctx, ctx->in);
    if (ret) {
        dev_err(ctx->dev, "f_adb: config in ep failed: %d\n", ret);
        goto err;
    }
    ret = adb_config_ep_prepare(ctx, ctx->out);
    if (ret) {
        dev_err(ctx->dev, "f_adb: config out ep failed: %d\n", ret);
        goto err;
    }

    /* enable endpoints (usb_ep_enable takes only the ep pointer) */
    ret = usb_ep_enable(ctx->in);
    if (ret) {
        dev_err(ctx->dev, "f_adb: usb_ep_enable(in) failed: %d\n", ret);
        goto err;
    }
    ctx->enabled_in_by_us = true;

    ret = usb_ep_enable(ctx->out);
    if (ret) {
        dev_err(ctx->dev, "f_adb: usb_ep_enable(out) failed: %d\n", ret);
        goto err_disable_in;
    }
    ctx->enabled_out_by_us = true;

    /* allocate pools */
    ctx->rx_pool = kzalloc(sizeof(struct pool_elem) * pool_size, GFP_KERNEL);
    if (!ctx->rx_pool) {
        ret = -ENOMEM;
        goto err_disable_out;
    }
    ctx->tx_pool = kzalloc(sizeof(struct pool_elem) * pool_size, GFP_KERNEL);
    if (!ctx->tx_pool) {
        ret = -ENOMEM;
        goto err_free_rx_pool;
    }
    ctx->pool_size = pool_size;

    /* allocate event ring */
    ctx->evt_cap = pool_size * EVT_RING_MULT;
    ctx->evt_ring = kzalloc(sizeof(struct out_evt) * ctx->evt_cap, GFP_KERNEL);
    if (!ctx->evt_ring) {
        ret = -ENOMEM;
        goto err_free_tx_pool;
    }
    ctx->evt_head = ctx->evt_tail = 0;
    spin_lock_init(&ctx->evt_lock);

    /* init queues and counters */
    skb_queue_head_init(&ctx->out_queue);
    atomic_set(&ctx->out_skb_count, 0);
    atomic_set(&ctx->primed_out_count, 0);
    atomic_set(&ctx->completed_out_count, 0);
    atomic_set(&ctx->dropped_on_error, 0);
    atomic_set(&ctx->is_sock_connected, 0);
    atomic_set(&ctx->drain_request, 0);
    atomic_set(&ctx->connect_request, 0);

    /* allocate rx_pool requests (OUT URBs) */
    for (i = 0; i < pool_size; i++) {
        struct pool_elem *pe = &ctx->rx_pool[i];
        pe->owner = ctx;
        pe->index = i;
        atomic_set(&pe->in_use, 0);

        pe->req = adb_alloc_req(ctx->out, out_req_size);
        if (!pe->req) {
            dev_err(ctx->dev, "f_adb: failed alloc rx req %u\n", i);
            ret = -ENOMEM;
            goto err_free_reqs;
        }
        pe->req->context = pe;
    }

    /* initialize tx_pool entries (skb allocated lazily) */
    for (i = 0; i < pool_size; i++) {
        struct pool_elem *pe = &ctx->tx_pool[i];
        pe->owner = ctx;
        pe->index = i;
        pe->skb = NULL;
        atomic_set(&pe->in_use, 0);
    }

    /* prime OUT URBs (queue them) */
    for (i = 0; i < pool_size; i++) {
        struct pool_elem *pe = &ctx->rx_pool[i];
        atomic_set(&pe->in_use, 1);
        pe->req->length = (unsigned int)out_req_size;
        pe->req->complete = adb_out_complete;
        pe->req->context = pe;
        if (usb_ep_queue(ctx->out, pe->req, GFP_ATOMIC)) {
            atomic_set(&pe->in_use, 0);
            dev_err(ctx->dev, "f_adb: usb_ep_queue failed for rx slot %u\n", i);
            /* continue trying to queue remaining; treat as non-fatal here */
        } else {
            atomic_inc(&ctx->primed_out_count);
        }
    }

    /* initialize waitqueues */
    init_waitqueue_head(&ctx->rx_wq);
    init_waitqueue_head(&ctx->tx_wq);

    /* start RX thread (owns socket lifecycle) */
    ctx->rx_task = kthread_run(adb_rx_kthread, ctx, "adb_rx/%p", ctx);
    if (IS_ERR(ctx->rx_task)) {
        ret = PTR_ERR(ctx->rx_task);
        ctx->rx_task = NULL;
        dev_err(ctx->dev, "f_adb: failed to start rx thread: %d\n", ret);
        goto err_stop_primed;
    }

    /* start sender thread */
    ctx->sender_task = kthread_run(adb_sender_thread, ctx, "adb_sender/%p", ctx);
    if (IS_ERR(ctx->sender_task)) {
        ret = PTR_ERR(ctx->sender_task);
        ctx->sender_task = NULL;
        dev_err(ctx->dev, "f_adb: failed to start sender thread: %d\n", ret);
        goto err_stop_rx;
    }

    /* success */
    dev_info(ctx->dev, "f_adb: runtime started (pool=%u out_size=%zu in_size=%zu)\n",
         pool_size, out_req_size, in_buf_size);
    return 0;

/* unwind on errors */
err_stop_rx:
    if (ctx->rx_task) {
        kthread_stop(ctx->rx_task);
        ctx->rx_task = NULL;
    }
err_stop_primed:
    /* try to cancel any queued rx reqs and free them */
    for (i = 0; i < pool_size; i++) {
        if (ctx->rx_pool && ctx->rx_pool[i].req) {
            /* best-effort: try to dequeue; ignore errors */
            usb_ep_dequeue(ctx->out, ctx->rx_pool[i].req);
        }
    }
err_free_reqs:
    /* free rx_pool reqs allocated so far */
    for (i = 0; i < pool_size; i++) {
        if (ctx->rx_pool && ctx->rx_pool[i].req) {
            adb_free_req(ctx->out, ctx->rx_pool[i].req);
            ctx->rx_pool[i].req = NULL;
        }
    }
    kfree(ctx->evt_ring);
    ctx->evt_ring = NULL;
err_free_tx_pool:
    kfree(ctx->tx_pool);
    ctx->tx_pool = NULL;
err_free_rx_pool:
    kfree(ctx->rx_pool);
    ctx->rx_pool = NULL;
err_disable_out:
    if (ctx->enabled_out_by_us) {
        usb_ep_disable(ctx->out);
        ctx->enabled_out_by_us = false;
    }
err_disable_in:
    if (ctx->enabled_in_by_us) {
        usb_ep_disable(ctx->in);
        ctx->enabled_in_by_us = false;
    }
err:
    /* free pending buffer if any */
    if (ctx->pending) {
        kfree(ctx->pending);
        ctx->pending = NULL;
        ctx->pending_len = 0;
        ctx->pending_expect = 0;
        ctx->pending_ts = 0;
        ctx->pending_cap = 0;
    }
    return ret;
}

/*
 * Tear down runtime state for the ADB function.
 *
 * Stops threads, disables endpoints, frees pools and event ring, and
 * frees the pending coalescing buffer. Uses only helpers already present
 * in this file (adb_free_req, usb_ep_disable, kthread_stop).
 */
static void adb_stop_runtime_internal(struct f_adb *ctx)
{
    unsigned int i;

    if (!ctx)
        return;

    dev_info(ctx->dev, "f_adb: stopping runtime\n");

    /* 1) stop sender thread */
    if (ctx->sender_task) {
        kthread_stop(ctx->sender_task);
        ctx->sender_task = NULL;
    }

    /* 2) stop RX thread (may be blocked on socket) */
    if (ctx->rx_task) {
        kthread_stop(ctx->rx_task);
        ctx->rx_task = NULL;
    }

    /* 3) try to dequeue any queued rx requests (best-effort) */
    if (ctx->rx_pool) {
        for (i = 0; i < ctx->pool_size; i++) {
            if (ctx->rx_pool[i].req)
                usb_ep_dequeue(ctx->out, ctx->rx_pool[i].req);
        }
    }

    /* 4) free rx_pool requests */
    if (ctx->rx_pool) {
        for (i = 0; i < ctx->pool_size; i++) {
            if (ctx->rx_pool[i].req) {
                adb_free_req(ctx->out, ctx->rx_pool[i].req);
                ctx->rx_pool[i].req = NULL;
            }
        }
        kfree(ctx->rx_pool);
        ctx->rx_pool = NULL;
    }

    /* 5) free tx_pool skbs (they were allocated lazily) */
    if (ctx->tx_pool) {
        for (i = 0; i < ctx->pool_size; i++) {
            if (ctx->tx_pool[i].skb) {
                kfree_skb(ctx->tx_pool[i].skb);
                ctx->tx_pool[i].skb = NULL;
            }
        }
        kfree(ctx->tx_pool);
        ctx->tx_pool = NULL;
    }

    /* 6) free event ring */
    if (ctx->evt_ring) {
        kfree(ctx->evt_ring);
        ctx->evt_ring = NULL;
        ctx->evt_head = ctx->evt_tail = 0;
    }

    /* 7) flush and free queued out_queue skbs */
    if (!skb_queue_empty(&ctx->out_queue)) {
        struct sk_buff *skb;
        while ((skb = skb_dequeue(&ctx->out_queue)) != NULL) {
            kfree_skb(skb);
        }
        atomic_set(&ctx->out_skb_count, 0);
    }

    /* 8) disable endpoints if we enabled them */
    if (ctx->enabled_out_by_us) {
        usb_ep_disable(ctx->out);
        ctx->enabled_out_by_us = false;
    }
    if (ctx->enabled_in_by_us) {
        usb_ep_disable(ctx->in);
        ctx->enabled_in_by_us = false;
    }

    /* 9) free pending buffer if present (after threads stopped) */
    if (ctx->pending) {
        kfree(ctx->pending);
        ctx->pending = NULL;
        ctx->pending_len = 0;
        ctx->pending_expect = 0;
        ctx->pending_ts = 0;
        ctx->pending_cap = 0;
    }

    dev_info(ctx->dev, "f_adb: runtime stopped\n");
}

/* --- Worker main --- */

static void adb_state_worker(struct work_struct *work)
{
    struct f_adb *ctx = container_of(work, struct f_adb, state_work);
    enum adb_state want, cur;
    int ret;

    if (!ctx) return;
    dev_dbg(ctx->dev, "f_adb: state_worker enter (ctx=%p)\n", ctx);

    for (;;) {
        spin_lock_irq(&ctx->state_lock);
        want = ctx->requested_state;
        cur  = ctx->current_state;
        spin_unlock_irq(&ctx->state_lock);

        if (want == cur) {
            adb_process_events(ctx);
            break;
        }

        switch (cur) {
        case ADB_STATE_STOPPED:
            if (want == ADB_STATE_RUNNING || want == ADB_STATE_STARTING) {
                spin_lock_irq(&ctx->state_lock);
                ctx->current_state = ADB_STATE_STARTING;
                spin_unlock_irq(&ctx->state_lock);

                ret = adb_start_runtime_internal(ctx);
                if (ret == 0) {
                    spin_lock_irq(&ctx->state_lock);
                    ctx->current_state = ADB_STATE_RUNNING;
                    ctx->state_retries = 0;
                    spin_unlock_irq(&ctx->state_lock);
                    dev_info(ctx->dev, "f_adb: transitioned to RUNNING\n");
                } else if (ret == -EAGAIN && ctx->state_retries++ < STATE_RETRY_MAX) {
                    dev_warn(ctx->dev, "f_adb: transient start failure, retrying (%u)\n", ctx->state_retries);
                    msleep(STATE_RETRY_DELAY_MS);
                } else {
                    dev_err(ctx->dev, "f_adb: start_runtime_internal failed: %d\n", ret);
                    spin_lock_irq(&ctx->state_lock);
                    ctx->current_state = ADB_STATE_STOPPED;
                    spin_unlock_irq(&ctx->state_lock);
                }
            }
            break;

        case ADB_STATE_RUNNING:
            if (want == ADB_STATE_STOPPING || want == ADB_STATE_STOPPED) {
                spin_lock_irq(&ctx->state_lock);
                ctx->current_state = ADB_STATE_STOPPING;
                spin_unlock_irq(&ctx->state_lock);

                adb_stop_runtime_internal(ctx);

                spin_lock_irq(&ctx->state_lock);
                ctx->current_state = ADB_STATE_STOPPED;
                spin_unlock_irq(&ctx->state_lock);
                dev_info(ctx->dev, "f_adb: transitioned to STOPPED\n");
            } else {
                dev_dbg(ctx->dev, "f_adb: state_worker processing events\n");
                adb_process_events(ctx);
                msleep(20);
            }
            break;

        case ADB_STATE_STARTING:
        case ADB_STATE_STOPPING:
        case ADB_STATE_ERROR:
            msleep(50);
            break;
        }
    }
}

static int adb_drain_out_queue_to_socket_once(struct f_adb *ctx)
{
    struct sk_buff *skb;
    int ret = 0;

    if (!ctx)
        return -EINVAL;

    /* Quick path: nothing to do */
    if (skb_queue_empty(&ctx->out_queue))
        return 0;

    for (;;) {
        struct socket *sock = NULL;
        struct sock *sk = NULL;

        /* Acquire a stable reference to the socket's sock under RCU */
        rcu_read_lock();
        sock = rcu_dereference(ctx->adbd_sock);
        if (sock) {
            sk = sock->sk;
            if (sk)
                sock_hold(sk); /* take sock ref while still under RCU */
        }
        rcu_read_unlock();

        if (!sock) {
            /* No socket published right now; caller can retry later */
            ret = -ENOTCONN;
            break;
        }

        /* Dequeue one skb to send */
        skb = skb_dequeue(&ctx->out_queue);
        if (!skb) {
            /* nothing to send */
            if (sk)
                sock_put(sk);
            ret = 0;
            break;
        }

        /* Prepare iov and msg for kernel_sendmsg */
        {
            struct kvec iov = { .iov_base = skb->data, .iov_len = skb->len };
            struct msghdr msg = { .msg_flags = 0 };
            int sret;

            /*
             * We hold a reference to sk (if non-NULL) so the socket won't be freed
             * while we call kernel_sendmsg. We are NOT in rcu_read_lock here.
             */
            sret = kernel_sendmsg(sock, &msg, &iov, 1, skb->len);

            if (sret < 0) {
                /* transient would-block: requeue at head and stop draining */
                if (sret == -EAGAIN || sret == -EWOULDBLOCK) {
                    skb_queue_head(&ctx->out_queue, skb);
                    atomic_set(&ctx->drain_request, 1);
                    ret = -EAGAIN;
                } else {
                    /* other errors: requeue and request reconnect */
                    skb_queue_head(&ctx->out_queue, skb);
                    atomic_set(&ctx->connect_request, 1);
                    atomic_inc(&ctx->dropped_on_error);
                    ret = sret;
                }

                if (sk)
                    sock_put(sk);
                break;
            }

            /* partial write: requeue remainder at head and retry later */
            if (sret != skb->len) {
                if (sret > 0)
                    skb_pull(skb, sret);
                skb_queue_head(&ctx->out_queue, skb);
                atomic_set(&ctx->drain_request, 1);
                ret = -EAGAIN;
                if (sk)
                    sock_put(sk);
                break;
            }

            /* fully sent */
            kfree_skb(skb);
            atomic_dec(&ctx->out_skb_count);
        }

        /* release the sock reference and loop to next skb (if any) */
        if (sk)
            sock_put(sk);

        /* quick exit if queue empty */
        if (skb_queue_empty(&ctx->out_queue)) {
            ret = 0;
            break;
        }
    }

    return ret;
}

static int adb_sender_thread(void *arg)
{
    struct f_adb *ctx = arg;

    /* lower priority to be polite */
    set_user_nice(current, 5);

    while (!kthread_should_stop()) {
        /* Sleep until there's work or thread should stop */
        wait_event_interruptible(ctx->tx_wq,
                                 kthread_should_stop() ||
                                 atomic_read(&ctx->drain_request) ||
                                 atomic_read(&ctx->connect_request));

        if (kthread_should_stop())
            break;

        /* If connect requested, wake RX thread to attempt connect */
        if (atomic_xchg(&ctx->connect_request, 0)) {
            wake_up_interruptible(&ctx->rx_wq);
        }

        /* Clear drain_request and attempt to drain queue */
        atomic_set(&ctx->drain_request, 0);

        /* Try to drain; helper handles socket absence and errors */
        adb_drain_out_queue_to_socket_once(ctx);

        /* If queue still non-empty, sleep a bit to avoid busy-loop */
        if (!skb_queue_empty(&ctx->out_queue))
            msleep(50);
    }

    /* On exit, try one final drain if socket present */
    adb_drain_out_queue_to_socket_once(ctx);

    return 0;
}

/*
 * Forward a complete skb (which may contain a full ADB header+payload or
 * raw payload) to the USB IN endpoint. Preserves an existing ADB header if
 * present; otherwise wraps payload into WRTE headers. Splits into multiple
 * IN URBs if the total exceeds ctx->in_buf_size.
 *
 * Caller: process context (RX thread). Returns 0 on success or negative errno.
 */
static int adb_forward_skb_to_usb_in(struct f_adb *ctx, struct sk_buff *skb)
{
    size_t in_cap;
    int ret = 0;

    if (!ctx || !skb)
        return -EINVAL;
    if (!ctx->in || !ctx->in->enabled)
        return -ENODEV;

    in_cap = ctx->in_buf_size; /* maximum payload per IN URB */

    /* Helper to allocate and queue a single IN URB with given buffer and len */
    #define QUEUE_IN_BUF(buf_ptr, buf_len)                                      \
        do {                                                                    \
            struct usb_request *in_req = NULL;                                  \
            size_t _len = (buf_len);                                            \
            in_req = adb_alloc_req(ctx->in, _len);                              \
            if (!in_req) {                                                      \
                dev_err(ctx->dev, "f_adb: failed to alloc in_req len=%zu\n", _len); \
                ret = -ENOMEM;                                                  \
                goto out;                                                       \
            }                                                                   \
            memcpy(in_req->buf, (buf_ptr), _len);                               \
            in_req->length = (unsigned int)_len;                                \
            in_req->complete = adb_in_complete;                                 \
            in_req->context = ctx;                                              \
            if (usb_ep_queue(ctx->in, in_req, GFP_KERNEL)) {                    \
                dev_err(ctx->dev, "f_adb: usb_ep_queue failed for IN len=%zu\n", _len); \
                adb_free_req(ctx->in, in_req);                                  \
                ret = -EIO;                                                     \
                goto out;                                                       \
            }                                                                   \
        } while (0)

    /* If skb contains at least an ADB header, inspect it */
    if (skb->len >= (int)sizeof(struct adb_header)) {
        struct adb_header hdr;
        memcpy(&hdr, skb->data, sizeof(hdr));

        /* Validate header magic (little-endian) */
        if ((le32_to_cpu(hdr.command) ^ 0xFFFFFFFFU) == le32_to_cpu(hdr.magic)) {
            u32 payload_len = le32_to_cpu(hdr.length);
            size_t total_needed = sizeof(hdr) + payload_len;

            /* If skb contains full header+payload, forward as-is (may need splitting) */
            if ((size_t)skb->len >= total_needed) {
                /* If total fits in one IN URB (header+payload), queue it directly */
                if (total_needed <= in_cap) {
                    QUEUE_IN_BUF(skb->data, total_needed);
                    ret = 0;
                    goto out;
                }

                /* Otherwise split payload into multiple URBs.
                 * Send original header + first chunk, then WRTE chunks for remaining.
                 */
                {
                    size_t remaining = payload_len;
                    const u8 *payload_ptr = skb->data + sizeof(hdr);

                    /* First: header + first chunk */
                    size_t first_payload = min(remaining, in_cap - sizeof(hdr));
                    size_t first_total = sizeof(hdr) + first_payload;
                    u8 *tmp = kzalloc(first_total, GFP_KERNEL);
                    if (!tmp) { ret = -ENOMEM; goto out; }
                    memcpy(tmp, skb->data, sizeof(hdr));
                    memcpy(tmp + sizeof(hdr), payload_ptr, first_payload);
                    QUEUE_IN_BUF(tmp, first_total);
                    kfree(tmp);
                    payload_ptr += first_payload;
                    remaining -= first_payload;

                    /* Remaining chunks with WRTE headers */
                    while (remaining > 0) {
                        size_t chunk = min(remaining, in_cap - sizeof(struct adb_header));
                        struct adb_header wrte;
                        u32 cmd = cpu_to_le32((u32)('W' | ('R'<<8) | ('T'<<16) | ('E'<<24)));
                        u32 checksum = 0;
                        size_t i;
                        for (i = 0; i < chunk; i++) checksum += payload_ptr[i];

                        wrte.command = cmd;
                        wrte.arg0 = cpu_to_le32(0);
                        wrte.arg1 = cpu_to_le32(0);
                        wrte.length = cpu_to_le32((u32)chunk);
                        wrte.checksum = cpu_to_le32(checksum);
                        wrte.magic = cpu_to_le32(le32_to_cpu(cmd) ^ 0xFFFFFFFFU);

                        size_t tot = sizeof(wrte) + chunk;
                        u8 *tmp2 = kzalloc(tot, GFP_KERNEL);
                        if (!tmp2) { ret = -ENOMEM; goto out; }
                        memcpy(tmp2, &wrte, sizeof(wrte));
                        memcpy(tmp2 + sizeof(wrte), payload_ptr, chunk);
                        QUEUE_IN_BUF(tmp2, tot);
                        kfree(tmp2);

                        payload_ptr += chunk;
                        remaining -= chunk;
                    }
                    ret = 0;
                    goto out;
                }
            }
            /* header present but payload not yet fully received in skb.
             * Caller should not call this forwarder until full message is available.
             */
        }
    }

    /* Treat as payload-only: wrap into WRTE header and send (may need splitting) */
    {
        size_t remaining = skb->len;
        const u8 *p = skb->data;

        while (remaining > 0) {
            size_t chunk = min(remaining, in_cap - sizeof(struct adb_header));
            struct adb_header wrte;
            u32 cmd = cpu_to_le32((u32)('W' | ('R'<<8) | ('T'<<16) | ('E'<<24)));
            u32 checksum = 0;
            size_t i;
            for (i = 0; i < chunk; i++) checksum += p[i];

            wrte.command = cmd;
            wrte.arg0 = cpu_to_le32(0);
            wrte.arg1 = cpu_to_le32(0);
            wrte.length = cpu_to_le32((u32)chunk);
            wrte.checksum = cpu_to_le32(checksum);
            wrte.magic = cpu_to_le32(le32_to_cpu(cmd) ^ 0xFFFFFFFFU);

            size_t tot = sizeof(wrte) + chunk;
            u8 *tmp = kzalloc(tot, GFP_KERNEL);
            if (!tmp) { ret = -ENOMEM; goto out; }
            memcpy(tmp, &wrte, sizeof(wrte));
            memcpy(tmp + sizeof(wrte), p, chunk);
            QUEUE_IN_BUF(tmp, tot);
            kfree(tmp);

            p += chunk;
            remaining -= chunk;
        }

        ret = 0;
        goto out;
    }

out:
    #undef QUEUE_IN_BUF
    return ret;
}

/*
 * Drain socket -> forward to USB IN, using ctx->pending to coalesce header+payload.
 *
 * Behavior:
 *  - If a valid ADB header is seen and full payload not yet available, store
 *    header+partial payload in ctx->pending and wait for remaining bytes.
 *  - When full message available, forward header+payload as a single skb via
 *    adb_forward_skb_to_usb_in().
 *  - If no valid header, wrap raw payload into WRTE and forward immediately.
 *
 * Caller: RX thread (process context). Returns 0 on success or negative errno.
 */
static int adb_drain_socket_to_in(struct f_adb *ctx, struct socket *sock)
{
    const size_t max_read = 4096;
    u8 *buf = NULL;
    u8 small_buf[256];
    u8 *read_buf = NULL;
    struct kvec iov;
    struct msghdr msg;
    int got;
    int ret = 0;
    struct sk_buff *skb = NULL;

    /* tuning parameters */
    const ktime_t pending_timeout = ms_to_ktime(500); /* 500 ms */
    const size_t max_payload_cap = 1024 * 1024;       /* 1 MiB sanity cap */

    if (!ctx || !sock)
        return -EINVAL;
    if (!ctx->in || !ctx->in->enabled)
        return -ENODEV;

    buf = kmalloc(max_read, GFP_KERNEL);
    read_buf = buf ? buf : small_buf;

    memset(&msg, 0, sizeof(msg));
    iov.iov_base = read_buf;
    iov.iov_len = buf ? max_read : sizeof(small_buf);

    got = kernel_recvmsg(sock, &msg, &iov, 1, iov.iov_len, MSG_DONTWAIT);
    if (got == -EAGAIN || got == -EWOULDBLOCK) { ret = 0; goto out; }
    if (got <= 0) { ret = got; goto out; }

    /* 1) If we already have a pending header+partial payload, append and check */
    if (ctx->pending_len) {
        size_t want = 24 + ctx->pending_expect;
        size_t new_len = ctx->pending_len + got;

        if (new_len > ctx->pending_cap) {
            size_t new_cap = max(new_len, ctx->pending_cap ? ctx->pending_cap * 2 : 1024);
            u8 *tmp = krealloc(ctx->pending, new_cap, GFP_KERNEL);
            if (!tmp) { ret = -ENOMEM; goto out; }
            ctx->pending = tmp;
            ctx->pending_cap = new_cap;
        }

        memcpy(ctx->pending + ctx->pending_len, read_buf, got);
        ctx->pending_len = new_len;

        /* timeout check: if pending too old, drop it */
        if (ktime_after(ktime_get(), ktime_add(ctx->pending_ts, pending_timeout))) {
            dev_warn(ctx->dev, "f_adb: pending header timed out, dropping\n");
            kfree(ctx->pending);
            ctx->pending = NULL;
            ctx->pending_len = 0;
            ctx->pending_expect = 0;
            ctx->pending_cap = 0;
            goto out;
        }

        if (ctx->pending_len >= want) {
            /* we have full header+payload: forward combined */
            skb = alloc_skb(want, GFP_KERNEL);
            if (!skb) { ret = -ENOMEM; goto out; }
            memcpy(skb_put(skb, want), ctx->pending, want);
            ret = adb_forward_skb_to_usb_in(ctx, skb);
            kfree_skb(skb);

            /* remove used bytes from pending (if any extra remain) */
            if (ctx->pending_len > want) {
                size_t remain = ctx->pending_len - want;
                memmove(ctx->pending, ctx->pending + want, remain);
                ctx->pending_len = remain;

                /* try to parse new pending header if present */
                if (ctx->pending_len >= 24) {
                    struct adb_header hdr;
                    memcpy(&hdr, ctx->pending, sizeof(hdr));
                    if ((le32_to_cpu(hdr.command) ^ 0xFFFFFFFFU) == le32_to_cpu(hdr.magic)) {
                        ctx->pending_expect = le32_to_cpu(hdr.length);
                        if (ctx->pending_expect > max_payload_cap) {
                            dev_warn(ctx->dev, "f_adb: pending expect too large, dropping\n");
                            kfree(ctx->pending);
                            ctx->pending = NULL;
                            ctx->pending_len = 0;
                            ctx->pending_expect = 0;
                            ctx->pending_cap = 0;
                        } else {
                            ctx->pending_ts = ktime_get();
                        }
                    } else {
                        /* treat remaining as raw next payload */
                        ctx->pending_expect = 0;
                    }
                } else {
                    ctx->pending_expect = 0;
                }
            } else {
                kfree(ctx->pending);
                ctx->pending = NULL;
                ctx->pending_len = 0;
                ctx->pending_expect = 0;
                ctx->pending_cap = 0;
            }
        }

        goto out;
    }

    /* 2) No pending: inspect this read for header or raw payload */
    if (got >= (int)sizeof(struct adb_header)) {
        struct adb_header hdr;
        memcpy(&hdr, read_buf, sizeof(hdr));
        if ((le32_to_cpu(hdr.command) ^ 0xFFFFFFFFU) == le32_to_cpu(hdr.magic)) {
            u32 payload_len = le32_to_cpu(hdr.length);
            size_t total = sizeof(hdr) + payload_len;

            if (payload_len > max_payload_cap) {
                dev_warn(ctx->dev, "f_adb: peer announced huge payload %u, dropping\n", payload_len);
                ret = -EINVAL;
                goto out;
            }

            if ((size_t)got >= total) {
                /* full header+payload in this read: forward as-is */
                skb = alloc_skb(total, GFP_KERNEL);
                if (!skb) { ret = -ENOMEM; goto out; }
                memcpy(skb_put(skb, total), read_buf, total);
                ret = adb_forward_skb_to_usb_in(ctx, skb);
                kfree_skb(skb);

                /* if extra bytes after message, stash them into pending for next loop */
                if ((size_t)got > total) {
                    size_t extra = got - total;
                    ctx->pending = kmalloc(extra, GFP_KERNEL);
                    if (ctx->pending) {
                        memcpy(ctx->pending, read_buf + total, extra);
                        ctx->pending_len = extra;
                        ctx->pending_expect = 0;
                        ctx->pending_ts = ktime_get();
                        ctx->pending_cap = extra;
                    } else {
                        dev_warn(ctx->dev, "f_adb: failed alloc pending for extra bytes\n");
                    }
                }
                goto out;
            } else {
                /* header-only or partial payload: store pending until full message arrives */
                ctx->pending = kmalloc(got, GFP_KERNEL);
                if (!ctx->pending) { ret = -ENOMEM; goto out; }
                memcpy(ctx->pending, read_buf, got);
                ctx->pending_len = got;
                ctx->pending_expect = payload_len;
                ctx->pending_ts = ktime_get();
                ctx->pending_cap = got;
                dev_dbg(ctx->dev, "f_adb: stored pending header expect=%u got=%zu\n",
                        (unsigned)payload_len, ctx->pending_len);
                goto out;
            }
        }
    }

    /* 3) No valid header: treat as raw payload and forward (wrap into WRTE) */
    skb = alloc_skb(got, GFP_KERNEL);
    if (!skb) { ret = -ENOMEM; goto out; }
    memcpy(skb_put(skb, got), read_buf, got);
    ret = adb_forward_skb_to_usb_in(ctx, skb);
    kfree_skb(skb);

out:
    if (buf)
        kfree(buf);
    return ret;
}

static int adb_rx_kthread(void *arg)
{
    struct f_adb *ctx = arg;
    int ret, connect_failures = 0;
    int first_enable = 1;

    allow_signal(SIGKILL);
    if (!ctx) return -EINVAL;

    dev_info(ctx->dev, "f_adb: adb_rx_kthread enter (pid=%d ctx=%p)\n", current->pid, ctx);

    for (;;) {
        struct socket *new_sock = NULL;

        wait_event_interruptible(ctx->rx_wq,
            rcu_dereference_protected(ctx->adbd_sock, true) != NULL ||
            atomic_read(&ctx->connect_request) ||
            kthread_should_stop());

        if (kthread_should_stop())
            break;

        if (atomic_xchg(&ctx->connect_request, 0)) {
            ret = try_connect_tcp(ctx->opts ? ctx->opts->port : bridge_port, &new_sock);
            if (ret) {
                if (new_sock) sock_release(new_sock);
                connect_failures++;
                if (connect_failures >= MAX_CONNECT_RETRIES) {
                    /* drop queued skbs to avoid unbounded buffering */
                    while (!skb_queue_empty(&ctx->out_queue)) {
                        struct sk_buff *s = skb_dequeue(&ctx->out_queue);
                        if (!s) break;
                        if (atomic_read(&ctx->out_skb_count) > 0)
                            atomic_dec(&ctx->out_skb_count);
                        kfree_skb(s);
                        atomic_inc(&ctx->dropped_on_error);
                    }
                    connect_failures = 0;
                }
                msleep(CONNECT_BACKOFF_MS);
                continue;
            }

            /* publish socket via RCU; RX keeps local reference */
            rcu_assign_pointer(ctx->adbd_sock, new_sock);
            smp_wmb();
            atomic_set(&ctx->is_sock_connected, 1);
            dev_info(ctx->dev, "f_adb: RX connected socket %p\n", new_sock);
            new_sock = NULL;
        }
#if 0
        /* If drain requested, RX will drain the out_queue */
        if (atomic_xchg(&ctx->drain_request, 0)) {
            adb_drain_out_queue_to_socket(ctx);
        }
#endif
	adb_drain_socket_to_in(ctx, ctx->adbd_sock);

    }

    /* RX exit: withdraw socket and release if present */
    rcu_assign_pointer(ctx->adbd_sock, NULL);
    synchronize_rcu();
    atomic_set(&ctx->is_sock_connected, 0);

    dev_info(ctx->dev, "f_adb: adb_rx_kthread exit\n");
    return 0;
}

/* --- USB function glue --- */

static int adb_bind(struct usb_configuration *c, struct usb_function *f)
{
    struct f_adb *ctx = container_of(f, struct f_adb, port_func);
    struct usb_gadget *g = c->cdev->gadget;
    int ret;

    ctx->dev = &g->dev;
    dev_info(ctx->dev, "f_adb: bind start (function=%p config=%p gadget=%p)\n", f, c, g);

    ret = usb_interface_id(c, f);
    if (ret < 0) return ret;
    ctx->data_id = ret;
    adb_interface_desc.bInterfaceNumber = ret;

    ctx->out = usb_ep_autoconfig(g, &adb_fs_out_desc);
    if (!ctx->out) return -ENODEV;
    ctx->in = usb_ep_autoconfig(g, &adb_fs_in_desc);
    if (!ctx->in) return -ENODEV;

    ctx->out->driver_data = ctx;
    ctx->in->driver_data = ctx;

    /* copy assigned endpoint addresses into HS/SS descriptors so they match */
    adb_hs_in_desc.bEndpointAddress = adb_fs_in_desc.bEndpointAddress;
    adb_hs_out_desc.bEndpointAddress = adb_fs_out_desc.bEndpointAddress;
    adb_ss_in_desc.bEndpointAddress = adb_fs_in_desc.bEndpointAddress;
    adb_ss_out_desc.bEndpointAddress = adb_fs_out_desc.bEndpointAddress;

    usb_assign_descriptors(f, adb_fs_function, adb_hs_function, adb_ss_function, NULL);

    skb_queue_head_init(&ctx->out_queue);
    atomic_set(&ctx->out_skb_count, 0);

    if (ctx->opts) {
        mutex_lock(&ctx->opts->lock);
        ctx->in_buf_size = ctx->opts->in_size;
        ctx->out_req_size = ctx->opts->out_size;
        ctx->out_req_count = ctx->opts->out_count;
        mutex_unlock(&ctx->opts->lock);
    } else {
        ctx->in_buf_size = ADB_IN_BUFSIZE_DEF;
        ctx->out_req_size = ADB_OUT_BUFSIZE_DEF;
        ctx->out_req_count = ADB_OUT_REQS_DEF;
    }

    atomic_set(&ctx->primed_out_count, 0);
    atomic_set(&ctx->completed_out_count, 0);
    atomic_set(&ctx->dropped_on_error, 0);

    ctx->enabled_in_by_us = false;
    ctx->enabled_out_by_us = false;

    ctx->current_state = ADB_STATE_STOPPED;
    ctx->requested_state = ADB_STATE_STOPPED;
    spin_lock_init(&ctx->state_lock);
    ctx->state_retries = 0;
    INIT_WORK(&ctx->state_work, adb_state_worker);

    spin_lock_init(&ctx->evt_lock);
    init_waitqueue_head(&ctx->rx_wq);

    atomic_set(&ctx->connect_request, 0);
    atomic_set(&ctx->drain_request, 0);
    atomic_set(&ctx->is_sock_connected, 0);

    dev_info(ctx->dev, "f_adb: bind complete in=%s out=%s out_req_count=%u out_req_size=%zu in_buf_size=%zu\n",
         ctx->in ? ctx->in->name : "NULL", ctx->out ? ctx->out->name : "NULL",
         ctx->out_req_count, ctx->out_req_size, ctx->in_buf_size);

    if (ctx->in->enabled && ctx->out->enabled && (ctx->opts ? ctx->opts->adb_enabled : adb_enabled)) {
        spin_lock_irq(&ctx->state_lock);
        ctx->requested_state = ADB_STATE_RUNNING;
        spin_unlock_irq(&ctx->state_lock);
        schedule_work(&ctx->state_work);
        dev_info(ctx->dev, "f_adb: bind requested runtime start\n");
    }

    return 0;
}

static void adb_unbind(struct usb_configuration *c, struct usb_function *f)
{
    struct f_adb *ctx = container_of(f, struct f_adb, port_func);

    dev_info(ctx->dev, "f_adb: unbind called\n");
    spin_lock_irq(&ctx->state_lock);
    ctx->requested_state = ADB_STATE_STOPPED;
    spin_unlock_irq(&ctx->state_lock);
    schedule_work(&ctx->state_work);
    flush_work(&ctx->state_work);

    usb_free_all_descriptors(f);
    dev_info(ctx->dev, "f_adb: unbind complete\n");
}

static void adb_request_state(struct f_adb *ctx, enum adb_state want)
{
    unsigned long flags;
    spin_lock_irqsave(&ctx->state_lock, flags);
    if (ctx->requested_state != want) {
        ctx->requested_state = want;
        if (!work_pending(&ctx->state_work))
            schedule_work(&ctx->state_work);
    }
    spin_unlock_irqrestore(&ctx->state_lock, flags);
}

static int adb_set_alt(struct usb_function *f, unsigned intf, unsigned alt)
{
    struct f_adb *ctx = container_of(f, struct f_adb, port_func);

    if (!ctx) return -EINVAL;

    dev_info(ctx->dev, "f_adb: set_alt intf=%u alt=%u (ctx=%p)\n", intf, alt, ctx);

    adb_request_state(ctx, ADB_STATE_RUNNING);
    return 0;
}

static void adb_disable(struct usb_function *f)
{
    struct f_adb *ctx = container_of(f, struct f_adb, port_func);

    dev_info(ctx->dev, "f_adb: disable (ctx=%p)\n", ctx);

    adb_request_state(ctx, ADB_STATE_STOPPED);
    flush_work(&ctx->state_work);
    if (ctx->in && ctx->in->enabled) usb_ep_disable(ctx->in);
    if (ctx->out && ctx->out->enabled) usb_ep_disable(ctx->out);
}

static void adb_free_func(struct usb_function *f)
{
    struct f_adb *ctx = container_of(f, struct f_adb, port_func);

    adb_request_state(ctx, ADB_STATE_STOPPED);
    flush_work(&ctx->state_work);
    pr_info("f_adb: free_func freeing ctx=%p\n", ctx);
    kfree(ctx);
}

/* allocate function instance (configfs instance already created elsewhere) */
static struct usb_function *adb_alloc(struct usb_function_instance *fi)
{
    struct adb_opts *opts = fi_to_adb_opts(fi);
    struct f_adb *ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
    if (!ctx) return ERR_PTR(-ENOMEM);

    ctx->opts = opts;
    ctx->in = NULL;
    ctx->out = NULL;
    rcu_assign_pointer(ctx->adbd_sock, NULL);
    ctx->rx_task = NULL;

    ctx->port_func.name = "adb";
    ctx->port_func.bind = adb_bind;
    ctx->port_func.unbind = adb_unbind;
    ctx->port_func.set_alt = adb_set_alt;
    ctx->port_func.disable = adb_disable;
    ctx->port_func.free_func = adb_free_func;

    pr_info("f_adb: adb_alloc created function ctx=%p (opts=%p)\n", ctx, opts);
    return &ctx->port_func;
}

/* function instance alloc/free */
static struct usb_function_instance *adb_alloc_inst(void)
{
    struct adb_opts *opts;

    opts = kzalloc(sizeof(*opts), GFP_KERNEL);
    if (!opts)
        return ERR_PTR(-ENOMEM);

    mutex_init(&opts->lock);
    atomic_set(&opts->refcnt, 0);

    /* defaults */
    opts->adb_enabled = adb_enabled;
    opts->port = bridge_port;
    opts->out_count = ADB_OUT_REQS_DEF;
    opts->out_size = ADB_OUT_BUFSIZE_DEF;
    opts->in_size = ADB_IN_BUFSIZE_DEF;

    /* init configfs group on the embedded usb_function_instance */
    config_group_init_type_name(&opts->func_inst.group, "adb", &adb_func_type);

    /* tell USB core how to free this instance */
    opts->func_inst.free_func_inst = adb_free_instance;

    pr_info("f_adb: adb_alloc_inst created opts=%p\n", opts);
    return &opts->func_inst;
}

/* Register function */
DECLARE_USB_FUNCTION_INIT(adb, adb_alloc_inst, adb_alloc);
MODULE_ALIAS("usbfunc:adb");

MODULE_AUTHOR("George Chan");
MODULE_DESCRIPTION("ADB gadget function with RCU socket, two pools, thin OUT completion, HS/SS descriptors");
MODULE_LICENSE("GPL");
