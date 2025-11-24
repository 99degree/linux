/* drivers/usb/gadget/legacy/g_adbd_bridge.c
 *
 * Composite gadget: ADB + gserial ACM (multi.c style)
 * - Uses usb_get_function_instance() + usb_get_function() to import functions
 * - Assigns non-zero iManufacturer/iProduct/iSerial/iConfiguration
 *
 * SPDX-License-Identifier: GPL-2.0
 */

#include <linux/module.h>
#include <linux/usb/composite.h>

/* ---- Strings (0409) ---- */
enum {
    STR_MANUFACTURER = 0,
    STR_PRODUCT,
    STR_SERIAL,
    STR_CONFIG,
};

static struct usb_string gadget_strings[] = {
    { .s = "Linux" },          /* iManufacturer */
    { .s = "ADB" },      /* iProduct */
    { .s = "000000000001" },    /* iSerialNumber (stable) */
    { .s = "adb+acm" },         /* iConfiguration */
    { }
};

static struct usb_gadget_strings stringtab = {
    .language = 0x0409,
    .strings  = gadget_strings,
};

static struct usb_gadget_strings *dev_strings[] = {
    &stringtab,
    NULL,
};

/* ---- Device descriptor (IAD composite) ---- */
static struct usb_device_descriptor device_desc = {
    .bLength            = sizeof(device_desc),
    .bDescriptorType    = USB_DT_DEVICE,
    .bcdUSB             = cpu_to_le16(0x0201),
    .bDeviceClass       = 0xEF,
    .bDeviceSubClass    = 0x02,
    .bDeviceProtocol    = 0x01,
    .bMaxPacketSize0    = 64,
    .idVendor           = cpu_to_le16(0x18D1),
    .idProduct          = cpu_to_le16(0xD00F),
    .bcdDevice          = cpu_to_le16(0x0100),
    .iManufacturer      = 0, /* patched in do_config */
    .iProduct           = 0,
    .iSerialNumber      = 0,
};

/* ---- Configuration builder ---- */
static int do_config(struct usb_configuration *c)
{
    int ret;
    struct usb_function_instance *fi;
    struct usb_function *f;

    /* Assign string IDs */
    ret = usb_string_ids_tab(c->cdev, gadget_strings);
    if (ret < 0)
        return ret;

    c->cdev->desc.iManufacturer = gadget_strings[STR_MANUFACTURER].id;
    c->cdev->desc.iProduct      = gadget_strings[STR_PRODUCT].id;
    c->cdev->desc.iSerialNumber = gadget_strings[STR_SERIAL].id;
    c->iConfiguration           = gadget_strings[STR_CONFIG].id;

    /* ADB function (from f_adb.c) */
    fi = usb_get_function_instance("adb");
    if (IS_ERR(fi))
        return PTR_ERR(fi);

    f = usb_get_function(fi);
    if (IS_ERR(f)) {
        usb_put_function_instance(fi);
        return PTR_ERR(f);
    }

    ret = usb_add_function(c, f);
    if (ret) {
        usb_put_function(f);
        usb_put_function_instance(fi);
        return ret;
    }

    /* ACM function (from gserial) */
    fi = usb_get_function_instance("acm");
    if (IS_ERR(fi))
        return PTR_ERR(fi);

    f = usb_get_function(fi);
    if (IS_ERR(f)) {
        usb_put_function_instance(fi);
        return PTR_ERR(f);
    }

    ret = usb_add_function(c, f);
    if (ret) {
        usb_put_function(f);
        usb_put_function_instance(fi);
        return ret;
    }

    return 0;
}

/* ---- Composite bind ---- */
static int bridge_bind(struct usb_composite_dev *cdev)
{
    static struct usb_configuration cfg = {
        .label               = "adb+acm",
        .bConfigurationValue = 1,
        .bmAttributes        = USB_CONFIG_ATT_ONE, /* bus powered */
        .MaxPower            = 250,                /* 500 mA */
    };

    /* pass base descriptors via driver struct; just add config here */
    return usb_add_config(cdev, &cfg, do_config);
}

/* ---- Composite driver ---- */
static struct usb_composite_driver adbd_bridge_driver = {
    .name      = "g_adbd_bridge",
    .dev       = &device_desc,
    .strings   = dev_strings,
    .max_speed = USB_SPEED_HIGH,
    .bind      = bridge_bind,
};

static int __init g_adbd_bridge_init(void)
{
    return usb_composite_probe(&adbd_bridge_driver);
}

static void __exit g_adbd_bridge_exit(void)
{
    usb_composite_unregister(&adbd_bridge_driver);
}

module_init(g_adbd_bridge_init);
module_exit(g_adbd_bridge_exit);

MODULE_DESCRIPTION("ADB + gserial ACM composite gadget (multi.c style)");
MODULE_AUTHOR("George");
MODULE_LICENSE("GPL");
