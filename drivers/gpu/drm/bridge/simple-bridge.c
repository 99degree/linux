// SPDX-License-Identifier: GPL-2.0
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/device.h>

#include <drm/drm_bridge.h>
#include <drm/drm_panel.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h> /* drm_mode_vrefresh */

#undef dev_dbg
#define dev_dbg dev_info

struct simple_bridge {
    struct drm_bridge bridge;
    struct drm_bridge *int_bridge;
    struct device *dev;

    struct mipi_dsi_host *host;
    struct mipi_dsi_device *panel_dsi;

    struct drm_panel *panel;
    struct drm_bridge *panel_bridge;
    struct device_node *panel_node;

//    struct drm_panel_follower follower;

    /* Follower-driven readiness */
//    bool panel_prepared;
    unsigned long prepared_timeout_ms; /* default 1500 ms */
};

/* ---------------------------------------------------------- */
/* DSI host and panel selection / registration                */
/* ---------------------------------------------------------- */

static struct mipi_dsi_host *simple_bridge_find_dsi_host(struct device *dev)
{
    struct device_node *ep;
    struct mipi_dsi_host *host = NULL;

    dev_dbg(dev, "simple-bridge: resolving DSI host via graph on %pOF\n",
        dev->of_node);

    for_each_endpoint_of_node(dev->of_node, ep) {
        struct device_node *remote_ep, *node, *next;

        remote_ep = of_graph_get_remote_endpoint(ep);
        if (!remote_ep)
            continue;

        node = of_get_parent(remote_ep);
        of_node_put(remote_ep);

        while (node &&
               (!strcmp(node->name, "endpoint") ||
            !strcmp(node->name, "port") ||
            !strcmp(node->name, "ports"))) {
            next = of_get_parent(node);
            of_node_put(node);
            node = next;
        }

        if (!node)
            continue;

        host = of_find_mipi_dsi_host_by_node(node);
        of_node_put(node);
        if (host)
            return host;
    }

    dev_err(dev, "no connected DSI host found; deferring\n");
    return ERR_PTR(-EPROBE_DEFER);
}

static int simple_bridge_select_and_register_dsi(struct device *dev,
                         struct mipi_dsi_host *host,
                         struct simple_bridge *ctx)
{
    struct device_node *child;

    for_each_child_of_node(dev->of_node, child) {
        u32 reg_val = 0;
        int gpio, mux_val;

        /* consider panel@X children only */
        if (!child->name || strncmp(child->name, "panel", 5) != 0)
            continue;

        /* use 'reg' as logical selector */
        if (of_property_read_u32(child, "reg", &reg_val))
            continue;
#if 0
        /* mux-gpio must match the child's reg to select this panel */
        gpio = of_get_named_gpio(child, "mux-gpios", 0);
        if (gpio_is_valid(gpio)) {
            if (gpio_request_one(gpio, GPIOF_IN, "panel-mux"))
                continue;
            mux_val = gpio_get_value(gpio);
            gpio_free(gpio);
            if (mux_val != reg_val)
                continue;
        }
#endif
        /* prefer reusing existing DSI device if host created it */
        ctx->panel_dsi = of_find_mipi_dsi_device_by_node(child);
        if (ctx->panel_dsi) {
            ctx->panel_node = child;
            dev_dbg(dev, "reusing DSI device %s for %pOF (reg=%u)\n",
                dev_name(&ctx->panel_dsi->dev), child, reg_val);
            return 0;
        }

        /* otherwise, explicitly register one against the host */
        {
            struct mipi_dsi_device_info info = {
                .type    = "",
                .channel = 0,       /* single active panel → channel 0 */
                .node    = child,   /* DT node of the selected panel endpoint */
            };
            struct mipi_dsi_device *dsi;

            dsi = mipi_dsi_device_register_full(host, &info);
            if (IS_ERR(dsi)) {
                int err = PTR_ERR(dsi);
                dev_err(dev, "mipi_dsi_device_register_full(%pOF) failed: %d\n",
                    child, err);
                continue;
            }

            ctx->panel_dsi = dsi;
            ctx->panel_node = child;
            dev_dbg(dev, "registered new DSI device %s for %pOF (reg=%u, ch=0)\n",
                dev_name(&dsi->dev), child, reg_val);
            return 0;
        }
    }

    dev_err(dev, "no eligible panel subnode under %pOF\n", dev->of_node);
    return -ENODEV;
}

#if 0
/* ---------------------------------------------------------- */
/* Panel follower                                             */
/* ---------------------------------------------------------- */

static int simple_bridge_panel_prepared(struct drm_panel_follower *follower)
{
    struct simple_bridge *ctx = container_of(follower, struct simple_bridge, follower);

    ctx->panel_prepared = true;

    dev_dbg(ctx->dev, "panel_prepared follower: prepared=1 (signaled)\n");
    return 0;
}

static int simple_bridge_panel_unpreparing(struct drm_panel_follower *follower)
{
    struct simple_bridge *ctx = container_of(follower, struct simple_bridge, follower);

    ctx->panel_prepared = false;

    dev_dbg(ctx->dev, "panel_unpreparing follower: prepared=0 (rearmed)\n");
    return 0;
}
#endif
/* ---------------------------------------------------------- */
/* Bridge lifecycle callbacks                                 */
/* ---------------------------------------------------------- */

static int simple_bridge_attach(struct drm_bridge *bridge,
                struct drm_encoder *encoder,
                enum drm_bridge_attach_flags flags)
{
    struct simple_bridge *ctx = container_of(bridge, struct simple_bridge, bridge);
    struct drm_panel *panel;

    dev_dbg(ctx->dev, "bridge.attach called\n");

    /* Build downstream panel_bridge on-demand, but never defer attach */
    if (!ctx->panel_bridge) {
        if (!ctx->panel_node) {
            dev_err(ctx->dev, "attach: no panel_node selected\n");
            return -ENODEV;
        }

        panel = of_drm_find_panel(ctx->panel_node);
        if (IS_ERR(panel)) {
            int err = PTR_ERR(panel);
            dev_info(ctx->dev, "attach: of_drm_find_panel returned %d\n", err);
            return err;
        }

        ctx->panel = panel;
        ctx->panel_bridge = drm_panel_bridge_add(panel);
        if (IS_ERR(ctx->panel_bridge)) {
            int err = PTR_ERR(ctx->panel_bridge);
            dev_err(ctx->dev, "attach: create panel_bridge failed: %d\n", err);
            ctx->panel_bridge = NULL;
            return err;
        }
        dev_dbg(ctx->dev, "attach: panel_bridge created\n");
    }

    return drm_bridge_attach(encoder, ctx->panel_bridge, bridge, flags);
}

static void simple_bridge_detach(struct drm_bridge *bridge)
{
    struct simple_bridge *ctx = container_of(bridge, struct simple_bridge, bridge);
    dev_dbg(ctx->dev, "bridge.detach called\n");
}

static enum drm_mode_status
simple_bridge_mode_valid(struct drm_bridge *bridge,
             const struct drm_display_info *info,
             const struct drm_display_mode *mode)
{
    struct simple_bridge *ctx = container_of(bridge, struct simple_bridge, bridge);
    dev_dbg(ctx->dev, "bridge.mode_valid: %dx%d@%d\n",
        mode->hdisplay, mode->vdisplay, drm_mode_vrefresh(mode));
    return MODE_OK;
}

static bool simple_bridge_mode_fixup(struct drm_bridge *bridge,
                     const struct drm_display_mode *mode,
                     struct drm_display_mode *adjusted_mode)
{
    struct simple_bridge *ctx = container_of(bridge, struct simple_bridge, bridge);
    dev_dbg(ctx->dev, "bridge.mode_fixup called\n");
    return true;
}

static void simple_bridge_mode_set(struct drm_bridge *bridge,
                   const struct drm_display_mode *mode,
                   const struct drm_display_mode *adj)
{
    struct simple_bridge *ctx = container_of(bridge, struct simple_bridge, bridge);
    dev_dbg(ctx->dev, "bridge.mode_set called\n");
}

/* Do NOT block here; panel prepare happens after downstream pre_enable */
static void simple_bridge_pre_enable(struct drm_bridge *bridge)
{
    struct simple_bridge *ctx = container_of(bridge, struct simple_bridge, bridge);
    dev_dbg(ctx->dev, "bridge.pre_enable called (no wait)\n");
}

/* Wait here: by the time enable runs, panel_bridge's pre_enable (and panel prepare) should have executed */
static void simple_bridge_enable(struct drm_bridge *bridge)
{
    struct simple_bridge *ctx = container_of(bridge, struct simple_bridge, bridge);
//    unsigned long timeout = msecs_to_jiffies(ctx->prepared_timeout_ms ?: 1500);
//    bool ready;

    dev_dbg(ctx->dev, "bridge.enable: waiting for panel prepared...\n");
#if 0
    if (ctx->panel_prepared) {
        dev_dbg(ctx->dev, "bridge.enable: already prepared\n");
        return;
    }

    if (!ready) {
        dev_info(ctx->dev, "bridge.enable: panel not prepared (timeout %lums), continuing\n",
             ctx->prepared_timeout_ms ?: 1500);
    }
#endif
}

static void simple_bridge_disable(struct drm_bridge *bridge)
{
    struct simple_bridge *ctx = container_of(bridge, struct simple_bridge, bridge);
    dev_dbg(ctx->dev, "bridge.disable called\n");
}

static void simple_bridge_post_disable(struct drm_bridge *bridge)
{
    struct simple_bridge *ctx = container_of(bridge, struct simple_bridge, bridge);
    dev_dbg(ctx->dev, "bridge.post_disable called\n");
}

/* ---------------------------------------------------------- */
/* Funcs table                                                */
/* ---------------------------------------------------------- */

static const struct drm_bridge_funcs simple_bridge_funcs = {
    .attach        = simple_bridge_attach,
    .detach        = simple_bridge_detach,
    .mode_valid    = simple_bridge_mode_valid,   /* 3-arg signature */
    .mode_fixup    = simple_bridge_mode_fixup,
    .mode_set      = simple_bridge_mode_set,
    .pre_enable    = simple_bridge_pre_enable,
    .enable        = simple_bridge_enable,
    .disable       = simple_bridge_disable,
    .post_disable  = simple_bridge_post_disable,
};

/* ---------------------------------------------------------- */
/* Probe / remove                                             */
/* ---------------------------------------------------------- */

static int simple_bridge_probe(struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct simple_bridge *ctx;
    struct mipi_dsi_host *host;
    int ret = 0;

    dev_dbg(dev, "%s: %d\n", __func__, ret);

    /* Allocate embedded bridge-managed context */
    ctx = devm_drm_bridge_alloc(dev, struct simple_bridge, bridge,
                    &simple_bridge_funcs);
    if (IS_ERR(ctx)) {
        ret = PTR_ERR(ctx);
        dev_err(dev, "devm_drm_bridge_alloc failed: %d\n", ret);
        return ret;
    }

    ctx->dev = dev;
    ctx->bridge.of_node = dev->of_node;
//    ctx->panel_prepared = false;
    ctx->prepared_timeout_ms = 1500; /* allow panel to prepare */
    platform_set_drvdata(pdev, ctx);

    /* Find DSI host */
    host = simple_bridge_find_dsi_host(dev);
    if (IS_ERR(host)) {
        ret = PTR_ERR(host);
        dev_err(dev, "resolve DSI host failed: %d\n", ret);
        return ret;
    }
    ctx->host = host;

    /* Select panel node and ensure a mipi_dsi_device exists (register_full if needed) */
    ret = simple_bridge_select_and_register_dsi(dev, host, ctx);
    if (ret) {
        dev_err(dev, "select/register DSI device failed: %d\n", ret);
        return ret;
    }
#if 0
    /* Add panel follower; ignore only -EPROBE_DEFER, fail other errors */
    ctx->follower.funcs = &(struct drm_panel_follower_funcs) {
        .panel_prepared    = simple_bridge_panel_prepared,
        .panel_unpreparing = simple_bridge_panel_unpreparing,
    };
    ret = devm_drm_panel_add_follower_fwnode(dev,
                         of_fwnode_handle(ctx->panel_node),
                         &ctx->follower);
    if (ret) {
        if (ret == -EPROBE_DEFER) {
            dev_info(dev, "panel follower target not ready; ignoring defer (%d)\n", ret);
            /* continue probing successfully; completion is initialized */
        } else {
            dev_err(dev, "add panel follower failed: %d\n", ret);
            return ret;
        }
    }
#endif
dev_dbg(dev, "%s: bridge_add %d\n", __func__, ret);
    /* Make the bridge visible so KMS can call attach and other hooks */
    ret = devm_drm_bridge_add(dev, &ctx->bridge);
    if (ret) {
        dev_err(dev, "devm_drm_bridge_add failed: %d\n", ret);
        return ret;
    }

    ctx->int_bridge = &ctx->bridge;

    dev_dbg(dev, "simple-bridge probe done\n");
    return 0;
}

static void simple_bridge_remove(struct platform_device *pdev)
{
    struct simple_bridge *ctx = platform_get_drvdata(pdev);

    if (ctx->panel_bridge) {
        drm_panel_bridge_remove(ctx->panel_bridge);
        ctx->panel_bridge = NULL;
    }

    if (ctx->int_bridge) {
        drm_panel_bridge_remove(ctx->int_bridge);
        ctx->int_bridge = NULL;
    }

    /* If we registered a DSI device manually, unregister it */
    if (ctx->panel_dsi) {
        mipi_dsi_device_unregister(ctx->panel_dsi);
        ctx->panel_dsi = NULL;
    }

//    ctx->panel_prepared = false;
}

/* ---------------------------------------------------------- */
/* Platform boilerplate                                       */
/* ---------------------------------------------------------- */

static const struct of_device_id simple_bridge_of_match[] = {
    { .compatible = "simple-dsi-bridge" },
    { }
};
MODULE_DEVICE_TABLE(of, simple_bridge_of_match);

static struct platform_driver simple_bridge_driver = {
    .probe  = simple_bridge_probe,
    .remove = simple_bridge_remove,
    .driver = {
        .name           = "simple-bridge",
        .of_match_table = simple_bridge_of_match,
    },
};

module_platform_driver(simple_bridge_driver);

MODULE_AUTHOR("George");
MODULE_DESCRIPTION("Simple DSI bridge driver with follower-guarded enable");
MODULE_LICENSE("GPL");
