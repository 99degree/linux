// SPDX-License-Identifier: GPL-2.0
/*
 * Simple DSI bridge driver (compacted, with mux GPIO selection)
 *
 * Author: George Chan
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/gpio/consumer.h>

#include <drm/drm_bridge.h>
#include <drm/drm_panel.h>
#include <drm/drm_mipi_dsi.h>

/* ------------------------------------------------------------------------- */
/* Context                                                                   */

struct simple_bridge {
    struct drm_bridge   bridge;
    struct drm_bridge   *next_bridge;
    struct device       *dev;

    struct mipi_dsi_host    *host;
    struct mipi_dsi_device  *panel_dsi;
    struct drm_panel    *panel;
    struct device_node  *panel_node;
};

#define drm_bridge_to_simple_bridge(b) \
    container_of(b, struct simple_bridge, bridge)

/* ------------------------------------------------------------------------- */
/* DSI and panel resolution (public API only)                                */

static struct mipi_dsi_host *simple_bridge_find_dsi_host(struct device *dev)
{
    struct device_node *ep;

    for_each_endpoint_of_node(dev->of_node, ep) {
        struct device_node *remote_parent;
        struct mipi_dsi_host *host;

        remote_parent = of_graph_get_remote_port_parent(ep);
        if (!remote_parent)
            continue;

        host = of_find_mipi_dsi_host_by_node(remote_parent);
        of_node_put(remote_parent);

        if (host) {
            of_node_put(ep);
            return host;
        }
    }

    return ERR_PTR(-EPROBE_DEFER);
}

static int simple_bridge_select_panel(struct device *dev,
                      struct mipi_dsi_host *host,
                      struct simple_bridge *sbridge)
{
    struct device_node *child;
    struct mipi_dsi_device *dsi;
    struct gpio_desc *mux_gpio;
    int mux_val = 0;
    u32 reg_val;

    /* Optional mux GPIO to select which panel to use */
    mux_gpio = devm_gpiod_get_optional(dev, "mux", GPIOD_IN);
    if (!IS_ERR(mux_gpio))
        mux_val = gpiod_get_value_cansleep(mux_gpio);

    for_each_child_of_node(dev->of_node, child) {
        if (!child->name || strncmp(child->name, "panel", 5))
            continue;

        /* If mux is present, only match the panel with matching reg */
        if (mux_val >= 0 &&
            !of_property_read_u32(child, "reg", &reg_val) &&
            mux_val != reg_val)
            continue;

        dsi = of_find_mipi_dsi_device_by_node(child);
        if (!dsi) {
            struct mipi_dsi_device_info info = {
                .type    = "",
                .channel = 0,
                .node    = child,
            };

            dsi = mipi_dsi_device_register_full(host, &info);
            if (IS_ERR(dsi))
                continue;
        }
        sbridge->panel_dsi = dsi;
        sbridge->panel_node = child;
        return 0;
    }

    return -ENODEV;
}

/* ------------------------------------------------------------------------- */
/* Bridge lifecycle                                                          */

static int simple_bridge_attach(struct drm_bridge *bridge,
                struct drm_encoder *encoder,
                enum drm_bridge_attach_flags flags)
{
    struct simple_bridge *sbridge = drm_bridge_to_simple_bridge(bridge);
    if (!sbridge->next_bridge) {
        struct drm_panel *panel;

        if (!sbridge->panel_node)
            return -ENODEV;

        panel = of_drm_find_panel(sbridge->panel_node);
        if (IS_ERR(panel))
            return PTR_ERR(panel);

        sbridge->panel = panel;
        sbridge->next_bridge = drm_panel_bridge_add(panel);
        if (IS_ERR(sbridge->next_bridge)) {
	    sbridge->next_bridge = NULL;
	    return PTR_ERR(sbridge->next_bridge);
	}
    }

    return drm_bridge_attach(encoder, sbridge->next_bridge, bridge, flags);
}

static const struct drm_bridge_funcs simple_bridge_funcs = {
    .attach = simple_bridge_attach,
};

/* ------------------------------------------------------------------------- */
/* Probe / remove                                                            */

static int simple_bridge_probe(struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct simple_bridge *sbridge;
    int ret;
    sbridge = devm_drm_bridge_alloc(dev, struct simple_bridge, bridge,
                    &simple_bridge_funcs);
    if (IS_ERR(sbridge))
        return PTR_ERR(sbridge);

    sbridge->dev = dev;
    sbridge->bridge.of_node = dev->of_node;
    platform_set_drvdata(pdev, sbridge);

    /* Resolve DSI host */
    sbridge->host = simple_bridge_find_dsi_host(dev);
    if (IS_ERR(sbridge->host))
        return PTR_ERR(sbridge->host);

    /* Resolve/select panel and ensure a DSI device exists */
    ret = simple_bridge_select_panel(dev, sbridge->host, sbridge);
    if (ret)
        return ret;

    /* Register our bridge */
    return devm_drm_bridge_add(dev, &sbridge->bridge);
}

static void simple_bridge_remove(struct platform_device *pdev)
{
    struct simple_bridge *sbridge = platform_get_drvdata(pdev);

    if (sbridge->panel && sbridge->next_bridge) {
        drm_panel_bridge_remove(sbridge->next_bridge);
        sbridge->next_bridge = NULL;
    }

    if (sbridge->panel_dsi) {
        mipi_dsi_device_unregister(sbridge->panel_dsi);
        sbridge->panel_dsi = NULL;
    }
}

/* ------------------------------------------------------------------------- */
/* Platform boilerplate                                                      */

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

MODULE_AUTHOR("George Chan <gchan9527@gmail.com>");
MODULE_DESCRIPTION("Simple DSI bridge driver (with mux GPIO selection)");
MODULE_LICENSE("GPL");
