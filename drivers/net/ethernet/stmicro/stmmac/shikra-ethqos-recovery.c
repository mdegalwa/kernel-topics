// SPDX-License-Identifier: GPL-2.0-only
/*
 * Shikra ETHQOS Thin Recovery Driver
 *
 * Copyright (c) 2026 Qualcomm Technologies, Inc.
 *
 * Minimal standalone platform driver that restores EMAC0 hardware
 * resources after qcom-ethqos has been unbound from 5d00000.ethernet
 * on the Qualcomm Shikra EVK, enabling vfio-platform to bind and the
 * DPDK PMD (stmmac_pmd) to take ownership.
 *
 * CONFIRMED HARDWARE OBSERVATIONS (Shikra EVK, EMAC0):
 *
 * Before unbind:
 *   GPIO135 = HIGH (physical 1), DP83867 PHY visible at MDIO addr 7
 *   (PHY ID 0x2000a231), eth0 present, all 6 EMAC0 clocks enabled.
 *
 * After 'echo 5d00000.ethernet > .../qcom-ethqos/unbind':
 *   GPIO135 = LOW (physical 0) -> DP83867 in reset
 *   PHY at MDIO addr 7 disappears, eth0 removed, MDIO bus gone
 *   All 6 EMAC0 clocks disabled
 *
 * Immediate rebind fails: "MDIO device at address 7 is missing",
 * "cannot attach to PHY (-ENODEV)".
 * After reboot: GPIO135 HIGH, PHY ID 0x2000a231 visible, eth0 restored.
 *
 * CLOCK NAME CORRECTION vs. original task description:
 *   The original task listed "axi" as one of five clocks.
 *   Hardware investigation confirms SIX clocks and NO "axi" entry.
 *   Confirmed DT clock-names: stmmaceth, pclk, ptp_ref, rgmii,
 *   axi-noc, pcie-tile-axi-noc.
 *   "axi" appears in shikra_noc_clks[] in dwmac-qcom-ethqos.c but was
 *   NOT observed disabled after unbind and is NOT in the DT clock-names.
 *
 * GPIO LOCATION CORRECTION:
 *   reset-gpios (GPIO135) is in the ethernet-phy@7 child node under
 *   the mdio sub-node, NOT in the parent ethernet@5d00000 node.
 *   devm_gpiod_get(&pdev->dev, "reset", ...) will NOT find it.
 *   This driver searches the mdio/ethernet-phy@7 child node explicitly.
 *
 * DPDK OWNERSHIP MODEL:
 *   1. qcom-ethqos unbind  -> GPIO135 LOW, 6 clocks OFF (broken)
 *   2. recovery bind       -> GPIO135 HIGH, 6 clocks ON (restored)
 *   3. recovery unbind     -> hardware state PRESERVED (DPDK handoff)
 *   4. vfio-platform bind  -> DPDK PMD owns hardware via VFIO
 *
 *   On unbind (step 3), devm releases handles but NOT hardware state:
 *   clk_put() does not decrement enable count -> clocks stay ON.
 *   GPIO descriptor release does not change pin level -> GPIO135 HIGH.
 *   This is the default (cleanup_on_remove=false).
 *
 * References:
 *   drivers/net/ethernet/stmicro/stmmac/dwmac-qcom-ethqos.c
 *   drivers/net/ethernet/stmicro/stmmac/stmmac_platform.c
 *   drivers/net/phy/dp83867.c
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reset.h>

#define SHIKRA_ETHQOS_NUM_CLKS		6
#define SHIKRA_PHY_RESET_ASSERT_US	10000U
#define SHIKRA_PHY_RESET_DEASSERT_US	50000U
#define SHIKRA_PHY_RESET_SLACK_US	5000U
#define DP83867_PHY_ID_CONFIRMED	0x2000a231U
#define DP83867_MDIO_ADDR		7

/**
 * struct shikra_ethqos_recovery - per-device driver state
 * @dev:              platform device's struct device
 * @rst:              optional GCC reset control (DT: resets = <&gcc 3>)
 * @clks:             EMAC0 clock handles (indexed by shikra_ethqos_clk_names)
 * @phy_reset_gpio:   GPIO135 descriptor (from mdio/ethernet-phy@7/reset-gpios)
 * @num_clks_enabled: clocks successfully enabled; bounds error-unwind loop
 */
struct shikra_ethqos_recovery {
	struct device		*dev;
	struct reset_control	*rst;
	struct clk		*clks[SHIKRA_ETHQOS_NUM_CLKS];
	struct gpio_desc	*phy_reset_gpio;
	int			num_clks_enabled;
};

/*
 * Six EMAC0 clock names confirmed from Shikra DT hardware investigation.
 * "axi" excluded: not observed disabled after unbind, not in DT.
 *
 * DT clock-name        GCC clock                     Source
 * -------------------  ----------------------------  ----------------
 * "stmmaceth"          gcc_emac0_axi_clk             stmmac_platform.c
 * "pclk"               gcc_emac0_ahb_clk             stmmac_platform.c
 * "ptp_ref"            gcc_emac0_ptp_clk             stmmac_platform.c
 * "rgmii"              gcc_emac0_rgmii_clk           dwmac-qcom-ethqos.c
 * "axi-noc"            gcc_emac0_axi_sys_noc_clk     shikra_noc_clks[]
 * "pcie-tile-axi-noc"  gcc_emac0_pcie_tile_noc_clk   shikra_noc_clks[]
 */
static const char * const shikra_ethqos_clk_names[SHIKRA_ETHQOS_NUM_CLKS] = {
	"stmmaceth",
	"pclk",
	"ptp_ref",
	"rgmii",
	"axi-noc",
	"pcie-tile-axi-noc",
};

/*
 * cleanup_on_remove=false (default): DPDK handoff mode.
 *   Hardware state PRESERVED on unbind. devm releases handles but NOT
 *   hardware state. clk_put() does not decrement enable count.
 *   GPIO descriptor release does not change pin level.
 *
 * cleanup_on_remove=true: clean shutdown mode.
 *   PHY reset asserted, clocks disabled, GCC reset asserted.
 */
static bool cleanup_on_remove;
module_param(cleanup_on_remove, bool, 0644);
MODULE_PARM_DESC(cleanup_on_remove,
		 "Tear down hardware on remove (default: false = DPDK handoff)");

static int shikra_ethqos_enable_clocks(struct shikra_ethqos_recovery *priv)
{
	struct device *dev = priv->dev;
	int i, ret;

	for (i = 0; i < SHIKRA_ETHQOS_NUM_CLKS; i++) {
		priv->clks[i] = devm_clk_get(dev, shikra_ethqos_clk_names[i]);
		if (IS_ERR(priv->clks[i])) {
			ret = PTR_ERR(priv->clks[i]);
			priv->clks[i] = NULL;
			dev_err(dev, "recovery: failed to get clock '%s': %d\n",
				shikra_ethqos_clk_names[i], ret);
			goto err_disable;
		}

		ret = clk_prepare_enable(priv->clks[i]);
		if (ret) {
			dev_err(dev, "recovery: failed to enable clock '%s': %d\n",
				shikra_ethqos_clk_names[i], ret);
			goto err_disable;
		}

		priv->num_clks_enabled = i + 1;
		dev_info(dev, "recovery: clock[%d] '%s' enabled (rate=%lu Hz)\n",
			 i, shikra_ethqos_clk_names[i],
			 clk_get_rate(priv->clks[i]));
	}
	return 0;

err_disable:
	while (--i >= 0) {
		if (priv->clks[i]) {
			clk_disable_unprepare(priv->clks[i]);
			dev_info(dev, "recovery: clock '%s' disabled (unwind)\n",
				 shikra_ethqos_clk_names[i]);
		}
	}
	priv->num_clks_enabled = 0;
	return ret;
}

static void shikra_ethqos_disable_clocks(struct shikra_ethqos_recovery *priv)
{
	struct device *dev = priv->dev;
	int i;

	for (i = priv->num_clks_enabled - 1; i >= 0; i--) {
		if (priv->clks[i]) {
			clk_disable_unprepare(priv->clks[i]);
			dev_info(dev, "recovery: clock '%s' disabled\n",
				 shikra_ethqos_clk_names[i]);
		}
	}
	priv->num_clks_enabled = 0;
}

/*
 * shikra_ethqos_get_phy_reset_gpio - acquire GPIO135 from correct DT node
 *
 * reset-gpios is in mdio/ethernet-phy@7, NOT the parent ethernet node.
 * Tries parent first (forward compat), then searches child node.
 */
static struct gpio_desc *
shikra_ethqos_get_phy_reset_gpio(struct shikra_ethqos_recovery *priv)
{
	struct device *dev = priv->dev;
	struct device_node *np = dev->of_node;
	struct device_node *mdio_node, *phy_node, *found_phy;
	struct gpio_desc *gpio;

	/* Attempt 1: reset-gpios in parent ethernet@5d00000 node */
	gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(gpio))
		return gpio;
	if (gpio) {
		dev_info(dev, "recovery: reset-gpios found in parent node\n");
		return gpio;
	}

	/*
	 * Attempt 2: reset-gpios in mdio/ethernet-phy@7 child node.
	 * Confirmed location on Shikra EVK:
	 *   ethernet@5d00000 { mdio { ethernet-phy@7 {
	 *       reset-gpios = <&tlmm 135 GPIO_ACTIVE_LOW>; }; }; };
	 */
	dev_info(dev, "recovery: searching mdio/ethernet-phy@%d for reset-gpios\n",
		 DP83867_MDIO_ADDR);

	mdio_node = of_get_child_by_name(np, "mdio");
	if (!mdio_node) {
		dev_err(dev, "recovery: no 'mdio' child node in DT\n");
		return ERR_PTR(-ENODEV);
	}

	found_phy = NULL;
	for_each_child_of_node(mdio_node, phy_node) {
		u32 reg = 0;

		if (!of_property_read_u32(phy_node, "reg", &reg) &&
		    reg == DP83867_MDIO_ADDR) {
			found_phy = phy_node;
			break;
		}
	}
	of_node_put(mdio_node);

	if (!found_phy) {
		dev_err(dev, "recovery: ethernet-phy@%d not found in mdio\n",
			DP83867_MDIO_ADDR);
		return ERR_PTR(-ENODEV);
	}

	dev_info(dev, "recovery: found ethernet-phy@%d; acquiring reset-gpios\n",
		 DP83867_MDIO_ADDR);

	gpio = devm_fwnode_gpiod_get_index(dev, of_fwnode_handle(found_phy),
					   "reset", 0, GPIOD_OUT_HIGH,
					   "phy-reset");
	of_node_put(found_phy);

	if (IS_ERR(gpio))
		dev_err(dev, "recovery: failed to get reset-gpios: %ld\n",
			PTR_ERR(gpio));
	else
		dev_info(dev, "recovery: GPIO135 acquired (GPIO_ACTIVE_LOW)\n");

	return gpio;
}

static int shikra_ethqos_phy_reset(struct shikra_ethqos_recovery *priv)
{
	struct device *dev = priv->dev;
	struct gpio_desc *gpio;
	int gpio_val;

	gpio = shikra_ethqos_get_phy_reset_gpio(priv);
	if (IS_ERR(gpio))
		return PTR_ERR(gpio);

	priv->phy_reset_gpio = gpio;

	/* Assert reset: logical 1 -> physical LOW (GPIO_ACTIVE_LOW) */
	gpiod_set_value(priv->phy_reset_gpio, 1);
	dev_info(dev, "recovery: PHY reset ASSERTED (GPIO135 logical=1, physical=LOW)\n");

	usleep_range(SHIKRA_PHY_RESET_ASSERT_US,
		     SHIKRA_PHY_RESET_ASSERT_US + SHIKRA_PHY_RESET_SLACK_US);
	dev_info(dev, "recovery: PHY reset hold complete (%u us)\n",
		 SHIKRA_PHY_RESET_ASSERT_US);

	/* Deassert reset: logical 0 -> physical HIGH */
	gpiod_set_value(priv->phy_reset_gpio, 0);
	dev_info(dev, "recovery: PHY reset DEASSERTED (GPIO135 logical=0, physical=HIGH)\n");

	usleep_range(SHIKRA_PHY_RESET_DEASSERT_US,
		     SHIKRA_PHY_RESET_DEASSERT_US + SHIKRA_PHY_RESET_SLACK_US);
	dev_info(dev, "recovery: PHY stabilisation complete (%u us)\n",
		 SHIKRA_PHY_RESET_DEASSERT_US);

	gpio_val = gpiod_get_value(priv->phy_reset_gpio);
	dev_info(dev, "recovery: GPIO135 logical=%d (expected 0 = physical HIGH = PHY OK)\n",
		 gpio_val);

	if (gpio_val != 0)
		dev_warn(dev, "recovery: WARNING GPIO135=%d; PHY may still be in reset\n",
			 gpio_val);
	else
		dev_info(dev, "recovery: DP83867 at MDIO addr %d (ID 0x%08x) accessible\n",
			 DP83867_MDIO_ADDR, DP83867_PHY_ID_CONFIRMED);

	return 0;
}

static void shikra_ethqos_verify_recovery(struct shikra_ethqos_recovery *priv)
{
	struct device *dev = priv->dev;
	int i;

	dev_info(dev, "recovery: ---- post-recovery verification ----\n");
	for (i = 0; i < priv->num_clks_enabled; i++) {
		unsigned long rate = priv->clks[i] ?
				     clk_get_rate(priv->clks[i]) : 0;

		dev_info(dev, "recovery: clock[%d] '%s' rate=%lu Hz %s\n",
			 i, shikra_ethqos_clk_names[i], rate,
			 rate > 0 ? "[OK]" : "[WARNING: rate=0]");
	}
	if (priv->phy_reset_gpio) {
		int val = gpiod_get_value(priv->phy_reset_gpio);

		dev_info(dev, "recovery: GPIO135 logical=%d physical=%s PHY=%s %s\n",
			 val, val == 0 ? "HIGH" : "LOW",
			 val == 0 ? "operational" : "IN RESET",
			 val == 0 ? "[OK]" : "[WARNING]");
	}
	dev_info(dev, "recovery: GCC reset = %s\n",
		 priv->rst ? "deasserted [OK]" : "not present (optional)");
	dev_info(dev, "recovery: ---- verification complete ----\n");
}

static int shikra_ethqos_recovery_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct shikra_ethqos_recovery *priv;
	int ret;

	dev_info(dev, "recovery: Shikra ETHQOS thin recovery driver probing\n");
	dev_info(dev, "recovery: restoring EMAC0 (GPIO135 LOW, 6 clocks OFF, PHY in reset)\n");

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = dev;
	platform_set_drvdata(pdev, priv);

	/* Step 1: GCC EMAC0 reset deassert (optional, DT: resets = <&gcc 3>) */
	priv->rst = devm_reset_control_get_optional_exclusive(dev, NULL);
	if (IS_ERR(priv->rst)) {
		ret = PTR_ERR(priv->rst);
		dev_err(dev, "recovery: failed to get GCC reset: %d\n", ret);
		return ret;
	}
	if (priv->rst) {
		dev_info(dev, "recovery: GCC EMAC0 reset acquired\n");
		ret = reset_control_deassert(priv->rst);
		if (ret) {
			dev_err(dev, "recovery: failed to deassert GCC reset: %d\n", ret);
			return ret;
		}
		dev_info(dev, "recovery: GCC EMAC0 reset deasserted\n");
	} else {
		dev_info(dev, "recovery: no GCC reset (optional, skipping)\n");
	}

	/* Step 2: Enable 6 confirmed EMAC0 clocks */
	dev_info(dev, "recovery: enabling %d clocks: stmmaceth pclk ptp_ref rgmii axi-noc pcie-tile-axi-noc\n",
		 SHIKRA_ETHQOS_NUM_CLKS);
	ret = shikra_ethqos_enable_clocks(priv);
	if (ret) {
		dev_err(dev, "recovery: clock enable failed: %d\n", ret);
		goto err_assert_reset;
	}
	dev_info(dev, "recovery: all %d EMAC0 clocks enabled\n", SHIKRA_ETHQOS_NUM_CLKS);

	/* Step 3: DP83867 PHY reset via GPIO135 (mdio/ethernet-phy@7/reset-gpios) */
	dev_info(dev, "recovery: PHY reset (GPIO135 in mdio/ethernet-phy@%d, assert=%u us, deassert=%u us)\n",
		 DP83867_MDIO_ADDR, SHIKRA_PHY_RESET_ASSERT_US, SHIKRA_PHY_RESET_DEASSERT_US);
	ret = shikra_ethqos_phy_reset(priv);
	if (ret) {
		dev_err(dev, "recovery: PHY reset failed: %d\n", ret);
		goto err_disable_clks;
	}

	shikra_ethqos_verify_recovery(priv);

	dev_info(dev, "recovery: *** Shikra ETHQOS recovery COMPLETE ***\n");
	dev_info(dev, "recovery: DPDK handoff: unbind this driver, bind vfio-platform\n");
	return 0;

err_disable_clks:
	shikra_ethqos_disable_clocks(priv);
err_assert_reset:
	if (priv->rst) {
		reset_control_assert(priv->rst);
		dev_info(dev, "recovery: GCC reset asserted (error unwind)\n");
	}
	return ret;
}

static void shikra_ethqos_recovery_remove(struct platform_device *pdev)
{
	struct shikra_ethqos_recovery *priv = platform_get_drvdata(pdev);
	struct device *dev = &pdev->dev;

	dev_info(dev, "recovery: removing\n");

	if (!cleanup_on_remove) {
		/* DPDK handoff: preserve hardware state */
		dev_info(dev, "recovery: cleanup_on_remove=false -- hardware PRESERVED\n");
		dev_info(dev, "recovery:   clocks=ENABLED, GPIO135=HIGH, GCC reset=DEASSERTED\n");
		dev_info(dev, "recovery: bind vfio-platform next\n");
		return;
	}

	/* Clean shutdown */
	if (priv->phy_reset_gpio) {
		gpiod_set_value(priv->phy_reset_gpio, 1);
		dev_info(dev, "recovery: GPIO135 asserted (PHY in reset)\n");
	}
	shikra_ethqos_disable_clocks(priv);
	if (priv->rst) {
		reset_control_assert(priv->rst);
		dev_info(dev, "recovery: GCC EMAC0 reset asserted\n");
	}
	dev_info(dev, "recovery: removed\n");
}

static const struct of_device_id shikra_ethqos_recovery_of_match[] = {
	{ .compatible = "qcom,shikra-ethqos" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, shikra_ethqos_recovery_of_match);

static struct platform_driver shikra_ethqos_recovery_driver = {
	.probe		= shikra_ethqos_recovery_probe,
	.remove		= shikra_ethqos_recovery_remove,
	.driver		= {
		.name		= "shikra-ethqos-recovery",
		.of_match_table	= shikra_ethqos_recovery_of_match,
	},
};
module_platform_driver(shikra_ethqos_recovery_driver);

MODULE_DESCRIPTION("Shikra ETHQOS thin recovery driver: restores 6 EMAC0 "
		   "clocks and DP83867 PHY reset (GPIO135) after "
		   "qcom-ethqos unbind; preserves state for DPDK/vfio handoff");
MODULE_AUTHOR("Qualcomm Technologies, Inc.");
MODULE_LICENSE("GPL v2");
