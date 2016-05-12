/*
 * (C) Copyright 2016 Beniamino Galvani <b.galvani@gmail.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <asm/io.h>
#include <asm/arch/gxbb.h>
#include <asm/arch/sm.h>
#include <asm/arch/sd_emmc.h>
#include <dm/platdata.h>
#include <phy.h>

#define EFUSE_SN_OFFSET		20
#define EFUSE_SN_SIZE		16
#define EFUSE_MAC_OFFSET	52
#define EFUSE_MAC_SIZE		6

int board_init(void)
{
	return 0;
}

static const struct eth_pdata gxbb_eth_pdata = {
	.iobase = GXBB_ETH_BASE,
	.phy_interface = PHY_INTERFACE_MODE_RGMII,
};

U_BOOT_DEVICE(meson_eth) = {
	.name = "eth_designware",
	.platdata = &gxbb_eth_pdata,
};

int misc_init_r(void)
{
	u8 mac_addr[EFUSE_MAC_SIZE];
	ssize_t len;

	/* Select Ethernet function */
	setbits_le32(GXBB_PINMUX(6), 0x3fff);

	/* Set RGMII mode */
	setbits_le32(GXBB_ETH_REG_0, GXBB_ETH_REG_0_PHY_INTF |
				     GXBB_ETH_REG_0_TX_PHASE(1) |
				     GXBB_ETH_REG_0_TX_RATIO(4) |
				     GXBB_ETH_REG_0_PHY_CLK_EN |
				     GXBB_ETH_REG_0_CLK_EN);

	/* Enable power and clock gate */
	setbits_le32(GXBB_GCLK_MPEG_1, GXBB_GCLK_MPEG_1_ETH);
	clrbits_le32(GXBB_MEM_PD_REG_0, GXBB_MEM_PD_REG_0_ETH_MASK);

	/* Reset PHY on GPIOZ_14 */
	clrbits_le32(GXBB_GPIO_EN(3), BIT(14));
	clrbits_le32(GXBB_GPIO_OUT(3), BIT(14));
	mdelay(10);
	setbits_le32(GXBB_GPIO_OUT(3), BIT(14));

	if (!eth_getenv_enetaddr("ethaddr", mac_addr)) {
		len = meson_sm_read_efuse(EFUSE_MAC_OFFSET,
					  mac_addr, EFUSE_MAC_SIZE);
		if (len == EFUSE_MAC_SIZE && is_valid_ethaddr(mac_addr))
			eth_setenv_enetaddr("ethaddr", mac_addr);
	}

	return 0;
}

#ifdef CONFIG_GENERIC_MMC

static const struct meson_mmc_platdata gxbb_sd_platdata[] = {
	{ .sd_emmc_reg = (struct meson_mmc_regs *)SD_EMMC_BASE_A },
	{ .sd_emmc_reg = (struct meson_mmc_regs *)SD_EMMC_BASE_B },
	{ .sd_emmc_reg = (struct meson_mmc_regs *)SD_EMMC_BASE_C },
};

U_BOOT_DEVICE(meson_mmc) = {
	.name = "meson_mmc",
	.platdata = &gxbb_sd_platdata[CONFIG_MMC_MESON_SD_PORT],
};

static void meson_mmc_pinmux_setup(unsigned int port)
{
	switch (port) {
	case SDIO_PORT_A:
		setbits_le32(GXBB_PINMUX(8), 0x3f);
		break;
	case SDIO_PORT_B:
		setbits_le32(GXBB_PINMUX(2), 0x3f << 10);
		break;
	case SDIO_PORT_C:
		clrbits_le32(GXBB_PINMUX(2), 0x1f << 22);
		setbits_le32(GXBB_PINMUX(4), (0x3 << 18) | (3 << 30));
		break;
	default:
		printf("meson: invalid MMC port %d for pinmux setup\n", port);
		break;
	}
}

int board_mmc_init(bd_t *bis)
{
	meson_mmc_pinmux_setup(CONFIG_MMC_MESON_SD_PORT);

	return 0;
}

#endif
