/*
 * (C) Copyright 2016 Carlo Caione <carlo@caione.org>
 *
 * SPDX-License-Identifier:    GPL-2.0+
 */

#ifndef __SD_EMMC_H__
#define __SD_EMMC_H__

#include <mmc.h>

#define SDIO_PORT_A			0
#define SDIO_PORT_B			1
#define SDIO_PORT_C			2

#define SD_EMMC_BASE_A			0xd0070000
#define SD_EMMC_BASE_B			0xd0072000
#define SD_EMMC_BASE_C			0xd0074000

#define SD_IRQ_ALL			0x3fff

#define SD_EMMC_CLKSRC_24M		24000000
#define SD_EMMC_CLKSRC_DIV2		1000000000

#define CLK_DIV				0
#define CLK_SRC				6
#define CLK_CO_PHASE			8
#define CLK_ALWAYS_ON			24

#define ADDR_USE_PING_BUF		BIT(1)

#define SD_EMMC_RXD_ERROR		BIT(0)
#define SD_EMMC_TXD_ERROR		BIT(1)
#define SD_EMMC_DESC_ERROR		BIT(2)
#define SD_EMMC_RESP_CRC_ERROR		BIT(3)
#define SD_EMMC_RESP_TIMEOUT_ERROR	BIT(4)
#define SD_EMMC_DESC_TIMEOUT_ERROR	BIT(5)

#define CFG_BUS_WIDTH			0
#define CFG_BUS_WIDTH_MASK		(0x3 << 0)
#define CFG_BL_LEN			4
#define CFG_BL_LEN_MASK			(0xf << 4)
#define CFG_RESP_TIMEOUT		8
#define CFG_RESP_TIMEOUT_MASK		(0xf << 8)
#define CFG_RC_CC			12
#define CFG_RC_CC_MASK			(0xf << 12)

#define STATUS_RXD_ERR_MASK		0xff
#define STATUS_TXD_ERR			BIT(8)
#define STATUS_DESC_ERR			BIT(9)
#define STATUS_RESP_ERR			BIT(10)
#define STATUS_RESP_TIMEOUT		BIT(11)
#define STATUS_DESC_TIMEOUT		BIT(12)
#define STATUS_END_OF_CHAIN		BIT(13)

#define CMD_CFG_LENGTH_MASK		0x1ff
#define CMD_CFG_CMD_INDEX		24
#define CMD_CFG_BLOCK_MODE		BIT(9)
#define CMD_CFG_R1B			BIT(10)
#define CMD_CFG_END_OF_CHAIN		BIT(11)
#define CMD_CFG_NO_RESP			BIT(16)
#define CMD_CFG_DATA_IO			BIT(18)
#define CMD_CFG_DATA_WR			BIT(19)
#define CMD_CFG_RESP_NOCRC		BIT(20)
#define CMD_CFG_RESP_128		BIT(21)
#define CMD_CFG_OWNER			BIT(31)

struct meson_mmc_regs {
	uint32_t gclock;
	uint32_t gdelay;
	uint32_t gadjust;
	uint32_t reserved_0c;
	uint32_t gcalout;
	uint32_t reserved_14[11];
	uint32_t gstart;
	uint32_t gcfg;
	uint32_t gstatus;
	uint32_t girq_en;
	uint32_t gcmd_cfg;
	uint32_t gcmd_arg;
	uint32_t gcmd_dat;
	uint32_t gcmd_rsp0;
	uint32_t gcmd_rsp1;
	uint32_t gcmd_rsp2;
	uint32_t gcmd_rsp3;
	uint32_t reserved_6c;
	uint32_t gcurr_cfg;
	uint32_t gcurr_arg;
	uint32_t gcurr_dat;
	uint32_t gcurr_rsp;
	uint32_t gnext_cfg;
	uint32_t gnext_arg;
	uint32_t gnext_dat;
	uint32_t gnext_rsp;
	uint32_t grxd;
	uint32_t gtxd;
	uint32_t reserved_98[90];
	uint32_t gdesc[128];
	uint32_t gping[128];
	uint32_t gpong[128];
};

struct meson_mmc_platdata {
	struct mmc_config cfg;
	struct meson_mmc_regs *sd_emmc_reg;
	char *w_buf;
};

#endif
