/*
 * (C) Copyright 2016 Carlo Caione <carlo@caione.org>
 *
 * SPDX-License-Identifier:    GPL-2.0+
 */

#include <common.h>
#include <fdtdec.h>
#include <malloc.h>
#include <mmc.h>
#include <asm/io.h>
#include <asm/arch/sd_emmc.h>
#include <dm/device.h>

static void meson_mmc_config_clock(struct mmc *mmc,
				   struct meson_mmc_regs *meson_mmc_reg)
{
	uint32_t meson_mmc_clk = 0;
	unsigned int clk, clk_src, clk_div;

	if (mmc->clock > 12000000) {
		clk = SD_EMMC_CLKSRC_DIV2;
		clk_src = 1;
	} else {
		clk = SD_EMMC_CLKSRC_24M;
		clk_src = 0;
	}
	clk_div = clk / mmc->clock;

	if (mmc->clock < mmc->cfg->f_min)
		mmc->clock = mmc->cfg->f_min;
	if (mmc->clock > mmc->cfg->f_max)
		mmc->clock = mmc->cfg->f_max;

	/* Keep the clock always on */
	meson_mmc_clk |= (1 << CLK_ALWAYS_ON);

	/* 180 phase */
	meson_mmc_clk |= (2 << CLK_CO_PHASE);

	/* clock settings */
	meson_mmc_clk |= (clk_src << CLK_SRC);
	meson_mmc_clk |= (clk_div << CLK_DIV);

	writel(meson_mmc_clk, &meson_mmc_reg->gclock);
}

static void meson_mmc_set_ios(struct mmc *mmc)
{
	struct meson_mmc_platdata *pdata;
	struct meson_mmc_regs *meson_mmc_reg;
	unsigned int bus_width;
	uint32_t meson_mmc_cfg = 0;

	pdata = mmc->priv;
	meson_mmc_reg = pdata->sd_emmc_reg;

	meson_mmc_config_clock(mmc, meson_mmc_reg);

	meson_mmc_cfg = readl(&meson_mmc_reg->gcfg);

	if (mmc->bus_width == 1)
		bus_width = 0;
	else
		bus_width = mmc->bus_width / 4;

	/* 1-bit mode */
	meson_mmc_cfg &= ~CFG_BUS_WIDTH_MASK;
	meson_mmc_cfg |= (bus_width << CFG_BUS_WIDTH);

	/* 512 bytes block lenght */
	meson_mmc_cfg &= ~CFG_BL_LEN_MASK;
	meson_mmc_cfg |= (9 << CFG_BL_LEN);

	/* Response timeout 256 clk */
	meson_mmc_cfg &= ~CFG_RESP_TIMEOUT_MASK;
	meson_mmc_cfg |= (7 << CFG_RESP_TIMEOUT);

	/* Command-command gap 1024 clk */
	meson_mmc_cfg &= ~CFG_RC_CC_MASK;
	meson_mmc_cfg |= (4 << CFG_RC_CC);

	writel(meson_mmc_cfg, &meson_mmc_reg->gcfg);

	return;
}

static int meson_mmc_init(struct mmc *mmc)
{
	mmc_set_clock(mmc, 400000);

	return 0;
}

static void meson_mmc_setup_cmd(struct mmc *mmc, struct mmc_data *data,
				struct mmc_cmd *cmd,
				struct meson_mmc_regs *meson_mmc_reg)
{
	uint32_t meson_mmc_cmd = 0;

	meson_mmc_cmd = ((0x80 | cmd->cmdidx) << CMD_CFG_CMD_INDEX);

	if (cmd->resp_type & MMC_RSP_PRESENT) {
		if (cmd->resp_type & MMC_RSP_136)
			meson_mmc_cmd |= CMD_CFG_RESP_128;

		if (cmd->resp_type & MMC_RSP_BUSY)
			meson_mmc_cmd |= CMD_CFG_R1B;

		if (!(cmd->resp_type & MMC_RSP_CRC))
			meson_mmc_cmd |= CMD_CFG_RESP_NOCRC;
	} else {
		meson_mmc_cmd |= CMD_CFG_NO_RESP;
	}

	if (data) {
		meson_mmc_cmd |= CMD_CFG_DATA_IO;

		if (data->flags == MMC_DATA_WRITE)
			meson_mmc_cmd |= CMD_CFG_DATA_WR;

		if (data->blocks > 1) {
			meson_mmc_cmd |= CMD_CFG_BLOCK_MODE;
			meson_mmc_cmd |= data->blocks;
		} else {
			meson_mmc_cmd |= (data->blocksize & CMD_CFG_LENGTH_MASK);
		}
	}

	meson_mmc_cmd |= CMD_CFG_OWNER;
	meson_mmc_cmd |= CMD_CFG_END_OF_CHAIN;

	writel(meson_mmc_cmd, &meson_mmc_reg->gcmd_cfg);

	return;
}

static void meson_mmc_setup_addr(struct mmc *mmc, struct mmc_data *data,
				 struct meson_mmc_regs *meson_mmc_reg)
{
	struct meson_mmc_platdata *pdata;
	unsigned int data_size = 0;
	uint32_t meson_mmc_data_addr = 0;

	pdata = mmc->priv;

	if (data) {
		data_size = data->blocks * data->blocksize;

		if (data->flags == MMC_DATA_READ) {
			if (data_size < 0x200) {
				meson_mmc_data_addr = (ulong) meson_mmc_reg->gping;
				meson_mmc_data_addr |= ADDR_USE_PING_BUF;
			} else {
				invalidate_dcache_range((ulong) data->dest,
							(ulong) (data->dest + data_size));
				meson_mmc_data_addr = (ulong) data->dest;
			}
		}

		if (data->flags == MMC_DATA_WRITE) {
			pdata->w_buf = calloc(data_size, sizeof(char));
			memcpy(pdata->w_buf, data->src, data_size);
			flush_dcache_range((ulong) pdata->w_buf,
					   (ulong) (pdata->w_buf + data_size));
			meson_mmc_data_addr = (ulong) pdata->w_buf;
		}
	}

	writel(meson_mmc_data_addr, &meson_mmc_reg->gcmd_dat);

	return;
}

static void meson_mmc_read_response(struct mmc *mmc, struct mmc_data *data,
				    struct mmc_cmd *cmd,
				    struct meson_mmc_regs *meson_mmc_reg)
{
	unsigned int data_size = 0;

	if (data) {
		data_size = data->blocks * data->blocksize;
		if ((data_size < 0x200) && (data->flags == MMC_DATA_READ))
			memcpy(data->dest, (const void *)meson_mmc_reg->gping, data_size);
	}

	if (cmd->resp_type & MMC_RSP_136) {
		cmd->response[0] = readl(&meson_mmc_reg->gcmd_rsp3);
		cmd->response[1] = readl(&meson_mmc_reg->gcmd_rsp2);
		cmd->response[2] = readl(&meson_mmc_reg->gcmd_rsp1);
		cmd->response[3] = readl(&meson_mmc_reg->gcmd_rsp0);
	} else {
		cmd->response[0] = readl(&meson_mmc_reg->gcmd_rsp0);
	}
}

static int meson_mmc_send_cmd(struct mmc *mmc, struct mmc_cmd *cmd,
			      struct mmc_data *data)
{
	struct meson_mmc_platdata *pdata;
	struct meson_mmc_regs *meson_mmc_reg;
	uint32_t meson_mmc_irq = 0;
	int ret = 0;

	pdata = mmc->priv;
	meson_mmc_reg = pdata->sd_emmc_reg;

	meson_mmc_setup_cmd(mmc, data, cmd, meson_mmc_reg);
	meson_mmc_setup_addr(mmc, data, meson_mmc_reg);

	writel(SD_IRQ_ALL, &meson_mmc_reg->gstatus);
	writel(cmd->cmdarg, &meson_mmc_reg->gcmd_arg);

	while (1) {
		meson_mmc_irq = readl(&meson_mmc_reg->gstatus);
		if (meson_mmc_irq & STATUS_END_OF_CHAIN)
			break;
	}

	if (meson_mmc_irq & STATUS_RXD_ERR_MASK)
		ret |= SD_EMMC_RXD_ERROR;
	if (meson_mmc_irq & STATUS_TXD_ERR)
		ret |= SD_EMMC_TXD_ERROR;
	if (meson_mmc_irq & STATUS_DESC_ERR)
		ret |= SD_EMMC_DESC_ERROR;
	if (meson_mmc_irq & STATUS_RESP_ERR)
		ret |= SD_EMMC_RESP_CRC_ERROR;
	if (meson_mmc_irq & STATUS_RESP_TIMEOUT)
		ret |= SD_EMMC_RESP_TIMEOUT_ERROR;
	if (meson_mmc_irq & STATUS_DESC_TIMEOUT)
		ret |= SD_EMMC_DESC_TIMEOUT_ERROR;

	meson_mmc_read_response(mmc, data, cmd, meson_mmc_reg);

	if (data && data->flags == MMC_DATA_WRITE)
		free(pdata->w_buf);

	if (ret) {
		if (meson_mmc_irq & STATUS_RESP_TIMEOUT)
			return TIMEOUT;
		return ret;
	}

	return 0;
}

static const struct mmc_ops meson_mmc_ops = {
	.send_cmd	= meson_mmc_send_cmd,
	.set_ios	= meson_mmc_set_ios,
	.init		= meson_mmc_init,
};

static int meson_mmc_ofdata_to_platdata(struct udevice *dev)
{
	struct meson_mmc_platdata *pdata = dev->platdata;
	fdt_addr_t addr;

	addr = dev_get_addr(dev);
	if (addr == FDT_ADDR_T_NONE)
		return -EINVAL;

	pdata->sd_emmc_reg = (struct meson_mmc_regs *)addr;

	return 0;
}

static int meson_mmc_probe(struct udevice *dev)
{
	struct meson_mmc_platdata *pdata = dev->platdata;
	struct mmc_uclass_priv *upriv = dev_get_uclass_priv(dev);
	struct mmc *mmc;
	struct mmc_config *cfg;

	cfg = &pdata->cfg;
	cfg->ops = &meson_mmc_ops;

	cfg->voltages = MMC_VDD_33_34 | MMC_VDD_32_33 |
			MMC_VDD_31_32 | MMC_VDD_165_195;
	cfg->host_caps = MMC_MODE_8BIT | MMC_MODE_4BIT |
			 MMC_MODE_HS_52MHz | MMC_MODE_HS;
	cfg->f_min = 400000;
	cfg->f_max = 50000000;
	cfg->b_max = 256;

	mmc = mmc_create(cfg, pdata);
	if (!mmc)
		return -ENOMEM;

	upriv->mmc = mmc;
	return 0;
}

static const struct udevice_id meson_mmc_match[] = {
	{ .compatible = "amlogic,meson-mmc" },
	{ /* sentinel */ }
};

U_BOOT_DRIVER(meson_mmc) = {
	.name = "meson_mmc",
	.id = UCLASS_MMC,
	.of_match = meson_mmc_match,
	.probe = meson_mmc_probe,
	.ofdata_to_platdata = meson_mmc_ofdata_to_platdata,
	.platdata_auto_alloc_size = sizeof(struct meson_mmc_platdata),
};
