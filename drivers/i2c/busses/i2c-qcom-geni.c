// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2017-2018, The Linux Foundation. All rights reserved.

#include <linux/acpi.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/dmaengine.h>
#include <linux/qcom-geni-se.h>
#include <linux/msm_gpi.h>
#include <linux/spinlock.h>

#define SE_I2C_TX_TRANS_LEN		0x26c
#define SE_I2C_RX_TRANS_LEN		0x270
#define SE_I2C_SCL_COUNTERS		0x278

#define SE_I2C_ERR  (M_CMD_OVERRUN_EN | M_ILLEGAL_CMD_EN | M_CMD_FAILURE_EN |\
			M_GP_IRQ_1_EN | M_GP_IRQ_3_EN | M_GP_IRQ_4_EN)
#define SE_I2C_ABORT		BIT(1)

/* M_CMD OP codes for I2C */
#define I2C_WRITE		0x1
#define I2C_READ		0x2
#define I2C_WRITE_READ		0x3
#define I2C_ADDR_ONLY		0x4
#define I2C_BUS_CLEAR		0x6
#define I2C_STOP_ON_BUS		0x7
/* M_CMD params for I2C */
#define PRE_CMD_DELAY		BIT(0)
#define TIMESTAMP_BEFORE	BIT(1)
#define STOP_STRETCH		BIT(2)
#define TIMESTAMP_AFTER		BIT(3)
#define POST_COMMAND_DELAY	BIT(4)
#define IGNORE_ADD_NACK		BIT(6)
#define READ_FINISHED_WITH_ACK	BIT(7)
#define BYPASS_ADDR_PHASE	BIT(8)
#define SLV_ADDR_MSK		GENMASK(15, 9)
#define SLV_ADDR_SHFT		9
/* I2C SCL COUNTER fields */
#define HIGH_COUNTER_MSK	GENMASK(29, 20)
#define HIGH_COUNTER_SHFT	20
#define LOW_COUNTER_MSK		GENMASK(19, 10)
#define LOW_COUNTER_SHFT	10
#define CYCLE_COUNTER_MSK	GENMASK(9, 0)

#define I2C_PACK_EN		(BIT(0) | BIT(1))

enum geni_i2c_err_code {
	GP_IRQ0,
	NACK,
	GP_IRQ2,
	BUS_PROTO,
	ARB_LOST,
	GP_IRQ5,
	GENI_OVERRUN,
	GENI_ILLEGAL_CMD,
	GENI_ABORT_DONE,
	GENI_TIMEOUT,
};

#define DM_I2C_CB_ERR		((BIT(NACK) | BIT(BUS_PROTO) | BIT(ARB_LOST)) \
									<< 5)

#define I2C_AUTO_SUSPEND_DELAY	250
#define KHZ(freq)		(1000 * freq)
#define PACKING_BYTES_PW	4

#define ABORT_TIMEOUT		HZ
#define XFER_TIMEOUT		HZ
#define RST_TIMEOUT		HZ

enum i2c_se_mode {
	UNINITIALIZED,
	FIFO_SE_DMA,
	GSI_ONLY,
};

struct geni_i2c_dev {
	struct geni_se se;
	u32 tx_wm;
	int irq;
	int err;
	struct i2c_adapter adap;
	struct completion done;
	struct i2c_msg *cur;
	int cur_wr;
	int cur_rd;
	spinlock_t lock;
	u32 clk_freq_out;
	const struct geni_i2c_clk_fld *clk_fld;
	int suspended;
	struct dma_chan *tx_c;
	struct dma_chan *rx_c;
	struct msm_gpi_tre lock_t;
	struct msm_gpi_tre unlock_t;
	struct msm_gpi_tre cfg0_t;
	struct msm_gpi_tre go_t;
	struct msm_gpi_tre tx_t;
	struct msm_gpi_tre rx_t;
	dma_addr_t tx_ph;
	dma_addr_t rx_ph;
	struct msm_gpi_ctrl tx_ev;
	struct msm_gpi_ctrl rx_ev;
	struct scatterlist tx_sg[5]; /* lock, cfg0, go, TX, unlock */
	struct scatterlist rx_sg;
	int cfg_sent;
	struct dma_async_tx_descriptor *tx_desc;
	struct dma_async_tx_descriptor *rx_desc;
	struct msm_gpi_dma_async_tx_cb_param tx_cb;
	struct msm_gpi_dma_async_tx_cb_param rx_cb;
	enum i2c_se_mode se_mode;
	bool cmd_done;
	bool is_shared;
};

struct geni_i2c_err_log {
	int err;
	const char *msg;
};

static const struct geni_i2c_err_log gi2c_log[] = {
	[GP_IRQ0] = {-EIO, "Unknown I2C err GP_IRQ0"},
	[NACK] = {-ENXIO, "NACK: slv unresponsive, check its power/reset-ln"},
	[GP_IRQ2] = {-EIO, "Unknown I2C err GP IRQ2"},
	[BUS_PROTO] = {-EPROTO, "Bus proto err, noisy/unepxected start/stop"},
	[ARB_LOST] = {-EAGAIN, "Bus arbitration lost, clock line undriveable"},
	[GP_IRQ5] = {-EIO, "Unknown I2C err GP IRQ5"},
	[GENI_OVERRUN] = {-EIO, "Cmd overrun, check GENI cmd-state machine"},
	[GENI_ILLEGAL_CMD] = {-EIO, "Illegal cmd, check GENI cmd-state machine"},
	[GENI_ABORT_DONE] = {-ETIMEDOUT, "Abort after timeout successful"},
	[GENI_TIMEOUT] = {-ETIMEDOUT, "I2C TXN timed out"},
};

struct geni_i2c_clk_fld {
	u32	clk_freq_out;
	u8	clk_div;
	u8	t_high_cnt;
	u8	t_low_cnt;
	u8	t_cycle_cnt;
};

/*
 * Hardware uses the underlying formula to calculate time periods of
 * SCL clock cycle. Firmware uses some additional cycles excluded from the
 * below formula and it is confirmed that the time periods are within
 * specification limits.
 *
 * time of high period of SCL: t_high = (t_high_cnt * clk_div) / source_clock
 * time of low period of SCL: t_low = (t_low_cnt * clk_div) / source_clock
 * time of full period of SCL: t_cycle = (t_cycle_cnt * clk_div) / source_clock
 * clk_freq_out = t / t_cycle
 * source_clock = 19.2 MHz
 */
static const struct geni_i2c_clk_fld geni_i2c_clk_map[] = {
	{KHZ(100), 7, 10, 11, 26},
	{KHZ(400), 2,  5, 12, 24},
	{KHZ(1000), 1, 3,  9, 18},
};

static int geni_i2c_clk_map_idx(struct geni_i2c_dev *gi2c)
{
	int i;
	const struct geni_i2c_clk_fld *itr = geni_i2c_clk_map;

	for (i = 0; i < ARRAY_SIZE(geni_i2c_clk_map); i++, itr++) {
		if (itr->clk_freq_out == gi2c->clk_freq_out) {
			gi2c->clk_fld = itr;
			return 0;
		}
	}
	return -EINVAL;
}

static void qcom_geni_i2c_conf(struct geni_i2c_dev *gi2c)
{
	const struct geni_i2c_clk_fld *itr = gi2c->clk_fld;
	u32 val;

	writel_relaxed(0, gi2c->se.base + SE_GENI_CLK_SEL);

	val = (itr->clk_div << CLK_DIV_SHFT) | SER_CLK_EN;
	writel_relaxed(val, gi2c->se.base + GENI_SER_M_CLK_CFG);

	val = itr->t_high_cnt << HIGH_COUNTER_SHFT;
	val |= itr->t_low_cnt << LOW_COUNTER_SHFT;
	val |= itr->t_cycle_cnt;
	writel_relaxed(val, gi2c->se.base + SE_I2C_SCL_COUNTERS);
}

static void geni_i2c_err_misc(struct geni_i2c_dev *gi2c)
{
	u32 m_cmd = readl_relaxed(gi2c->se.base + SE_GENI_M_CMD0);
	u32 m_stat = readl_relaxed(gi2c->se.base + SE_GENI_M_IRQ_STATUS);
	u32 geni_s = readl_relaxed(gi2c->se.base + SE_GENI_STATUS);
	u32 geni_ios = readl_relaxed(gi2c->se.base + SE_GENI_IOS);
	u32 dma = readl_relaxed(gi2c->se.base + SE_GENI_DMA_MODE_EN);
	u32 rx_st, tx_st;

	if (dma) {
		rx_st = readl_relaxed(gi2c->se.base + SE_DMA_RX_IRQ_STAT);
		tx_st = readl_relaxed(gi2c->se.base + SE_DMA_TX_IRQ_STAT);
	} else {
		rx_st = readl_relaxed(gi2c->se.base + SE_GENI_RX_FIFO_STATUS);
		tx_st = readl_relaxed(gi2c->se.base + SE_GENI_TX_FIFO_STATUS);
	}
	dev_dbg(gi2c->se.dev, "DMA:%d tx_stat:0x%x, rx_stat:0x%x, irq-stat:0x%x\n",
		dma, tx_st, rx_st, m_stat);
	dev_dbg(gi2c->se.dev, "m_cmd:0x%x, geni_status:0x%x, geni_ios:0x%x\n",
		m_cmd, geni_s, geni_ios);
}

static void geni_i2c_err(struct geni_i2c_dev *gi2c, int err)
{
	if (!gi2c->err)
		gi2c->err = gi2c_log[err].err;
	if (gi2c->cur)
		dev_dbg(gi2c->se.dev, "len:%d, slv-addr:0x%x, RD/WR:%d\n",
			gi2c->cur->len, gi2c->cur->addr, gi2c->cur->flags);

	if (err != NACK && err != GENI_ABORT_DONE) {
		dev_err(gi2c->se.dev, "%s\n", gi2c_log[err].msg);
		geni_i2c_err_misc(gi2c);
	}
}

static irqreturn_t geni_i2c_irq(int irq, void *dev)
{
	struct geni_i2c_dev *gi2c = dev;
	void __iomem *base = gi2c->se.base;
	int j, p;
	u32 m_stat;
	u32 rx_st;
	u32 dm_tx_st;
	u32 dm_rx_st;
	u32 dma;
	u32 val;
	struct i2c_msg *cur;
	unsigned long flags;

	spin_lock_irqsave(&gi2c->lock, flags);
	m_stat = readl_relaxed(base + SE_GENI_M_IRQ_STATUS);
	rx_st = readl_relaxed(base + SE_GENI_RX_FIFO_STATUS);
	dm_tx_st = readl_relaxed(base + SE_DMA_TX_IRQ_STAT);
	dm_rx_st = readl_relaxed(base + SE_DMA_RX_IRQ_STAT);
	dma = readl_relaxed(base + SE_GENI_DMA_MODE_EN);
	cur = gi2c->cur;

	if (!cur ||
	    m_stat & (M_CMD_FAILURE_EN | M_CMD_ABORT_EN) ||
	    dm_rx_st & (DM_I2C_CB_ERR)) {
		if (m_stat & M_GP_IRQ_1_EN)
			geni_i2c_err(gi2c, NACK);
		if (m_stat & M_GP_IRQ_3_EN)
			geni_i2c_err(gi2c, BUS_PROTO);
		if (m_stat & M_GP_IRQ_4_EN)
			geni_i2c_err(gi2c, ARB_LOST);
		if (m_stat & M_CMD_OVERRUN_EN)
			geni_i2c_err(gi2c, GENI_OVERRUN);
		if (m_stat & M_ILLEGAL_CMD_EN)
			geni_i2c_err(gi2c, GENI_ILLEGAL_CMD);
		if (m_stat & M_CMD_ABORT_EN)
			geni_i2c_err(gi2c, GENI_ABORT_DONE);
		if (m_stat & M_GP_IRQ_0_EN)
			geni_i2c_err(gi2c, GP_IRQ0);

		/* Disable the TX Watermark interrupt to stop TX */
		if (!dma)
			writel_relaxed(0, base + SE_GENI_TX_WATERMARK_REG);
	} else if (dma) {
		dev_dbg(gi2c->se.dev, "i2c dma tx:0x%x, dma rx:0x%x\n",
			dm_tx_st, dm_rx_st);
	} else if (cur->flags & I2C_M_RD &&
		   m_stat & (M_RX_FIFO_WATERMARK_EN | M_RX_FIFO_LAST_EN)) {
		u32 rxcnt = rx_st & RX_FIFO_WC_MSK;

		for (j = 0; j < rxcnt; j++) {
			p = 0;
			val = readl_relaxed(base + SE_GENI_RX_FIFOn);
			while (gi2c->cur_rd < cur->len && p < sizeof(val)) {
				cur->buf[gi2c->cur_rd++] = val & 0xff;
				val >>= 8;
				p++;
			}
			if (gi2c->cur_rd == cur->len)
				break;
		}
	} else if (!(cur->flags & I2C_M_RD) &&
		   m_stat & M_TX_FIFO_WATERMARK_EN) {
		for (j = 0; j < gi2c->tx_wm; j++) {
			u32 temp;

			val = 0;
			p = 0;
			while (gi2c->cur_wr < cur->len && p < sizeof(val)) {
				temp = cur->buf[gi2c->cur_wr++];
				val |= temp << (p * 8);
				p++;
			}
			writel_relaxed(val, base + SE_GENI_TX_FIFOn);
			/* TX Complete, Disable the TX Watermark interrupt */
			if (gi2c->cur_wr == cur->len) {
				writel_relaxed(0, base + SE_GENI_TX_WATERMARK_REG);
				break;
			}
		}
	}

	if (m_stat)
		writel_relaxed(m_stat, base + SE_GENI_M_IRQ_CLEAR);

	if (dma && dm_tx_st)
		writel_relaxed(dm_tx_st, base + SE_DMA_TX_IRQ_CLR);
	if (dma && dm_rx_st)
		writel_relaxed(dm_rx_st, base + SE_DMA_RX_IRQ_CLR);

	/* if this is err with done-bit not set, handle that through timeout. */
	if (m_stat & M_CMD_DONE_EN || m_stat & M_CMD_ABORT_EN ||
	    dm_tx_st & TX_DMA_DONE || dm_tx_st & TX_RESET_DONE ||
	    dm_rx_st & RX_DMA_DONE || dm_rx_st & RX_RESET_DONE)
		complete(&gi2c->done);

	spin_unlock_irqrestore(&gi2c->lock, flags);

	return IRQ_HANDLED;
}

static void geni_i2c_abort_xfer(struct geni_i2c_dev *gi2c)
{
	u32 val;
	unsigned long time_left = ABORT_TIMEOUT;
	unsigned long flags;

	spin_lock_irqsave(&gi2c->lock, flags);
	geni_i2c_err(gi2c, GENI_TIMEOUT);
	gi2c->cur = NULL;
	geni_se_abort_m_cmd(&gi2c->se);
	spin_unlock_irqrestore(&gi2c->lock, flags);
	do {
		time_left = wait_for_completion_timeout(&gi2c->done, time_left);
		val = readl_relaxed(gi2c->se.base + SE_GENI_M_IRQ_STATUS);
	} while (!(val & M_CMD_ABORT_EN) && time_left);

	if (!(val & M_CMD_ABORT_EN))
		dev_err(gi2c->se.dev, "Timeout abort_m_cmd\n");
}

static void geni_i2c_rx_fsm_rst(struct geni_i2c_dev *gi2c)
{
	u32 val;
	unsigned long time_left = RST_TIMEOUT;

	writel_relaxed(1, gi2c->se.base + SE_DMA_RX_FSM_RST);
	do {
		time_left = wait_for_completion_timeout(&gi2c->done, time_left);
		val = readl_relaxed(gi2c->se.base + SE_DMA_RX_IRQ_STAT);
	} while (!(val & RX_RESET_DONE) && time_left);

	if (!(val & RX_RESET_DONE))
		dev_err(gi2c->se.dev, "Timeout resetting RX_FSM\n");
}

static void geni_i2c_tx_fsm_rst(struct geni_i2c_dev *gi2c)
{
	u32 val;
	unsigned long time_left = RST_TIMEOUT;

	writel_relaxed(1, gi2c->se.base + SE_DMA_TX_FSM_RST);
	do {
		time_left = wait_for_completion_timeout(&gi2c->done, time_left);
		val = readl_relaxed(gi2c->se.base + SE_DMA_TX_IRQ_STAT);
	} while (!(val & TX_RESET_DONE) && time_left);

	if (!(val & TX_RESET_DONE))
		dev_err(gi2c->se.dev, "Timeout resetting TX_FSM\n");
}

static int geni_i2c_rx_one_msg(struct geni_i2c_dev *gi2c, struct i2c_msg *msg,
				u32 m_param)
{
	dma_addr_t rx_dma;
	unsigned long time_left;
	void *dma_buf = NULL;
	struct geni_se *se = &gi2c->se;
	size_t len = msg->len;

	if (!of_machine_is_compatible("lenovo,yoga-c630"))
		dma_buf = i2c_get_dma_safe_msg_buf(msg, 32);

	if (dma_buf)
		geni_se_select_mode(se, GENI_SE_DMA);
	else
		geni_se_select_mode(se, GENI_SE_FIFO);

	writel_relaxed(len, se->base + SE_I2C_RX_TRANS_LEN);

	if (dma_buf && geni_se_rx_dma_prep(se, dma_buf, len, &rx_dma)) {
		geni_se_select_mode(se, GENI_SE_FIFO);
		i2c_put_dma_safe_msg_buf(dma_buf, msg, false);
		dma_buf = NULL;
	}

	geni_se_setup_m_cmd(se, I2C_READ, m_param);

	time_left = wait_for_completion_timeout(&gi2c->done, XFER_TIMEOUT);
	if (!time_left)
		geni_i2c_abort_xfer(gi2c);

	gi2c->cur_rd = 0;
	if (dma_buf) {
		if (gi2c->err)
			geni_i2c_rx_fsm_rst(gi2c);
		geni_se_rx_dma_unprep(se, rx_dma, len);
		i2c_put_dma_safe_msg_buf(dma_buf, msg, !gi2c->err);
	}

	return gi2c->err;
}

static int geni_i2c_tx_one_msg(struct geni_i2c_dev *gi2c, struct i2c_msg *msg,
				u32 m_param)
{
	dma_addr_t tx_dma;
	unsigned long time_left;
	void *dma_buf = NULL;
	struct geni_se *se = &gi2c->se;
	size_t len = msg->len;

	if (!of_machine_is_compatible("lenovo,yoga-c630"))
		dma_buf = i2c_get_dma_safe_msg_buf(msg, 32);

	if (dma_buf)
		geni_se_select_mode(se, GENI_SE_DMA);
	else
		geni_se_select_mode(se, GENI_SE_FIFO);

	writel_relaxed(len, se->base + SE_I2C_TX_TRANS_LEN);

	if (dma_buf && geni_se_tx_dma_prep(se, dma_buf, len, &tx_dma)) {
		geni_se_select_mode(se, GENI_SE_FIFO);
		i2c_put_dma_safe_msg_buf(dma_buf, msg, false);
		dma_buf = NULL;
	}

	geni_se_setup_m_cmd(se, I2C_WRITE, m_param);

	if (!dma_buf) /* Get FIFO IRQ */
		writel_relaxed(1, se->base + SE_GENI_TX_WATERMARK_REG);

	time_left = wait_for_completion_timeout(&gi2c->done, XFER_TIMEOUT);
	if (!time_left)
		geni_i2c_abort_xfer(gi2c);

	gi2c->cur_wr = 0;
	if (dma_buf) {
		if (gi2c->err)
			geni_i2c_tx_fsm_rst(gi2c);
		geni_se_tx_dma_unprep(se, tx_dma, len);
		i2c_put_dma_safe_msg_buf(dma_buf, msg, !gi2c->err);
	}

	return gi2c->err;
}

static void gi2c_ev_cb(struct dma_chan *ch, struct msm_gpi_cb const *cb_str,
		       void *ptr)
{
	struct geni_i2c_dev *gi2c = ptr;
	u32 m_stat = cb_str->status;

	switch (cb_str->cb_event) {
		case MSM_GPI_QUP_ERROR:
		case MSM_GPI_QUP_SW_ERROR:
		case MSM_GPI_QUP_MAX_EVENT:
			/* fall through to stall impacted channel */
		case MSM_GPI_QUP_CH_ERROR:
		case MSM_GPI_QUP_PENDING_EVENT:
		case MSM_GPI_QUP_EOT_DESC_MISMATCH:
			break;
		case MSM_GPI_QUP_NOTIFY:
			if (m_stat & M_GP_IRQ_1_EN)
				geni_i2c_err(gi2c, NACK);
			if (m_stat & M_GP_IRQ_3_EN)
				geni_i2c_err(gi2c, BUS_PROTO);
			if (m_stat & M_GP_IRQ_4_EN)
				geni_i2c_err(gi2c, ARB_LOST);
			complete(&gi2c->done);
		default:
			break;
			break;
	}

	if (cb_str->cb_event != MSM_GPI_QUP_NOTIFY)
		dev_err(gi2c->se.dev,
			"GSI QN err:0x%x, status:0x%x, err:%d slv_addr: 0x%x R/W: %d\n",
			cb_str->error_log.error_code, m_stat,
			cb_str->cb_event, gi2c->cur->addr, gi2c->cur->flags);
}

static void gi2c_gsi_cb_err(struct msm_gpi_dma_async_tx_cb_param *cb,
								char *xfer)
{
	struct geni_i2c_dev *gi2c = cb->userdata;

	if (cb->status & DM_I2C_CB_ERR) {
		dev_dbg(gi2c->se.dev, "%s TCE Unexpected Err, stat:0x%x\n",
				xfer, cb->status);
		if (cb->status & M_GP_IRQ_1_EN)
			geni_i2c_err(gi2c, NACK);
		if (cb->status & M_GP_IRQ_3_EN)
			geni_i2c_err(gi2c, BUS_PROTO);
		if (cb->status & M_GP_IRQ_4_EN)
			geni_i2c_err(gi2c, ARB_LOST);
	}
}

static void gi2c_gsi_tx_cb(void *ptr)
{
	struct msm_gpi_dma_async_tx_cb_param *tx_cb = ptr;
	struct geni_i2c_dev *gi2c = tx_cb->userdata;

	if (tx_cb->completion_code == MSM_GPI_TCE_EOB) {
		complete(&gi2c->done);
	} else if (!(gi2c->cur->flags & I2C_M_RD)) {
		gi2c_gsi_cb_err(tx_cb, "TX");
		complete(&gi2c->done);
	}
}

static void gi2c_gsi_rx_cb(void *ptr)
{
	struct msm_gpi_dma_async_tx_cb_param *rx_cb = ptr;
	struct geni_i2c_dev *gi2c = rx_cb->userdata;

	if (gi2c->cur->flags & I2C_M_RD) {
		gi2c_gsi_cb_err(rx_cb, "RX");
		complete(&gi2c->done);
	}
}

static int geni_i2c_gsi_xfer(struct i2c_adapter *adap, struct i2c_msg msgs[],
			     int num)
{
	struct geni_i2c_dev *gi2c = i2c_get_adapdata(adap);
	int i, ret = 0, timeout = 0;

	if (!gi2c->tx_c) {
		gi2c->tx_c = dma_request_slave_channel(gi2c->se.dev, "tx");
		if (!gi2c->tx_c) {
			dev_err(gi2c->se.dev,
				    "tx dma req slv chan ret :%d\n", ret);
			ret = -EIO;
			goto geni_i2c_gsi_xfer_out;
		}
		gi2c->tx_ev.init.callback = gi2c_ev_cb;
		gi2c->tx_ev.init.cb_param = gi2c;
		gi2c->tx_ev.cmd = MSM_GPI_INIT;
		gi2c->tx_c->private = &gi2c->tx_ev;
		ret = dmaengine_slave_config(gi2c->tx_c, NULL);
		if (ret) {
			dev_err(gi2c->se.dev,
				    "tx dma slave config ret :%d\n", ret);
			goto geni_i2c_gsi_xfer_out;
		}
	}

	if (!gi2c->rx_c) {
		gi2c->rx_c = dma_request_slave_channel(gi2c->se.dev, "rx");
		if (!gi2c->rx_c) {
			dev_err(gi2c->se.dev,
				    "rx dma req slv chan ret :%d\n", ret);
			ret = -EIO;
			goto geni_i2c_gsi_xfer_out;
		}
		gi2c->rx_ev.init.cb_param = gi2c;
		gi2c->rx_ev.init.callback = gi2c_ev_cb;
		gi2c->rx_ev.cmd = MSM_GPI_INIT;
		gi2c->rx_c->private = &gi2c->rx_ev;
		ret = dmaengine_slave_config(gi2c->rx_c, NULL);
		if (ret) {
			dev_err(gi2c->se.dev,
				    "rx dma slave config ret :%d\n", ret);
			goto geni_i2c_gsi_xfer_out;
		}
	}

	if (gi2c->is_shared) {
		struct msm_gpi_tre *lock_t = &gi2c->lock_t;
		struct msm_gpi_tre *unlock_t = &gi2c->unlock_t;

		/* lock */
		lock_t->dword[0] = MSM_GPI_LOCK_TRE_DWORD0;
		lock_t->dword[1] = MSM_GPI_LOCK_TRE_DWORD1;
		lock_t->dword[2] = MSM_GPI_LOCK_TRE_DWORD2;
		lock_t->dword[3] = MSM_GPI_LOCK_TRE_DWORD3(0, 0, 0, 0, 1);

		/* unlock */
		unlock_t->dword[0] = MSM_GPI_UNLOCK_TRE_DWORD0;
		unlock_t->dword[1] = MSM_GPI_UNLOCK_TRE_DWORD1;
		unlock_t->dword[2] = MSM_GPI_UNLOCK_TRE_DWORD2;
		unlock_t->dword[3] = MSM_GPI_UNLOCK_TRE_DWORD3(0, 0, 0, 1, 0);
	}

	if (!gi2c->cfg_sent) {
		const struct geni_i2c_clk_fld *itr = gi2c->clk_fld;

		struct msm_gpi_tre *cfg0 = &gi2c->cfg0_t;

		/* config0 */
		cfg0->dword[0] = MSM_GPI_I2C_CONFIG0_TRE_DWORD0(I2C_PACK_EN,
								itr->t_cycle_cnt,
								itr->t_high_cnt,
								itr->t_low_cnt);
		cfg0->dword[1] = MSM_GPI_I2C_CONFIG0_TRE_DWORD1(0, 0);
		cfg0->dword[2] = MSM_GPI_I2C_CONFIG0_TRE_DWORD2(0,
								itr->clk_div);
		cfg0->dword[3] = MSM_GPI_I2C_CONFIG0_TRE_DWORD3(0, 0, 0, 0, 1);

		gi2c->tx_cb.userdata = gi2c;
		gi2c->rx_cb.userdata = gi2c;
	}

	for (i = 0; i < num; i++) {
		u8 op = (msgs[i].flags & I2C_M_RD) ? 2 : 1;
		int segs = 3 - op;
		int index = 0;
		u8 *dma_buf = NULL;
		int stretch = (i < (num - 1));
		dma_cookie_t tx_cookie, rx_cookie;
		struct msm_gpi_tre *go_t = &gi2c->go_t;
		struct device *rx_dev = gi2c->se.wrapper->dev;
		struct device *tx_dev = gi2c->se.wrapper->dev;

		gi2c->cur = &msgs[i];

		dma_buf = i2c_get_dma_safe_msg_buf(&msgs[i], 1);
		if (!dma_buf) {
			ret = -ENOMEM;
			goto geni_i2c_gsi_xfer_out;
		}

		if (!gi2c->cfg_sent)
			segs++;
		if (gi2c->is_shared && (i == 0 || i == num-1)) {
			segs++;
			if (num == 1)
				segs++;
			sg_init_table(gi2c->tx_sg, segs);
			if (i == 0)
				sg_set_buf(&gi2c->tx_sg[index++], &gi2c->lock_t,
					sizeof(gi2c->lock_t));
		} else {
			sg_init_table(gi2c->tx_sg, segs);
		}

		if (!gi2c->cfg_sent) {
			sg_set_buf(&gi2c->tx_sg[index++], &gi2c->cfg0_t,
						sizeof(gi2c->cfg0_t));
			gi2c->cfg_sent = 1;
		}

		go_t->dword[0] = MSM_GPI_I2C_GO_TRE_DWORD0((stretch << 2),
							   msgs[i].addr, op);
		go_t->dword[1] = MSM_GPI_I2C_GO_TRE_DWORD1;

		if (msgs[i].flags & I2C_M_RD) {
			go_t->dword[2] = MSM_GPI_I2C_GO_TRE_DWORD2(msgs[i].len);
			go_t->dword[3] = MSM_GPI_I2C_GO_TRE_DWORD3(1, 0, 0, 0,
									0);
		} else {
			go_t->dword[2] = MSM_GPI_I2C_GO_TRE_DWORD2(0);
			go_t->dword[3] = MSM_GPI_I2C_GO_TRE_DWORD3(0, 0, 0, 0,
									1);
		}

		sg_set_buf(&gi2c->tx_sg[index++], &gi2c->go_t,
						  sizeof(gi2c->go_t));

		if (msgs[i].flags & I2C_M_RD) {
			dev_dbg(gi2c->se.dev,
				"msg[%d].len:%d R\n", i, gi2c->cur->len);
			sg_init_table(&gi2c->rx_sg, 1);
			gi2c->rx_ph = dma_map_single(rx_dev, dma_buf,
						msgs[i].len,
						DMA_FROM_DEVICE);
			if (dma_mapping_error(rx_dev, gi2c->rx_ph)) {
				dev_err(gi2c->se.dev,
					    "dma_map_single for rx failed :%d\n",
					    ret);
				i2c_put_dma_safe_msg_buf(dma_buf, &msgs[i],
								false);
				goto geni_i2c_gsi_xfer_out;

			}

			gi2c->rx_t.dword[0] =
				MSM_GPI_DMA_W_BUFFER_TRE_DWORD0(gi2c->rx_ph);
			gi2c->rx_t.dword[1] =
				MSM_GPI_DMA_W_BUFFER_TRE_DWORD1(gi2c->rx_ph);
			gi2c->rx_t.dword[2] =
				MSM_GPI_DMA_W_BUFFER_TRE_DWORD2(msgs[i].len);
			gi2c->rx_t.dword[3] =
				MSM_GPI_DMA_W_BUFFER_TRE_DWORD3(0, 0, 1, 0, 0);

			sg_set_buf(&gi2c->rx_sg, &gi2c->rx_t,
						 sizeof(gi2c->rx_t));
			gi2c->rx_desc = dmaengine_prep_slave_sg(gi2c->rx_c,
							&gi2c->rx_sg, 1,
							DMA_DEV_TO_MEM,
							(DMA_PREP_INTERRUPT |
							 DMA_CTRL_ACK));
			if (!gi2c->rx_desc) {
				dev_err(gi2c->se.dev,
					    "prep_slave_sg for rx failed\n");
				gi2c->err = -ENOMEM;
				goto geni_i2c_err_prep_sg;
			}
			gi2c->rx_desc->callback = gi2c_gsi_rx_cb;
			gi2c->rx_desc->callback_param = &gi2c->rx_cb;

			/* Issue RX */
			rx_cookie = dmaengine_submit(gi2c->rx_desc);
			dma_async_issue_pending(gi2c->rx_c);
		} else {
			dev_dbg(gi2c->se.dev,
				"msg[%d].len:%d W\n", i, gi2c->cur->len);
			gi2c->tx_ph = dma_map_single(tx_dev, dma_buf,
						msgs[i].len,
						DMA_TO_DEVICE);
			if (dma_mapping_error(tx_dev, gi2c->tx_ph)) {
				dev_err(gi2c->se.dev,
					    "dma_map_single for tx failed :%d\n",
					    ret);
				i2c_put_dma_safe_msg_buf(dma_buf, &msgs[i],
								false);
				goto geni_i2c_gsi_xfer_out;

			}

			gi2c->tx_t.dword[0] =
				MSM_GPI_DMA_W_BUFFER_TRE_DWORD0(gi2c->tx_ph);
			gi2c->tx_t.dword[1] =
				MSM_GPI_DMA_W_BUFFER_TRE_DWORD1(gi2c->tx_ph);
			gi2c->tx_t.dword[2] =
				MSM_GPI_DMA_W_BUFFER_TRE_DWORD2(msgs[i].len);
			if (gi2c->is_shared && i == num-1)
				gi2c->tx_t.dword[3] =
				MSM_GPI_DMA_W_BUFFER_TRE_DWORD3(0, 0, 1, 0, 1);
			else
				gi2c->tx_t.dword[3] =
				MSM_GPI_DMA_W_BUFFER_TRE_DWORD3(0, 0, 1, 0, 0);

			sg_set_buf(&gi2c->tx_sg[index++], &gi2c->tx_t,
							  sizeof(gi2c->tx_t));
		}

		if (gi2c->is_shared && i == num-1) {
			sg_set_buf(&gi2c->tx_sg[index++],
				&gi2c->unlock_t, sizeof(gi2c->unlock_t));
		}

		gi2c->tx_desc = dmaengine_prep_slave_sg(gi2c->tx_c, gi2c->tx_sg,
						segs, DMA_MEM_TO_DEV,
						(DMA_PREP_INTERRUPT |
						 DMA_CTRL_ACK));
		if (!gi2c->tx_desc) {
			dev_err(gi2c->se.dev,
				    "prep_slave_sg for tx failed\n");
			gi2c->err = -ENOMEM;
			goto geni_i2c_err_prep_sg;
		}
		gi2c->tx_desc->callback = gi2c_gsi_tx_cb;
		gi2c->tx_desc->callback_param = &gi2c->tx_cb;

		/* Issue TX */
		tx_cookie = dmaengine_submit(gi2c->tx_desc);
		dma_async_issue_pending(gi2c->tx_c);

		timeout = wait_for_completion_timeout(&gi2c->done,
						XFER_TIMEOUT);
		if (!timeout) {
			dev_err(gi2c->se.dev,
				"I2C timeout gsi flags:%d addr:0x%x\n",
				gi2c->cur->flags,
				gi2c->cur->addr);
			gi2c->err = -ETIMEDOUT;
		}
geni_i2c_err_prep_sg:
		if (gi2c->err) {
			dmaengine_terminate_all(gi2c->tx_c);
			gi2c->cfg_sent = 0;
		}
		if (msgs[i].flags & I2C_M_RD)
			dma_unmap_single(rx_dev, gi2c->rx_ph,
				msgs[i].len, DMA_FROM_DEVICE);
		else
			dma_unmap_single(tx_dev, gi2c->tx_ph,
				msgs[i].len, DMA_TO_DEVICE);
		i2c_put_dma_safe_msg_buf(dma_buf, &msgs[i], !gi2c->err);
		if (gi2c->err)
			goto geni_i2c_gsi_xfer_out;
	}

geni_i2c_gsi_xfer_out:
	if (!ret && gi2c->err)
		ret = gi2c->err;
	return ret;
}

static int geni_i2c_xfer(struct i2c_adapter *adap,
			 struct i2c_msg msgs[],
			 int num)
{
	struct geni_i2c_dev *gi2c = i2c_get_adapdata(adap);
	int i, ret;

	gi2c->err = 0;
	reinit_completion(&gi2c->done);
	ret = pm_runtime_get_sync(gi2c->se.dev);
	if (ret < 0) {
		dev_err(gi2c->se.dev, "error turning SE resources:%d\n", ret);
		pm_runtime_put_noidle(gi2c->se.dev);
		/* Set device in suspended since resume failed */
		pm_runtime_set_suspended(gi2c->se.dev);
		return ret;
	}

	qcom_geni_i2c_conf(gi2c);

	if (gi2c->se_mode == GSI_ONLY) {
		ret = geni_i2c_gsi_xfer(adap, msgs, num);
		goto geni_i2c_txn_ret;
	} else {
		/* Don't set shared flag in non-GSI mode */
		gi2c->is_shared = false;
	}

	for (i = 0; i < num; i++) {
		u32 m_param = i < (num - 1) ? STOP_STRETCH : 0;

		m_param |= ((msgs[i].addr << SLV_ADDR_SHFT) & SLV_ADDR_MSK);

		gi2c->cur = &msgs[i];
		if (msgs[i].flags & I2C_M_RD)
			ret = geni_i2c_rx_one_msg(gi2c, &msgs[i], m_param);
		else
			ret = geni_i2c_tx_one_msg(gi2c, &msgs[i], m_param);

		if (ret)
			break;
	}
geni_i2c_txn_ret:
	if (ret == 0)
		ret = num;

	pm_runtime_mark_last_busy(gi2c->se.dev);
	pm_runtime_put_autosuspend(gi2c->se.dev);
	gi2c->cur = NULL;
	gi2c->err = 0;
	return ret;
}

static u32 geni_i2c_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | (I2C_FUNC_SMBUS_EMUL & ~I2C_FUNC_SMBUS_QUICK);
}

static const struct i2c_algorithm geni_i2c_algo = {
	.master_xfer	= geni_i2c_xfer,
	.functionality	= geni_i2c_func,
};

#ifdef CONFIG_ACPI
static const struct acpi_device_id geni_i2c_acpi_match[] = {
	{ "QCOM0220"},
	{ },
};
MODULE_DEVICE_TABLE(acpi, geni_i2c_acpi_match);
#endif

static int geni_i2c_probe(struct platform_device *pdev)
{
	struct geni_i2c_dev *gi2c;
	struct resource *res;
	u32 proto, tx_depth;
	int ret;
	struct device *dev = &pdev->dev;

	gi2c = devm_kzalloc(dev, sizeof(*gi2c), GFP_KERNEL);
	if (!gi2c)
		return -ENOMEM;

	gi2c->se.dev = dev;
	gi2c->se.wrapper = dev_get_drvdata(dev->parent);
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	gi2c->se.base = devm_ioremap_resource(dev, res);
	if (IS_ERR(gi2c->se.base))
		return PTR_ERR(gi2c->se.base);

	gi2c->se.clk = devm_clk_get(dev, "se");
	if (IS_ERR(gi2c->se.clk) && !has_acpi_companion(dev))
		return PTR_ERR(gi2c->se.clk);

	ret = device_property_read_u32(dev, "clock-frequency",
				       &gi2c->clk_freq_out);
	if (ret) {
		dev_info(dev, "Bus frequency not specified, default to 100kHz.\n");
		gi2c->clk_freq_out = KHZ(100);
	}

	if (has_acpi_companion(dev))
		ACPI_COMPANION_SET(&gi2c->adap.dev, ACPI_COMPANION(dev));

	gi2c->irq = platform_get_irq(pdev, 0);
	if (gi2c->irq < 0)
		return gi2c->irq;

	ret = geni_i2c_clk_map_idx(gi2c);
	if (ret) {
		dev_err(dev, "Invalid clk frequency %d Hz: %d\n",
			gi2c->clk_freq_out, ret);
		return ret;
	}

	gi2c->adap.algo = &geni_i2c_algo;
	init_completion(&gi2c->done);
	spin_lock_init(&gi2c->lock);
	platform_set_drvdata(pdev, gi2c);
	ret = devm_request_irq(dev, gi2c->irq, geni_i2c_irq, 0,
			       dev_name(dev), gi2c);
	if (ret) {
		dev_err(dev, "Request_irq failed:%d: err:%d\n",
			gi2c->irq, ret);
		return ret;
	}
	/* Disable the interrupt so that the system can enter low-power mode */
	disable_irq(gi2c->irq);
	i2c_set_adapdata(&gi2c->adap, gi2c);
	gi2c->adap.dev.parent = dev;
	gi2c->adap.dev.of_node = dev->of_node;
	strlcpy(gi2c->adap.name, "Geni-I2C", sizeof(gi2c->adap.name));

	ret = geni_se_resources_on(&gi2c->se);
	if (ret) {
		dev_err(dev, "Error turning on resources %d\n", ret);
		return ret;
	}
	proto = geni_se_read_proto(&gi2c->se);
	tx_depth = geni_se_get_tx_fifo_depth(&gi2c->se);
	if (proto != GENI_SE_I2C) {
		dev_err(dev, "Invalid proto %d\n", proto);
		geni_se_resources_off(&gi2c->se);
		return -ENXIO;
	}
	gi2c->tx_wm = tx_depth - 1;
	geni_se_init(&gi2c->se, gi2c->tx_wm, tx_depth);
	geni_se_config_packing(&gi2c->se, BITS_PER_BYTE, PACKING_BYTES_PW,
							true, true, true);
	ret = geni_se_resources_off(&gi2c->se);
	if (ret) {
		dev_err(dev, "Error turning off resources %d\n", ret);
		return ret;
	}

	dev_dbg(dev, "i2c fifo/se-dma mode. fifo depth:%d\n", tx_depth);

	gi2c->suspended = 1;
	pm_runtime_set_suspended(gi2c->se.dev);
	pm_runtime_set_autosuspend_delay(gi2c->se.dev, I2C_AUTO_SUSPEND_DELAY);
	pm_runtime_use_autosuspend(gi2c->se.dev);
	pm_runtime_enable(gi2c->se.dev);

	ret = i2c_add_adapter(&gi2c->adap);
	if (ret) {
		dev_err(dev, "Error adding i2c adapter %d\n", ret);
		pm_runtime_disable(gi2c->se.dev);
		return ret;
	}

	dev_dbg(dev, "Geni-I2C adaptor successfully added\n");

	return 0;
}

static int geni_i2c_remove(struct platform_device *pdev)
{
	struct geni_i2c_dev *gi2c = platform_get_drvdata(pdev);

	i2c_del_adapter(&gi2c->adap);
	pm_runtime_disable(gi2c->se.dev);
	return 0;
}

static int __maybe_unused geni_i2c_runtime_suspend(struct device *dev)
{
	int ret;
	struct geni_i2c_dev *gi2c = dev_get_drvdata(dev);

	if (gi2c->se_mode == FIFO_SE_DMA)
		disable_irq(gi2c->irq);
	ret = geni_se_resources_off(&gi2c->se);
	if (ret) {
		enable_irq(gi2c->irq);
		return ret;

	} else {
		gi2c->suspended = 1;
	}

	return 0;
}

static int __maybe_unused geni_i2c_runtime_resume(struct device *dev)
{
	int ret;
	struct geni_i2c_dev *gi2c = dev_get_drvdata(dev);

	ret = geni_se_resources_on(&gi2c->se);
	if (ret)
		return ret;
	if (gi2c->se_mode == UNINITIALIZED) {
		int proto = geni_se_read_proto(&gi2c->se);
		u32 se_mode;

		if (unlikely(proto != GENI_SE_I2C)) {
			dev_err(gi2c->se.dev, "Invalid proto %d\n", proto);
			geni_se_resources_off(&gi2c->se);
			return -ENXIO;
		}

		se_mode = readl_relaxed(gi2c->se.base + GENI_IF_DISABLE_RO) &
				FIFO_IF_DISABLE;
		if (se_mode) {
			gi2c->se_mode = GSI_ONLY;
			geni_se_select_mode(&gi2c->se, GENI_GPI_DMA);
			dev_dbg(gi2c->se.dev, "i2c GSI mode\n");
		} else {
			int gi2c_tx_depth = geni_se_get_tx_fifo_depth(&gi2c->se);

			gi2c->se_mode = FIFO_SE_DMA;
			gi2c->tx_wm = gi2c_tx_depth - 1;
			geni_se_init(&gi2c->se, gi2c->tx_wm, gi2c_tx_depth);
			geni_se_config_packing(&gi2c->se, BITS_PER_BYTE, PACKING_BYTES_PW,
							true, true, true);
			qcom_geni_i2c_conf(gi2c);
			dev_dbg(gi2c->se.dev,
				    "i2c fifo/se-dma mode. fifo depth:%d\n",
				    gi2c_tx_depth);
		}
		dev_dbg(gi2c->se.dev, "i2c-%d: %s\n",
			gi2c->adap.nr, dev_name(gi2c->se.dev));
	}

	if (gi2c->se_mode == FIFO_SE_DMA)
		enable_irq(gi2c->irq);

	gi2c->suspended = 0;
	return 0;
}

static int __maybe_unused geni_i2c_suspend_noirq(struct device *dev)
{
	struct geni_i2c_dev *gi2c = dev_get_drvdata(dev);

	if (!gi2c->suspended) {
		geni_i2c_runtime_suspend(dev);
		pm_runtime_disable(dev);
		pm_runtime_set_suspended(dev);
		pm_runtime_enable(dev);
	}
	return 0;
}

static const struct dev_pm_ops geni_i2c_pm_ops = {
	SET_NOIRQ_SYSTEM_SLEEP_PM_OPS(geni_i2c_suspend_noirq, NULL)
	SET_RUNTIME_PM_OPS(geni_i2c_runtime_suspend, geni_i2c_runtime_resume,
									NULL)
};

static const struct of_device_id geni_i2c_dt_match[] = {
	{ .compatible = "qcom,geni-i2c" },
	{}
};
MODULE_DEVICE_TABLE(of, geni_i2c_dt_match);

static struct platform_driver geni_i2c_driver = {
	.probe  = geni_i2c_probe,
	.remove = geni_i2c_remove,
	.driver = {
		.name = "geni_i2c",
		.pm = &geni_i2c_pm_ops,
		.of_match_table = geni_i2c_dt_match,
		.acpi_match_table = ACPI_PTR(geni_i2c_acpi_match),
	},
};

module_platform_driver(geni_i2c_driver);

MODULE_DESCRIPTION("I2C Controller Driver for GENI based QUP cores");
MODULE_LICENSE("GPL v2");
