/*
 * mcp2515.c: driver for Microchip MCP2515 SPI CAN controller
 *
 * Copyright 2010 Andre B. Oliveira
 * Copyright 2013 Michael Luxen
 * Copyright 2013 Tower Technologies
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the version 2 of the GNU General Public License
 * as published by the Free Software Foundation
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * Example of mcp2515 platform spi_board_info definition:
 *
 * static struct mcp251x_platform_data mcp251x_info = {
 *	 .oscillator_frequency = 8000000,
 *	 .model = CAN_MCP251X_MCP2515,
 *	 .board_specific_setup = mcp251x_setup,
 *	 .power_enable = mcp251x_power_enable,
 *	 .transceiver_enable = NULL,
 * };
 *
 * static struct spi_board_info spi_board_info[] = {
 *	{
 *		.modalias = "mcp2515",
 *		.bus_num = 2,
 *		.chip_select = 0,
 *		.irq = IRQ_GPIO(28),
 *		.max_speed_hz = 10000000,
 *		.platform_data = &mcp251x_info,
 *	},
 * };
 */

/*
 * References: Microchip MCP2515 data sheet, DS21801E, 2007.
 */

#include <linux/dma-mapping.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/skbuff.h>
#include <linux/spi/spi.h>
#include <linux/spinlock.h>
#include <linux/can.h>
#include <linux/can/dev.h>
#include <linux/can/platform/mcp251x.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>


MODULE_DESCRIPTION("Driver for Microchip MCP2515 SPI CAN controller");
MODULE_AUTHOR("Andre B. Oliveira <anbadeol@gmail.com>");
MODULE_LICENSE("GPL");

/* Registers */
#define CANCTRL		0x0f
#define CANCTRL_REQOP_MASK		0xe0
#define CANCTRL_REQOP_CONF		0x80
#define CANCTRL_REQOP_LISTEN_ONLY	0x60
#define CANCTRL_REQOP_LOOPBACK		0x40
#define CANCTRL_REQOP_SLEEP		0x20
#define CANCTRL_REQOP_NORMAL		0x00
#define CANCTRL_ABAT			0x10
#define CANCTRL_OSM			0x08
#define CANCTRL_CLKEN			0x04

#define RXB0CTRL	0x60
#define RXB1CTRL	0x70

/* RXBnCTRL bits */
#define RXBCTRL_RXM1	0x40
#define RXBCTRL_RXM0	0x20
#define RXBCTRL_BUKT	0x04

/* RXBnSIDL bits */
#define RXBSIDL_SRR	0x10
#define RXBSIDL_IDE	0x08

/* RXBnDLC bits */
#define RXBDLC_RTR	0x40

/* CANINTF bits */
#define CANINTF_ERRIF	0x20
#define CANINTF_TX0IF	0x04
#define CANINTF_RX1IF	0x02
#define CANINTF_RX0IF	0x01

/* EFLG bits */
#define EFLG_RX1OVR	0x80
#define EFLG_RX0OVR	0x40

/* Network device private data */
struct mcp2515_priv {

	struct can_priv can;	/* must be first for all CAN network devices */
	struct spi_device *spi;	/* SPI device */
	struct net_device *net;

	struct mcp251x_platform_data *pdata;

	u8 canintf;		/* last read value of CANINTF register */
	u8 eflg;		/* last read value of EFLG register */

	struct sk_buff *skb;	/* skb to transmit or currently transmitting */

	spinlock_t lock;	/* Lock for the following flags: */
	unsigned busy:1;	/* set when pending async spi transaction */
	unsigned interrupt:1;	/* set when pending interrupt handling */
	unsigned transmit:1;	/* set when pending transmission */

	/* Message, transfer and buffers for one async spi transaction */
	struct spi_message message;
	struct spi_transfer transfer;
	u8 rx_buf[14] __attribute__((aligned(8)));
	u8 tx_buf[14] __attribute__((aligned(8)));
};

static struct can_bittiming_const mcp2515_bittiming_const = {
	.name = "mcp2515",
	.tseg1_min = 2,
	.tseg1_max = 16,
	.tseg2_min = 2,
	.tseg2_max = 8,
	.sjw_max = 4,
	.brp_min = 1,
	.brp_max = 64,
	.brp_inc = 1,
};

/*
 * SPI asynchronous completion callback functions.
 */
static void mcp2515_read_flags_complete(void *context);
static void mcp2515_read_rxb0_complete(void *context);
static void mcp2515_read_rxb1_complete(void *context);
static void mcp2515_clear_canintf_complete(void *context);
static void mcp2515_clear_eflg_complete(void *context);
static void mcp2515_load_txb0_complete(void *context);
static void mcp2515_rts_txb0_complete(void *context);

/*
 * Write VALUE to register at address ADDR.
 * Synchronous.
 */
static int mcp2515_write(struct spi_device *spi, unsigned addr, unsigned value)
{
	const u8 buf[3] __attribute__((aligned(8))) = {
		[0] = 2,	/* write instruction */
		[1] = addr,	/* address */
		[2] = value,	/* data */
	};

	return spi_write(spi, buf, sizeof(buf));
}

/*
 * Reset internal registers to default state and enter configuration mode.
 * Synchronous.
 */
static int mcp2515_reset(struct spi_device *spi)
{
	const u8 reset = 0xc0;

	return spi_write(spi, &reset, sizeof(reset));
}

/*
 * Set the bit timing configuration registers, the interrupt enable register
 * and the receive buffers control registers.
 * Synchronous.
 */
static int mcp2515_config(struct net_device *dev)
{
	struct mcp2515_priv *priv = netdev_priv(dev);
	struct spi_device *spi = priv->spi;
	struct can_bittiming *bt = &priv->can.bittiming;
	u8 buf[6] __attribute__((aligned(8)));
	int err;

	buf[0] = 2;	/* write instruction */
	buf[1] = 0x28;	/* address of CNF3 */

	/* CNF3 */
	buf[2] = bt->phase_seg2 - 1;

	/* CNF2 */
	buf[3] = (priv->can.ctrlmode & CAN_CTRLMODE_3_SAMPLES ? 0xc0 : 0x80) |
		(bt->phase_seg1 - 1) << 3 | (bt->prop_seg - 1);

	/* CNF1 */
	buf[4] = (bt->sjw - 1) << 6 | (bt->brp - 1);

	/* CANINTE */
	buf[5] = ~0;	/* enable all interrupts */

	netdev_info(dev, "CNF: 0x%02x 0x%02x 0x%02x\n",
		 buf[4], buf[3], buf[2]);

	err = spi_write(spi, buf, sizeof(buf));
	if (err)
		return err;

	err = mcp2515_write(spi, RXB0CTRL,
			    RXBCTRL_RXM1 | RXBCTRL_RXM0 | RXBCTRL_BUKT);
	if (err)
		return err;

	err = mcp2515_write(spi, RXB1CTRL, RXBCTRL_RXM1 | RXBCTRL_RXM0);
	if (err)
		return err;

	/* Finally, enter normal operation mode. */
	err = mcp2515_write(spi, CANCTRL, CANCTRL_REQOP_NORMAL | (priv->pdata->enable_clkout ? CANCTRL_CLKEN : 0));
	if (err)
		return err;

	netdev_info(priv->net, "CANCTRL: 0x%02x\n",
		CANCTRL_REQOP_NORMAL | (priv->pdata->enable_clkout ? CANCTRL_CLKEN : 0));

	return 0;
}

/*
 * Start an asynchronous SPI transaction.
 */
static void mcp2515_spi_async(struct net_device *dev)
{
	struct mcp2515_priv *priv = netdev_priv(dev);
	int err;

	err = spi_async(priv->spi, &priv->message);
	if (err)
		netdev_err(dev, "%s failed with err=%d\n", __func__, err);
}

/*
 * Read CANINTF and EFLG registers in one shot.
 * Asynchronous.
 */
static void mcp2515_read_flags(struct net_device *dev)
{
	struct mcp2515_priv *priv = netdev_priv(dev);
	u8 *buf = (u8 *)priv->transfer.tx_buf;

	buf[0] = 3;	/* read instruction */
	buf[1] = 0x2c;	/* address of CANINTF */
	buf[2] = 0;	/* CANINTF */
	buf[3] = 0;	/* EFLG */
	priv->transfer.len = 4;
	priv->message.complete = mcp2515_read_flags_complete;

	mcp2515_spi_async(dev);
}

/*
 * Read receive buffer 0 (instruction 0x90) or 1 (instruction 0x94).
 * Asynchronous.
 */
static void mcp2515_read_rxb(struct net_device *dev, u8 instruction,
			     void (*complete)(void *))
{
	struct mcp2515_priv *priv = netdev_priv(dev);
	u8 *buf = (u8 *)priv->transfer.tx_buf;

	memset(buf, 0, 14);
	buf[0] = instruction;
	priv->transfer.len = 14; /* instruction + id(4) + dlc + data(8) */
	priv->message.complete = complete;

	mcp2515_spi_async(dev);
}

/*
 * Read receive buffer 0.
 * Asynchronous.
 */
static void mcp2515_read_rxb0(struct net_device *dev)
{
	mcp2515_read_rxb(dev, 0x90, mcp2515_read_rxb0_complete);
}

/*
 * Read receive buffer 1.
 * Asynchronous.
 */
static void mcp2515_read_rxb1(struct net_device *dev)
{
	mcp2515_read_rxb(dev, 0x94, mcp2515_read_rxb1_complete);
}

/*
 * Clear CANINTF bits.
 * Asynchronous.
 */
static void mcp2515_clear_canintf(struct net_device *dev)
{
	struct mcp2515_priv *priv = netdev_priv(dev);
	u8 *buf = (u8 *)priv->transfer.tx_buf;

	buf[0] = 5;	/* bit modify instruction */
	buf[1] = 0x2c;	/* address of CANINTF */
	buf[2] = priv->canintf & ~(CANINTF_RX0IF | CANINTF_RX1IF); /* mask */
	buf[3] = 0;	/* data */
	priv->transfer.len = 4;
	priv->message.complete = mcp2515_clear_canintf_complete;

	mcp2515_spi_async(dev);
}

/*
 * Clear EFLG bits.
 * Asynchronous.
 */
static void mcp2515_clear_eflg(struct net_device *dev)
{
	struct mcp2515_priv *priv = netdev_priv(dev);
	u8 *buf = (u8 *)priv->transfer.tx_buf;

	buf[0] = 5;		/* bit modify instruction */
	buf[1] = 0x2d;		/* address of EFLG */
	buf[2] = priv->eflg;	/* mask */
	buf[3] = 0;		/* data */
	priv->transfer.len = 4;
	priv->message.complete = mcp2515_clear_eflg_complete;

	mcp2515_spi_async(dev);
}

/*
 * Set the transmit buffer, starting at TXB0SIDH, for an skb.
 */
static int mcp2515_set_txbuf(u8 *buf, const struct sk_buff *skb)
{
	struct can_frame *frame = (struct can_frame *)skb->data;

	if (frame->can_id & CAN_EFF_FLAG) {
		buf[0] = frame->can_id >> 21;
		buf[1] = (frame->can_id >> 13 & 0xe0) | 8 |
			(frame->can_id >> 16 & 3);
		buf[2] = frame->can_id >> 8;
		buf[3] = frame->can_id;
	} else {
		buf[0] = frame->can_id >> 3;
		buf[1] = frame->can_id << 5;
		buf[2] = 0;
		buf[3] = 0;
	}

	if (frame->can_id & CAN_RTR_FLAG)
		buf[4] = frame->can_dlc | 0x40;
	else
		buf[4] = frame->can_dlc;

	memcpy(buf + 5, frame->data, frame->can_dlc);

	return 5 + frame->can_dlc;
}

/*
 * Send the "load transmit buffer 0" SPI message.
 * Asynchronous.
 */
static void mcp2515_load_txb0(struct sk_buff *skb, struct net_device *dev)
{
	struct mcp2515_priv *priv = netdev_priv(dev);
	u8 *buf = (u8 *)priv->transfer.tx_buf;

	buf[0] = 0x40;	/* load txb0 instruction */
	priv->transfer.len = mcp2515_set_txbuf(buf + 1, skb) + 1;
	priv->message.complete = mcp2515_load_txb0_complete;

	mcp2515_spi_async(dev);
}

/*
 * Send the "request to send transmit buffer 0" SPI message.
 * Asynchronous.
 */
static void mcp2515_rts_txb0(struct net_device *dev)
{
	struct mcp2515_priv *priv = netdev_priv(dev);
	u8 *buf = (u8 *)priv->transfer.tx_buf;

	buf[0] = 0x81;	/* request to send txb0 instruction */
	priv->transfer.len = 1;
	priv->message.complete = mcp2515_rts_txb0_complete;

	mcp2515_spi_async(dev);
}

/*
 * Called when the "read CANINTF and EFLG registers" SPI message completes.
 */
static void mcp2515_read_flags_complete(void *context)
{
	struct net_device *dev = context;
	struct mcp2515_priv *priv = netdev_priv(dev);
	u8 *buf = priv->transfer.rx_buf;
	unsigned canintf;
	unsigned long flags;

	priv->canintf = canintf = buf[2];
	priv->eflg = buf[3];

	if (canintf & CANINTF_RX0IF)
		mcp2515_read_rxb0(dev);
	else if (canintf & CANINTF_RX1IF)
		mcp2515_read_rxb1(dev);
	else if (canintf)
		mcp2515_clear_canintf(dev);
	else {
		spin_lock_irqsave(&priv->lock, flags);
		if (priv->transmit) {
			priv->transmit = 0;
			spin_unlock_irqrestore(&priv->lock, flags);
			mcp2515_load_txb0(priv->skb, dev);
		} else if (priv->interrupt) {
			priv->interrupt = 0;
			spin_unlock_irqrestore(&priv->lock, flags);
			mcp2515_read_flags(dev);
		} else {
			priv->busy = 0;
			spin_unlock_irqrestore(&priv->lock, flags);
		}
	}
}

/*
 * Called when one of the "read receive buffer i" SPI message completes.
 */
static void mcp2515_read_rxb_complete(void *context)
{
	struct net_device *dev = context;
	struct mcp2515_priv *priv = netdev_priv(dev);
	struct sk_buff *skb;
	struct can_frame *frame;
	u8 *buf = priv->transfer.rx_buf;

	skb = alloc_can_skb(dev, &frame);
	if (!skb) {
		dev->stats.rx_dropped++;
		return;
	}

	if (buf[2] & RXBSIDL_IDE) {
		frame->can_id = buf[1] << 21 | (buf[2] & 0xe0) << 13 |
			(buf[2] & 3) << 16 | buf[3] << 8 | buf[4] |
			 CAN_EFF_FLAG;
		if (buf[5] & RXBDLC_RTR)
			frame->can_id |= CAN_RTR_FLAG;
	} else {
		frame->can_id = buf[1] << 3 | buf[2] >> 5;
		if (buf[2] & RXBSIDL_SRR)
			frame->can_id |= CAN_RTR_FLAG;
	}

	frame->can_dlc = get_can_dlc(buf[5] & 0xf);

	memcpy(frame->data, buf + 6, frame->can_dlc);

	dev->stats.rx_packets++;
	dev->stats.rx_bytes += frame->can_dlc;

	netif_rx(skb);
}

/*
 * Transmit a frame if transmission pending, else read and process flags.
 */
static void mcp2515_transmit_or_read_flags(struct net_device *dev)
{
	struct mcp2515_priv *priv = netdev_priv(dev);
	unsigned long flags;

	spin_lock_irqsave(&priv->lock, flags);
	if (priv->transmit) {
		priv->transmit = 0;
		spin_unlock_irqrestore(&priv->lock, flags);
		mcp2515_load_txb0(priv->skb, dev);
	} else {
		spin_unlock_irqrestore(&priv->lock, flags);
		mcp2515_read_flags(dev);
	}
}

/*
 * Called when the "read receive buffer 0" SPI message completes.
 */
static void mcp2515_read_rxb0_complete(void *context)
{
	struct net_device *dev = context;
	struct mcp2515_priv *priv = netdev_priv(dev);

	mcp2515_read_rxb_complete(context);

	if (priv->canintf & CANINTF_RX1IF)
		mcp2515_read_rxb1(dev);
	else
		mcp2515_transmit_or_read_flags(dev);
}

/*
 * Called when the "read receive buffer 1" SPI message completes.
 */
static void mcp2515_read_rxb1_complete(void *context)
{
	struct net_device *dev = context;

	mcp2515_read_rxb_complete(context);

	mcp2515_transmit_or_read_flags(dev);
}

/*
 * Called when the "clear CANINTF bits" SPI message completes.
 */
static void mcp2515_clear_canintf_complete(void *context)
{
	struct net_device *dev = context;
	struct mcp2515_priv *priv = netdev_priv(dev);

	if (priv->canintf & CANINTF_TX0IF) {
		struct sk_buff *skb = priv->skb;
		if (skb) {
			struct can_frame *f = (struct can_frame *)skb->data;
			dev->stats.tx_bytes += f->can_dlc;
			dev->stats.tx_packets++;
			can_put_echo_skb(skb, dev, 0);
			can_get_echo_skb(dev, 0);
		}
		priv->skb = NULL;
		netif_wake_queue(dev);
	}

	if (priv->eflg)
		mcp2515_clear_eflg(dev);
	else
		mcp2515_read_flags(dev);
}

/*
 * Called when the "clear EFLG bits" SPI message completes.
 */
static void mcp2515_clear_eflg_complete(void *context)
{
	struct net_device *dev = context;
	struct mcp2515_priv *priv = netdev_priv(dev);

	/*
	 * The receive flow chart (figure 4-3) of the data sheet (DS21801E)
	 * says that, if RXB0CTRL.BUKT is set (our case), the overflow
	 * flag that is set is EFLG.RX1OVR, when in fact it is EFLG.RX0OVR
	 * that is set.  To be safe, we test for any one of them.
	 */
	if (priv->eflg & (EFLG_RX0OVR | EFLG_RX1OVR))
		dev->stats.rx_over_errors++;

	mcp2515_read_flags(dev);
}

/*
 * Called when the "load transmit buffer 0" SPI message completes.
 */
static void mcp2515_load_txb0_complete(void *context)
{
	struct net_device *dev = context;

	mcp2515_rts_txb0(dev);
}

/*
 * Called when the "request to send transmit buffer 0" SPI message completes.
 */
static void mcp2515_rts_txb0_complete(void *context)
{
	struct net_device *dev = context;

	mcp2515_read_flags(dev);
}

/*
 * Interrupt handler.
 */
static irqreturn_t mcp2515_interrupt(int irq, void *dev_id)
{
	struct net_device *dev = dev_id;
	struct mcp2515_priv *priv = netdev_priv(dev);

	spin_lock(&priv->lock);
	if (priv->busy) {
		priv->interrupt = 1;
		spin_unlock(&priv->lock);
		return IRQ_HANDLED;
	}
	priv->busy = 1;
	spin_unlock(&priv->lock);

	mcp2515_read_flags(dev);

	return IRQ_HANDLED;
}

/*
 * Transmit a frame.
 */
static netdev_tx_t mcp2515_start_xmit(struct sk_buff *skb,
				      struct net_device *dev)
{
	struct mcp2515_priv *priv = netdev_priv(dev);
	unsigned long flags;

	if (can_dropped_invalid_skb(dev, skb))
		return NETDEV_TX_OK;

	netif_stop_queue(dev);
	priv->skb = skb;

	spin_lock_irqsave(&priv->lock, flags);
	if (priv->busy) {
		priv->transmit = 1;
		spin_unlock_irqrestore(&priv->lock, flags);
		return NETDEV_TX_OK;
	}
	priv->busy = 1;
	spin_unlock_irqrestore(&priv->lock, flags);

	mcp2515_load_txb0(skb, dev);

	return NETDEV_TX_OK;
}

/*
 * Called when the network device transitions to the up state.
 */
static int mcp2515_open(struct net_device *dev)
{
	struct mcp2515_priv *priv = netdev_priv(dev);
	struct spi_device *spi = priv->spi;
	int err;

	err = mcp2515_reset(spi);
	if (err)
		return err;

	err = open_candev(dev);
	if (err)
		return err;

	err = request_irq(spi->irq, mcp2515_interrupt,
			  IRQF_TRIGGER_FALLING, dev->name, dev);
	if (err)
		goto err1;

	err = mcp2515_config(dev);
	if (err)
		goto err2;

	netif_wake_queue(dev);

	return 0;

err2:	mcp2515_reset(spi);
	free_irq(spi->irq, dev);
err1:	close_candev(dev);
	return err;
}

/*
 * Called when the network device transitions to the down state.
 */
static int mcp2515_stop(struct net_device *dev)
{
	struct mcp2515_priv *priv = netdev_priv(dev);
	struct spi_device *spi = priv->spi;

	mcp2515_reset(spi);
	close_candev(dev);
	free_irq(spi->irq, dev);

	return 0;
}

/*
 * Set up SPI messages.
 */
static void mcp2515_setup_spi_messages(struct net_device *dev)
{
	struct mcp2515_priv *priv = netdev_priv(dev);
	struct device *device;
	void *buf;
	dma_addr_t dma;

	spi_message_init(&priv->message);
	priv->message.context = dev;

	/* FIXME */
	device = &priv->spi->dev;
	device->coherent_dma_mask = 0xffffffff;

	buf = dma_alloc_coherent(device, 32, &dma, GFP_KERNEL);
	if (buf) {
		priv->transfer.tx_buf = buf;
		priv->transfer.rx_buf = buf + 16;
		priv->transfer.tx_dma = dma;
		priv->transfer.rx_dma = dma + 16;
		priv->message.is_dma_mapped = 1;
	} else {
		priv->transfer.tx_buf = priv->tx_buf;
		priv->transfer.rx_buf = priv->rx_buf;
	}

	spi_message_add_tail(&priv->transfer, &priv->message);
}

static int mcp2515_set_mode(struct net_device *dev, enum can_mode mode)
{
	return 0;
}

/*
 * Network device operations.
 */
static const struct net_device_ops mcp2515_netdev_ops = {
	.ndo_open = mcp2515_open,
	.ndo_stop = mcp2515_stop,
	.ndo_start_xmit = mcp2515_start_xmit,
};

static struct mcp251x_platform_data *
mcp251x_platform_data_dt_get(struct device *dev)
{
	struct device_node *np = dev->of_node;
	struct mcp251x_platform_data *pd;

	u32 tmp;
	int err, gpio;

	dev_dbg(dev, "reading device tree\n");

	if (!np) {
		dev_err(dev, "no device tree node defined\n");
		return NULL;
	}

	pd = devm_kzalloc(dev, sizeof(struct mcp251x_platform_data), GFP_KERNEL);
	if (!pd) {
		dev_err(dev, "cannot allocate platform data memory\n");
		return NULL;
	}

	of_property_read_u32(np, "mcp251x,oscillator-frequency", &tmp);
	pd->oscillator_frequency = tmp;

	pd->stay_awake = of_property_read_bool(np, "mcp251x,stay-awake");
	pd->enable_clkout = of_property_read_bool(np, "mcp251x,enable-clkout");

	gpio = of_get_named_gpio(np, "mcp251x,irq-gpios", 0);
	if (gpio_is_valid(gpio)) {
		err = devm_gpio_request_one(dev, gpio, GPIOF_DIR_IN, "mcp251x-irq");
		if (err) {
			dev_err(dev, "cannot request gpio %d\n", gpio);
			return NULL;
		}
	} else {
			dev_warn(dev, "mcp251x,irq-gpios missing or invalid\n");
	}

	return pd;
}

/*
 * Binds this driver to the spi device.
 */
static int mcp2515_probe(struct spi_device *spi)
{
	struct net_device *dev;
	struct mcp2515_priv *priv;
	struct mcp251x_platform_data *pdata = spi->dev.platform_data;
	struct pinctrl *pinctrl;
	int err;

	dev_dbg(&spi->dev, "probing\n");

	if (!pdata && IS_ENABLED(CONFIG_OF))
		pdata = mcp251x_platform_data_dt_get(&spi->dev);

	if (!pdata) {
		/* Platform data is required for osc freq */
		dev_err(&spi->dev, "missing platform data and/or device tree\n");
		return -ENODEV;
	}

	dev_info(&spi->dev, "mode %d, cs %d, irq %d, awake %d, clkout %d, oscillator freq %ld\n",
		spi->mode, spi->chip_select, spi->irq, pdata->stay_awake, pdata->enable_clkout, pdata->oscillator_frequency);

	pinctrl = devm_pinctrl_get_select_default(&spi->dev);
	if (IS_ERR(pinctrl))
		dev_warn(&spi->dev, "pinctrl pins are not configured from the driver");

	err = mcp2515_reset(spi);
	if (err) {
		dev_err(&spi->dev, "cannot reset the mcp2515\n");
		return err;
	}

	dev = alloc_candev(sizeof(struct mcp2515_priv), 1);
	if (!dev) {
		dev_err(&spi->dev, "alloc_candev failed\n");
		return -ENOMEM;
	}

	dev_set_drvdata(&spi->dev, dev);
	SET_NETDEV_DEV(dev, &spi->dev);

	dev->netdev_ops = &mcp2515_netdev_ops;
	dev->flags |= IFF_ECHO;

	priv = netdev_priv(dev);
	priv->can.bittiming_const = &mcp2515_bittiming_const;
	priv->can.do_set_mode = mcp2515_set_mode;
	priv->can.clock.freq = pdata->oscillator_frequency / 2;
	priv->spi = spi;
	priv->pdata = pdata;
	priv->net = dev;

	spin_lock_init(&priv->lock);

	mcp2515_setup_spi_messages(dev);

	err = register_candev(dev);
	if (err) {
		netdev_info(priv->net, "probed\n");
		free_candev(dev);
		return err;
	}

	return 0;
}

static int mcp2515_remove(struct spi_device *spi)
{
	struct net_device *dev = dev_get_drvdata(&spi->dev);

	unregister_candev(dev);
	dev_set_drvdata(&spi->dev, NULL);
	free_candev(dev);

	return 0;
}

static struct spi_driver mcp2515_driver = {

	.driver = {
		.name = "mcp2515",
		.bus = &spi_bus_type,
		.owner = THIS_MODULE,
	},

	.probe = mcp2515_probe,
	.remove = mcp2515_remove,
};

static int __init mcp2515_init(void)
{
	return spi_register_driver(&mcp2515_driver);
}

static void __exit mcp2515_exit(void)
{
	spi_unregister_driver(&mcp2515_driver);
}

module_init(mcp2515_init);
module_exit(mcp2515_exit);

MODULE_DESCRIPTION("Driver for Microchip MCP2515 SPI CAN controller");
MODULE_AUTHOR("Andre B. Oliveira <anbadeol@gmail.com>");
MODULE_LICENSE("GPL v2");
