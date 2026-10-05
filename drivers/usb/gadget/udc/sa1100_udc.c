// SPDX-License-Identifier: GPL-2.0-only
/*
 * SA-1110 on-chip full speed USB device controller (UDC).
 *
 * Copyright (C) 2026 Koen Kooi
 *
 * A PIO-only, three-endpoint driver for the fixed-function UDC integrated
 * into Intel/Marvell SA-1110 (and, per its shared register layout,
 * SA-1100). Unlike the PXA2xx/IXP4xx generation (pxa25x_udc.c, the
 * structural template for this file), this silicon implements exactly
 * three endpoints: ep0 control (an 8-byte FIFO shared between OUT and IN),
 * ep1 bulk-OUT and ep2 bulk-IN. There is no interrupt endpoint, no
 * software D+ pull-up (the 1.5 kohm D+ resistor is external and fixed, so
 * the host sees an attach whenever VBUS is present; UDCCR_UDD only
 * tristates UDC+/UDC-), and no DMA support in this driver yet (the
 * silicon has it: Ser0UDCTr/Ser0UDCRc in drivers/dma/sa11x0-dma.c).
 *
 * Reference material, cited by function below: the SA-1110 Developer's
 * Manual (278240-002) ss11.8 ("DM"), the handhelds.org SA-1100 USB stack
 * (arch/arm/mach-sa1100/usb/ in the 2.4-hh and 2.6-hh trees, "hh"), and
 * the WinCE usb driver of the h3870 ("WinCE"). Where they disagree, the
 * behaviour at least two of them share is what is implemented.
 *
 * Hardware properties that shape this driver:
 *
 * - A write to any UDC register or FIFO is not guaranteed to have taken
 *   effect by the time the next access happens, and the first read after
 *   a write returns stale data (DM ss11.8.2 note). Every register write
 *   is therefore retried until a second read-back shows its effect,
 *   bounded, with a message on the bound (hh UDC_write()/UDC_flip(),
 *   WinCE's identical write-until-read-back loops).
 *
 * - UDCCS0/1/2 mix write-1-to-clear status (SST, RPC, TPC), write-1
 *   strobes (SO, SSE) and read-only live status with, on UDCCS1/2, a
 *   plain read/write force-stall bit. A read-modify-write would ack any
 *   W1C bit that happened to be set, so these registers are only ever
 *   written with the exact bits intended (WinCE writes literal values);
 *   the FST hold bit of ep1/ep2 is carried along explicitly in every
 *   write so an ack never un-halts the endpoint.
 *
 * - An IN packet on ep2 is exactly UDCIMP + 1 bytes long (DM ss11.8.6);
 *   a short packet is sent by reprogramming UDCIMP for that packet (hh
 *   ep2_start(), WinCE's ep2 transmit path), and a zero-length packet
 *   cannot be sent at all.
 *
 * - The PIO-reachable FIFOs are 16 bytes (transmit) and 20 bytes
 *   (receive) deep (DM ss11.8.8.1, ss11.8.9.1), and the only interrupt
 *   for either endpoint is packet completion: there is no "FIFO needs
 *   service" interrupt, only the DMA request line. Without DMA a whole
 *   packet has to fit its FIFO, so both bulk endpoints are limited to
 *   16-byte packets -- exactly the bulk-IN wMaxPacketSize WinCE's own
 *   descriptors advertise for its PIO-driven ep2.
 *
 * Known silicon limits no driver can work around (SA-1110 specification
 * update 278259-015 errata 6 and 12, SA-1100 278105-025 errata 8 and 11):
 * the UDC can stall when another device shares its bus segment, and it
 * sporadically fails to NAK an IN token after an SOF with ep2 empty.
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

#include <linux/usb/ch9.h>
#include <linux/usb/composite.h>
#include <linux/usb/gadget.h>
#include <linux/usb/otg.h>
#include <linux/usb/phy.h>

/*
 * Register layout (DM ss11.8.14, Table 11-13). Offsets are relative to
 * the devicetree/platform "reg" base, Ser0UDCCR (0x80000000 physical).
 * Defined locally rather than taken from mach/SA-1100.h so the driver
 * builds under COMPILE_TEST, as pxa25x_udc.c does.
 */

#define UDCCR		0x00	/* Control */
#define UDCAR		0x04	/* Address */
#define UDCOMP		0x08	/* Output (ep1) Maximum Packet size - 1 */
#define UDCIMP		0x0c	/* Input (ep2) packet size - 1 */
#define UDCCS0		0x10	/* Control/status, ep0 */
#define UDCCS1		0x14	/* Control/status, ep1 (OUT) */
#define UDCCS2		0x18	/* Control/status, ep2 (IN) */
#define UDCD0		0x1c	/* Data FIFO, ep0 (byte-wide) */
#define UDCWC		0x20	/* Write Count, ep0's own FIFO byte counter */
#define UDCDR		0x28	/* Data FIFO, ep1/ep2 (byte-wide, shared) */
#define UDCSR		0x30	/* Status (interrupt sources, W1C) */

#define UDC_REG_WINDOW	0x34	/* Ser0UDCCR .. Ser0UDCSR inclusive */

/*
 * UDCCR bit 2 is reserved and must be written as zero (DM ss11.8.3.3);
 * a single SRM bit masks suspend and resume together (DM ss11.8.3.7).
 * mach/SA-1100.h's "UDCCR_RESIM ... per errata" has no support in either
 * specification update, and WinCE never sets bit 2.
 */
#define UDCCR_UDD	BIT(0)	/* UDC Disable */
#define UDCCR_UDA	BIT(1)	/* UDC Active (read-only) */
#define UDCCR_EIM	BIT(3)	/* ep0 interrupt mask */
#define UDCCR_RIM	BIT(4)	/* ep1 (receive) interrupt mask */
#define UDCCR_TIM	BIT(5)	/* ep2 (transmit) interrupt mask */
#define UDCCR_SRM	BIT(6)	/* Suspend/resume interrupt mask */
#define UDCCR_REM	BIT(7)	/* Reset interrupt mask */

/* Disabled: the reset value of the register (DM ss11.8.3 table). */
#define UDCCR_IDLE	(UDCCR_UDD | UDCCR_SRM)

#define UDCCS0_OPR	BIT(0)	/* Output Packet Ready (read) */
#define UDCCS0_IPR	BIT(1)	/* Input Packet Ready (write 1 to set) */
#define UDCCS0_SST	BIT(2)	/* Sent STall (W1C) */
#define UDCCS0_FST	BIT(3)	/* Force STall (write 1 to set) */
#define UDCCS0_DE	BIT(4)	/* Data End (write 1 to set) */
#define UDCCS0_SE	BIT(5)	/* Setup End (read) */
#define UDCCS0_SO	BIT(6)	/* Serviced Output packet ready (write) */
#define UDCCS0_SSE	BIT(7)	/* Serviced Setup End (write) */

#define UDCCS1_RFS	BIT(0)	/* Receive FIFO >=12 bytes service (read) */
#define UDCCS1_RPC	BIT(1)	/* Receive Packet Complete (W1C) */
#define UDCCS1_RPE	BIT(2)	/* Receive Packet Error (read) */
#define UDCCS1_SST	BIT(3)	/* Sent STall (W1C) */
#define UDCCS1_FST	BIT(4)	/* Force STall (read/write) */
#define UDCCS1_RNE	BIT(5)	/* Receive FIFO Not Empty (read) */

#define UDCCS2_TFS	BIT(0)	/* Transmit FIFO <=8 bytes service (read) */
#define UDCCS2_TPC	BIT(1)	/* Transmit Packet Complete (W1C) */
#define UDCCS2_TPE	BIT(2)	/* Transmit Packet Error (read) */
#define UDCCS2_TUR	BIT(3)	/* Transmit FIFO Under-Run (read) */
#define UDCCS2_SST	BIT(4)	/* Sent STall (W1C) */
#define UDCCS2_FST	BIT(5)	/* Force STall (read/write) */

#define UDCSR_EIR	BIT(0)	/* ep0 interrupt request */
#define UDCSR_RIR	BIT(1)	/* ep1 (receive) interrupt request */
#define UDCSR_TIR	BIT(2)	/* ep2 (transmit) interrupt request */
#define UDCSR_SUSIR	BIT(3)	/* Suspend interrupt request */
#define UDCSR_RESIR	BIT(4)	/* Resume interrupt request */
#define UDCSR_RSTIR	BIT(5)	/* Reset interrupt request */
#define UDCSR_ALL	(UDCSR_EIR | UDCSR_RIR | UDCSR_TIR | UDCSR_SUSIR | \
			 UDCSR_RESIR | UDCSR_RSTIR)

#define UDCAR_ADDR_MASK		0x7f

/* UDCOMP/UDCIMP hold (packet size - 1) in bits [7:0] */
#define UDC_MAXP_FIELD(size)	(((size) - 1) & 0xff)

#define EP0_FIFO_SIZE		8U
#define BULK_PIO_MAXPACKET	16U	/* see the file header comment */
#define RX_FIFO_DEPTH		20U	/* DM ss11.8.8.1 */

#define DRIVER_NAME	"sa1100_udc"
#define DRIVER_DESC	"SA-1110 USB Device Controller driver"

/*
 * Driver data structures
 */

enum ep0_state {
	EP0_IDLE,
	EP0_IN_DATA,
	EP0_OUT_DATA,
	EP0_STATUS,	/* no-data request: OPR still set, awaiting status */
};

struct sa1100_udc;
struct sa1100_request;

struct sa1100_ep {
	struct usb_ep		ep;
	struct sa1100_udc	*dev;
	struct list_head	queue;
	const char		*name;
	u8			bEndpointAddress;
	u8			bmAttributes;
	u8			cs_reg;		/* UDCCS1 or UDCCS2 offset */
	u8			mask_bit;	/* UDCCR_RIM or UDCCR_TIM */
	unsigned		stopped:1;
	unsigned		is_in:1;
	unsigned		halted:1;	/* FST currently requested */

	/* ep2 only: the packet currently loaded into the transmit FIFO */
	unsigned int in_len;
	struct sa1100_request	*in_req;	/* NULL once dequeued */
};

struct sa1100_request {
	struct usb_request	req;
	struct list_head	queue;
};

struct sa1100_udc {
	struct usb_gadget	gadget;
	struct usb_gadget_driver *driver;
	struct device		*dev;
	void __iomem		*regs;
	int			irq;
	spinlock_t		lock;	/* protects the state below and the registers */

	struct usb_phy		*transceiver;

	struct sa1100_ep	ep[3];		/* 0 = ep0, 1 = ep1out, 2 = ep2in */
	enum ep0_state		ep0state;
	struct usb_ctrlrequest	setup;

	u32			udccr;		/* shadow of the last value written */
	u8			address;	/* last SET_ADDRESS value */
	unsigned		vbus:1;
	unsigned		pullup:1;
	unsigned		active:1;	/* UDC currently enabled (UDD=0) */
	unsigned		got_irq:1;
};

static const char ep0name[] = "ep0";

/*
 * Low-level register access
 */

static inline u32 udc_readl(struct sa1100_udc *dev, unsigned int reg)
{
	return readl_relaxed(dev->regs + reg);
}

static inline void udc_writel(struct sa1100_udc *dev, unsigned int reg, u32 val)
{
	writel_relaxed(val, dev->regs + reg);
}

/* The first read after a write returns stale data (DM ss11.8.2 note). */
static u32 udc_readback(struct sa1100_udc *dev, unsigned int reg)
{
	udc_readl(dev, reg);
	return udc_readl(dev, reg);
}

#define UDC_WRITE_RETRIES	10000

/*
 * Write @val to a plain data/control register and re-write it until it
 * reads back. UDCCR's read-only UDA bit follows bus activity, so it is
 * excluded from the comparison there.
 */
static void udc_write_verify(struct sa1100_udc *dev, unsigned int reg, u32 val)
{
	u32 ignore = reg == UDCCR ? UDCCR_UDA : 0;
	unsigned int i = UDC_WRITE_RETRIES;

	do {
		udc_writel(dev, reg, val);
	} while (((udc_readback(dev, reg) ^ val) & ~ignore) && --i);

	if (!i)
		dev_err_ratelimited(dev->dev,
				    "erratum: write %#x to reg %#x did not stick (read %#x)\n",
			val, reg, udc_readl(dev, reg));
}

/* Acknowledge write-1-to-clear bits in UDCSR. */
static void udc_flip_verify(struct sa1100_udc *dev, unsigned int reg, u32 bits)
{
	unsigned int i = UDC_WRITE_RETRIES;

	do {
		udc_writel(dev, reg, bits);
	} while ((udc_readback(dev, reg) & bits) && --i);

	if (!i)
		dev_err_ratelimited(dev->dev,
				    "erratum: ack %#x of reg %#x did not clear (read %#x)\n",
			bits, reg, udc_readl(dev, reg));
}

static void udccr_update(struct sa1100_udc *dev, u32 set, u32 clear)
{
	dev->udccr = (dev->udccr | set) & ~clear;
	udc_write_verify(dev, UDCCR, dev->udccr);
}

/*
 * Write @bits to UDCCS0 and retry until the write is seen to have taken
 * effect. Each bit class has its own completion test:
 *  - SO (clears OPR): done once OPR is clear or, when DE was written
 *    with it, once DE reads set (WinCE's 0x50 loops);
 *  - IPR/DE: done once they read set, abandoned if the host aborted the
 *    transfer meanwhile (SST/SE set; hh set_ipr()/set_de());
 *  - SSE/SST: done once SE/SST read clear (WinCE);
 *  - FST: a single write, since the UDC clears it itself once the stall
 *    handshake is sent and a retry could stall the next transfer (hh
 *    set_cs_bits()).
 */
static void ep0_cs_write(struct sa1100_udc *dev, u32 bits)
{
	unsigned int i = UDC_WRITE_RETRIES;
	u32 set = bits & (UDCCS0_IPR | UDCCS0_DE);
	u32 cs;
	bool ok;

	do {
		udc_writel(dev, UDCCS0, bits);
		cs = udc_readback(dev, UDCCS0);

		if (bits & UDCCS0_SO)
			ok = !(cs & UDCCS0_OPR) ||
			     ((bits & UDCCS0_DE) && (cs & UDCCS0_DE));
		else if (set)
			ok = (cs & set) == set ||
			     (cs & (UDCCS0_SST | UDCCS0_SE));
		else if (bits & UDCCS0_SSE)
			ok = !(cs & UDCCS0_SE);
		else if (bits & UDCCS0_SST)
			ok = !(cs & UDCCS0_SST);
		else
			ok = true;
	} while (!ok && --i);

	if (!i)
		dev_err_ratelimited(dev->dev,
				    "erratum: UDCCS0 write %#x did not take (read %#x)\n",
			bits, cs);
}

/*
 * Write ep1/ep2's control/status register: @w1c are the status bits to
 * acknowledge, and the endpoint's FST hold bit is always written with
 * its intended value so that no acknowledge clears a halt (FST is plain
 * read/write, DM ss11.8.8.5/ss11.8.9.6).
 */
static void ep_cs_write(struct sa1100_ep *ep, u32 w1c)
{
	struct sa1100_udc *dev = ep->dev;
	u32 fst = ep->is_in ? UDCCS2_FST : UDCCS1_FST;
	u32 want = ep->halted ? fst : 0;
	unsigned int i = UDC_WRITE_RETRIES;
	u32 cs;

	do {
		udc_writel(dev, ep->cs_reg, w1c | want);
		cs = udc_readback(dev, ep->cs_reg);
	} while (((cs & w1c) || (cs & fst) != want) && --i);

	if (!i)
		dev_err_ratelimited(dev->dev,
				    "erratum: %s CS write %#x did not take (read %#x)\n",
			ep->ep.name, w1c | want, cs);
}

/*
 * The ep0 FIFO (UDCD0) has no ready flag; the only progress signal is
 * UDCWC, which changes as each byte is consumed or produced. Each byte
 * access is retried if the count does not move (hh udc_ep0_write_fifo()/
 * udc_ep0_read_fifo(), WinCE's ep0 FIFO loops), bounded per byte.
 */
#define EP0_BYTE_RETRIES	10
#define EP0_WC_POLLS		20	/* 1 us each */
#define EP0_SETUP_WAIT_US	1000

static bool ep0_wc_moved(struct sa1100_udc *dev, u32 wc)
{
	unsigned int i;

	for (i = 0; i < EP0_WC_POLLS; i++) {
		if ((udc_readl(dev, UDCWC) & 0xf) != wc)
			return true;
		udelay(1);
	}
	return false;
}

static int ep0_write_byte(struct sa1100_udc *dev, u8 val)
{
	u32 wc = udc_readl(dev, UDCWC) & 0xf;
	unsigned int i;

	for (i = 0; i < EP0_BYTE_RETRIES; i++) {
		udc_writel(dev, UDCD0, val);
		if (ep0_wc_moved(dev, wc))
			return 0;
	}

	dev_err_ratelimited(dev->dev, "ep0 FIFO write stuck\n");
	return -EIO;
}

static int ep0_read_byte(struct sa1100_udc *dev, u8 *val)
{
	u32 wc = udc_readl(dev, UDCWC) & 0xf;
	unsigned int i;

	for (i = 0; i < EP0_BYTE_RETRIES; i++) {
		*val = udc_readl(dev, UDCD0);
		if (ep0_wc_moved(dev, wc))
			return 0;
	}

	dev_err_ratelimited(dev->dev, "ep0 FIFO read stuck\n");
	return -EIO;
}

/*
 * Request queue plumbing
 */

static struct usb_request *sa1100_ep_alloc_request(struct usb_ep *_ep,
						   gfp_t gfp_flags)
{
	struct sa1100_request *req = kzalloc_obj(*req, gfp_flags);

	if (!req)
		return NULL;
	INIT_LIST_HEAD(&req->queue);
	return &req->req;
}

static void sa1100_ep_free_request(struct usb_ep *_ep, struct usb_request *_req)
{
	struct sa1100_request *req = container_of(_req, struct sa1100_request, req);

	WARN_ON(!list_empty(&req->queue));
	kfree(req);
}

/* caller holds dev->lock */
static void done(struct sa1100_ep *ep, struct sa1100_request *req, int status)
{
	unsigned int stopped = ep->stopped;

	list_del_init(&req->queue);
	if (ep->in_req == req)
		ep->in_req = NULL;

	if (likely(req->req.status == -EINPROGRESS))
		req->req.status = status;
	else
		status = req->req.status;

	ep->stopped = 1;
	spin_unlock(&ep->dev->lock);
	usb_gadget_giveback_request(&ep->ep, &req->req);
	spin_lock(&ep->dev->lock);
	ep->stopped = stopped;
}

/* caller holds dev->lock */
static void nuke(struct sa1100_ep *ep, int status)
{
	struct sa1100_request *req;

	while (!list_empty(&ep->queue)) {
		req = list_entry(ep->queue.next, struct sa1100_request, queue);
		done(ep, req, status);
	}
}

/*
 * ep1 (bulk OUT) / ep2 (bulk IN) PIO, over the shared UDCDR FIFO port.
 */

/*
 * hh re-loads UDCAR before every IN packet and whenever it reads back
 * wrong in the ep2 interrupt ("fighting stupid silicon bug", usb_send.c
 * ep2_start()/udc_ep2_int_hndlr()). Neither DM nor WinCE mention it;
 * only restoring on a mismatch keeps it harmless, and the warning makes
 * a real occurrence visible.
 */
static void udc_check_address(struct sa1100_udc *dev)
{
	u32 car = udc_readl(dev, UDCAR) & UDCAR_ADDR_MASK;

	if (car != dev->address) {
		dev_warn_ratelimited(dev->dev,
				     "UDCAR lost its address (%u, expected %u), restoring\n",
			car, dev->address);
		udc_writel(dev, UDCAR, dev->address);
	}
}

/*
 * Load one IN packet. TPC is acknowledged first -- the UDC NAKs IN tokens
 * while it is set -- then UDCIMP is set to this packet's length, then the
 * FIFO is filled (hh ep2_start(), WinCE's ep2 transmit path, same order).
 * The whole packet fits the 16-byte transmit FIFO, so it is written
 * without consulting TFS, which reads "8 bytes or fewer queued" and is
 * only a DMA request line (DM ss11.8.9.1).
 */
static void sa1100_write_fifo(struct sa1100_ep *ep, struct sa1100_request *req,
			      unsigned int n)
{
	struct sa1100_udc *dev = ep->dev;
	u8 *buf = req->req.buf + req->req.actual;
	unsigned int i;

	if (udc_readl(dev, UDCCS2) & UDCCS2_TPC)
		ep_cs_write(ep, UDCCS2_TPC);

	udc_write_verify(dev, UDCIMP, UDC_MAXP_FIELD(n));
	udc_check_address(dev);

	for (i = 0; i < n; i++)
		udc_writel(dev, UDCDR, buf[i]);

	ep->in_len = n;
	ep->in_req = req;
}

static void handle_bulk_in(struct sa1100_ep *ep)
{
	struct sa1100_udc *dev = ep->dev;
	struct sa1100_request *req;
	u32 cs = udc_readl(dev, UDCCS2);

	if (cs & UDCCS2_SST) {
		/* clearing SST flushes the transmit FIFO (DM ss11.8.9.5) */
		ep_cs_write(ep, UDCCS2_SST);
		ep->in_len = 0;
		ep->in_req = NULL;
	}

	if (ep->in_len) {
		unsigned int len = ep->in_len;

		if (!(cs & UDCCS2_TPC))
			return;		/* still waiting for the host */

		/* cleared first: done() may re-enter via a nested queue() */
		req = ep->in_req;
		ep->in_len = 0;
		ep->in_req = NULL;

		if (cs & (UDCCS2_TPE | UDCCS2_TUR)) {
			/*
			 * Not acknowledged by the host (TPE), or shortened
			 * with a corrupted CRC after an underrun (TUR): the
			 * host will ask for the same data again, so the
			 * request is not advanced and the packet is loaded
			 * afresh below (WinCE re-sends its saved copy).
			 */
			dev_dbg(dev->dev, "ep2 packet error %#x, resending\n",
				cs);
		} else if (req) {
			req->req.actual += len;
			if (req->req.actual == req->req.length)
				done(ep, req, 0);
		}
	}

	if (ep->halted)
		return;

	while (!ep->in_len && !list_empty(&ep->queue)) {
		unsigned int n;

		req = list_entry(ep->queue.next, struct sa1100_request, queue);
		n = min(req->req.length - req->req.actual, ep->ep.maxpacket);
		if (!n) {
			/* zero-length packets cannot be sent (DM ss11.8.6) */
			done(ep, req, 0);
			continue;
		}
		sa1100_write_fifo(ep, req, n);
		return;
	}
}

/* Discard a packet flagged bad by RPE; the host re-sends it. */
static void sa1100_drain_rx(struct sa1100_udc *dev)
{
	unsigned int i;

	for (i = 0; i < RX_FIFO_DEPTH &&
	     (udc_readl(dev, UDCCS1) & UDCCS1_RNE); i++)
		udc_readl(dev, UDCDR);
}

static void sa1100_read_fifo(struct sa1100_ep *ep, struct sa1100_request *req)
{
	struct sa1100_udc *dev = ep->dev;
	unsigned int left = req->req.length - req->req.actual;
	u8 *buf = req->req.buf + req->req.actual;
	unsigned int count = 0, stored = 0;
	u8 byte;

	/* RNE, not a count, says how much arrived (DM ss11.8.8.6) */
	while (count < RX_FIFO_DEPTH && (udc_readl(dev, UDCCS1) & UDCCS1_RNE)) {
		byte = udc_readl(dev, UDCDR);
		if (stored < left)
			buf[stored++] = byte;
		count++;
	}

	/* RPC NAKs further OUT tokens until acknowledged (DM ss11.8.8.2) */
	ep_cs_write(ep, UDCCS1_RPC);

	req->req.actual += stored;
	if (stored < count)
		done(ep, req, -EOVERFLOW);
	else if (count < ep->ep.maxpacket || req->req.actual == req->req.length)
		done(ep, req, 0);
}

#define HANDLE_EP_MAX_LOOPS	64

static void handle_bulk_out(struct sa1100_ep *ep)
{
	struct sa1100_udc *dev = ep->dev;
	struct sa1100_request *req;
	unsigned int loops;
	u32 cs;

	for (loops = 0; loops < HANDLE_EP_MAX_LOOPS; loops++) {
		cs = udc_readl(dev, UDCCS1);

		if (cs & UDCCS1_SST) {
			/* a stall handshake went out; there is no data */
			ep_cs_write(ep, UDCCS1_SST | (cs & UDCCS1_RPC));
			continue;
		}

		if (!(cs & UDCCS1_RPC))
			return;

		if (cs & UDCCS1_RPE) {
			dev_dbg(dev->dev, "ep1 receive error %#x\n", cs);
			sa1100_drain_rx(dev);
			ep_cs_write(ep, UDCCS1_RPC);
			continue;
		}

		/* no buffer: leave RPC set, so the UDC keeps NAKing */
		if (list_empty(&ep->queue))
			return;

		req = list_entry(ep->queue.next, struct sa1100_request, queue);
		sa1100_read_fifo(ep, req);
	}

	dev_err_ratelimited(dev->dev, "%s: %u passes in one call, bailing out\n",
			    ep->ep.name, loops);
}

/*
 * ep0 control transfers
 */

/*
 * A no-data request's OPR is only cleared together with setting DE, which
 * lets the UDC run the status stage (DM ss11.8.7.1/ss11.8.7.5; hh RET_ACK,
 * WinCE's 0x50 writes). Stalling such a request writes FST in the same
 * access (hh RET_REQERROR).
 */
static void ep0_stall(struct sa1100_udc *dev)
{
	if (dev->ep0state == EP0_STATUS)
		ep0_cs_write(dev, UDCCS0_SO | UDCCS0_DE | UDCCS0_FST);
	else
		ep0_cs_write(dev, UDCCS0_FST);
	dev->ep0state = EP0_IDLE;
}

static void ep0_ack_status(struct sa1100_udc *dev)
{
	ep0_cs_write(dev, UDCCS0_SO | UDCCS0_DE);
	dev->ep0state = EP0_IDLE;
}

static void ep0_write_data(struct sa1100_udc *dev, struct sa1100_ep *ep,
			   struct sa1100_request *req)
{
	unsigned int left = req->req.length - req->req.actual;
	unsigned int n = min(left, EP0_FIFO_SIZE);
	u8 *buf = req->req.buf + req->req.actual;
	unsigned int i;
	bool is_last;

	/* the previous packet has not gone out yet */
	if (udc_readl(dev, UDCCS0) & UDCCS0_IPR)
		return;

	for (i = 0; i < n; i++) {
		if (ep0_write_byte(dev, buf[i]) < 0)
			break;
	}
	req->req.actual += i;
	is_last = (i < EP0_FIFO_SIZE) ||
		  (req->req.actual == req->req.length && !req->req.zero);

	if (is_last) {
		ep0_cs_write(dev, UDCCS0_IPR | UDCCS0_DE);
		done(ep, req, 0);
		dev->ep0state = EP0_IDLE;
	} else {
		ep0_cs_write(dev, UDCCS0_IPR);
	}
}

static void ep0_read_data(struct sa1100_udc *dev, struct sa1100_ep *ep,
			  struct sa1100_request *req)
{
	unsigned int left = req->req.length - req->req.actual;
	unsigned int avail = udc_readl(dev, UDCWC) & 0xf;
	unsigned int n = min3(left, avail, EP0_FIFO_SIZE);
	u8 *buf = req->req.buf + req->req.actual;
	unsigned int i;

	for (i = 0; i < n; i++) {
		if (ep0_read_byte(dev, &buf[i]) < 0)
			break;
	}
	req->req.actual += i;

	if (avail < EP0_FIFO_SIZE || req->req.actual == req->req.length) {
		ep0_ack_status(dev);
		done(ep, req, 0);
	} else {
		ep0_cs_write(dev, UDCCS0_SO);
	}
}

/* caller holds dev->lock; parses a freshly-arrived SETUP packet */
static void ep0_setup(struct sa1100_udc *dev)
{
	struct sa1100_ep *ep0 = &dev->ep[0];
	struct usb_ctrlrequest *r = &dev->setup;
	u8 raw[8];
	unsigned int i;
	u32 wc = 0;
	int ret;

	nuke(ep0, -EPROTO);
	dev->ep0state = EP0_IDLE;

	/* WinCE waits for all eight SETUP bytes before reading any */
	for (i = 0; i < EP0_SETUP_WAIT_US; i++) {
		wc = udc_readl(dev, UDCWC) & 0xf;
		if (wc >= sizeof(raw))
			break;
		udelay(1);
	}
	if (wc != sizeof(raw)) {
		dev_err_ratelimited(dev->dev, "SETUP with %u bytes\n", wc);
		ep0_cs_write(dev, UDCCS0_SO | UDCCS0_FST);
		return;
	}

	for (i = 0; i < sizeof(raw); i++) {
		if (ep0_read_byte(dev, &raw[i]) < 0) {
			ep0_cs_write(dev, UDCCS0_SO | UDCCS0_FST);
			return;
		}
	}
	memcpy(r, raw, sizeof(raw));

	dev_dbg(dev->dev, "SETUP %02x.%02x v%04x i%04x l%04x\n",
		r->bRequestType, r->bRequest, le16_to_cpu(r->wValue),
		le16_to_cpu(r->wIndex), le16_to_cpu(r->wLength));

	/*
	 * Unlike PXA2xx, this UDC leaves SET_ADDRESS to software. UDCAR is
	 * loaded before the status stage is released: the UDC latches the
	 * new address only once that stage is acknowledged (DM ss11.8.4;
	 * hh udc_set_address() then RET_ACK). It is a plain write because
	 * the read-back keeps the old value until then (hh's comment on
	 * udc_set_address(); WinCE writes it once, too).
	 */
	if (r->bRequestType == (USB_DIR_OUT | USB_TYPE_STANDARD |
				USB_RECIP_DEVICE) &&
	    r->bRequest == USB_REQ_SET_ADDRESS) {
		dev->address = le16_to_cpu(r->wValue) & UDCAR_ADDR_MASK;
		udc_writel(dev, UDCAR, dev->address);
		ep0_ack_status(dev);
		usb_gadget_set_state(&dev->gadget, dev->address ?
				     USB_STATE_ADDRESS : USB_STATE_DEFAULT);
		return;
	}

	if (le16_to_cpu(r->wLength) == 0) {
		dev->ep0state = EP0_STATUS;
	} else {
		/* no data phase may start before OPR is clear (DM ss11.8.7.1) */
		ep0_cs_write(dev, UDCCS0_SO);
		dev->ep0state = (r->bRequestType & USB_DIR_IN) ?
				EP0_IN_DATA : EP0_OUT_DATA;
	}

	spin_unlock(&dev->lock);
	ret = dev->driver->setup(&dev->gadget, r);
	spin_lock(&dev->lock);

	if (ret < 0) {
		ep0_stall(dev);
		return;
	}

	/*
	 * A no-data request the gadget answered without queueing a status
	 * request; USB_GADGET_DELAYED_STATUS leaves it for a later
	 * zero-length queue instead.
	 */
	if (dev->ep0state == EP0_STATUS && ret != USB_GADGET_DELAYED_STATUS)
		ep0_ack_status(dev);
}

static void handle_ep0(struct sa1100_udc *dev)
{
	struct sa1100_ep *ep0 = &dev->ep[0];
	struct sa1100_request *req = NULL;
	u32 cs0 = udc_readl(dev, UDCCS0);

	if (cs0 & UDCCS0_SST) {
		ep0_cs_write(dev, UDCCS0_SST);
		nuke(ep0, -EPIPE);
		dev->ep0state = EP0_IDLE;
	}

	if (cs0 & UDCCS0_SE) {
		/*
		 * The host ended the control transfer early, typically with
		 * a new SETUP (DM ss11.8.7.6), which OPR below then picks up.
		 */
		ep0_cs_write(dev, UDCCS0_SSE);
		nuke(ep0, -ECONNRESET);
		dev->ep0state = EP0_IDLE;
	}

	if (cs0 & (UDCCS0_SST | UDCCS0_SE))
		cs0 = udc_readl(dev, UDCCS0);

	if (!list_empty(&ep0->queue))
		req = list_entry(ep0->queue.next, struct sa1100_request, queue);

	switch (dev->ep0state) {
	case EP0_IDLE:
		if (cs0 & UDCCS0_OPR)
			ep0_setup(dev);
		break;
	case EP0_IN_DATA:
		if (req && !(cs0 & UDCCS0_IPR))
			ep0_write_data(dev, ep0, req);
		break;
	case EP0_OUT_DATA:
		if ((cs0 & UDCCS0_OPR) && req)
			ep0_read_data(dev, ep0, req);
		break;
	case EP0_STATUS:
		break;
	}
}

/*
 * usb_ep_ops
 */

static int sa1100_ep_enable(struct usb_ep *_ep,
			    const struct usb_endpoint_descriptor *desc)
{
	struct sa1100_ep *ep = container_of(_ep, struct sa1100_ep, ep);
	struct sa1100_udc *dev = ep->dev;
	unsigned long flags;
	unsigned int maxp;

	if (!desc || _ep->name == ep0name ||
	    desc->bDescriptorType != USB_DT_ENDPOINT ||
	    ep->bEndpointAddress != desc->bEndpointAddress)
		return -EINVAL;

	if (usb_endpoint_xfer_bulk(desc) != (ep->bmAttributes == USB_ENDPOINT_XFER_BULK))
		return -EINVAL;

	maxp = usb_endpoint_maxp(desc);
	if (!maxp || maxp > _ep->maxpacket_limit)
		return -ERANGE;

	if (!dev->driver || dev->gadget.speed == USB_SPEED_UNKNOWN)
		return -ESHUTDOWN;

	spin_lock_irqsave(&dev->lock, flags);

	_ep->desc = desc;
	_ep->maxpacket = maxp;
	ep->stopped = 0;
	ep->halted = 0;

	if (ep->is_in)
		udc_write_verify(dev, UDCIMP, UDC_MAXP_FIELD(maxp));
	else
		udc_write_verify(dev, UDCOMP, UDC_MAXP_FIELD(maxp));
	ep_cs_write(ep, 0);

	udccr_update(dev, 0, ep->mask_bit);

	spin_unlock_irqrestore(&dev->lock, flags);
	return 0;
}

static int sa1100_ep_disable(struct usb_ep *_ep)
{
	struct sa1100_ep *ep = container_of(_ep, struct sa1100_ep, ep);
	struct sa1100_udc *dev = ep->dev;
	unsigned long flags;

	if (!_ep->desc)
		return -EINVAL;

	spin_lock_irqsave(&dev->lock, flags);

	udccr_update(dev, ep->mask_bit, 0);
	nuke(ep, -ESHUTDOWN);
	_ep->desc = NULL;
	ep->stopped = 1;
	if (ep->halted) {
		ep->halted = 0;
		ep_cs_write(ep, 0);
	}

	spin_unlock_irqrestore(&dev->lock, flags);
	return 0;
}

static int sa1100_ep_queue(struct usb_ep *_ep, struct usb_request *_req,
			   gfp_t gfp_flags)
{
	struct sa1100_ep *ep = container_of(_ep, struct sa1100_ep, ep);
	struct sa1100_request *req = container_of(_req, struct sa1100_request, req);
	struct sa1100_udc *dev = ep->dev;
	unsigned long flags;
	int ret = 0;

	if (unlikely(!_req || !_req->complete || !_req->buf ||
		     !list_empty(&req->queue)))
		return -EINVAL;

	if (ep != &dev->ep[0] && !_ep->desc)
		return -ESHUTDOWN;

	if (!dev->driver || dev->gadget.speed == USB_SPEED_UNKNOWN)
		return -ESHUTDOWN;

	spin_lock_irqsave(&dev->lock, flags);

	_req->status = -EINPROGRESS;
	_req->actual = 0;

	if (ep == &dev->ep[0]) {
		switch (dev->ep0state) {
		case EP0_STATUS:
			if (_req->length) {
				ret = -EINVAL;
				break;
			}
			list_add_tail(&req->queue, &ep->queue);
			ep0_ack_status(dev);
			done(ep, req, 0);
			break;
		case EP0_IN_DATA:
			list_add_tail(&req->queue, &ep->queue);
			if (list_is_singular(&ep->queue))
				ep0_write_data(dev, ep, req);
			break;
		case EP0_OUT_DATA:
			list_add_tail(&req->queue, &ep->queue);
			if (list_is_singular(&ep->queue) &&
			    (udc_readl(dev, UDCCS0) & UDCCS0_OPR))
				ep0_read_data(dev, ep, req);
			break;
		default:
			/* no control transfer is waiting for this request */
			ret = -EL2HLT;
			break;
		}
	} else {
		bool was_empty = list_empty(&ep->queue);

		list_add_tail(&req->queue, &ep->queue);
		if (was_empty) {
			if (ep->is_in)
				handle_bulk_in(ep);
			else
				handle_bulk_out(ep);
		}
	}

	spin_unlock_irqrestore(&dev->lock, flags);
	return ret;
}

static int sa1100_ep_dequeue(struct usb_ep *_ep, struct usb_request *_req)
{
	struct sa1100_ep *ep = container_of(_ep, struct sa1100_ep, ep);
	struct sa1100_udc *dev = ep->dev;
	struct sa1100_request *req;
	unsigned long flags;

	spin_lock_irqsave(&dev->lock, flags);

	list_for_each_entry(req, &ep->queue, queue) {
		if (&req->req == _req) {
			/*
			 * An IN packet already in the FIFO cannot be pulled
			 * back; done() detaches it from the request, and it
			 * is dropped from the accounting when it completes.
			 */
			done(ep, req, -ECONNRESET);
			spin_unlock_irqrestore(&dev->lock, flags);
			return 0;
		}
	}

	spin_unlock_irqrestore(&dev->lock, flags);
	return -EINVAL;
}

static int sa1100_ep_set_halt(struct usb_ep *_ep, int value)
{
	struct sa1100_ep *ep = container_of(_ep, struct sa1100_ep, ep);
	struct sa1100_udc *dev = ep->dev;
	unsigned long flags;
	u32 sst;

	spin_lock_irqsave(&dev->lock, flags);

	if (ep == &dev->ep[0]) {
		/* ep0 cannot stay halted: this stalls the current request */
		if (value)
			ep0_stall(dev);
		spin_unlock_irqrestore(&dev->lock, flags);
		return 0;
	}

	if (value && ep->is_in && !list_empty(&ep->queue)) {
		spin_unlock_irqrestore(&dev->lock, flags);
		return -EAGAIN;
	}

	ep->halted = !!value;
	ep_cs_write(ep, 0);

	if (!value) {
		/*
		 * The stall state is left only once FST and SST are both
		 * clear (DM ss11.8.8.5/ss11.8.9.6; hh udc_ep2_halt()).
		 */
		sst = ep->is_in ? UDCCS2_SST : UDCCS1_SST;
		if (udc_readl(dev, ep->cs_reg) & sst)
			ep_cs_write(ep, sst);
		if (ep->is_in) {
			ep->in_len = 0;
			ep->in_req = NULL;
			handle_bulk_in(ep);
		} else {
			handle_bulk_out(ep);
		}
	}

	spin_unlock_irqrestore(&dev->lock, flags);
	return 0;
}

static int sa1100_ep_fifo_status(struct usb_ep *_ep)
{
	/* no byte-count register exists for ep1/ep2 on this silicon */
	return -EOPNOTSUPP;
}

static void sa1100_ep_fifo_flush(struct usb_ep *_ep)
{
	/*
	 * No flush control exists for any of the three endpoints (unlike
	 * PXA's UDCCS_BI_FTF); some function drivers call this defensively
	 * on disable, so it must exist and must not fault.
	 */
}

static const struct usb_ep_ops sa1100_ep_ops = {
	.enable		= sa1100_ep_enable,
	.disable	= sa1100_ep_disable,
	.alloc_request	= sa1100_ep_alloc_request,
	.free_request	= sa1100_ep_free_request,
	.queue		= sa1100_ep_queue,
	.dequeue	= sa1100_ep_dequeue,
	.set_halt	= sa1100_ep_set_halt,
	.fifo_status	= sa1100_ep_fifo_status,
	.fifo_flush	= sa1100_ep_fifo_flush,
};

/*
 * Hardware enable/disable and gadget-scoped ops
 */

static void stop_activity(struct sa1100_udc *dev)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(dev->ep); i++)
		nuke(&dev->ep[i], -ESHUTDOWN);
	dev->ep0state = EP0_IDLE;
}

/* The UDC's own reset clears address, FIFOs and endpoint state. */
static void udc_forget_hw_state(struct sa1100_udc *dev)
{
	dev->address = 0;
	dev->ep[1].halted = 0;
	dev->ep[2].halted = 0;
	dev->ep[2].in_len = 0;
	dev->ep[2].in_req = NULL;
	dev->ep0state = EP0_IDLE;
}

/* caller holds dev->lock */
static void udc_hw_disable(struct sa1100_udc *dev)
{
	dev->udccr = UDCCR_IDLE;
	udc_write_verify(dev, UDCCR, dev->udccr);
	dev->active = 0;
	udc_forget_hw_state(dev);
}

/*
 * caller holds dev->lock. hh udc_start()'s sequence: enable the block
 * with every interrupt masked, let it settle, clear stale status, then
 * unmask reset/suspend/resume/ep0 while ep1/ep2 stay masked until a
 * function enables them.
 */
static void udc_hw_enable(struct sa1100_udc *dev)
{
	udc_write_verify(dev, UDCCR,
			 UDCCR_SRM | UDCCR_RIM | UDCCR_TIM | UDCCR_EIM);
	udelay(100);

	udc_flip_verify(dev, UDCSR, UDCSR_ALL);

	dev->udccr = UDCCR_RIM | UDCCR_TIM;
	udc_write_verify(dev, UDCCR, dev->udccr);
	dev->active = 1;
	udc_forget_hw_state(dev);
}

/*
 * A host-issued reset resets the entire UDC (DM ss11.8.13.6), so every
 * piece of state software programmed is re-established: WinCE re-runs
 * its whole UDC init on RSTIR. Endpoints are unconfigured by the reset,
 * so ep1/ep2 go back to masked until a function re-enables them.
 */
static void udc_bus_reset(struct sa1100_udc *dev)
{
	struct usb_gadget_driver *driver = dev->driver;

	udc_forget_hw_state(dev);
	dev->udccr = UDCCR_RIM | UDCCR_TIM;
	udc_write_verify(dev, UDCCR, dev->udccr);

	nuke(&dev->ep[0], -ECONNRESET);

	if (driver && dev->gadget.speed != USB_SPEED_UNKNOWN) {
		spin_unlock(&dev->lock);
		usb_gadget_udc_reset(&dev->gadget, driver);
		spin_lock(&dev->lock);
	}
	dev->gadget.speed = USB_SPEED_FULL;
}

/*
 * Toggle the UDC on/off based on vbus presence, pullup request, and
 * whether a gadget driver is bound. UDCCR_UDD is not a pull-up: the D+
 * resistor is external and fixed, so with the UDC disabled the host still
 * sees an attach but gets no answer (DM ss11.8.3.1).
 *
 * The gadget core tells the gadget driver about a pullup-initiated
 * disconnect itself, but not about loss of VBUS, so @notify is set only
 * on the VBUS path (as pxa25x_udc.c's stop_activity() does).
 */
static void udc_reconfigure(struct sa1100_udc *dev, bool notify)
{
	struct usb_gadget_driver *driver = dev->driver;
	unsigned long flags;
	int want_active = driver && dev->vbus && dev->pullup;

	spin_lock_irqsave(&dev->lock, flags);
	if (want_active && !dev->active) {
		udc_hw_enable(dev);
	} else if (!want_active && dev->active) {
		bool was_connected = dev->gadget.speed != USB_SPEED_UNKNOWN;

		if (was_connected)
			stop_activity(dev);
		udc_hw_disable(dev);
		dev->gadget.speed = USB_SPEED_UNKNOWN;

		if (notify && was_connected && driver && driver->disconnect) {
			spin_unlock(&dev->lock);
			driver->disconnect(&dev->gadget);
			spin_lock(&dev->lock);
		}
	}
	spin_unlock_irqrestore(&dev->lock, flags);
}

static int sa1100_udc_vbus_session(struct usb_gadget *_gadget, int is_active)
{
	struct sa1100_udc *dev = container_of(_gadget, struct sa1100_udc, gadget);

	dev->vbus = !!is_active;
	udc_reconfigure(dev, true);
	return 0;
}

static int sa1100_udc_pullup(struct usb_gadget *_gadget, int is_active)
{
	struct sa1100_udc *dev = container_of(_gadget, struct sa1100_udc, gadget);

	dev->pullup = !!is_active;
	udc_reconfigure(dev, false);
	return 0;
}

static int sa1100_udc_vbus_draw(struct usb_gadget *_gadget, unsigned int mA)
{
	struct sa1100_udc *dev = container_of(_gadget, struct sa1100_udc, gadget);

	if (dev->transceiver)
		return usb_phy_set_power(dev->transceiver, mA);
	return -EOPNOTSUPP;
}

static void sa1100_udc_find_transceiver(struct sa1100_udc *dev)
{
	struct usb_phy *phy;

	if (dev->transceiver)
		return;

	phy = devm_usb_get_phy(dev->dev, USB_PHY_TYPE_USB2);
	if (IS_ERR(phy)) {
		dev_dbg(dev->dev, "no USB PHY/VBUS transceiver (%ld)\n",
			PTR_ERR(phy));
		return;
	}
	dev->transceiver = phy;
}

static int sa1100_udc_start(struct usb_gadget *g,
			    struct usb_gadget_driver *driver)
{
	struct sa1100_udc *dev = container_of(g, struct sa1100_udc, gadget);
	int ret;

	if (!driver || driver->max_speed < USB_SPEED_FULL || !driver->setup)
		return -EINVAL;
	if (dev->driver)
		return -EBUSY;

	/*
	 * The VBUS sensor (a gpio-vbus usb_phy on a companion-chip GPIO)
	 * usually probes after this SoC device, so it is looked up again
	 * at bind time. Without one, VBUS is assumed present.
	 */
	sa1100_udc_find_transceiver(dev);

	dev->driver = driver;
	dev->pullup = 1;

	if (dev->transceiver && dev->transceiver->otg) {
		/* the transceiver reports VBUS through vbus_session() */
		dev->vbus = 0;
		ret = otg_set_peripheral(dev->transceiver->otg, &dev->gadget);
		if (ret) {
			dev_err(dev->dev, "can't bind to transceiver: %d\n", ret);
			dev->driver = NULL;
			return ret;
		}
	} else {
		dev->vbus = 1;
	}

	udc_reconfigure(dev, false);
	return 0;
}

static int sa1100_udc_stop(struct usb_gadget *g)
{
	struct sa1100_udc *dev = container_of(g, struct sa1100_udc, gadget);

	dev->pullup = 0;
	udc_reconfigure(dev, false);

	if (dev->transceiver && dev->transceiver->otg)
		otg_set_peripheral(dev->transceiver->otg, NULL);

	dev->driver = NULL;
	return 0;
}

static const struct usb_gadget_ops sa1100_udc_ops = {
	.vbus_session	= sa1100_udc_vbus_session,
	.pullup		= sa1100_udc_pullup,
	.vbus_draw	= sa1100_udc_vbus_draw,
	.udc_start	= sa1100_udc_start,
	.udc_stop	= sa1100_udc_stop,
};

/*
 * Interrupt handler
 */

static irqreturn_t sa1100_udc_irq(int irq, void *_dev)
{
	struct sa1100_udc *dev = _dev;
	struct usb_gadget_driver *driver;
	u32 status;

	spin_lock(&dev->lock);

	status = udc_readl(dev, UDCSR);

	if (!status) {
		spin_unlock(&dev->lock);
		return IRQ_NONE;
	}

	/*
	 * Ack first, then service (hh udc_interrupt(); pxa25x_udc.c's irq
	 * handler documents the lost-event hazard of the reverse order).
	 * Every handler below re-reads its own control/status register
	 * rather than trusting the edge, so a source re-asserting between
	 * the ack and its service is not lost.
	 */
	udc_flip_verify(dev, UDCSR, status);

	if (status & UDCSR_RSTIR)
		udc_bus_reset(dev);

	driver = dev->driver;
	if ((status & UDCSR_SUSIR) && driver && driver->suspend &&
	    dev->gadget.speed != USB_SPEED_UNKNOWN) {
		spin_unlock(&dev->lock);
		driver->suspend(&dev->gadget);
		spin_lock(&dev->lock);
	}

	if ((status & UDCSR_RESIR) && driver && driver->resume &&
	    dev->gadget.speed != USB_SPEED_UNKNOWN) {
		spin_unlock(&dev->lock);
		driver->resume(&dev->gadget);
		spin_lock(&dev->lock);
	}

	if (status & UDCSR_EIR)
		handle_ep0(dev);

	if (status & UDCSR_RIR)
		handle_bulk_out(&dev->ep[1]);

	if (status & UDCSR_TIR)
		handle_bulk_in(&dev->ep[2]);

	spin_unlock(&dev->lock);
	return IRQ_HANDLED;
}

/*
 * Endpoint table and gadget scaffolding
 */

static void sa1100_udc_init_eps(struct sa1100_udc *dev)
{
	struct sa1100_ep *ep;

	ep = &dev->ep[0];
	ep->dev = dev;
	ep->name = ep0name;
	ep->bEndpointAddress = 0;
	ep->bmAttributes = USB_ENDPOINT_XFER_CONTROL;
	ep->cs_reg = UDCCS0;
	INIT_LIST_HEAD(&ep->queue);
	usb_ep_set_maxpacket_limit(&ep->ep, EP0_FIFO_SIZE);
	ep->ep.ops = &sa1100_ep_ops;
	ep->ep.name = ep0name;
	ep->ep.caps.type_control = true;
	ep->ep.caps.dir_in = true;
	ep->ep.caps.dir_out = true;
	dev->gadget.ep0 = &ep->ep;

	ep = &dev->ep[1];
	ep->dev = dev;
	ep->name = "ep1out";
	ep->bEndpointAddress = USB_DIR_OUT | 1;
	ep->bmAttributes = USB_ENDPOINT_XFER_BULK;
	ep->cs_reg = UDCCS1;
	ep->mask_bit = UDCCR_RIM;
	ep->is_in = 0;
	INIT_LIST_HEAD(&ep->queue);
	usb_ep_set_maxpacket_limit(&ep->ep, BULK_PIO_MAXPACKET);
	ep->ep.ops = &sa1100_ep_ops;
	ep->ep.name = ep->name;
	ep->ep.caps.type_bulk = true;
	ep->ep.caps.dir_out = true;

	ep = &dev->ep[2];
	ep->dev = dev;
	ep->name = "ep2in";
	ep->bEndpointAddress = USB_DIR_IN | 2;
	ep->bmAttributes = USB_ENDPOINT_XFER_BULK;
	ep->cs_reg = UDCCS2;
	ep->mask_bit = UDCCR_TIM;
	ep->is_in = 1;
	INIT_LIST_HEAD(&ep->queue);
	usb_ep_set_maxpacket_limit(&ep->ep, BULK_PIO_MAXPACKET);
	ep->ep.ops = &sa1100_ep_ops;
	ep->ep.name = ep->name;
	ep->ep.caps.type_bulk = true;
	ep->ep.caps.dir_in = true;

	INIT_LIST_HEAD(&dev->gadget.ep_list);
	list_add_tail(&dev->ep[1].ep.ep_list, &dev->gadget.ep_list);
	list_add_tail(&dev->ep[2].ep.ep_list, &dev->gadget.ep_list);
}

static void sa1100_udc_release(struct device *_dev)
{
	/* static allocation below; nothing to free */
}

/*
 * Platform driver
 */

static int sa1100_udc_probe(struct platform_device *pdev)
{
	struct sa1100_udc *dev;
	struct resource *res;
	int irq, ret;

	dev = devm_kzalloc(&pdev->dev, sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	dev->regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(dev->regs))
		return PTR_ERR(dev->regs);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	dev->dev = &pdev->dev;
	dev->irq = irq;
	spin_lock_init(&dev->lock);

	dev->gadget.ops = &sa1100_udc_ops;
	dev->gadget.ep0 = NULL; /* set by sa1100_udc_init_eps() */
	dev->gadget.name = DRIVER_NAME;
	dev->gadget.max_speed = USB_SPEED_FULL;
	dev->gadget.speed = USB_SPEED_UNKNOWN;
	dev->gadget.quirk_zlp_not_supp = 1;
	dev->gadget.dev.parent = &pdev->dev;
	dev->gadget.dev.release = sa1100_udc_release;

	sa1100_udc_init_eps(dev);

	sa1100_udc_find_transceiver(dev);

	platform_set_drvdata(pdev, dev);

	udc_hw_disable(dev);

	ret = devm_request_irq(&pdev->dev, irq, sa1100_udc_irq, 0,
			       DRIVER_NAME, dev);
	if (ret) {
		dev_err(&pdev->dev, "can't get irq %d: %d\n", irq, ret);
		return ret;
	}
	dev->got_irq = 1;

	ret = usb_add_gadget_udc(&pdev->dev, &dev->gadget);
	if (ret)
		return ret;

	return 0;
}

static void sa1100_udc_remove(struct platform_device *pdev)
{
	struct sa1100_udc *dev = platform_get_drvdata(pdev);

	usb_del_gadget_udc(&dev->gadget);
	udc_hw_disable(dev);
}

static void sa1100_udc_shutdown(struct platform_device *pdev)
{
	struct sa1100_udc *dev = platform_get_drvdata(pdev);

	if (dev)
		udc_hw_disable(dev);
}

static const struct of_device_id sa1100_udc_of_match[] = {
	{ .compatible = "intel,sa1110-udc" },
	{ .compatible = "intel,sa1100-udc" },
	{ }
};
MODULE_DEVICE_TABLE(of, sa1100_udc_of_match);

/*
 * The driver name is "sa11x0-udc", not "sa1100-udc": that
 * is the exact platform_device name arch/arm/mach-sa1100/generic.c has
 * registered for this hardware since before this driver existed
 * (sa11x0udc_device.name, still registered unconditionally on a non-DT
 * boot). Matching it here gives that legacy registration a real driver
 * to bind to as well as the devicetree node above, via plain
 * platform_device name matching (platform_match() falls back to
 * comparing pdev->name against drv->name when there is no OF/ACPI/
 * id_table match), with no separate platform_device_id table needed for
 * a single fixed name.
 */
static struct platform_driver sa1100_udc_driver = {
	.probe		= sa1100_udc_probe,
	.remove		= sa1100_udc_remove,
	.shutdown	= sa1100_udc_shutdown,
	.driver		= {
		.name		= "sa11x0-udc",
		.of_match_table	= sa1100_udc_of_match,
	},
};
module_platform_driver(sa1100_udc_driver);

MODULE_ALIAS("platform:sa11x0-udc");
MODULE_AUTHOR("Koen Kooi");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
