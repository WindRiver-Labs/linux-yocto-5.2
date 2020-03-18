/*
 * Freescale linflexuart serial port driver
 *
 * Copyright 2012-2016 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#if defined(CONFIG_SERIAL_FSL_LINFLEXUART_CONSOLE) && defined(CONFIG_MAGIC_SYSRQ)
#define SUPPORT_SYSRQ
#endif

#include <linux/clk.h>
#include <linux/console.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/dmapool.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_dma.h>
#include <linux/serial_core.h>
#include <linux/slab.h>
#include <linux/tty_flip.h>

/* All registers are 32-bit width */

#define LINCR1	0x0000	/* LIN control register				*/
#define LINIER	0x0004	/* LIN interrupt enable register		*/
#define LINSR	0x0008	/* LIN status register				*/
#define LINESR	0x000C	/* LIN error status register			*/
#define UARTCR	0x0010	/* UART mode control register			*/
#define UARTSR	0x0014	/* UART mode status register			*/
#define LINTCSR	0x0018	/* LIN timeout control status register		*/
#define LINOCR	0x001C	/* LIN output compare register			*/
#define LINTOCR	0x0020	/* LIN timeout control register			*/
#define LINFBRR	0x0024	/* LIN fractional baud rate register		*/
#define LINIBRR	0x0028	/* LIN integer baud rate register		*/
#define LINCFR	0x002C	/* LIN checksum field register			*/
#define LINCR2	0x0030	/* LIN control register 2			*/
#define BIDR	0x0034	/* Buffer identifier register			*/
#define BDRL	0x0038	/* Buffer data register least significant	*/
#define BDRM	0x003C	/* Buffer data register most significant	*/
#define IFER	0x0040	/* Identifier filter enable register		*/
#define IFMI	0x0044	/* Identifier filter match index		*/
#define IFMR	0x0048	/* Identifier filter mode register		*/
#define GCR	0x004C	/* Global control register			*/
#define UARTPTO	0x0050	/* UART preset timeout register			*/
#define UARTCTO	0x0054	/* UART current timeout register		*/
/* The offsets for DMARXE/DMATXE in master mode only			*/
#define DMATXE	0x0058	/* DMA Tx enable register			*/
#define DMARXE	0x005C	/* DMA Rx enable register			*/

/*
 *	CONSTANT DEFINITIONS
 */

#define LINFLEXD_LINCR1_INIT		(1<<0)
#define LINFLEXD_LINCR1_MME		(1<<4)
#define LINFLEXD_LINCR1_BF		(1<<7)

#define LINFLEXD_LINSR_LINS_INITMODE	(1<<12)
#define LINFLEXD_LINSR_LINS_MASK	(0xF<<12)

#define LINFLEXD_LINIER_SZIE		(1<<15)
#define LINFLEXD_LINIER_OCIE		(1<<14)
#define LINFLEXD_LINIER_BEIE		(1<<13)
#define LINFLEXD_LINIER_CEIE		(1<<12)
#define LINFLEXD_LINIER_HEIE		(1<<11)
#define LINFLEXD_LINIER_FEIE		(1<<8)
#define LINFLEXD_LINIER_BOIE		(1<<7)
#define LINFLEXD_LINIER_LSIE		(1<<6)
#define LINFLEXD_LINIER_WUIE		(1<<5)
#define LINFLEXD_LINIER_DBFIE		(1<<4)
#define LINFLEXD_LINIER_DBEIETOIE	(1<<3)
#define LINFLEXD_LINIER_DRIE		(1<<2)
#define LINFLEXD_LINIER_DTIE		(1<<1)
#define LINFLEXD_LINIER_HRIE		(1<<0)

#define LINFLEXD_UARTCR_OSR_MASK	(0xF<<24)
#define LINFLEXD_UARTCR_OSR(uartcr)	(((uartcr) \
					& LINFLEXD_UARTCR_OSR_MASK) >> 24)

#define LINFLEXD_UARTCR_ROSE		(1<<23)

#define LINFLEXD_UARTCR_SBUR_MASK	(0x3<<17)
#define LINFLEXD_UARTCR_SBUR_1SBITS	(0x0<<17)
#define LINFLEXD_UARTCR_SBUR_2SBITS	(0x1<<17)
#define LINFLEXD_UARTCR_SBUR_3SBITS	(0x2<<17)

#define LINFLEXD_UARTCR_RFBM		(1<<9)
#define LINFLEXD_UARTCR_TFBM		(1<<8)
#define LINFLEXD_UARTCR_WL1		(1<<7)
#define LINFLEXD_UARTCR_PC1		(1<<6)

#define LINFLEXD_UARTCR_RXEN		(1<<5)
#define LINFLEXD_UARTCR_TXEN		(1<<4)
#define LINFLEXD_UARTCR_PC0		(1<<3)

#define LINFLEXD_UARTCR_PCE		(1<<2)
#define LINFLEXD_UARTCR_WL0		(1<<1)
#define LINFLEXD_UARTCR_UART		(1<<0)

#define LINFLEXD_UARTSR_SZF		(1<<15)
#define LINFLEXD_UARTSR_OCF		(1<<14)
#define LINFLEXD_UARTSR_PE3		(1<<13)
#define LINFLEXD_UARTSR_PE2		(1<<12)
#define LINFLEXD_UARTSR_PE1		(1<<11)
#define LINFLEXD_UARTSR_PE0		(1<<10)
#define LINFLEXD_UARTSR_RMB		(1<<9)
#define LINFLEXD_UARTSR_FEF		(1<<8)
#define LINFLEXD_UARTSR_BOF		(1<<7)
#define LINFLEXD_UARTSR_RPS		(1<<6)
#define LINFLEXD_UARTSR_WUF		(1<<5)
#define LINFLEXD_UARTSR_4		(1<<4)

#define LINFLEXD_UARTSR_TO		(1<<3)

#define LINFLEXD_UARTSR_DRFRFE		(1<<2)
#define LINFLEXD_UARTSR_DTFTFF		(1<<1)
#define LINFLEXD_UARTSR_NF		(1<<0)
#define LINFLEXD_UARTSR_PE		(LINFLEXD_UARTSR_PE0|LINFLEXD_UARTSR_PE1|LINFLEXD_UARTSR_PE2|LINFLEXD_UARTSR_PE3)

#define LINFLEXD_GCR_STOP_MASK		(1<<1)
#define LINFLEXD_GCR_STOP_1SBITS	(0<<1)
#define LINFLEXD_GCR_STOP_2SBITS	(1<<1)

#define DMA_MAXBURST			(16)
#define DMA_MAXBURST_MASK		(DMA_MAXBURST - 1)
#define FSL_UART_RX_DMA_BUFFER_SIZE	(PAGE_SIZE)

#define LINFLEXD_UARTCR_TXFIFO_SIZE	(4)
#define LINFLEXD_UARTCR_RXFIFO_SIZE	(4)

#define LINFLEX_LDIV_MULTIPLIER		(16)

#define DRIVER_NAME	"fsl-linflexuart"
#define DEV_NAME	"ttyLF"
#define UART_NR		2

#define prd_info(a)	;//pr_info(a);

struct linflex_port {
	struct uart_port	port;
	struct clk		*clk;
	struct clk		*clk_ipg;
	unsigned int		txfifo_size;
	unsigned int		rxfifo_size;
	bool			dma_tx_use;
	bool			dma_rx_use;
	struct dma_chan		*dma_tx_chan;
	struct dma_chan		*dma_rx_chan;
	struct dma_async_tx_descriptor  *dma_tx_desc;
	struct dma_async_tx_descriptor  *dma_rx_desc;
	dma_addr_t		dma_tx_buf_bus;
	dma_addr_t		dma_rx_buf_bus;
	dma_cookie_t		dma_tx_cookie;
	dma_cookie_t		dma_rx_cookie;
	unsigned char		*dma_tx_buf_virt;
	unsigned char		*dma_rx_buf_virt;
	unsigned int		dma_tx_bytes;
	int			dma_tx_in_progress;
	int			dma_rx_in_progress;
	unsigned int		dma_rx_timeout;
	struct timer_list	timer;
};

static struct of_device_id linflex_dt_ids[] = {
	{
		.compatible = "fsl,s32v234-linflexuart",
	},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, linflex_dt_ids);

/* Forward declare this for the dma callbacks. */
static void linflex_dma_tx_complete(void *arg);
static void linflex_dma_rx_complete(void *arg);

static void linflex_copy_rx_to_tty(struct linflex_port *sport,
		struct tty_port *tty, int count)
{
	int copied;

	sport->port.icount.rx += count;

	if (!tty) {
		dev_err(sport->port.dev, "No tty port\n");
		return;
	}

	dma_sync_single_for_cpu(sport->port.dev, sport->dma_rx_buf_bus,
			FSL_UART_RX_DMA_BUFFER_SIZE, DMA_FROM_DEVICE);
	copied = tty_insert_flip_string(tty,
			((unsigned char *)(sport->dma_rx_buf_virt)), count);

	if (copied != count) {
		WARN_ON(1);
		dev_err(sport->port.dev, "RxData copy to tty layer failed\n");
	}

	dma_sync_single_for_device(sport->port.dev, sport->dma_rx_buf_bus,
			FSL_UART_RX_DMA_BUFFER_SIZE, DMA_TO_DEVICE);
}

static void linflex_stop_tx(struct uart_port *port)
{
	unsigned long temp;
	unsigned int count;
	struct dma_tx_state state;
	struct linflex_port *sport = container_of(port,
					struct linflex_port, port);
	struct circ_buf *xmit = &sport->port.state->xmit;

	if (!sport->dma_tx_use) {
		temp = readl(port->membase + LINIER);
		temp &= ~(LINFLEXD_LINIER_DTIE);
		writel(temp, port->membase + LINIER);
	} else if (sport->dma_tx_in_progress) {
		dmaengine_pause(sport->dma_tx_chan);
		dmaengine_tx_status(sport->dma_tx_chan,
				sport->dma_tx_cookie, &state);
		dmaengine_terminate_all(sport->dma_tx_chan);
		dma_sync_single_for_cpu(sport->port.dev, sport->dma_tx_buf_bus,
			sport->dma_tx_bytes, DMA_TO_DEVICE);
		count = sport->dma_tx_bytes - state.residue;
		xmit->tail = (xmit->tail + count) & (UART_XMIT_SIZE - 1);
		port->icount.tx += count;

		sport->dma_tx_in_progress = 0;
	}
}

static void linflex_stop_rx(struct uart_port *port)
{
	unsigned long temp;
	unsigned int count;
	struct dma_tx_state state;
	struct linflex_port *sport = container_of(port,
					struct linflex_port, port);

	if (!sport->dma_rx_use) {
		temp = readl(port->membase + LINIER);
		writel(temp & ~LINFLEXD_LINIER_DRIE, port->membase + LINIER);
	} else if (sport->dma_rx_in_progress) {
		del_timer(&sport->timer);
		dmaengine_pause(sport->dma_rx_chan);
		dmaengine_tx_status(sport->dma_rx_chan,
				sport->dma_rx_cookie, &state);
		dmaengine_terminate_all(sport->dma_rx_chan);
		count = FSL_UART_RX_DMA_BUFFER_SIZE - state.residue;

		sport->dma_rx_in_progress = 0;
		linflex_copy_rx_to_tty(sport, &sport->port.state->port, count);
		tty_flip_buffer_push(&sport->port.state->port);
	}
}

static inline void linflex_transmit_buffer(struct linflex_port *sport)
{
	struct circ_buf *xmit = &sport->port.state->xmit;
	unsigned char c;
	unsigned long status;

	while (!uart_circ_empty(xmit)) {
		c = xmit->buf[xmit->tail];
		writeb(c, sport->port.membase + BDRL);

		/* Waiting for data transmission completed. */
		if (!sport->dma_tx_use) {
			while (((status = readl(sport->port.membase + UARTSR)) &
						LINFLEXD_UARTSR_DTFTFF) !=
						LINFLEXD_UARTSR_DTFTFF)
				;
		} else {
			while (((status = readl(sport->port.membase + UARTSR)) &
						LINFLEXD_UARTSR_DTFTFF))
				;
		}

		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		sport->port.icount.tx++;

		if (!sport->dma_tx_use)
			writel(status | LINFLEXD_UARTSR_DTFTFF,
					sport->port.membase + UARTSR);
	}

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(&sport->port);

	if (uart_circ_empty(xmit))
		linflex_stop_tx(&sport->port);
}

static int linflex_dma_tx(struct linflex_port *sport, unsigned long count)
{
	struct circ_buf *xmit = &sport->port.state->xmit;
	dma_addr_t tx_bus_addr;

	while ((readl(sport->port.membase + UARTSR) & LINFLEXD_UARTSR_DTFTFF))
		;

	dma_sync_single_for_device(sport->port.dev, sport->dma_tx_buf_bus,
				UART_XMIT_SIZE, DMA_TO_DEVICE);
	sport->dma_tx_bytes = count;
	tx_bus_addr = sport->dma_tx_buf_bus + xmit->tail;
	sport->dma_tx_desc = dmaengine_prep_slave_single(sport->dma_tx_chan,
			tx_bus_addr, sport->dma_tx_bytes, DMA_MEM_TO_DEV,
			DMA_PREP_INTERRUPT | DMA_CTRL_ACK);

	if (!sport->dma_tx_desc) {
		dev_err(sport->port.dev, "Not able to get desc for tx\n");
		return -EIO;
	}

	sport->dma_tx_desc->callback = linflex_dma_tx_complete;
	sport->dma_tx_desc->callback_param = sport;
	sport->dma_tx_in_progress = 1;
	sport->dma_tx_cookie = dmaengine_submit(sport->dma_tx_desc);
	dma_async_issue_pending(sport->dma_tx_chan);

	return 0;
}

static void linflex_prepare_tx(struct linflex_port *sport)
{
	struct circ_buf *xmit = &sport->port.state->xmit;
	unsigned long count =  CIRC_CNT_TO_END(xmit->head,
					xmit->tail, UART_XMIT_SIZE);

	if (!count || sport->dma_tx_in_progress)
		return;

	linflex_dma_tx(sport, count);
}

static void linflex_dma_tx_complete(void *arg)
{
	struct linflex_port *sport = arg;
	struct circ_buf *xmit = &sport->port.state->xmit;
	unsigned long flags;

	spin_lock_irqsave(&sport->port.lock, flags);

	xmit->tail = (xmit->tail + sport->dma_tx_bytes) & (UART_XMIT_SIZE - 1);
	sport->port.icount.tx += sport->dma_tx_bytes;
	sport->dma_tx_in_progress = 0;

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(&sport->port);

	linflex_prepare_tx(sport);

	spin_unlock_irqrestore(&sport->port.lock, flags);
}

static void linflex_flush_buffer(struct uart_port *port)
{
	struct linflex_port *sport = container_of(port,
					struct linflex_port, port);

	if (sport->dma_tx_use) {
		dmaengine_terminate_all(sport->dma_tx_chan);
		sport->dma_tx_in_progress = 0;
	}
}

static int linflex_dma_rx(struct linflex_port *sport)
{
	dma_sync_single_for_device(sport->port.dev, sport->dma_rx_buf_bus,
			FSL_UART_RX_DMA_BUFFER_SIZE, DMA_FROM_DEVICE);
	sport->dma_rx_desc = dmaengine_prep_slave_single(sport->dma_rx_chan,
			sport->dma_rx_buf_bus, FSL_UART_RX_DMA_BUFFER_SIZE,
			DMA_DEV_TO_MEM, DMA_PREP_INTERRUPT | DMA_CTRL_ACK);

	if (!sport->dma_rx_desc) {
		dev_err(sport->port.dev, "Not able to get desc for rx\n");
		return -EIO;
	}

	sport->dma_rx_desc->callback = linflex_dma_rx_complete;
	sport->dma_rx_desc->callback_param = sport;
	sport->dma_rx_in_progress = 1;
	sport->dma_rx_cookie = dmaengine_submit(sport->dma_rx_desc);
	dma_async_issue_pending(sport->dma_rx_chan);

	return 0;
}

static void linflex_dma_rx_complete(void *arg)
{
	struct linflex_port *sport = arg;
	struct tty_port *port = &sport->port.state->port;
	unsigned long flags;

	mod_timer(&sport->timer, jiffies + sport->dma_rx_timeout);

	spin_lock_irqsave(&sport->port.lock, flags);

	sport->dma_rx_in_progress = 0;
	linflex_copy_rx_to_tty(sport, port, FSL_UART_RX_DMA_BUFFER_SIZE);
	tty_flip_buffer_push(port);
	linflex_dma_rx(sport);

	spin_unlock_irqrestore(&sport->port.lock, flags);
}

static void linflex_timer_func(unsigned long data)
{
	struct linflex_port *sport = (struct linflex_port *)data;
	struct tty_port *port = &sport->port.state->port;
	struct dma_tx_state state;
	unsigned long flags;
	int count;

	del_timer(&sport->timer);
	dmaengine_pause(sport->dma_rx_chan);
	dmaengine_tx_status(sport->dma_rx_chan, sport->dma_rx_cookie, &state);
	dmaengine_terminate_all(sport->dma_rx_chan);
	count = FSL_UART_RX_DMA_BUFFER_SIZE - state.residue;

	spin_lock_irqsave(&sport->port.lock, flags);

	sport->dma_rx_in_progress = 0;
	linflex_copy_rx_to_tty(sport, port, count);
	tty_flip_buffer_push(port);

	linflex_dma_rx(sport);

	spin_unlock_irqrestore(&sport->port.lock, flags);
	mod_timer(&sport->timer, jiffies + sport->dma_rx_timeout);
}

static void linflex_start_tx(struct uart_port *port)
{
	struct linflex_port *sport = container_of(port,
					struct linflex_port, port);
	unsigned long temp;

	if (sport->dma_tx_use) {
		linflex_prepare_tx(sport);
	} else {
		linflex_transmit_buffer(sport);
		temp = readl(port->membase + LINIER);
		writel(temp | LINFLEXD_LINIER_DTIE, port->membase + LINIER);
	}
}

static irqreturn_t linflex_txint(int irq, void *dev_id)
{
	struct linflex_port *sport = dev_id;
	struct circ_buf *xmit = &sport->port.state->xmit;
	unsigned long flags;
	unsigned long status;

	spin_lock_irqsave(&sport->port.lock, flags);

	if (sport->port.x_char) {

		writeb(sport->port.x_char, sport->port.membase + BDRL);

		/* waiting for data transmission completed */
		while (((status = readl(sport->port.membase + UARTSR)) &
			LINFLEXD_UARTSR_DTFTFF) != LINFLEXD_UARTSR_DTFTFF)
			;

		writel(status | LINFLEXD_UARTSR_DTFTFF,
				sport->port.membase + UARTSR);

		goto out;
	}

	if (uart_circ_empty(xmit) || uart_tx_stopped(&sport->port)) {
		linflex_stop_tx(&sport->port);
		goto out;
	}

	linflex_transmit_buffer(sport);

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(&sport->port);

out:
	spin_unlock_irqrestore(&sport->port.lock, flags);
	return IRQ_HANDLED;
}

static irqreturn_t linflex_rxint(int irq, void *dev_id)
{
	struct linflex_port *sport = dev_id;
	unsigned int flg;
	struct tty_port *port = &sport->port.state->port;
	unsigned long flags, status;
	unsigned char rx;

	spin_lock_irqsave(&sport->port.lock, flags);

	status = readl(sport->port.membase + UARTSR);
	while (status & LINFLEXD_UARTSR_RMB) {

		rx = readb(sport->port.membase + BDRM);
		flg = TTY_NORMAL;
		sport->port.icount.rx++;

		if (status & (LINFLEXD_UARTSR_BOF|LINFLEXD_UARTSR_SZF|LINFLEXD_UARTSR_FEF|LINFLEXD_UARTSR_PE)) {

			if (status & LINFLEXD_UARTSR_SZF) {
				status |= LINFLEXD_UARTSR_SZF;
			}
			if (status & LINFLEXD_UARTSR_BOF) {
				status |= LINFLEXD_UARTSR_BOF;
			}
			if (status & LINFLEXD_UARTSR_FEF) {
				status |= LINFLEXD_UARTSR_FEF;
			}
			if (status & LINFLEXD_UARTSR_PE) {
				status |=  LINFLEXD_UARTSR_PE;
			}
		}

		writel(status | LINFLEXD_UARTSR_RMB | LINFLEXD_UARTSR_DRFRFE,
				sport->port.membase + UARTSR);
		status = readl(sport->port.membase + UARTSR);

		if (uart_handle_sysrq_char(&sport->port, (unsigned char)rx))
			continue;

		#ifdef SUPPORT_SYSRQ
			sport->port.sysrq = 0;
		#endif
		tty_insert_flip_char(port, rx, flg);
	}

	spin_unlock_irqrestore(&sport->port.lock, flags);

	tty_flip_buffer_push(port);

	return IRQ_HANDLED;
}

static irqreturn_t linflex_int(int irq, void *dev_id)
{
	struct linflex_port *sport = dev_id;
	unsigned long status;

	status = readl(sport->port.membase + UARTSR);

	if ((status & LINFLEXD_UARTSR_DRFRFE) && !sport->dma_rx_use)
			linflex_rxint(irq, dev_id);
	if ((status & LINFLEXD_UARTSR_DTFTFF) && !sport->dma_tx_use)
			linflex_txint(irq, dev_id);

	return IRQ_HANDLED;
}

/* return TIOCSER_TEMT when transmitter is not busy */
static unsigned int linflex_tx_empty(struct uart_port *port)
{
	unsigned long status;
	struct linflex_port *sport = container_of(port,
					struct linflex_port, port);

	status = readl(sport->port.membase + UARTSR) & LINFLEXD_UARTSR_DTFTFF;

	if (!sport->dma_tx_use)
		return status ? TIOCSER_TEMT : 0;
	else
		return status ? 0 : TIOCSER_TEMT;
}

static unsigned int linflex_get_mctrl(struct uart_port *port)
{
	return 0;
}

static void linflex_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
}
static void linflex_break_ctl(struct uart_port *port, int break_state)
{
}
static void linflex_setup_watermark(struct linflex_port *sport)
{
	unsigned long cr, ier, cr1;

	cr = readl(sport->port.membase + UARTCR);
	/* Disable transmission/reception */
	cr &= ~(LINFLEXD_UARTCR_RXEN | LINFLEXD_UARTCR_TXEN);

	writel(cr, sport->port.membase + UARTCR);

	/* Enter initialization mode by setting INIT bit */

	/* set the Linflex in master mode amd activate by-pass filter */
	cr1 = LINFLEXD_LINCR1_BF | LINFLEXD_LINCR1_MME
	      | LINFLEXD_LINCR1_INIT;
	writel(cr1, sport->port.membase + LINCR1);

	/* wait for init mode entry */
	while ((readl(sport->port.membase + LINSR)
		& LINFLEXD_LINSR_LINS_MASK)
		!= LINFLEXD_LINSR_LINS_INITMODE)
		;

	/*
		UART = 0x1;		- Linflex working in UART mode
		TXEN = 0x1;		- Enable transmission of data now
		RXEn = 0x1;		- Receiver enabled
		WL0 = 0x1;		- 8 bit data
		PCE = 0x0;		- No parity
	*/


	/* set UART bit to allow writing other bits */
	writel(LINFLEXD_UARTCR_UART, sport->port.membase + UARTCR);

	cr = (LINFLEXD_UARTCR_RXEN | LINFLEXD_UARTCR_TXEN |
	      LINFLEXD_UARTCR_WL0 | LINFLEXD_UARTCR_UART);

	/* FIFO mode enabled for DMA Rx mode. */
	if (sport->dma_rx_use)
		cr |= LINFLEXD_UARTCR_RFBM;

	/* FIFO mode enabled for DMA Tx mode. */
	if (sport->dma_tx_use)
		cr |= LINFLEXD_UARTCR_TFBM;

	writel(cr, sport->port.membase + UARTCR);

	cr1 &= ~(LINFLEXD_LINCR1_INIT);

	writel(cr1, sport->port.membase + LINCR1);

	ier = readl(sport->port.membase + LINIER);
	if (!sport->dma_rx_use)
		ier |= LINFLEXD_LINIER_DRIE;

	if (!sport->dma_tx_use)
		ier |= LINFLEXD_LINIER_DTIE;

	writel(ier, sport->port.membase + LINIER);
}


static int linflex_dma_tx_request(struct uart_port *port)
{
	struct linflex_port *sport = container_of(port,
					struct linflex_port, port);
	struct dma_slave_config dma_tx_sconfig;
	dma_addr_t dma_bus;
	unsigned char *dma_buf;
	int ret;

	dma_bus = dma_map_single(sport->dma_tx_chan->device->dev,
				sport->port.state->xmit.buf,
				UART_XMIT_SIZE, DMA_TO_DEVICE);

	if (dma_mapping_error(sport->dma_tx_chan->device->dev, dma_bus)) {
		dev_err(sport->port.dev, "dma_map_single tx failed\n");
		return -ENOMEM;
	}

	dma_buf = sport->port.state->xmit.buf;
	dma_tx_sconfig.dst_addr = sport->port.mapbase + BDRL;
	dma_tx_sconfig.dst_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	dma_tx_sconfig.dst_maxburst = 1;
	dma_tx_sconfig.direction = DMA_MEM_TO_DEV;
	ret = dmaengine_slave_config(sport->dma_tx_chan, &dma_tx_sconfig);

	if (ret < 0) {
		dev_err(sport->port.dev,
				"Dma slave config failed, err = %d\n", ret);
		return ret;
	}

	sport->dma_tx_buf_virt = dma_buf;
	sport->dma_tx_buf_bus = dma_bus;
	sport->dma_tx_in_progress = 0;

	return 0;
}

static int linflex_dma_rx_request(struct uart_port *port)
{
	struct linflex_port *sport = container_of(port,
					struct linflex_port, port);
	struct dma_slave_config dma_rx_sconfig;
	dma_addr_t dma_bus;
	unsigned char *dma_buf;
	int ret;

	dma_buf = devm_kzalloc(sport->port.dev,
				FSL_UART_RX_DMA_BUFFER_SIZE, GFP_KERNEL);

	if (!dma_buf) {
		dev_err(sport->port.dev, "Dma rx alloc failed\n");
		return -ENOMEM;
	}

	dma_bus = dma_map_single(sport->dma_rx_chan->device->dev, dma_buf,
				FSL_UART_RX_DMA_BUFFER_SIZE, DMA_FROM_DEVICE);

	if (dma_mapping_error(sport->dma_rx_chan->device->dev, dma_bus)) {
		dev_err(sport->port.dev, "dma_map_single rx failed\n");
		return -ENOMEM;
	}

	dma_rx_sconfig.src_addr = sport->port.mapbase + BDRM;
	dma_rx_sconfig.src_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	dma_rx_sconfig.src_maxburst = 1;
	dma_rx_sconfig.direction = DMA_DEV_TO_MEM;
	ret = dmaengine_slave_config(sport->dma_rx_chan, &dma_rx_sconfig);

	if (ret < 0) {
		dev_err(sport->port.dev,
				"Dma slave config failed, err = %d\n", ret);
		return ret;
	}

	sport->dma_rx_buf_virt = dma_buf;
	sport->dma_rx_buf_bus = dma_bus;
	sport->dma_rx_in_progress = 0;

	return 0;
}

static void linflex_dma_tx_free(struct uart_port *port)
{
	struct linflex_port *sport = container_of(port,
					struct linflex_port, port);

	dma_unmap_single(sport->port.dev, sport->dma_tx_buf_bus,
			UART_XMIT_SIZE, DMA_TO_DEVICE);

	sport->dma_tx_buf_bus = 0;
	sport->dma_tx_buf_virt = NULL;
}

static void linflex_dma_rx_free(struct uart_port *port)
{
	struct linflex_port *sport = container_of(port,
					struct linflex_port, port);

	dma_unmap_single(sport->port.dev, sport->dma_rx_buf_bus,
			FSL_UART_RX_DMA_BUFFER_SIZE, DMA_FROM_DEVICE);

	sport->dma_rx_buf_bus = 0;
	sport->dma_rx_buf_virt = NULL;
}

static int linflex_startup(struct uart_port *port)
{
	struct linflex_port *sport = container_of(port,
					struct linflex_port, port);
	int ret;
	unsigned long flags, temp;

	sport->txfifo_size = LINFLEXD_UARTCR_TXFIFO_SIZE;
	sport->rxfifo_size = LINFLEXD_UARTCR_RXFIFO_SIZE;
	sport->port.fifosize = sport->txfifo_size;

	if (sport->dma_rx_chan && !linflex_dma_rx_request(port)) {
		sport->dma_rx_use = true;
		temp = readl(port->membase + DMARXE);
		writel(temp | 0x1, port->membase + DMARXE);
		setup_timer(&sport->timer, linflex_timer_func,
				(unsigned long)sport);

		linflex_dma_rx(sport);
		sport->timer.expires = jiffies + sport->dma_rx_timeout;
		add_timer(&sport->timer);
	} else
		sport->dma_rx_use = false;

	if (sport->dma_tx_chan && !linflex_dma_tx_request(port)) {
		sport->dma_tx_use = true;
		temp = readl(port->membase + DMATXE);
		writel(temp | 0x1, port->membase + DMATXE);

	} else
		sport->dma_tx_use = false;

	if (!sport->dma_rx_use || !sport->dma_tx_use) {
		ret = devm_request_irq(port->dev, port->irq, linflex_int, 0,
						DRIVER_NAME, sport);
		if (ret)
			return ret;
	}

	spin_lock_irqsave(&sport->port.lock, flags);

	linflex_setup_watermark(sport);

	spin_unlock_irqrestore(&sport->port.lock, flags);
	return 0;
}

static void linflex_shutdown(struct uart_port *port)
{
	struct linflex_port *sport = container_of(port,
					struct linflex_port, port);
	unsigned long cr, ier;
	unsigned long flags, temp;

	spin_lock_irqsave(&port->lock, flags);

	/* disable Rx/Tx and interrupts*/
	ier = readl(port->membase + LINIER);

	writel(ier, port->membase + LINIER);

	cr = readl(port->membase + UARTCR);

	cr &= ~(LINFLEXD_UARTCR_RXEN |	LINFLEXD_UARTCR_TXEN);
	writel(ier, port->membase + UARTCR);

	spin_unlock_irqrestore(&port->lock, flags);

	if (!sport->dma_rx_use || !sport->dma_tx_use)
		devm_free_irq(port->dev, port->irq, sport);

	if (sport->dma_rx_use) {
		del_timer(&sport->timer);
		dmaengine_terminate_all(sport->dma_rx_chan);

		temp = readl(sport->port.membase + DMARXE);
		writel(temp & 0xFFFF0000, sport->port.membase + DMARXE);

		linflex_dma_rx_free(&sport->port);
		sport->dma_rx_in_progress = 0;
	}

	if (sport->dma_tx_use) {
		dmaengine_terminate_all(sport->dma_tx_chan);

		temp = readl(sport->port.membase + DMATXE);
		writel(temp & 0xFFFF0000, sport->port.membase + DMATXE);

		linflex_dma_tx_free(&sport->port);
		sport->dma_tx_in_progress = 0;
	}

}

static int
linflex_ldiv_multiplier(struct linflex_port *sport)
{
	unsigned int mul = LINFLEX_LDIV_MULTIPLIER;
	unsigned long cr;

	cr = readl(sport->port.membase + UARTCR);
	if (cr & LINFLEXD_UARTCR_ROSE)
		mul = LINFLEXD_UARTCR_OSR(cr);

	return mul;
}

static void
linflex_set_termios(struct uart_port *port, struct ktermios *termios,
			struct ktermios *old )
{
#ifndef CONFIG_S32V234_PALLADIUM
	/*on Palladium we trust the configuration provided by u-boot*/
	struct linflex_port *sport = container_of(port,
					struct linflex_port, port);
	unsigned long flags;
	unsigned long cr, old_cr, cr1, gcr;
	unsigned int  baud;
	unsigned int old_csize = old ? old->c_cflag & CSIZE : CS8;
	unsigned long ibr, fbr, divisr, dividr;

	cr = old_cr = readl(sport->port.membase + UARTCR);

	/* Enter initialization mode by setting INIT bit */
	cr1=readl(sport->port.membase + LINCR1);
	cr1 |= LINFLEXD_LINCR1_INIT;
	writel(cr1, sport->port.membase + LINCR1);

	/* wait for init mode entry */
	while ((readl(sport->port.membase + LINSR)
		& LINFLEXD_LINSR_LINS_MASK)
		!= LINFLEXD_LINSR_LINS_INITMODE)
		;

	/*
	 * only support CS8 and CS7, and for CS7 must enable PE.
	 * supported mode:
	 *	- (7,e/o,1)
	 *	- (8,n,1)
	 *	- (8,m/s,1)
	 *	- (8,e/o,1)
	 */
	/* enter the UART into configuration mode */

	while ((termios->c_cflag & CSIZE) != CS8 &&
	(termios->c_cflag & CSIZE) != CS7) {
		termios->c_cflag &= ~CSIZE;
		termios->c_cflag |= old_csize;
		old_csize = CS8;
	}

	if ((termios->c_cflag & CSIZE) == CS7) {
		/* Word length: WL1WL0:00 */
		cr = old_cr & ~LINFLEXD_UARTCR_WL1 & ~LINFLEXD_UARTCR_WL0;
	}

	if ((termios->c_cflag & CSIZE) == CS8) {
		/* Word length: WL1WL0:01 */
		cr = (old_cr | LINFLEXD_UARTCR_WL0)& ~LINFLEXD_UARTCR_WL1;
	}

	if (termios->c_cflag & CMSPAR) {
		if ((termios->c_cflag & CSIZE) != CS8) {
			termios->c_cflag &= ~CSIZE;
			termios->c_cflag |= CS8;
		}
		/* has a space/sticky bit */
		cr |= LINFLEXD_UARTCR_WL0;
	}

	gcr = readl(port->membase + GCR);

	if (termios->c_cflag & CSTOPB) {
		/* Use 2 stop bits. */
		cr = (cr & ~LINFLEXD_UARTCR_SBUR_MASK) |
			LINFLEXD_UARTCR_SBUR_2SBITS;
		/* Set STOP in GCR field for 2 stop bits. */
		gcr = (gcr & ~LINFLEXD_GCR_STOP_MASK) |
			LINFLEXD_GCR_STOP_2SBITS;
	} else {
		/* Use 1 stop bit. */
		cr = (cr & ~LINFLEXD_UARTCR_SBUR_MASK) |
			LINFLEXD_UARTCR_SBUR_1SBITS;
		/* Set STOP in GCR field for 1 stop bit. */
		gcr = (gcr & ~LINFLEXD_GCR_STOP_MASK) |
			LINFLEXD_GCR_STOP_1SBITS;
	}
	/* Update GCR register. */
	writel(gcr, port->membase + GCR);

	/* parity must be enabled when CS7 to match 8-bits format */
	if ((termios->c_cflag & CSIZE) == CS7)
		termios->c_cflag |= PARENB;

	if ((termios->c_cflag & PARENB)) {
		if (termios->c_cflag & CMSPAR) {
			cr &= ~LINFLEXD_UARTCR_PCE;

		} else {

			cr |= LINFLEXD_UARTCR_PCE;
			if ((termios->c_cflag & CSIZE) == CS8)
			{
				if (termios->c_cflag & PARODD)
					cr = (cr | LINFLEXD_UARTCR_PC0 )&(~LINFLEXD_UARTCR_PC1);
				else
					cr = cr & (~LINFLEXD_UARTCR_PC1 & ~LINFLEXD_UARTCR_PC0);
			}
		}
	}

	/* ask the core to calculate the divisor */
	baud = uart_get_baud_rate(port, termios, old, 50, port->uartclk / 16);

	spin_lock_irqsave(&sport->port.lock, flags);

	sport->port.read_status_mask = 0;

	if (termios->c_iflag & INPCK)
		sport->port.read_status_mask |=	(LINFLEXD_UARTSR_FEF | LINFLEXD_UARTSR_PE0|LINFLEXD_UARTSR_PE1
										|LINFLEXD_UARTSR_PE2|LINFLEXD_UARTSR_PE3);
	if (termios->c_iflag & (IGNBRK | BRKINT | PARMRK))
		sport->port.read_status_mask |= LINFLEXD_UARTSR_FEF;

	/* characters to ignore */
	sport->port.ignore_status_mask = 0;
	if (termios->c_iflag & IGNPAR)
		sport->port.ignore_status_mask |= LINFLEXD_UARTSR_PE;
	if (termios->c_iflag & IGNBRK) {
		sport->port.ignore_status_mask |= LINFLEXD_UARTSR_PE;
		/*
		 * if we're ignoring parity and break indicators,
		 * ignore overruns too (for real raw support).
		 */
		if (termios->c_iflag & IGNPAR)
			sport->port.ignore_status_mask |= LINFLEXD_UARTSR_BOF;
	}

	/* update the per-port timeout */
	uart_update_timeout(port, termios->c_cflag, baud);
	sport->dma_rx_timeout = msecs_to_jiffies(20);

	/* disable transmit and receive */
	writel(old_cr & ~(LINFLEXD_UARTCR_RXEN | LINFLEXD_UARTCR_TXEN),
		sport->port.membase + UARTCR);

	divisr = sport->port.uartclk;	//freq in Hz
	dividr = (baud * linflex_ldiv_multiplier(sport));

	ibr = divisr / dividr;
	fbr = ((divisr % dividr) * 16 / dividr) & 0xF;

	writel(ibr, sport->port.membase + LINIBRR);
	writel(fbr, sport->port.membase + LINFBRR);

	writel(cr, sport->port.membase + UARTCR);

	cr1 &= ~(LINFLEXD_LINCR1_INIT);

	writel(cr1, sport->port.membase + LINCR1);

	spin_unlock_irqrestore(&sport->port.lock, flags);
#endif

}

static const char *linflex_type(struct uart_port *port)
{
	return "FSL_LINFLEX";
}

static void linflex_release_port(struct uart_port *port)
{
	/* nothing to do */
}

static int linflex_request_port(struct uart_port *port)
{
	return 0;
}

/* configure/auto-configure the port */
static void linflex_config_port(struct uart_port *port, int flags)
{
	if (flags & UART_CONFIG_TYPE)
		port->type = PORT_LINFLEXUART;
}

static int linflex_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	int ret = 0;
	if (ser->type != PORT_UNKNOWN && ser->type != PORT_LINFLEXUART)
		ret = -EINVAL;
	if (port->irq != ser->irq)
		ret = -EINVAL;
	if (ser->io_type != UPIO_MEM)
		ret = -EINVAL;
	if (port->uartclk / 16 != ser->baud_base)
		ret = -EINVAL;
	if (port->iobase != ser->port)
		ret = -EINVAL;
	if (ser->hub6 != 0)
		ret = -EINVAL;
	return ret;
}

static struct uart_ops linflex_pops = {
	.tx_empty	= linflex_tx_empty,
	.set_mctrl	= linflex_set_mctrl,
	.get_mctrl	= linflex_get_mctrl,
	.stop_tx	= linflex_stop_tx,
	.start_tx	= linflex_start_tx,
	.stop_rx	= linflex_stop_rx,
	.break_ctl	= linflex_break_ctl,
	.startup	= linflex_startup,
	.shutdown	= linflex_shutdown,
	.set_termios	= linflex_set_termios,
	.type		= linflex_type,
	.request_port	= linflex_request_port,
	.release_port	= linflex_release_port,
	.config_port	= linflex_config_port,
	.verify_port	= linflex_verify_port,
	.flush_buffer	= linflex_flush_buffer,
};

static struct linflex_port *linflex_ports[UART_NR];

#ifdef CONFIG_SERIAL_FSL_LINFLEXUART_CONSOLE
static void linflex_console_putchar(struct uart_port *port, int ch)
{
	struct linflex_port *sport = container_of(port,
					struct linflex_port, port);

	writeb(ch, port->membase + BDRL);

	if (!sport->dma_tx_use)
		while ((readl(port->membase + UARTSR) &
					LINFLEXD_UARTSR_DTFTFF)
				!= LINFLEXD_UARTSR_DTFTFF)
			;
	else
		while (readl(port->membase + UARTSR) &
					LINFLEXD_UARTSR_DTFTFF)
			;

	if (!sport->dma_tx_use) {
		writel((readl(port->membase + UARTSR) |
					LINFLEXD_UARTSR_DTFTFF),
					port->membase + UARTSR);
	}
}
static void
linflex_console_write(struct console *co, const char *s, unsigned int count)
{
	struct linflex_port *sport = linflex_ports[co->index];
	unsigned long cr, ier, old_ier;

	/* First save CR2 and then disable interrupts. */
	ier = old_ier = readl(sport->port.membase + LINIER);
	if (!sport->dma_tx_use) {
		ier &= ~(LINFLEXD_LINIER_DTIE);
		writel(ier, sport->port.membase + LINIER);
	}

	cr = readl(sport->port.membase + UARTCR);
	cr |= (LINFLEXD_UARTCR_TXEN);
	writel(cr, sport->port.membase + UARTCR);

	uart_console_write(&sport->port, s, count, linflex_console_putchar);

	if (!sport->dma_tx_use)
		writel(old_ier, sport->port.membase + LINIER);
}

/*
 * if the port was already initialised (eg, by a boot loader),
 * try to determine the current setup.
 */
static void __init
linflex_console_get_options(struct linflex_port *sport, int *baud,
				int *parity, int *bits)
{
#ifndef CONFIG_S32V234_PALLADIUM
	/*unsigned char cr, bdh, bdl, brfa;*/
	unsigned long cr, fbr, ibr;
	unsigned int uartclk, baud_raw;

	prd_info("8\n");
	cr = readl(sport->port.membase + UARTCR);
	cr &= LINFLEXD_UARTCR_RXEN | LINFLEXD_UARTCR_TXEN;

	if (!cr)
		return;

	/* ok, the port was enabled */

	*parity = 'n';
	if (cr & LINFLEXD_UARTCR_PCE) {
		if (cr & LINFLEXD_UARTCR_PC0)
			*parity = 'o';
		else
			*parity = 'e';
	}

	if ((cr & LINFLEXD_UARTCR_WL0) && ((cr & LINFLEXD_UARTCR_WL1) == 0)) {
		if (cr & LINFLEXD_UARTCR_PCE)
			*bits = 9;
		else
			*bits = 8;
	}

	fbr = readl(sport->port.membase + LINFBRR);

	ibr = readl(sport->port.membase + LINIBRR);

	uartclk = clk_get_rate(sport->clk);

	baud_raw = uartclk / (linflex_ldiv_multiplier(sport) * ibr);

	if (*baud != baud_raw)
		printk(KERN_INFO "Serial: Console linflex rounded baud rate"
				"from %d to %d\n", baud_raw, *baud);
	#endif
}

static int __init linflex_console_setup(struct console *co, char *options)
{
	struct linflex_port *sport;
	int baud = 115200;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';
	/*
	 * check whether an invalid uart number has been specified, and
	 * if so, search for the first available port that does have
	 * console support.
	 */
	if (co->index == -1 || co->index >= ARRAY_SIZE(linflex_ports))
		co->index = 0;

	sport = linflex_ports[co->index];
	if (sport == NULL)
		return -ENODEV;

	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);
	else
		linflex_console_get_options(sport, &baud, &parity, &bits);

	linflex_setup_watermark(sport);

	return uart_set_options(&sport->port, co, baud, parity, bits, flow);
}

static struct uart_driver linflex_reg;
static struct console linflex_console = {
	.name		= DEV_NAME,
	.write		= linflex_console_write,
	.device		= uart_console_device,
	.setup		= linflex_console_setup,
	.flags		= CON_PRINTBUFFER,
	.index		= -1,
	.data		= &linflex_reg,
};

#define LINFLEX_CONSOLE	(&linflex_console)
#else
#define LINFLEX_CONSOLE	NULL
#endif

static struct uart_driver linflex_reg = {
	.owner		= THIS_MODULE,
	.driver_name	= DRIVER_NAME,
	.dev_name	= DEV_NAME,
	.nr		= ARRAY_SIZE(linflex_ports),
	.cons		= LINFLEX_CONSOLE,
};

static int linflex_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct linflex_port *sport;
	struct resource *res;
	int ret;

	sport = devm_kzalloc(&pdev->dev, sizeof(*sport), GFP_KERNEL);
	if (!sport)
		return -ENOMEM;

	pdev->dev.coherent_dma_mask = 0;

	ret = of_alias_get_id(np, "serial");
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to get alias id, errno %d\n", ret);
		return ret;
	}
	sport->port.line = ret;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;

	sport->port.mapbase = res->start;
	sport->port.membase = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(sport->port.membase))
		return PTR_ERR(sport->port.membase);

	sport->port.dev = &pdev->dev;
	sport->port.type = PORT_LINFLEXUART;
	sport->port.iotype = UPIO_MEM;
	sport->port.irq = platform_get_irq(pdev, 0);
	sport->port.ops = &linflex_pops;
	sport->port.flags = UPF_BOOT_AUTOCONF;
#ifndef CONFIG_S32V234_PALLADIUM
	sport->clk = devm_clk_get(&pdev->dev, "lin");
	if (IS_ERR(sport->clk)) {
		ret = PTR_ERR(sport->clk);
		dev_err(&pdev->dev, "failed to get uart clk: %d\n", ret);
		return ret;
	}

	ret = clk_prepare_enable(sport->clk);
	if (ret) {
		dev_err(&pdev->dev, "failed to enable uart clk: %d\n", ret);
		return ret;
	}

	sport->clk_ipg = devm_clk_get(&pdev->dev, "ipg");
	if (IS_ERR(sport->clk_ipg)) {
		ret = PTR_ERR(sport->clk_ipg);
		dev_err(&pdev->dev, "failed to get ipg uart clk: %d\n", ret);
		clk_disable_unprepare(sport->clk);
		return ret;
	}

	ret = clk_prepare_enable(sport->clk_ipg);
	if (ret) {
		clk_disable_unprepare(sport->clk);
		dev_err(&pdev->dev, "failed to enable ipg uart clk: %d\n", ret);
		return ret;
	}

	sport->port.uartclk = clk_get_rate(sport->clk);
#endif
	linflex_ports[sport->port.line] = sport;

	platform_set_drvdata(pdev, &sport->port);

	ret = uart_add_one_port(&linflex_reg, &sport->port);
	if (ret) {
		clk_disable_unprepare(sport->clk);
		clk_disable_unprepare(sport->clk_ipg);
		return ret;
	}

	sport->dma_tx_chan = dma_request_slave_channel(sport->port.dev, "tx");
	if (!sport->dma_tx_chan)
		dev_info(sport->port.dev,
			"DMA tx channel request failed, operating without tx DMA\n");

	sport->dma_rx_chan = dma_request_slave_channel(sport->port.dev, "rx");
	if (!sport->dma_rx_chan)
		dev_info(sport->port.dev,
			"DMA rx channel request failed, operating without rx DMA\n");

	sport->txfifo_size = LINFLEXD_UARTCR_TXFIFO_SIZE;
	sport->rxfifo_size = LINFLEXD_UARTCR_RXFIFO_SIZE;

	return 0;
}

static int linflex_remove(struct platform_device *pdev)
{
	struct linflex_port *sport = platform_get_drvdata(pdev);
	uart_remove_one_port(&linflex_reg, &sport->port);

	clk_disable_unprepare(sport->clk);
	clk_disable_unprepare(sport->clk_ipg);

	if (sport->dma_tx_chan)
		dma_release_channel(sport->dma_tx_chan);

	if (sport->dma_rx_chan)
		dma_release_channel(sport->dma_rx_chan);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int linflex_suspend(struct device *dev)
{
	struct linflex_port *sport = dev_get_drvdata(dev);
	uart_suspend_port(&linflex_reg, &sport->port);

	return 0;
}

static int linflex_resume(struct device *dev)
{
	struct linflex_port *sport = dev_get_drvdata(dev);
	uart_resume_port(&linflex_reg, &sport->port);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(linflex_pm_ops, linflex_suspend, linflex_resume);

static struct platform_driver linflex_driver = {
	.probe		= linflex_probe,
	.remove		= linflex_remove,
	.driver		= {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
		.of_match_table	= linflex_dt_ids,
		.pm	= &linflex_pm_ops,
	},
};

static int __init linflex_serial_init(void)
{
	int ret;

	prd_info("serial: Freescale linflex driver\n");
	ret = uart_register_driver(&linflex_reg);
	if (ret)
		return ret;

	ret = platform_driver_register(&linflex_driver);
	if (ret)
		uart_unregister_driver(&linflex_reg);

	return ret;
}

static void __exit linflex_serial_exit(void)
{
	platform_driver_unregister(&linflex_driver);
	uart_unregister_driver(&linflex_reg);
}

module_init(linflex_serial_init);
module_exit(linflex_serial_exit);

MODULE_DESCRIPTION("Freescale linflex serial port driver");
MODULE_LICENSE("GPL v2");
