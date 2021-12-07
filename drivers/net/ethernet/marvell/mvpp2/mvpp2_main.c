// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for Marvell PPv2 network controller for Armada 375 SoC.
 *
 * Copyright (C) 2014 Marvell
 *
 * Marcin Wojtas <mw@semihalf.com>
 */

#include <linux/acpi.h>
#include <linux/dma-direct.h>
#include <linux/kernel.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/platform_device.h>
#include <linux/skbuff.h>
#include <linux/inetdevice.h>
#include <linux/mbus.h>
#include <linux/module.h>
#include <linux/mfd/syscon.h>
#include <linux/if_vlan.h>
#include <linux/interrupt.h>
#include <linux/cpumask.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_mdio.h>
#include <linux/of_net.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_reserved_mem.h>
#include <linux/genalloc.h>
#include <linux/phy.h>
#include <linux/phylink.h>
#include <linux/phy/phy.h>
#include <linux/clk.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/regmap.h>
#include <uapi/linux/ppp_defs.h>
#include <net/dsa.h>
#include <net/ip.h>
#include <net/ipv6.h>
#include <net/tso.h>
#include <net/busy_poll.h>

#include "mvpp2.h"
#include "mvpp2_prs.h"
#include "mvpp2_cls.h"

/* RX-TX fast-forwarding path optimization */
#define MVPP2_RXTX_HASH			0xbac0
#define MVPP2_RXTX_HASH_CONST_MASK	0xfff0
#define MVPP2_RXTX_HASH_BMID_MASK	0xf
/* HashBits[31..16] contain skb->head[22..7], the head is aligned and [6..0]=0,
 * so skb->head is shifted left for (16-7) bits.
 * This hash permits to detect 2 non-recyclable cases:
 * - new skb with old hash inside
 * - same skb but NET-stack has replaced the data-buffer with another one
 */
#define MVPP2_HEAD_HASH_SHIFT	(16 - 7)
#define MVPP2_RXTX_HASH_GENER(skb, bm_pool_id) \
	(((u32)(phys_addr_t)skb->head << MVPP2_HEAD_HASH_SHIFT) | \
					MVPP2_RXTX_HASH | bm_pool_id)
#define MVPP2_RXTX_HASH_IS_OK(skb, hash) \
	(MVPP2_RXTX_HASH_GENER(skb, 0) == (hash & ~MVPP2_RXTX_HASH_BMID_MASK))
#define MVPP2_RXTX_HASH_IS_OK_TX(skb, hash) \
	(((((u32)(phys_addr_t)skb->head << MVPP2_HEAD_HASH_SHIFT) | \
			MVPP2_RXTX_HASH) ^ hash) <= MVPP2_RXTX_HASH_BMID_MASK)

/* The recycle pool size should be "effectively big" but limited (to eliminate
 * memory-wasting on TX-pick). It should be >8 (Net-stack-forwarding-buffer)
 * and >pkt-coalescing. For "effective" >=NAPI_POLL_WEIGHT.
 * For 4 ports we need more buffers but not x4, statistically it is enough x3.
 * SKB-pool is shared for Small/Large/Jumbo buffers so we need more SKBs,
 * statistically it is enough x5.
 */
#define MVPP2_RECYCLE_FULL	(NAPI_POLL_WEIGHT * 3)
#define MVPP2_RECYCLE_FULL_SKB	(NAPI_POLL_WEIGHT * 5)

struct mvpp2_recycle_pool {
	void *pbuf[MVPP2_RECYCLE_FULL_SKB];
};

struct mvpp2_recycle_pcpu {
	/* All pool-indexes are in 1 cache-line */
	short int idx[MVPP2_BM_POOLS_NUM_MAX];
	/* BM/SKB-buffer pools */
	struct mvpp2_recycle_pool pool[MVPP2_BM_POOLS_NUM_MAX];
} __aligned(L1_CACHE_BYTES);

struct mvpp2_share {
	struct mvpp2_recycle_pcpu *recycle;
	void *recycle_base;

	/* Counters set by Probe/Init/Open */
	int num_open_ports;
} __aligned(L1_CACHE_BYTES);

struct mvpp2_share mvpp2_share;

static inline void mvpp2_recycle_put(struct mvpp2_port *port,
				     struct mvpp2_txq_pcpu *txq_pcpu,
				     struct mvpp2_txq_pcpu_buf *tx_buf);

static void mvpp2_tx_done_guard_force_irq(struct mvpp2_port *port,
					  int sw_thread, u8 to_zero_map);
static inline void mvpp2_tx_done_guard_timer_set(struct mvpp2_port *port,
						 int sw_thread);
static u32 mvpp2_tx_done_guard_get_stats(struct mvpp2_port *port, int cpu);

/* The prototype is added here to be used in start_dev when using ACPI. This
 * will be removed once phylink is used for all modes (dt+ACPI).
 */
static void mvpp2_mac_config(struct phylink_config *config, unsigned int mode,
			     const struct phylink_link_state *state);
static void mvpp2_mac_link_up(struct phylink_config *config, unsigned int mode,
			      phy_interface_t interface, struct phy_device *phy);

/* Branch prediction switches */
DEFINE_STATIC_KEY_FALSE(mvpp21_variant);
DEFINE_STATIC_KEY_FALSE(mvpp2_recycle_ena);

/* Queue modes */
#define MVPP2_QDIST_SINGLE_MODE	0
#define MVPP2_QDIST_MULTI_MODE	1

static int queue_mode = MVPP2_QDIST_MULTI_MODE;
static int tx_fifo_protection;
static int bm_underrun_protect = 1;
static int recycle;
static u32 tx_fifo_map;

module_param(queue_mode, int, 0444);
MODULE_PARM_DESC(queue_mode, "Set queue_mode (single=0, multi=1)");

module_param(tx_fifo_protection, int, 0444);
MODULE_PARM_DESC(tx_fifo_protection, "Set tx_fifo_protection (off=0, on=1)");

module_param(bm_underrun_protect, int, 0444);
MODULE_PARM_DESC(bm_underrun_protect, "Set BM underrun protect feature (0-1), def=1");

module_param(recycle, int, 0444);
MODULE_PARM_DESC(recycle, "Recycle: 0:disable(default), >=1:enable");

module_param(tx_fifo_map, uint, 0444);
MODULE_PARM_DESC(tx_fifo_map, "Set PPv2 TX FIFO ports map");

static dma_addr_t mvpp2_txdesc_dma_addr_get(struct mvpp2_port *port,
					    struct mvpp2_tx_desc *tx_desc)
{
	if (static_branch_unlikely(&mvpp21_variant))
		return le32_to_cpu(tx_desc->pp21.buf_dma_addr);
	else
		return le64_to_cpu(tx_desc->pp22.buf_dma_addr_ptp) &
		       MVPP2_DESC_DMA_MASK;
}

static void mvpp2_txdesc_dma_addr_set(struct mvpp2_port *port,
				      struct mvpp2_tx_desc *tx_desc,
				      dma_addr_t dma_addr)
{
	dma_addr_t addr, offset;

	addr = dma_addr & ~MVPP2_TX_DESC_ALIGN;
	offset = dma_addr & MVPP2_TX_DESC_ALIGN;

	if (static_branch_unlikely(&mvpp21_variant)) {
		tx_desc->pp21.buf_dma_addr = cpu_to_le32(addr);
		tx_desc->pp21.packet_offset = offset;
	} else {
		__le64 val = cpu_to_le64(addr);

		tx_desc->pp22.buf_dma_addr_ptp &= ~cpu_to_le64(MVPP2_DESC_DMA_MASK);
		tx_desc->pp22.buf_dma_addr_ptp |= val;
		tx_desc->pp22.packet_offset = offset;
	}
}

static size_t mvpp2_txdesc_size_get(struct mvpp2_port *port,
				    struct mvpp2_tx_desc *tx_desc)
{
	if (static_branch_unlikely(&mvpp21_variant))
		return le16_to_cpu(tx_desc->pp21.data_size);
	else
		return le16_to_cpu(tx_desc->pp22.data_size);
}

static void mvpp2_txdesc_size_set(struct mvpp2_port *port,
				  struct mvpp2_tx_desc *tx_desc,
				  size_t size)
{
	if (static_branch_unlikely(&mvpp21_variant))
		tx_desc->pp21.data_size = cpu_to_le16(size);
	else
		tx_desc->pp22.data_size = cpu_to_le16(size);
}

static void mvpp2_txdesc_txq_set(struct mvpp2_port *port,
				 struct mvpp2_tx_desc *tx_desc,
				 unsigned int txq)
{
	if (static_branch_unlikely(&mvpp21_variant))
		tx_desc->pp21.phys_txq = txq;
	else
		tx_desc->pp22.phys_txq = txq;
}

static void mvpp2_txdesc_cmd_set(struct mvpp2_port *port,
				 struct mvpp2_tx_desc *tx_desc,
				 unsigned int command)
{
	if (static_branch_unlikely(&mvpp21_variant))
		tx_desc->pp21.command = cpu_to_le32(command);
	else
		tx_desc->pp22.command = cpu_to_le32(command);
}

static unsigned int mvpp2_txdesc_offset_get(struct mvpp2_port *port,
					    struct mvpp2_tx_desc *tx_desc)
{
	if (static_branch_unlikely(&mvpp21_variant))
		return tx_desc->pp21.packet_offset;
	else
		return tx_desc->pp22.packet_offset;
}

static dma_addr_t mvpp2_rxdesc_dma_addr_get(struct mvpp2_port *port,
					    struct mvpp2_rx_desc *rx_desc)
{
	if (static_branch_unlikely(&mvpp21_variant))
		return le32_to_cpu(rx_desc->pp21.buf_dma_addr);
	else
		return le64_to_cpu(rx_desc->pp22.buf_dma_addr_key_hash) &
		       MVPP2_DESC_DMA_MASK;
}

static size_t mvpp2_rxdesc_size_get(struct mvpp2_port *port,
				    struct mvpp2_rx_desc *rx_desc)
{
	if (static_branch_unlikely(&mvpp21_variant))
		return le16_to_cpu(rx_desc->pp21.data_size);
	else
		return le16_to_cpu(rx_desc->pp22.data_size);
}

static u32 mvpp2_rxdesc_status_get(struct mvpp2_port *port,
				   struct mvpp2_rx_desc *rx_desc)
{
	if (static_branch_unlikely(&mvpp21_variant))
		return le32_to_cpu(rx_desc->pp21.status);
	else
		return le32_to_cpu(rx_desc->pp22.status);
}

static void mvpp2_txq_inc_get(struct mvpp2_txq_pcpu *txq_pcpu)
{
	txq_pcpu->txq_get_index++;
	if (txq_pcpu->txq_get_index == txq_pcpu->size)
		txq_pcpu->txq_get_index = 0;
}

static void mvpp2_txq_inc_put(struct mvpp2_port *port,
			      struct mvpp2_txq_pcpu *txq_pcpu,
			      void *skb_or_tso_mark,
			      struct mvpp2_tx_desc *tx_desc)
{
	struct mvpp2_txq_pcpu_buf *tx_buf =
		txq_pcpu->buffs + txq_pcpu->txq_put_index;
	tx_buf->skb = (struct sk_buff *)skb_or_tso_mark;
	tx_buf->size = mvpp2_txdesc_size_get(port, tx_desc);
	tx_buf->dma = mvpp2_txdesc_dma_addr_get(port, tx_desc) +
		mvpp2_txdesc_offset_get(port, tx_desc);
	txq_pcpu->txq_put_index++;
	if (txq_pcpu->txq_put_index == txq_pcpu->size)
		txq_pcpu->txq_put_index = 0;
}

/* Get number of physical egress port */
static inline int mvpp2_egress_port(struct mvpp2_port *port)
{
	return MVPP2_MAX_TCONT + port->id;
}

/* Get number of physical TXQ */
static inline int mvpp2_txq_phys(int port, int txq)
{
	return (MVPP2_MAX_TCONT + port) * MVPP2_MAX_TXQ + txq;
}

static void *mvpp2_frag_alloc(const struct mvpp2_bm_pool *pool)
{
	if (likely(pool->frag_size <= PAGE_SIZE))
		return netdev_alloc_frag(pool->frag_size);
	else
		return kmalloc(pool->frag_size, GFP_ATOMIC);
}

static void mvpp2_frag_free(const struct mvpp2_bm_pool *pool, void *data)
{
	if (likely(pool->frag_size <= PAGE_SIZE))
		skb_free_frag(data);
	else
		kfree(data);
}

/* Buffer Manager configuration routines */

/* Get default packet size for given BM pool type */
static int mvpp2_bm_pool_default_pkt_size(enum mvpp2_bm_pool_type bm_pool_type)
{
	switch (bm_pool_type) {
	case MVPP2_BM_SHORT:
		return MVPP2_BM_SHORT_PKT_SIZE;
	case MVPP2_BM_JUMBO:
		return MVPP2_BM_JUMBO_PKT_SIZE;
	case MVPP2_BM_LONG:
		return MVPP2_BM_LONG_PKT_SIZE;
	default:
		return -EINVAL;
	}
}

/* Get default buffer count for given BM pool type */
static int mvpp2_bm_pool_default_buf_num(enum mvpp2_bm_pool_type bm_pool_type)
{
	switch (bm_pool_type) {
	case MVPP2_BM_SHORT:
		return MVPP2_BM_SHORT_BUF_NUM;
	case MVPP2_BM_JUMBO:
		return MVPP2_BM_JUMBO_BUF_NUM;
	case MVPP2_BM_LONG:
		return MVPP2_BM_LONG_BUF_NUM;
	default:
		return -EINVAL;
	}
}

/* Get BM pool type mapping - return the hardware Buffer Manager pools
 * type according to the mapping to its ID:
 * POOL#0 - short packets
 * POOL#1 - jumbo packets
 * POOL#2 - long packets
 * In case the KS recycling feature is enabled, ID = 2 is
 * the first (CPU#0) out of the per-CPU pools for long packets.
 */
static enum mvpp2_bm_pool_type mvpp2_bm_pool_get_type(int id)
{
	switch (id) {
	case 0:
		return MVPP2_BM_SHORT;
	case 1:
		return MVPP2_BM_JUMBO;
	case 2:
		return MVPP2_BM_LONG;
	default:
		if (recycle)
			return MVPP2_BM_LONG;
		return -EINVAL;
	}
}

/* Get BM pool ID mapping - return the hardware Buffer Manager pools
 * ID according to the mapping to its type:
 * Short packets - POOL#0
 * Jumbo packets - POOL#1
 * Long packets - POOL#2
 * In case the KS recycling feature is enabled, ID = 2 is
 * the first (CPU#0) out of the per-CPU pools for long packets.
 */
static int mvpp2_bm_pool_get_id(enum mvpp2_bm_pool_type bm_pool_type)
{
	switch (bm_pool_type) {
	case MVPP2_BM_SHORT:
		return 0;
	case MVPP2_BM_JUMBO:
		return 1;
	case MVPP2_BM_LONG:
		return 2;
	default:
		return -EINVAL;
	}
}

/* Create pool */
static int mvpp2_bm_pool_create(struct platform_device *pdev,
				struct mvpp2 *priv,
				struct mvpp2_bm_pool *bm_pool, int size)
{
	u32 val;

	/* Number of buffer pointers must be a multiple of 16, as per
	 * hardware constraints
	 */
	if (!IS_ALIGNED(size, 16))
		return -EINVAL;

	/* PPv2.1 needs 8 bytes per buffer pointer, PPv2.2 and PPv2.3 needs 16
	 * bytes per buffer pointer
	 */
	if (priv->hw_version == MVPP21)
		bm_pool->size_bytes = 2 * sizeof(u32) * size;
	else
		bm_pool->size_bytes = 2 * sizeof(u64) * size;

	bm_pool->virt_addr = dma_alloc_coherent(&pdev->dev, bm_pool->size_bytes,
						&bm_pool->dma_addr,
						GFP_KERNEL);
	if (!bm_pool->virt_addr)
		return -ENOMEM;

	if (!IS_ALIGNED((unsigned long)bm_pool->virt_addr,
			MVPP2_BM_POOL_PTR_ALIGN)) {
		dma_free_coherent(&pdev->dev, bm_pool->size_bytes,
				  bm_pool->virt_addr, bm_pool->dma_addr);
		dev_err(&pdev->dev, "BM pool %d is not %d bytes aligned\n",
			bm_pool->id, MVPP2_BM_POOL_PTR_ALIGN);
		return -ENOMEM;
	}

	mvpp2_write(priv, MVPP2_BM_POOL_BASE_REG(bm_pool->id),
		    lower_32_bits(bm_pool->dma_addr));
	mvpp2_write(priv, MVPP2_BM_POOL_SIZE_REG(bm_pool->id), size);

	val = mvpp2_read(priv, MVPP2_BM_POOL_CTRL_REG(bm_pool->id));
	val |= MVPP2_BM_START_MASK;

	val &= ~MVPP2_BM_LOW_THRESH_MASK;
	val &= ~MVPP2_BM_HIGH_THRESH_MASK;

	/* Set 8 Pools BPPI threshold if BM underrun protection feature
	 * were enabled
	 */
	if (priv->hw_version == MVPP23 && bm_underrun_protect) {
		val |= MVPP2_BM_LOW_THRESH_VALUE(MVPP23_BM_BPPI_LOW_THRESH);
		val |= MVPP2_BM_HIGH_THRESH_VALUE(MVPP23_BM_BPPI_HIGH_THRESH);
	} else {
		val |= MVPP2_BM_LOW_THRESH_VALUE(MVPP2_BM_BPPI_LOW_THRESH);
		val |= MVPP2_BM_HIGH_THRESH_VALUE(MVPP2_BM_BPPI_HIGH_THRESH);
	}

	mvpp2_write(priv, MVPP2_BM_POOL_CTRL_REG(bm_pool->id), val);

	bm_pool->size = size;
	bm_pool->pkt_size = 0;
	bm_pool->buf_num = 0;
	bm_pool->type = mvpp2_bm_pool_get_type(bm_pool->id);

	return 0;
}

/* Set pool buffer size */
static void mvpp2_bm_pool_bufsize_set(struct mvpp2 *priv,
				      struct mvpp2_bm_pool *bm_pool,
				      int buf_size)
{
	u32 val;

	bm_pool->buf_size = buf_size;

	val = ALIGN(buf_size, 1 << MVPP2_POOL_BUF_SIZE_OFFSET);
	mvpp2_write(priv, MVPP2_POOL_BUF_SIZE_REG(bm_pool->id), val);
}

static void mvpp2_bm_bufs_get_addrs(struct device *dev, struct mvpp2 *priv,
				    struct mvpp2_bm_pool *bm_pool,
				    dma_addr_t *dma_addr,
				    phys_addr_t *phys_addr)
{
	unsigned int thread = mvpp2_cpu_to_thread(priv, get_cpu());

	*dma_addr = mvpp2_thread_read(priv, thread,
				      MVPP2_BM_PHY_ALLOC_REG(bm_pool->id));

	if (priv->hw_version != MVPP21 && sizeof(dma_addr_t) == 8) {
		u32 val;
		u32 dma_addr_highbits;

		val = mvpp2_thread_read(priv, thread, MVPP22_BM_ADDR_HIGH_ALLOC);
		dma_addr_highbits = (val & MVPP22_BM_ADDR_HIGH_PHYS_MASK);
		*dma_addr |= (u64)dma_addr_highbits << 32;
	}
	*phys_addr = dma_to_phys(dev, *dma_addr);

	put_cpu();
}

/* Free all buffers from the pool */
static void mvpp2_bm_bufs_free(struct device *dev, struct mvpp2 *priv,
			       struct mvpp2_bm_pool *bm_pool, int buf_num)
{
	int i;

	if (buf_num > bm_pool->buf_num) {
		WARN(1, "Pool does not have so many bufs pool(%d) bufs(%d)\n",
		     bm_pool->id, buf_num);
		buf_num = bm_pool->buf_num;
	}

	for (i = 0; i < buf_num; i++) {
		dma_addr_t buf_dma_addr;
		phys_addr_t buf_phys_addr;
		void *data;

		mvpp2_bm_bufs_get_addrs(dev, priv, bm_pool,
					&buf_dma_addr, &buf_phys_addr);

		dma_unmap_single(dev, buf_dma_addr,
				 bm_pool->buf_size, DMA_FROM_DEVICE);

		data = (void *)phys_to_virt(buf_phys_addr);
		if (!data)
			break;

		mvpp2_frag_free(bm_pool, data);
	}

	/* Update BM driver with number of buffers removed from pool */
	bm_pool->buf_num -= i;
}

/* Check number of buffers in BM pool */
static int mvpp2_check_hw_buf_num(struct mvpp2 *priv, struct mvpp2_bm_pool *bm_pool)
{
	int buf_num = 0;

	buf_num += mvpp2_read(priv, MVPP2_BM_POOL_PTRS_NUM_REG(bm_pool->id)) &
				    MVPP22_BM_POOL_PTRS_NUM_MASK;
	buf_num += mvpp2_read(priv, MVPP2_BM_BPPI_PTRS_NUM_REG(bm_pool->id)) &
				    MVPP2_BM_BPPI_PTR_NUM_MASK;

	/* HW has one buffer ready which is not reflected in the counters */
	if (buf_num)
		buf_num += 1;

	return buf_num;
}

/* Cleanup pool */
static int mvpp2_bm_pool_destroy(struct platform_device *pdev,
				 struct mvpp2 *priv,
				 struct mvpp2_bm_pool *bm_pool)
{
	int buf_num;
	u32 val;

	buf_num = mvpp2_check_hw_buf_num(priv, bm_pool);
	mvpp2_bm_bufs_free(&pdev->dev, priv, bm_pool, buf_num);

	/* Check buffer counters after free */
	buf_num = mvpp2_check_hw_buf_num(priv, bm_pool);
	if (buf_num) {
		WARN(1, "cannot free all buffers in pool %d, buf_num left %d\n",
		     bm_pool->id, bm_pool->buf_num);
		return 0;
	}

	val = mvpp2_read(priv, MVPP2_BM_POOL_CTRL_REG(bm_pool->id));
	val |= MVPP2_BM_STOP_MASK;
	mvpp2_write(priv, MVPP2_BM_POOL_CTRL_REG(bm_pool->id), val);

	dma_free_coherent(&pdev->dev, bm_pool->size_bytes,
			  bm_pool->virt_addr,
			  bm_pool->dma_addr);
	return 0;
}

static int mvpp2_bm_pools_init(struct platform_device *pdev,
			       struct mvpp2 *priv)
{
	int i, err, size, cpu;
	struct mvpp2_bm_pool *bm_pool;

	if (recycle) {
		/* Allocate per-CPU long pools array */
		priv->pools_pcpu = devm_kcalloc(&pdev->dev, num_present_cpus(),
						sizeof(*priv->pools_pcpu),
						GFP_KERNEL);
		if (!priv->pools_pcpu)
			return -ENOMEM;
	}

	/* Initialize Virtual with 0x0 */
	for_each_present_cpu(cpu)
		mvpp2_thread_write(priv, cpu, MVPP2_BM_VIRT_RLS_REG, 0x0);

	/* Create all pools with maximum size */
	size = MVPP2_BM_POOL_SIZE_MAX;
	for (i = 0; i < MVPP2_BM_POOLS_NUM; i++) {
		bm_pool = &priv->bm_pools[i];
		bm_pool->id = i;
		err = mvpp2_bm_pool_create(pdev, priv, bm_pool, size);
		if (err)
			goto err_unroll_pools;
		mvpp2_bm_pool_bufsize_set(priv, bm_pool, 0);
	}
	return 0;

err_unroll_pools:
	dev_err(&pdev->dev, "failed to create BM pool %d, size %d\n", i, size);
	for (i = i - 1; i >= 0; i--)
		mvpp2_bm_pool_destroy(pdev, priv, &priv->bm_pools[i]);
	return err;
}

/* Routine enable PPv23 8 pool mode */
static void mvpp23_bm_set_8pool_mode(struct mvpp2 *priv)
{
	int val;

	val = mvpp2_read(priv, MVPP22_BM_POOL_BASE_ADDR_HIGH_REG);
	val |= MVPP23_BM_8POOL_MODE;
	mvpp2_write(priv, MVPP22_BM_POOL_BASE_ADDR_HIGH_REG, val);
}

/* Cleanup pool before actual initialization in the OS */
static void mvpp2_bm_pool_cleanup(struct mvpp2 *priv, int pool_id)
{
	u32 val;
	int i;

	/* Drain the BM from all possible residues left by firmware */
	for (i = 0; i < MVPP2_BM_POOL_SIZE_MAX; i++)
		mvpp2_read(priv, MVPP2_BM_PHY_ALLOC_REG(pool_id));

	/* Stop the BM pool */
	val = mvpp2_read(priv, MVPP2_BM_POOL_CTRL_REG(pool_id));
	val |= MVPP2_BM_STOP_MASK;
	mvpp2_write(priv, MVPP2_BM_POOL_CTRL_REG(pool_id), val);

	/* Mask BM all interrupts */
	mvpp2_write(priv, MVPP2_BM_INTR_MASK_REG(pool_id), 0);
	/* Clear BM cause register */
	mvpp2_write(priv, MVPP2_BM_INTR_CAUSE_REG(pool_id), 0);
}

static int mvpp2_bm_init(struct platform_device *pdev, struct mvpp2 *priv)
{
	int i, err;

	for (i = 0; i < MVPP2_BM_POOLS_NUM; i++) {
		/* Make sure about the pool state in case it was
		 * used by firmware.
		 */
		mvpp2_bm_pool_cleanup(priv, i);
	}

	/* Allocate and initialize BM pools */
	priv->bm_pools = devm_kcalloc(&pdev->dev, MVPP2_BM_POOLS_NUM,
				      sizeof(*priv->bm_pools), GFP_KERNEL);
	if (!priv->bm_pools)
		return -ENOMEM;

	if (priv->hw_version == MVPP23 && bm_underrun_protect)
		mvpp23_bm_set_8pool_mode(priv);

	err = mvpp2_bm_pools_init(pdev, priv);
	if (err < 0)
		return err;
	return 0;
}

/* Attach long pool to rxq */
static void mvpp2_rxq_long_pool_set(struct mvpp2_port *port,
				    int lrxq, int long_pool)
{
	u32 val, mask;
	int prxq;

	/* Get queue physical ID */
	prxq = port->rxqs[lrxq]->id;

	if (port->priv->hw_version == MVPP21)
		mask = MVPP21_RXQ_POOL_LONG_MASK;
	else
		mask = MVPP22_RXQ_POOL_LONG_MASK;

	val = mvpp2_read(port->priv, MVPP2_RXQ_CONFIG_REG(prxq));
	val &= ~mask;
	val |= (long_pool << MVPP2_RXQ_POOL_LONG_OFFS) & mask;
	mvpp2_write(port->priv, MVPP2_RXQ_CONFIG_REG(prxq), val);
}

/* Attach short pool to rxq */
static void mvpp2_rxq_short_pool_set(struct mvpp2_port *port,
				     int lrxq, int short_pool)
{
	u32 val, mask;
	int prxq;

	/* Get queue physical ID */
	prxq = port->rxqs[lrxq]->id;

	if (port->priv->hw_version == MVPP21)
		mask = MVPP21_RXQ_POOL_SHORT_MASK;
	else
		mask = MVPP22_RXQ_POOL_SHORT_MASK;

	val = mvpp2_read(port->priv, MVPP2_RXQ_CONFIG_REG(prxq));
	val &= ~mask;
	val |= (short_pool << MVPP2_RXQ_POOL_SHORT_OFFS) & mask;
	mvpp2_write(port->priv, MVPP2_RXQ_CONFIG_REG(prxq), val);
}

static dma_addr_t mvpp2_buf_alloc(struct mvpp2_port *port,
				  struct mvpp2_bm_pool *bm_pool,
				  gfp_t gfp_mask)
{
	dma_addr_t dma_addr;
	void *data;

	data = mvpp2_frag_alloc(bm_pool);
	if (!data)
		return (dma_addr_t)data;

	dma_addr = dma_map_single(port->dev->dev.parent, data,
				  bm_pool->buf_size, DMA_FROM_DEVICE);
	if (unlikely(dma_mapping_error(port->dev->dev.parent, dma_addr))) {
		mvpp2_frag_free(bm_pool, data);
		dma_addr = 0;
	}
	return dma_addr;
}

/* Routine calculate single queue shares address space */
static int mvpp22_calc_shared_addr_space(struct mvpp2_port *port)
{
	/* If number of CPU's greater than number of threads, return last
	 * address space
	 */
	if (num_active_cpus() >= MVPP2_MAX_THREADS)
		return MVPP2_MAX_THREADS - 1;

	return num_active_cpus();
}

/* Routine enable flow control for RXQs conditon */
void mvpp2_rxq_enable_fc(struct mvpp2_port *port)
{
	int val, cm3_state, host_id, q;
	int fq = port->first_rxq;
	unsigned long flags;

	spin_lock_irqsave(&port->priv->mss_spinlock, flags);

	/* Remove Flow control enable bit to prevent race between FW and Kernel
	 * If Flow control were enabled, it would be re-enabled.
	 */
	val = mvpp2_cm3_read(port->priv, MSS_FC_COM_REG);
	cm3_state = (val & FLOW_CONTROL_ENABLE_BIT);
	val &= ~FLOW_CONTROL_ENABLE_BIT;
	mvpp2_cm3_write(port->priv, MSS_FC_COM_REG, val);

	/* Set same Flow control for all RXQs */
	for (q = 0; q < port->nrxqs; q++) {
		/* Set stop and start Flow control RXQ thresholds */
		val = MSS_THRESHOLD_START;
		val |= (MSS_THRESHOLD_STOP << MSS_RXQ_TRESH_STOP_OFFS);
		mvpp2_cm3_write(port->priv, MSS_RXQ_TRESH_REG(q, fq), val);

		val = mvpp2_cm3_read(port->priv, MSS_RXQ_ASS_REG(q, fq));
		/* Set RXQ port ID */
		val &= ~(MSS_RXQ_ASS_PORTID_MASK << MSS_RXQ_ASS_Q_BASE(q, fq));
		val |= (port->id << MSS_RXQ_ASS_Q_BASE(q, fq));
		val &= ~(MSS_RXQ_ASS_HOSTID_MASK << (MSS_RXQ_ASS_Q_BASE(q, fq)
			+ MSS_RXQ_ASS_HOSTID_OFFS));

		/* Calculate RXQ host ID:
		 * In Single queue mode: Host ID equal to Host ID used for
		 *			 shared RX interrupt
		 * In Multi queue mode: Host ID equal to number of
		 *			RXQ ID / number of CoS queues
		 * In Single resource mode: Host ID always equal to 0
		 */
		if (queue_mode == MVPP2_QDIST_SINGLE_MODE)
			host_id = mvpp22_calc_shared_addr_space(port);
		else if (queue_mode == MVPP2_QDIST_MULTI_MODE)
			host_id = q;
		else
			host_id = 0;

		/* Set RXQ host ID */
		val |= (host_id << (MSS_RXQ_ASS_Q_BASE(q, fq)
			+ MSS_RXQ_ASS_HOSTID_OFFS));

		mvpp2_cm3_write(port->priv, MSS_RXQ_ASS_REG(q, fq), val);
	}

	/* Notify Firmware that Flow control config space ready for update */
	val = mvpp2_cm3_read(port->priv, MSS_FC_COM_REG);
	val |= FLOW_CONTROL_UPDATE_COMMAND_BIT;
	val |= cm3_state;
	mvpp2_cm3_write(port->priv, MSS_FC_COM_REG, val);

	spin_unlock_irqrestore(&port->priv->mss_spinlock, flags);
}

/* Routine disable flow control for RXQs conditon */
void mvpp2_rxq_disable_fc(struct mvpp2_port *port)
{
	int val, cm3_state, q;
	unsigned long flags;
	int fq = port->first_rxq;

	spin_lock_irqsave(&port->priv->mss_spinlock, flags);

	/* Remove Flow control enable bit to prevent race between FW and Kernel
	 * If Flow control were enabled, it would be re-enabled.
	 */
	val = mvpp2_cm3_read(port->priv, MSS_FC_COM_REG);
	cm3_state = (val & FLOW_CONTROL_ENABLE_BIT);
	val &= ~FLOW_CONTROL_ENABLE_BIT;
	mvpp2_cm3_write(port->priv, MSS_FC_COM_REG, val);

	/* Disable Flow control for all RXQs */
	for (q = 0; q < port->nrxqs; q++) {
		/* Set threshold 0 to disable Flow control */
		val = 0;
		val |= (0 << MSS_RXQ_TRESH_STOP_OFFS);
		mvpp2_cm3_write(port->priv, MSS_RXQ_TRESH_REG(q, fq), val);

		val = mvpp2_cm3_read(port->priv, MSS_RXQ_ASS_REG(q, fq));

		val &= ~(MSS_RXQ_ASS_PORTID_MASK << MSS_RXQ_ASS_Q_BASE(q, fq));

		val &= ~(MSS_RXQ_ASS_HOSTID_MASK << (MSS_RXQ_ASS_Q_BASE(q, fq)
			+ MSS_RXQ_ASS_HOSTID_OFFS));

		mvpp2_cm3_write(port->priv, MSS_RXQ_ASS_REG(q, fq), val);
	}

	/* Notify Firmware that Flow control config space ready for update */
	val = mvpp2_cm3_read(port->priv, MSS_FC_COM_REG);
	val |= FLOW_CONTROL_UPDATE_COMMAND_BIT;
	val |= cm3_state;
	mvpp2_cm3_write(port->priv, MSS_FC_COM_REG, val);

	spin_unlock_irqrestore(&port->priv->mss_spinlock, flags);
}

/* Routine disable/enable flow control for BM pool conditon */
void mvpp2_bm_pool_update_fc(struct mvpp2_port *port,
			     struct mvpp2_bm_pool *pool,
			     bool en)
{
	int val, cm3_state;
	unsigned long flags;

	spin_lock_irqsave(&port->priv->mss_spinlock, flags);

	/* Remove Flow control enable bit to prevent race between FW and Kernel
	 * If Flow control were enabled, it would be re-enabled.
	 */
	val = mvpp2_cm3_read(port->priv, MSS_FC_COM_REG);
	cm3_state = (val & FLOW_CONTROL_ENABLE_BIT);
	val &= ~FLOW_CONTROL_ENABLE_BIT;
	mvpp2_cm3_write(port->priv, MSS_FC_COM_REG, val);

	/* Check if BM pool should be enabled/disable */
	if (en) {
		/* Set BM pool start and stop thresholds per port */
		val = mvpp2_cm3_read(port->priv, MSS_BUF_POOL_REG(pool->id));
		val |= MSS_BUF_POOL_PORT_OFFS(port->id);
		val &= ~MSS_BUF_POOL_START_MASK;
		val |= (MSS_THRESHOLD_START << MSS_BUF_POOL_START_OFFS);
		val &= ~MSS_BUF_POOL_STOP_MASK;
		val |= MSS_THRESHOLD_STOP;
		mvpp2_cm3_write(port->priv, MSS_BUF_POOL_REG(pool->id), val);
	} else {
		/* Remove BM pool from the port */
		val = mvpp2_cm3_read(port->priv, MSS_BUF_POOL_REG(pool->id));
		val &= ~MSS_BUF_POOL_PORT_OFFS(port->id);

		/* Zero BM pool start and stop thresholds to disable pool
		 * flow control if pool empty (not used by any port)
		 */
		if (!pool->buf_num) {
			val &= ~MSS_BUF_POOL_START_MASK;
			val &= ~MSS_BUF_POOL_STOP_MASK;
		}

		mvpp2_cm3_write(port->priv, MSS_BUF_POOL_REG(pool->id), val);
	}

	/* Notify Firmware that Flow control config space ready for update */
	val = mvpp2_cm3_read(port->priv, MSS_FC_COM_REG);
	val |= FLOW_CONTROL_UPDATE_COMMAND_BIT;
	val |= cm3_state;
	mvpp2_cm3_write(port->priv, MSS_FC_COM_REG, val);

	spin_unlock_irqrestore(&port->priv->mss_spinlock, flags);
}

static int mvpp2_enable_global_fc(struct mvpp2 *priv)
{
	int val, timeout = 0;

	/* Enable global flow control. In this stage global
	 * flow control enabled, but still disabled per port.
	 */
	val = mvpp2_cm3_read(priv, MSS_FC_COM_REG);
	val |= FLOW_CONTROL_ENABLE_BIT;
	mvpp2_cm3_write(priv, MSS_FC_COM_REG, val);

	/* Check if Firmware running and disable FC if not*/
	val |= FLOW_CONTROL_UPDATE_COMMAND_BIT;
	mvpp2_cm3_write(priv, MSS_FC_COM_REG, val);

	while (timeout < MSS_FC_MAX_TIMEOUT) {
		val = mvpp2_cm3_read(priv, MSS_FC_COM_REG);

		if (!(val & FLOW_CONTROL_UPDATE_COMMAND_BIT))
			return 0;
		usleep_range(10, 20);
		timeout++;
	}

	priv->global_tx_fc = false;
	return -ENOTSUPP;
}

/* Release buffer to BM */
static inline void mvpp2_bm_pool_put(struct mvpp2_port *port, int pool,
				     dma_addr_t buf_dma_addr)
{
	unsigned int thread = mvpp2_cpu_to_thread(port->priv, get_cpu());
	unsigned long flags = 0;

	if (test_bit(thread, &port->priv->lock_map))
		spin_lock_irqsave(&port->bm_lock[thread], flags);

	/* MVPP2_BM_VIRT_RLS_REG is not interpreted by HW, and simply
	 * returned in the "cookie" field of the RX descriptor.
	 * For performance reasons don't store VA|PA and don't use "cookie".
	 * VA/PA obtained faster from dma_to_phys(dma-addr) and phys_to_virt.
	 */
#if defined(CONFIG_ARCH_DMA_ADDR_T_64BIT) && defined(CONFIG_PHYS_ADDR_T_64BIT)
	if (!static_branch_unlikely(&mvpp21_variant)) {
		u32 val = upper_32_bits(buf_dma_addr) &
				MVPP22_BM_ADDR_HIGH_PHYS_RLS_MASK;

		mvpp2_thread_write_relaxed(port->priv, thread,
					   MVPP22_BM_ADDR_HIGH_RLS_REG, val);
	}
#endif

	mvpp2_thread_write_relaxed(port->priv, thread,
				   MVPP2_BM_PHY_RLS_REG(pool), buf_dma_addr);

	if (test_bit(thread, &port->priv->lock_map))
		spin_unlock_irqrestore(&port->bm_lock[thread], flags);

	put_cpu();
}

/* Allocate buffers for the pool */
static int mvpp2_bm_bufs_add(struct mvpp2_port *port,
			     struct mvpp2_bm_pool *bm_pool, int buf_num)
{
	int i;
	dma_addr_t dma_addr;

	if (buf_num < 0 ||
	    (buf_num + bm_pool->buf_num > bm_pool->size)) {
		netdev_err(port->dev,
			   "cannot allocate %d buffers for pool %d\n",
			   buf_num, bm_pool->id);
		return 0;
	}

	for (i = 0; i < buf_num; i++) {
		dma_addr = mvpp2_buf_alloc(port, bm_pool, GFP_KERNEL);
		if (!dma_addr)
			break;

		mvpp2_bm_pool_put(port, bm_pool->id, dma_addr);
	}

	/* Update BM driver with number of buffers added to pool */
	bm_pool->buf_num += i;

	netdev_dbg(port->dev,
		   "pool %d: pkt_size=%4d, buf_size=%4d, total_size=%4d\n",
		   bm_pool->id, bm_pool->pkt_size,
		   MVPP2_RX_BUF_SIZE(bm_pool->pkt_size),
		   bm_pool->frag_size);

	netdev_dbg(port->dev,
		   "pool %d: %d of %d buffers added\n",
		   bm_pool->id, i, buf_num);
	return i;
}

/* Notify the driver that BM pool is being used as specific type and return the
 * pool pointer on success
 */
static struct mvpp2_bm_pool *
mvpp2_bm_pool_use(struct mvpp2_port *port, unsigned pool, int pkt_size)
{
	struct mvpp2_bm_pool *new_pool = &port->priv->bm_pools[pool];
	enum mvpp2_bm_pool_type pool_type = mvpp2_bm_pool_get_type(pool);
	int num;

	if (pool >= MVPP2_BM_POOLS_NUM) {
		netdev_err(port->dev, "Invalid pool %d\n", pool);
		return NULL;
	}

	/* Allocate buffers in case BM pool is used as long pool, but packet
	 * size doesn't match MTU or BM pool hasn't being used yet
	 */
	if (new_pool->pkt_size == 0) {
		int pkts_num;

		/* Set default buffer number or free all the buffers in case
		 * the pool is not empty
		 */
		pkts_num = new_pool->buf_num;
		if (pkts_num == 0) {
			pkts_num = mvpp2_bm_pool_default_buf_num(pool_type);
			if (pkts_num < 0)
				return NULL;
		} else {
			mvpp2_bm_bufs_free(port->dev->dev.parent,
					   port->priv, new_pool, pkts_num);
		}

		new_pool->pkt_size = pkt_size;
		new_pool->frag_size =
			SKB_DATA_ALIGN(MVPP2_RX_BUF_SIZE(pkt_size +
						MVPP2_VLAN_TAG_EDSA_LEN)) +
						MVPP2_SKB_SHINFO_SIZE;

		/* Allocate buffers for this pool */
		num = mvpp2_bm_bufs_add(port, new_pool, pkts_num);
		if (num != pkts_num) {
			WARN(1, "pool %d: %d of %d allocated\n",
			     new_pool->id, num, pkts_num);
			return NULL;
		}
	}

	mvpp2_bm_pool_bufsize_set(port->priv, new_pool,
				  MVPP2_RX_BUF_SIZE(new_pool->pkt_size));

	return new_pool;
}

/* Create long pool per-CPU */
static void mvpp2_bm_pool_pcpu_use(void *arg)
{
	struct mvpp2_port *port = arg;
	struct mvpp2_bm_pool **pools_pcpu = port->priv->pools_pcpu;
	int cpu = smp_processor_id();
	int pool_id, pkt_size;

	if (pools_pcpu[cpu])
		return;

	pool_id = mvpp2_bm_pool_get_id(MVPP2_BM_LONG) + cpu,
	pkt_size = mvpp2_bm_pool_default_pkt_size(MVPP2_BM_LONG);

	pools_pcpu[cpu] = mvpp2_bm_pool_use(port, pool_id, pkt_size);
}

/* Initialize pools for swf */
static int mvpp2_swf_bm_pool_pcpu_init(struct mvpp2_port *port)
{
	enum mvpp2_bm_pool_type long_pool_type, short_pool_type;
	int rxq, pkt_size, pool_id, cpu;

	/* If port pkt_size is higher than 1518B:
	 * HW Long pool - SW Jumbo pool, HW Short pool - SW Long pool
	 * else: HW Long pool - SW Long pool, HW Short pool - SW Short pool
	 */
	if (port->pkt_size > MVPP2_BM_LONG_PKT_SIZE) {
		long_pool_type = MVPP2_BM_JUMBO;
		short_pool_type = MVPP2_BM_LONG;
	} else {
		long_pool_type = MVPP2_BM_LONG;
		short_pool_type = MVPP2_BM_SHORT;
	}

	/* First handle the per-CPU long pools,
	 * as they are used in both cases.
	 */
	on_each_cpu(mvpp2_bm_pool_pcpu_use, port, 1);
	/* Sanity check */
	for_each_present_cpu(cpu) {
		if (!port->priv->pools_pcpu[cpu])
			return -ENOMEM;
	}

	if (!port->pool_long && long_pool_type == MVPP2_BM_JUMBO) {
		/* HW Long pool - SW Jumbo pool */
		pool_id = mvpp2_bm_pool_get_id(long_pool_type);
		pkt_size = mvpp2_bm_pool_default_pkt_size(long_pool_type);

		port->pool_long = mvpp2_bm_pool_use(port, pool_id, pkt_size);
		if (!port->pool_long)
			return -ENOMEM;

		port->pool_long->port_map |= BIT(port->id);

		for (rxq = 0; rxq < port->nrxqs; rxq++)
			mvpp2_rxq_long_pool_set(port, rxq, port->pool_long->id);

		/* HW Short pool - SW Long pool (per-CPU) */
		port->pool_short = port->priv->pools_pcpu[0];
		for (rxq = 0; rxq < port->nrxqs; rxq++)
			mvpp2_rxq_short_pool_set(port, rxq,
						 port->pool_short->id + rxq);

	} else if (!port->pool_long) {
		/* HW Long pool - SW Long pool (per-CPU) */
		port->pool_long = port->priv->pools_pcpu[0];
		for (rxq = 0; rxq < port->nrxqs; rxq++)
			mvpp2_rxq_long_pool_set(port, rxq,
						port->pool_long->id + rxq);
	}

	if (!port->pool_short) {
		/* HW Short pool - SW Short pool */
		pool_id = mvpp2_bm_pool_get_id(short_pool_type);
		pkt_size = mvpp2_bm_pool_default_pkt_size(short_pool_type);

		port->pool_short = mvpp2_bm_pool_use(port, pool_id, pkt_size);
		if (!port->pool_short)
			return -ENOMEM;

		port->pool_short->port_map |= BIT(port->id);

		for (rxq = 0; rxq < port->nrxqs; rxq++)
			mvpp2_rxq_short_pool_set(port, rxq,
						 port->pool_short->id);
	}

	/* Fill per-CPU Long pools' port map */
	for_each_present_cpu(cpu)
		port->priv->pools_pcpu[cpu]->port_map |= BIT(port->id);

	return 0;
}

/* Initialize pools for swf */
static int mvpp2_swf_bm_pool_init(struct mvpp2_port *port)
{
	enum mvpp2_bm_pool_type long_pool_type, short_pool_type;
	int rxq;

	/* If port pkt_size is higher than 1518B:
	 * HW Long pool - SW Jumbo pool, HW Short pool - SW Long pool
	 * else: HW Long pool - SW Long pool, HW Short pool - SW Short pool
	 */
	if (port->pkt_size > MVPP2_BM_LONG_PKT_SIZE) {
		long_pool_type = MVPP2_BM_JUMBO;
		short_pool_type = MVPP2_BM_LONG;
	} else {
		long_pool_type = MVPP2_BM_LONG;
		short_pool_type = MVPP2_BM_SHORT;
	}

	if (!port->pool_long) {
		port->pool_long =
			mvpp2_bm_pool_use(port,
					  mvpp2_bm_pool_get_id(long_pool_type),
				mvpp2_bm_pool_default_pkt_size(long_pool_type));
		if (!port->pool_long)
			return -ENOMEM;

		port->pool_long->port_map |= BIT(port->id);

		for (rxq = 0; rxq < port->nrxqs; rxq++)
			mvpp2_rxq_long_pool_set(port, rxq, port->pool_long->id);
	}

	if (!port->pool_short) {
		port->pool_short =
			mvpp2_bm_pool_use(port,
					  mvpp2_bm_pool_get_id(short_pool_type),
			       mvpp2_bm_pool_default_pkt_size(short_pool_type));
		if (!port->pool_short)
			return -ENOMEM;

		port->pool_short->port_map |= BIT(port->id);

		for (rxq = 0; rxq < port->nrxqs; rxq++)
			mvpp2_rxq_short_pool_set(port, rxq,
						 port->pool_short->id);
	}

	return 0;
}

static int mvpp2_bm_update_mtu(struct net_device *dev, int mtu)
{
	struct mvpp2_port *port = netdev_priv(dev);
	enum mvpp2_bm_pool_type new_long_pool_type;
	struct mvpp2_bm_pool **pools_pcpu = port->priv->pools_pcpu;
	int pkt_size = MVPP2_RX_PKT_SIZE(mtu);
	int err, cpu;

	/* If port MTU is higher than 1518B:
	 * HW Long pool - SW Jumbo pool, HW Short pool - SW Long pool
	 * else: HW Long pool - SW Long pool, HW Short pool - SW Short pool
	 */
	if (pkt_size > MVPP2_BM_LONG_PKT_SIZE)
		new_long_pool_type = MVPP2_BM_JUMBO;
	else
		new_long_pool_type = MVPP2_BM_LONG;

	if (new_long_pool_type != port->pool_long->type) {
		if (port->tx_fc) {
			if (recycle) {
				for_each_present_cpu(cpu)
					mvpp2_bm_pool_update_fc(port,
								pools_pcpu[cpu],
								false);
			} else if (pkt_size > MVPP2_BM_LONG_PKT_SIZE)
				mvpp2_bm_pool_update_fc(port,
							port->pool_short,
							false);
			else
				mvpp2_bm_pool_update_fc(port, port->pool_long,
							false);
		}

		/* Remove port from old short & long pool */
		port->pool_long->port_map &= ~BIT(port->id);
		port->pool_long = NULL;

		port->pool_short->port_map &= ~BIT(port->id);
		port->pool_short = NULL;

		port->pkt_size =  pkt_size;

		/* Add port to new short & long pool */
		if (recycle) {
			for_each_present_cpu(cpu)
				pools_pcpu[cpu]->port_map &= ~BIT(port->id);
			err = mvpp2_swf_bm_pool_pcpu_init(port);
		} else {
			err = mvpp2_swf_bm_pool_init(port);
		}
		if (err)
			return err;

		if (port->tx_fc) {
			if (recycle) {
				for_each_present_cpu(cpu)
					mvpp2_bm_pool_update_fc(port,
								pools_pcpu[cpu],
								false);
			} else if (pkt_size > MVPP2_BM_LONG_PKT_SIZE)
				mvpp2_bm_pool_update_fc(port, port->pool_long,
							true);
			else
				mvpp2_bm_pool_update_fc(port, port->pool_short,
							true);
		}

		/* Update L4 checksum when jumbo enable/disable on port */
		if (new_long_pool_type == MVPP2_BM_JUMBO && port->id != 0) {
			dev->features &= ~(NETIF_F_IP_CSUM | NETIF_F_IPV6_CSUM);
			dev->hw_features &= ~(NETIF_F_IP_CSUM |
					      NETIF_F_IPV6_CSUM);
		} else {
			dev->features |= NETIF_F_IP_CSUM | NETIF_F_IPV6_CSUM;
			dev->hw_features |= NETIF_F_IP_CSUM | NETIF_F_IPV6_CSUM;
		}
	}

	dev->mtu = mtu;
	dev->wanted_features = dev->features;

	netdev_update_features(dev);
	return 0;
}

static inline void mvpp2_interrupts_enable(struct mvpp2_port *port)
{
	int i, sw_thread_mask = 0;

	for (i = 0; i < port->nqvecs; i++)
		sw_thread_mask |= port->qvecs[i].sw_thread_mask;

	mvpp2_write(port->priv, MVPP2_ISR_ENABLE_REG(port->id),
		    MVPP2_ISR_ENABLE_INTERRUPT(sw_thread_mask));
}

static inline void mvpp2_interrupts_disable(struct mvpp2_port *port)
{
	int i, sw_thread_mask = 0;

	for (i = 0; i < port->nqvecs; i++)
		sw_thread_mask |= port->qvecs[i].sw_thread_mask;

	mvpp2_write(port->priv, MVPP2_ISR_ENABLE_REG(port->id),
		    MVPP2_ISR_DISABLE_INTERRUPT(sw_thread_mask));
}

static inline void mvpp2_qvec_interrupt_enable(struct mvpp2_queue_vector *qvec)
{
	struct mvpp2_port *port = qvec->port;

	mvpp2_write(port->priv, MVPP2_ISR_ENABLE_REG(port->id),
		    MVPP2_ISR_ENABLE_INTERRUPT(qvec->sw_thread_mask));
}

static inline void mvpp2_qvec_interrupt_disable(struct mvpp2_queue_vector *qvec)
{
	struct mvpp2_port *port = qvec->port;

	mvpp2_write(port->priv, MVPP2_ISR_ENABLE_REG(port->id),
		    MVPP2_ISR_DISABLE_INTERRUPT(qvec->sw_thread_mask));
}

/* Mask the current thread's Rx/Tx interrupts
 * Called by on_each_cpu(), guaranteed to run with migration disabled,
 * using smp_processor_id() is OK.
 */
static void mvpp2_interrupts_mask(void *arg)
{
	struct mvpp2_port *port = arg;

	/* If the thread isn't used, don't do anything */
	if (smp_processor_id() > port->priv->nthreads)
		return;

	mvpp2_thread_write(port->priv,
			   mvpp2_cpu_to_thread(port->priv, smp_processor_id()),
			   MVPP2_ISR_RX_TX_MASK_REG(port->id), 0);
	mvpp2_thread_write(port->priv,
			   mvpp2_cpu_to_thread(port->priv, smp_processor_id()),
			   MVPP2_ISR_RX_ERR_CAUSE_REG(port->id), 0);
}

/* Unmask the current thread's Rx/Tx interrupts.
 * Called by on_each_cpu(), guaranteed to run with migration disabled,
 * using smp_processor_id() is OK.
 */
static void mvpp2_interrupts_unmask(void *arg)
{
	struct mvpp2_port *port = arg;
	u32 val;

	/* If the thread isn't used, don't do anything */
	if (smp_processor_id() > port->priv->nthreads)
		return;

	if (port->flags & MVPP22_F_IF_MUSDK)
		return;

	val = MVPP2_CAUSE_RXQ_OCCUP_DESC_ALL_MASK(mvpp21_variant);
	if (port->has_tx_irqs)
		val |= MVPP2_CAUSE_TXQ_OCCUP_DESC_ALL_MASK;

	mvpp2_thread_write(port->priv,
			   mvpp2_cpu_to_thread(port->priv, smp_processor_id()),
			   MVPP2_ISR_RX_TX_MASK_REG(port->id), val);
	mvpp2_thread_write(port->priv,
			   mvpp2_cpu_to_thread(port->priv, smp_processor_id()),
			   MVPP2_ISR_RX_ERR_CAUSE_REG(port->id),
			   MVPP2_ISR_RX_ERR_CAUSE_NONOCC_MASK);
}

static void
mvpp2_shared_interrupt_mask_unmask(struct mvpp2_port *port, bool mask)
{
	u32 val;
	int i;

	if (port->priv->hw_version == MVPP21)
		return;

	if (mask)
		val = 0;
	else
		val = MVPP2_CAUSE_RXQ_OCCUP_DESC_ALL_MASK(mvpp21_variant);

	for (i = 0; i < port->nqvecs; i++) {
		struct mvpp2_queue_vector *v = port->qvecs + i;

		if (v->type != MVPP2_QUEUE_VECTOR_SHARED)
			continue;

		mvpp2_thread_write(port->priv, v->sw_thread_id,
				   MVPP2_ISR_RX_TX_MASK_REG(port->id), val);
		mvpp2_thread_write(port->priv, v->sw_thread_id,
				   MVPP2_ISR_RX_ERR_CAUSE_REG(port->id),
				   MVPP2_ISR_RX_ERR_CAUSE_NONOCC_MASK);
	}
}

/* Port configuration routines */

static void mvpp22_gop_init_rgmii(struct mvpp2_port *port)
{
	struct mvpp2 *priv = port->priv;
	u32 val;

	regmap_read(priv->sysctrl_base, GENCONF_PORT_CTRL0, &val);
	val |= GENCONF_PORT_CTRL0_BUS_WIDTH_SELECT;
	regmap_write(priv->sysctrl_base, GENCONF_PORT_CTRL0, val);

	regmap_read(priv->sysctrl_base, GENCONF_CTRL0, &val);
	if (port->gop_id == 2)
		val |= GENCONF_CTRL0_PORT0_RGMII;
	else if (port->gop_id == 3)
		val |= GENCONF_CTRL0_PORT1_RGMII_MII;
	regmap_write(priv->sysctrl_base, GENCONF_CTRL0, val);
}

static void mvpp22_gop_init_mii(struct mvpp2_port *port)
{
	struct mvpp2 *priv = port->priv;
	u32 val;

	regmap_read(priv->sysctrl_base, GENCONF_PORT_CTRL0, &val);
	val |= GENCONF_PORT_CTRL0_BUS_WIDTH_SELECT;
	regmap_write(priv->sysctrl_base, GENCONF_PORT_CTRL0, val);

	regmap_read(priv->sysctrl_base, GENCONF_CTRL0, &val);
	val |= GENCONF_CTRL0_PORT1_RGMII_MII;
	val &= ~GENCONF_CTRL0_PORT1_RGMII;
	regmap_write(priv->sysctrl_base, GENCONF_CTRL0, val);
}

static void mvpp22_gop_init_sgmii(struct mvpp2_port *port)
{
	struct mvpp2 *priv = port->priv;
	u32 val;

	regmap_read(priv->sysctrl_base, GENCONF_PORT_CTRL0, &val);
	val |= GENCONF_PORT_CTRL0_BUS_WIDTH_SELECT |
	       GENCONF_PORT_CTRL0_RX_DATA_SAMPLE;
	regmap_write(priv->sysctrl_base, GENCONF_PORT_CTRL0, val);

	if (port->gop_id > 1) {
		regmap_read(priv->sysctrl_base, GENCONF_CTRL0, &val);
		if (port->gop_id == 2)
			val &= ~GENCONF_CTRL0_PORT0_RGMII;
		else if (port->gop_id == 3)
			val &= ~GENCONF_CTRL0_PORT1_RGMII_MII;
		regmap_write(priv->sysctrl_base, GENCONF_CTRL0, val);
	}
}

static void mvpp22_gop_init_xpcs(struct mvpp2_port *port)
{
	struct mvpp2 *priv = port->priv;
	void __iomem *xpcs = priv->iface_base + MVPP22_XPCS_BASE(port->gop_id);
	u32 val;

	/* Reset the XPCS when reconfiguring the lanes */
	val = readl(xpcs + MVPP22_XPCS_CFG0);
	writel(val & ~MVPP22_XPCS_CFG0_RESET_DIS, xpcs + MVPP22_XPCS_CFG0);

	/* XPCS */
	val = readl(xpcs + MVPP22_XPCS_CFG0);
	val &= ~(MVPP22_XPCS_CFG0_PCS_MODE(0x3) |
		 MVPP22_XPCS_CFG0_ACTIVE_LANE(0x3));
	val |= MVPP22_XPCS_CFG0_ACTIVE_LANE(2);
	writel(val, xpcs + MVPP22_XPCS_CFG0);

	/* Release lanes from reset */
	val = readl(xpcs + MVPP22_XPCS_CFG0);
	writel(val | MVPP22_XPCS_CFG0_RESET_DIS, xpcs + MVPP22_XPCS_CFG0);

}

static void mvpp22_gop_init_mpcs(struct mvpp2_port *port)
{
	struct mvpp2 *priv = port->priv;
	void __iomem *mpcs = priv->iface_base + MVPP22_MPCS_BASE(port->gop_id);
	u32 val;

	/* MPCS */
	val = readl(mpcs + MVPP22_MPCS_CTRL);
	val &= ~MVPP22_MPCS_CTRL_FWD_ERR_CONN;
	writel(val, mpcs + MVPP22_MPCS_CTRL);

	val = readl(mpcs + MVPP22_MPCS_CLK_RESET);
	val &= ~(MVPP22_MPCS_CLK_RESET_DIV_RATIO(0x7) | MAC_CLK_RESET_MAC |
		 MAC_CLK_RESET_SD_RX | MAC_CLK_RESET_SD_TX);
	val |= MVPP22_MPCS_CLK_RESET_DIV_RATIO(1);
	writel(val, mpcs + MVPP22_MPCS_CLK_RESET);

	val &= ~MVPP22_MPCS_CLK_RESET_DIV_SET;
	val |= MAC_CLK_RESET_MAC | MAC_CLK_RESET_SD_RX | MAC_CLK_RESET_SD_TX;
	writel(val, mpcs + MVPP22_MPCS_CLK_RESET);
}

static void mvpp22_gop_fca_enable_periodic(struct mvpp2_port *port, bool en)
{
	struct mvpp2 *priv = port->priv;
	void __iomem *fca = priv->iface_base + MVPP22_FCA_BASE(port->gop_id);
	u32 val;

	val = readl(fca + MVPP22_FCA_CONTROL_REG);
	val &= ~MVPP22_FCA_ENABLE_PERIODIC;
	if (en)
		val |= MVPP22_FCA_ENABLE_PERIODIC;
	writel(val, fca + MVPP22_FCA_CONTROL_REG);
}

static void mvpp22_gop_fca_set_timer(struct mvpp2_port *port, u32 timer)
{
	struct mvpp2 *priv = port->priv;
	void __iomem *fca = priv->iface_base + MVPP22_FCA_BASE(port->gop_id);
	u32 lsb, msb;

	lsb = timer & MVPP22_FCA_REG_MASK;
	msb = timer >> MVPP22_FCA_REG_SIZE;

	writel(lsb, fca + MVPP22_PERIODIC_COUNTER_LSB_REG);
	writel(msb, fca + MVPP22_PERIODIC_COUNTER_MSB_REG);
}

/* Set Flow Control timer x140 faster than pause quanta to ensure that link
 * partner won't send taffic if port in XOFF mode.
 */
static void mvpp22_gop_fca_set_periodic_timer(struct mvpp2_port *port)
{
	u32 timer;

	timer = (port->priv->tclk / (USEC_PER_SEC * FC_CLK_DIVIDER))
		* FC_QUANTA;

	mvpp22_gop_fca_enable_periodic(port, false);

	mvpp22_gop_fca_set_timer(port, timer);

	mvpp22_gop_fca_enable_periodic(port, true);
}

static int mvpp22_gop_init(struct mvpp2_port *port)
{
	struct mvpp2 *priv = port->priv;
	u32 val;

	if (!priv->sysctrl_base)
		return 0;

	switch (port->phy_interface) {
	case PHY_INTERFACE_MODE_MII:
		if (port->gop_id == 0 || port->gop_id == 2)
			goto invalid_conf;
		mvpp22_gop_init_mii(port);
		break;
	case PHY_INTERFACE_MODE_RGMII:
	case PHY_INTERFACE_MODE_RGMII_ID:
	case PHY_INTERFACE_MODE_RGMII_RXID:
	case PHY_INTERFACE_MODE_RGMII_TXID:
		if (port->gop_id == 0)
			goto invalid_conf;
		mvpp22_gop_init_rgmii(port);
		break;
	case PHY_INTERFACE_MODE_SGMII:
	case PHY_INTERFACE_MODE_1000BASEX:
	case PHY_INTERFACE_MODE_2500BASEX:
	case PHY_INTERFACE_MODE_2500BASET:
		mvpp22_gop_init_sgmii(port);
		break;
	case PHY_INTERFACE_MODE_RXAUI:
		if (port->gop_id != 0)
			goto invalid_conf;
		mvpp22_gop_init_xpcs(port);
		break;
	case PHY_INTERFACE_MODE_10GKR:
	case PHY_INTERFACE_MODE_5GKR:
		if (!port->has_xlg_mac)
			goto invalid_conf;
		mvpp22_gop_init_mpcs(port);
		break;
	case PHY_INTERFACE_MODE_INTERNAL:
		return 0;
	default:
		goto unsupported_conf;
	}

	regmap_read(priv->sysctrl_base, GENCONF_PORT_CTRL1, &val);
	val |= GENCONF_PORT_CTRL1_RESET(port->gop_id) |
	       GENCONF_PORT_CTRL1_EN(port->gop_id);
	regmap_write(priv->sysctrl_base, GENCONF_PORT_CTRL1, val);

	regmap_read(priv->sysctrl_base, GENCONF_PORT_CTRL0, &val);
	val |= GENCONF_PORT_CTRL0_CLK_DIV_PHASE_CLR;
	regmap_write(priv->sysctrl_base, GENCONF_PORT_CTRL0, val);

	regmap_read(priv->sysctrl_base, GENCONF_SOFT_RESET1, &val);
	val |= GENCONF_SOFT_RESET1_GOP;
	regmap_write(priv->sysctrl_base, GENCONF_SOFT_RESET1, val);

	mvpp22_gop_fca_set_periodic_timer(port);

unsupported_conf:
	return 0;

invalid_conf:
	netdev_err(port->dev, "Invalid port configuration\n");
	return -EINVAL;
}

static void mvpp22_gop_unmask_irq(struct mvpp2_port *port)
{
	u32 val;

	if (phy_interface_mode_is_rgmii(port->phy_interface) ||
	    phy_interface_mode_is_8023z(port->phy_interface) ||
	    port->phy_interface == PHY_INTERFACE_MODE_MII ||
	    port->phy_interface == PHY_INTERFACE_MODE_SGMII ||
	    port->phy_interface == PHY_INTERFACE_MODE_2500BASET) {
		/* Enable the GMAC link status irq for this port */
		val = readl(port->base + MVPP22_GMAC_INT_SUM_MASK);
		val |= MVPP22_GMAC_INT_SUM_MASK_LINK_STAT;
		writel(val, port->base + MVPP22_GMAC_INT_SUM_MASK);
	}

	if (port->has_xlg_mac) {
		/* Enable the XLG/GIG irqs for this port */
		val = readl(port->base + MVPP22_XLG_EXT_INT_MASK);
		if (port->phy_interface == PHY_INTERFACE_MODE_10GKR ||
		    port->phy_interface == PHY_INTERFACE_MODE_5GKR ||
		    port->phy_interface == PHY_INTERFACE_MODE_INTERNAL)
			val |= MVPP22_XLG_EXT_INT_MASK_XLG;
		else
			val |= MVPP22_XLG_EXT_INT_MASK_GIG;
		writel(val, port->base + MVPP22_XLG_EXT_INT_MASK);
	}
}

static void mvpp22_gop_mask_irq(struct mvpp2_port *port)
{
	u32 val;

	if (port->has_xlg_mac) {
		val = readl(port->base + MVPP22_XLG_EXT_INT_MASK);
		val &= ~(MVPP22_XLG_EXT_INT_MASK_XLG |
			 MVPP22_XLG_EXT_INT_MASK_GIG);
		writel(val, port->base + MVPP22_XLG_EXT_INT_MASK);
	}

	if (phy_interface_mode_is_rgmii(port->phy_interface) ||
	    phy_interface_mode_is_8023z(port->phy_interface) ||
	    port->phy_interface == PHY_INTERFACE_MODE_MII ||
	    port->phy_interface == PHY_INTERFACE_MODE_SGMII ||
	    port->phy_interface == PHY_INTERFACE_MODE_2500BASET) {
		val = readl(port->base + MVPP22_GMAC_INT_SUM_MASK);
		val &= ~MVPP22_GMAC_INT_SUM_MASK_LINK_STAT;
		writel(val, port->base + MVPP22_GMAC_INT_SUM_MASK);
	}
}

static void mvpp22_gop_setup_irq(struct mvpp2_port *port)
{
	u32 val;

	if (port->phylink ||
	    phy_interface_mode_is_rgmii(port->phy_interface) ||
	    phy_interface_mode_is_8023z(port->phy_interface) ||
	    port->phy_interface == PHY_INTERFACE_MODE_MII ||
	    port->phy_interface == PHY_INTERFACE_MODE_SGMII) {
		val = readl(port->base + MVPP22_GMAC_INT_MASK);
		val |= MVPP22_GMAC_INT_MASK_LINK_STAT;
		writel(val, port->base + MVPP22_GMAC_INT_MASK);
	}

	if (port->has_xlg_mac) {
		val = readl(port->base + MVPP22_XLG_INT_MASK);
		val |= MVPP22_XLG_INT_MASK_LINK;
		writel(val, port->base + MVPP22_XLG_INT_MASK);
	}

	mvpp22_gop_unmask_irq(port);
}

/* Sets the PHY mode of the COMPHY (which configures the serdes lanes).
 *
 * The PHY mode used by the PPv2 driver comes from the network subsystem, while
 * the one given to the COMPHY comes from the generic PHY subsystem. Hence they
 * differ.
 *
 * The COMPHY configures the serdes lanes regardless of the actual use of the
 * lanes by the physical layer. This is why configurations like
 * "PPv2 (2500BaseX) - COMPHY (2500SGMII)" are valid.
 */
static int mvpp22_comphy_init(struct mvpp2_port *port)
{
	int ret;

	if (!port->comphy)
		return 0;

	ret = phy_set_mode_ext(port->comphy, PHY_MODE_ETHERNET,
			       port->phy_interface);
	if (ret)
		return ret;

	return phy_power_on(port->comphy);
}

static void mvpp2_port_enable(struct mvpp2_port *port)
{
	u32 val;

	if (port->phy_interface == PHY_INTERFACE_MODE_INTERNAL)
		return;

	if (port->has_xlg_mac &&
	    (port->phy_interface == PHY_INTERFACE_MODE_RXAUI ||
	     port->phy_interface == PHY_INTERFACE_MODE_10GKR ||
	     port->phy_interface == PHY_INTERFACE_MODE_5GKR)) {
		val = readl(port->base + MVPP22_XLG_CTRL0_REG);
		val |= MVPP22_XLG_CTRL0_PORT_EN |
		       MVPP22_XLG_CTRL0_MAC_RESET_DIS;
		val &= ~MVPP22_XLG_CTRL0_MIB_CNT_DIS;
		writel(val, port->base + MVPP22_XLG_CTRL0_REG);
	} else {
		val = readl(port->base + MVPP2_GMAC_CTRL_0_REG);
		val |= MVPP2_GMAC_PORT_EN_MASK;
		val |= MVPP2_GMAC_MIB_CNTR_EN_MASK;
		writel(val, port->base + MVPP2_GMAC_CTRL_0_REG);
	}
}

static void mvpp2_port_disable(struct mvpp2_port *port)
{
	u32 val;

	if (port->phy_interface == PHY_INTERFACE_MODE_INTERNAL)
		return;

	if (port->has_xlg_mac &&
	    (port->phy_interface == PHY_INTERFACE_MODE_RXAUI ||
	     port->phy_interface == PHY_INTERFACE_MODE_10GKR ||
	     port->phy_interface == PHY_INTERFACE_MODE_5GKR)) {
		val = readl(port->base + MVPP22_XLG_CTRL0_REG);
		val &= ~MVPP22_XLG_CTRL0_PORT_EN;
		writel(val, port->base + MVPP22_XLG_CTRL0_REG);
	} else {
		val = readl(port->base + MVPP2_GMAC_CTRL_0_REG);
		val &= ~(MVPP2_GMAC_PORT_EN_MASK);
		writel(val, port->base + MVPP2_GMAC_CTRL_0_REG);
	}
}

/* Set IEEE 802.3x Flow Control Xon Packet Transmission Mode */
static void mvpp2_port_periodic_xon_disable(struct mvpp2_port *port)
{
	u32 val;

	val = readl(port->base + MVPP2_GMAC_CTRL_1_REG) &
		    ~MVPP2_GMAC_PERIODIC_XON_EN_MASK;
	writel(val, port->base + MVPP2_GMAC_CTRL_1_REG);
}

/* Configure loopback port */
static void mvpp2_port_loopback_set(struct mvpp2_port *port,
				    const struct phylink_link_state *state)
{
	u32 val;

	val = readl(port->base + MVPP2_GMAC_CTRL_1_REG);

	if (state->speed == 1000)
		val |= MVPP2_GMAC_GMII_LB_EN_MASK;
	else
		val &= ~MVPP2_GMAC_GMII_LB_EN_MASK;

	if (phy_interface_mode_is_8023z(port->phy_interface) ||
	    port->phy_interface == PHY_INTERFACE_MODE_SGMII)
		val |= MVPP2_GMAC_PCS_LB_EN_MASK;
	else
		val &= ~MVPP2_GMAC_PCS_LB_EN_MASK;

	writel(val, port->base + MVPP2_GMAC_CTRL_1_REG);
}

struct mvpp2_ethtool_counter {
	unsigned int offset;
	const char string[ETH_GSTRING_LEN];
	bool reg_is_64b;
};

static u64 mvpp2_read_count(struct mvpp2_port *port,
			    const struct mvpp2_ethtool_counter *counter)
{
	u64 val;

	val = readl(port->stats_base + counter->offset);
	if (counter->reg_is_64b)
		val += (u64)readl(port->stats_base + counter->offset + 4) << 32;

	return val;
}

/* Due to the fact that software statistics and hardware statistics are, by
 * design, incremented at different moments in the chain of packet processing,
 * it is very likely that incoming packets could have been dropped after being
 * counted by hardware but before reaching software statistics (most probably
 * multicast packets), and in the oppposite way, during transmission, FCS bytes
 * are added in between as well as TSO skb will be split and header bytes added.
 * Hence, statistics gathered from userspace with ifconfig (software) and
 * ethtool (hardware) cannot be compared.
 */
static const struct mvpp2_ethtool_counter mvpp2_ethtool_regs[] = {
	{ MVPP2_MIB_GOOD_OCTETS_RCVD, "good_octets_received", true },
	{ MVPP2_MIB_BAD_OCTETS_RCVD, "bad_octets_received" },
	{ MVPP2_MIB_CRC_ERRORS_SENT, "crc_errors_sent" },
	{ MVPP2_MIB_UNICAST_FRAMES_RCVD, "unicast_frames_received" },
	{ MVPP2_MIB_BROADCAST_FRAMES_RCVD, "broadcast_frames_received" },
	{ MVPP2_MIB_MULTICAST_FRAMES_RCVD, "multicast_frames_received" },
	{ MVPP2_MIB_FRAMES_64_OCTETS, "frames_64_octets" },
	{ MVPP2_MIB_FRAMES_65_TO_127_OCTETS, "frames_65_to_127_octet" },
	{ MVPP2_MIB_FRAMES_128_TO_255_OCTETS, "frames_128_to_255_octet" },
	{ MVPP2_MIB_FRAMES_256_TO_511_OCTETS, "frames_256_to_511_octet" },
	{ MVPP2_MIB_FRAMES_512_TO_1023_OCTETS, "frames_512_to_1023_octet" },
	{ MVPP2_MIB_FRAMES_1024_TO_MAX_OCTETS, "frames_1024_to_max_octet" },
	{ MVPP2_MIB_GOOD_OCTETS_SENT, "good_octets_sent", true },
	{ MVPP2_MIB_UNICAST_FRAMES_SENT, "unicast_frames_sent" },
	{ MVPP2_MIB_MULTICAST_FRAMES_SENT, "multicast_frames_sent" },
	{ MVPP2_MIB_BROADCAST_FRAMES_SENT, "broadcast_frames_sent" },
	{ MVPP2_MIB_FC_SENT, "fc_sent" },
	{ MVPP2_MIB_FC_RCVD, "fc_received" },
	{ MVPP2_MIB_RX_FIFO_OVERRUN, "rx_fifo_overrun" },
	{ MVPP2_MIB_UNDERSIZE_RCVD, "undersize_received" },
	{ MVPP2_MIB_FRAGMENTS_ERR_RCVD, "fragments_err_received" },
	{ MVPP2_MIB_OVERSIZE_RCVD, "oversize_received" },
	{ MVPP2_MIB_JABBER_RCVD, "jabber_received" },
	{ MVPP2_MIB_MAC_RCV_ERROR, "mac_receive_error" },
	{ MVPP2_MIB_BAD_CRC_EVENT, "bad_crc_event" },
	{ MVPP2_MIB_COLLISION, "collision" },
	{ MVPP2_MIB_LATE_COLLISION, "late_collision" },
#define MVPP2_LAST_MIB		MVPP2_MIB_LATE_COLLISION

	/* Extend counters */
	{ MVPP2_OVERRUN_DROP_REG(0),	"rx_ppv2_overrun" },
	{ MVPP2_CLS_DROP_REG(0),	"rx_cls_drop" },
	{ MVPP2_RX_PKT_FULLQ_DROP_REG,	"rx_fullq_drop" },
	{ MVPP2_RX_PKT_EARLY_DROP_REG,	"rx_early_drop" },
	{ MVPP2_RX_PKT_BM_DROP_REG,	"rx_bm_drop" },

	/* Extend SW counters (not registers) */
#define MVPP2_FIRST_CNT_SW		0xf000
#define MVPP2_TX_GUARD_CNT(cpu)	(MVPP2_FIRST_CNT_SW + cpu)
	{ MVPP2_TX_GUARD_CNT(0),	"tx-guard-cpu0" },
	{ MVPP2_TX_GUARD_CNT(1),	"tx-guard-cpu1" },
	{ MVPP2_TX_GUARD_CNT(2),	"tx-guard-cpu2" },
	{ MVPP2_TX_GUARD_CNT(3),	"tx-guard-cpu3" },
};

static const char mvpp22_priv_flags_strings[][ETH_GSTRING_LEN] = {
	"musdk",
};

#define MVPP22_F_IF_MUSDK_PRIV	BIT(0)

static int mvpp2_ethtool_get_mib_cntr_size(void)
{
	int i = 0;

	while (i < ARRAY_SIZE(mvpp2_ethtool_regs)) {
		if (mvpp2_ethtool_regs[i++].offset == MVPP2_LAST_MIB)
			break;
	}
	return i; /* mib_size */
}

static int mvpp2_ethtool_get_cntr_index(u32 offset)
{
	int i = 0;

	while (i < ARRAY_SIZE(mvpp2_ethtool_regs)) {
		if (mvpp2_ethtool_regs[i].offset == offset)
			break;
		i++;
	}
	return i;
}

/* hw_get_stats - update the ethtool_stats accumulator from HW-registers
 * The HW-registers/counters are cleared on read.
 */
static void mvpp2_hw_get_stats(struct mvpp2_port *port, u64 *pstats)
{
	int i, mib_size, queue, cpu;
	unsigned int reg_offs;
	u32 val, cls_drops;
	u64 *ptmp;

	mib_size = mvpp2_ethtool_get_mib_cntr_size();

	cls_drops = mvpp2_read(port->priv, MVPP2_OVERRUN_DROP_REG(port->id));

	for (i = 0; i < mib_size; i++) {
		if (mvpp2_ethtool_regs[i].offset == MVPP2_MIB_COLLISION) {
			val = mvpp2_read_count(port, &mvpp2_ethtool_regs[i]);
			port->dev->stats.collisions += val;
			*pstats++ += val;
			continue;
		}
		*pstats++ += mvpp2_read_count(port, &mvpp2_ethtool_regs[i]);
	}

	/* Extend HW counters */
	*pstats++ += cls_drops;
	*pstats++ += mvpp2_read(port->priv, MVPP2_CLS_DROP_REG(port->id));
	ptmp = pstats;
	queue = port->first_rxq;
	while (queue < (port->first_rxq + port->nrxqs)) {
		mvpp2_write(port->priv, MVPP2_CNT_IDX_REG, queue++);
		pstats = ptmp;
		i = mib_size + 2;
		while (i < ARRAY_SIZE(mvpp2_ethtool_regs)) {
			reg_offs = mvpp2_ethtool_regs[i++].offset;
			if (reg_offs == MVPP2_FIRST_CNT_SW)
				break;
			*pstats++ += mvpp2_read(port->priv, reg_offs);
		}
	}

	/* Extend SW counters (i=MVPP2_FIRST_CNT_SW) */
	for_each_present_cpu(cpu)
		*pstats++ = mvpp2_tx_done_guard_get_stats(port, cpu);
}

static void mvpp2_hw_clear_stats(struct mvpp2_port *port)
{
	int i, mib_size, queue;
	unsigned int reg_offs;

	mib_size = mvpp2_ethtool_get_mib_cntr_size();

	for (i = 0; i < mib_size; i++)
		mvpp2_read_count(port, &mvpp2_ethtool_regs[i]);

	/* Extend counters */
	mvpp2_read(port->priv, MVPP2_OVERRUN_DROP_REG(port->id));
	mvpp2_read(port->priv, MVPP2_CLS_DROP_REG(port->id));
	queue = port->first_rxq;
	while (queue < (port->first_rxq + port->nrxqs)) {
		mvpp2_write(port->priv, MVPP2_CNT_IDX_REG, queue++);
		i = mib_size + 2;
		while (i < ARRAY_SIZE(mvpp2_ethtool_regs)) {
			reg_offs = mvpp2_ethtool_regs[i++].offset;
			if (reg_offs == MVPP2_FIRST_CNT_SW)
				break;
			mvpp2_read(port->priv, reg_offs);
		}
	}
	/* Extend SW counters (i=MVPP2_FIRST_CNT_SW) */
	/* no clear */
}

static void mvpp2_ethtool_get_strings(struct net_device *netdev, u32 sset,
				      u8 *data)
{
	int i;

	switch (sset) {
	case ETH_SS_STATS:
		for (i = 0; i < ARRAY_SIZE(mvpp2_ethtool_regs); i++)
			memcpy(data + i * ETH_GSTRING_LEN,
			       &mvpp2_ethtool_regs[i].string, ETH_GSTRING_LEN);
		break;
	case ETH_SS_PRIV_FLAGS:
		memcpy(data, mvpp22_priv_flags_strings,
		       ARRAY_SIZE(mvpp22_priv_flags_strings) * ETH_GSTRING_LEN);
	}
}

static void mvpp2_gather_hw_statistics(struct work_struct *work)
{
	struct delayed_work *del_work = to_delayed_work(work);
	struct mvpp2_port *port = container_of(del_work, struct mvpp2_port,
					       stats_work);

	/* Update the statistic buffer by q-work only, not by ethtool-S */
	mutex_lock(&port->gather_stats_lock);
	mvpp2_hw_get_stats(port, port->ethtool_stats);
	mutex_unlock(&port->gather_stats_lock);
	queue_delayed_work(port->priv->stats_queue, &port->stats_work,
			   MVPP2_MIB_COUNTERS_STATS_DELAY);
}

static void mvpp2_ethtool_get_stats(struct net_device *dev,
				    struct ethtool_stats *stats, u64 *data)
{
	struct mvpp2_port *port = netdev_priv(dev);
	int cls_drp, fc_rcv;

	/* Use statistic already accumulated in ethtool_stats by q-work
	 * and copy under mutex-lock it into given ethtool-data-buffer.
	 */
	mutex_lock(&port->gather_stats_lock);
	memcpy(data, port->ethtool_stats,
	       sizeof(u64) * ARRAY_SIZE(mvpp2_ethtool_regs));
	mutex_unlock(&port->gather_stats_lock);

	/* Do not count flow control receive frames as classifier drops */
	cls_drp = mvpp2_ethtool_get_cntr_index(MVPP2_CLS_DROP_REG(0));
	fc_rcv = mvpp2_ethtool_get_cntr_index(MVPP2_MIB_FC_RCVD);
	data[cls_drp] =
		data[fc_rcv] > data[cls_drp] ? 0 : data[cls_drp] - data[fc_rcv];
}

static int mvpp2_ethtool_get_sset_count(struct net_device *dev, int sset)
{
	struct mvpp2_port *port = netdev_priv(dev);

	switch (sset) {
	case ETH_SS_STATS:
		return ARRAY_SIZE(mvpp2_ethtool_regs);
	case ETH_SS_PRIV_FLAGS:
		return (port->priv->hw_version == MVPP21) ?
			0 : ARRAY_SIZE(mvpp22_priv_flags_strings);
	}
	return -EOPNOTSUPP;
}

static void mvpp2_port_reset(struct mvpp2_port *port)
{
	u32 val;

	/* Read the GOP statistics to reset the hardware counters */
	mvpp2_hw_clear_stats(port);

	val = readl(port->base + MVPP2_GMAC_CTRL_2_REG) |
	      MVPP2_GMAC_PORT_RESET_MASK;
	writel(val, port->base + MVPP2_GMAC_CTRL_2_REG);

	if (port->has_xlg_mac) {
		/* Set the XLG MAC in reset */
		val = readl(port->base + MVPP22_XLG_CTRL0_REG) &
		      ~MVPP22_XLG_CTRL0_MAC_RESET_DIS;
		writel(val, port->base + MVPP22_XLG_CTRL0_REG);

		while (readl(port->base + MVPP22_XLG_CTRL0_REG) &
		       MVPP22_XLG_CTRL0_MAC_RESET_DIS)
			continue;
	}
}

/* Change maximum receive size of the port */
static inline void mvpp2_gmac_max_rx_size_set(struct mvpp2_port *port)
{
	u32 val;

	if (port->flags & MVPP22_F_IF_MUSDK)
		return;

	val = readl(port->base + MVPP2_GMAC_CTRL_0_REG);
	val &= ~MVPP2_GMAC_MAX_RX_SIZE_MASK;
	val |= (((port->pkt_size - MVPP2_MH_SIZE) / 2) <<
		    MVPP2_GMAC_MAX_RX_SIZE_OFFS);
	writel(val, port->base + MVPP2_GMAC_CTRL_0_REG);
}

/* Change maximum receive size of the port */
static inline void mvpp2_xlg_max_rx_size_set(struct mvpp2_port *port)
{
	u32 val;

	if (port->flags & MVPP22_F_IF_MUSDK)
		return;

	val =  readl(port->base + MVPP22_XLG_CTRL1_REG);
	val &= ~MVPP22_XLG_CTRL1_FRAMESIZELIMIT_MASK;
	val |= ((port->pkt_size - MVPP2_MH_SIZE) / 2) <<
	       MVPP22_XLG_CTRL1_FRAMESIZELIMIT_OFFS;
	writel(val, port->base + MVPP22_XLG_CTRL1_REG);
}

static void mvpp2_gmac_tx_fifo_configure(struct mvpp2_port *port)
{
	u32 val, tx_fifo_min_th;
	u8 low_wm, hi_wm;

	tx_fifo_min_th = MVPP2_GMAC_TX_FIFO_MIN_TH;
	low_wm = MVPP2_GMAC_TX_FIFO_LOW_WM;
	hi_wm = MVPP2_GMAC_TX_FIFO_HI_WM;

	/* Update TX FIFO MIN Threshold */
	val = readl(port->base + MVPP2_GMAC_PORT_FIFO_CFG_1_REG);
	val &= ~MVPP2_GMAC_TX_FIFO_MIN_TH_ALL_MASK;
	val |= tx_fifo_min_th;
	writel(val, port->base + MVPP2_GMAC_PORT_FIFO_CFG_1_REG);

	/* Update TX FIFO levels of assertion/deassertion
	 * of p2mem_ready_signal, which indicates readiness
	 * for fetching the data from DRAM.
	 */
	val = readl(port->base + MVPP2_GMAC_PORT_FIFO_CFG_0_REG);
	val &= ~MVPP2_GMAC_TX_FIFO_WM_MASK;
	val |= (low_wm << MVPP2_GMAC_TX_FIFO_WM_LOW_OFFSET) | hi_wm;
	writel(val, port->base + MVPP2_GMAC_PORT_FIFO_CFG_0_REG);
}

/* Set defaults to the MVPP2 port */
static void mvpp2_defaults_set(struct mvpp2_port *port)
{
	int tx_port_num, val, queue, ptxq, lrxq;

	if (phy_interface_mode_is_rgmii(port->phy_interface) ||
	    port->phy_interface == PHY_INTERFACE_MODE_MII ||
	    port->phy_interface == PHY_INTERFACE_MODE_SGMII ||
	    port->phy_interface == PHY_INTERFACE_MODE_1000BASEX ||
	    port->phy_interface == PHY_INTERFACE_MODE_2500BASEX)
		mvpp2_gmac_tx_fifo_configure(port);

	/* Disable Legacy WRR, Disable EJP, Release from reset */
	tx_port_num = mvpp2_egress_port(port);
	mvpp2_write(port->priv, MVPP2_TXP_SCHED_PORT_INDEX_REG,
		    tx_port_num);
	mvpp2_write(port->priv, MVPP2_TXP_SCHED_CMD_1_REG, 0);

	/* Set TXQ scheduling to Round-Robin */
	mvpp2_write(port->priv, MVPP2_TXP_SCHED_FIXED_PRIO_REG, 0);

	/* Close bandwidth for all queues */
	for (queue = 0; queue < MVPP2_MAX_TXQ; queue++) {
		ptxq = mvpp2_txq_phys(port->id, queue);
		mvpp2_write(port->priv,
			    MVPP2_TXQ_SCHED_TOKEN_CNTR_REG(ptxq), 0);
	}

	/* Set refill period to 1 usec, refill tokens
	 * and bucket size to maximum
	 */
	mvpp2_write(port->priv, MVPP2_TXP_SCHED_PERIOD_REG,
		    port->priv->tclk / USEC_PER_SEC);
	val = mvpp2_read(port->priv, MVPP2_TXP_SCHED_REFILL_REG);
	val &= ~MVPP2_TXP_REFILL_PERIOD_ALL_MASK;
	val |= MVPP2_TXP_REFILL_PERIOD_MASK(1);
	val |= MVPP2_TXP_REFILL_TOKENS_ALL_MASK;
	mvpp2_write(port->priv, MVPP2_TXP_SCHED_REFILL_REG, val);
	val = MVPP2_TXP_TOKEN_SIZE_MAX;
	mvpp2_write(port->priv, MVPP2_TXP_SCHED_TOKEN_SIZE_REG, val);

	/* Set MaximumLowLatencyPacketSize value to 256 */
	mvpp2_write(port->priv, MVPP2_RX_CTRL_REG(port->id),
		    MVPP2_RX_USE_PSEUDO_FOR_CSUM_MASK |
		    MVPP2_RX_LOW_LATENCY_PKT_SIZE(256));

	/* Enable Rx cache snoop */
	for (lrxq = 0; lrxq < port->nrxqs; lrxq++) {
		queue = port->rxqs[lrxq]->id;
		val = mvpp2_read(port->priv, MVPP2_RXQ_CONFIG_REG(queue));
		val |= MVPP2_SNOOP_PKT_SIZE_MASK |
			   MVPP2_SNOOP_BUF_HDR_MASK;
		mvpp2_write(port->priv, MVPP2_RXQ_CONFIG_REG(queue), val);
	}

	/* At default, mask all interrupts to all present cpus */
	mvpp2_interrupts_disable(port);
}

/* Enable/disable receiving packets */
static void mvpp2_ingress_enable(struct mvpp2_port *port)
{
	u32 val;
	int lrxq, queue;

	for (lrxq = 0; lrxq < port->nrxqs; lrxq++) {
		queue = port->rxqs[lrxq]->id;
		val = mvpp2_read(port->priv, MVPP2_RXQ_CONFIG_REG(queue));
		val &= ~MVPP2_RXQ_DISABLE_MASK;
		mvpp2_write(port->priv, MVPP2_RXQ_CONFIG_REG(queue), val);
	}
}

static void mvpp2_ingress_disable(struct mvpp2_port *port)
{
	u32 val;
	int lrxq, queue;

	for (lrxq = 0; lrxq < port->nrxqs; lrxq++) {
		queue = port->rxqs[lrxq]->id;
		val = mvpp2_read(port->priv, MVPP2_RXQ_CONFIG_REG(queue));
		val |= MVPP2_RXQ_DISABLE_MASK;
		mvpp2_write(port->priv, MVPP2_RXQ_CONFIG_REG(queue), val);
	}
}

/* Enable transmit via physical egress queue
 * - HW starts take descriptors from DRAM
 */
static void mvpp2_egress_enable(struct mvpp2_port *port)
{
	u32 qmap;
	int queue;
	int tx_port_num = mvpp2_egress_port(port);

	if (port->flags & MVPP22_F_IF_MUSDK)
		return;

	/* Enable all initialized TXs. */
	qmap = 0;
	for (queue = 0; queue < port->ntxqs; queue++) {
		struct mvpp2_tx_queue *txq = port->txqs[queue];

		if (txq->descs)
			qmap |= (1 << queue);
	}

	mvpp2_write(port->priv, MVPP2_TXP_SCHED_PORT_INDEX_REG, tx_port_num);
	mvpp2_write(port->priv, MVPP2_TXP_SCHED_Q_CMD_REG, qmap);
}

/* Disable transmit via physical egress queue
 * - HW doesn't take descriptors from DRAM
 */
static void mvpp2_egress_disable(struct mvpp2_port *port)
{
	u32 reg_data;
	int delay;
	int tx_port_num = mvpp2_egress_port(port);

	if (port->flags & MVPP22_F_IF_MUSDK)
		return;

	/* Issue stop command for active channels only */
	mvpp2_write(port->priv, MVPP2_TXP_SCHED_PORT_INDEX_REG, tx_port_num);
	reg_data = (mvpp2_read(port->priv, MVPP2_TXP_SCHED_Q_CMD_REG)) &
		    MVPP2_TXP_SCHED_ENQ_MASK;
	if (reg_data != 0)
		mvpp2_write(port->priv, MVPP2_TXP_SCHED_Q_CMD_REG,
			    (reg_data << MVPP2_TXP_SCHED_DISQ_OFFSET));

	/* Wait for all Tx activity to terminate. */
	delay = 0;
	do {
		if (delay >= MVPP2_TX_DISABLE_TIMEOUT_MSEC) {
			netdev_warn(port->dev,
				    "Tx stop timed out, status=0x%08x\n",
				    reg_data);
			break;
		}
		mdelay(1);
		delay++;

		/* Check port TX Command register that all
		 * Tx queues are stopped
		 */
		reg_data = mvpp2_read(port->priv, MVPP2_TXP_SCHED_Q_CMD_REG);
	} while (reg_data & MVPP2_TXP_SCHED_ENQ_MASK);
}

/* Rx descriptors helper methods */

/* Get number of Rx descriptors occupied by received packets */
static inline int
mvpp2_rxq_received(struct mvpp2_port *port, int rxq_id)
{
	u32 val = mvpp2_read(port->priv, MVPP2_RXQ_STATUS_REG(rxq_id));

	return val & MVPP2_RXQ_OCCUPIED_MASK;
}

/* Update Rx queue status with the number of occupied and available
 * Rx descriptor slots.
 */
static inline void
mvpp2_rxq_status_update(struct mvpp2_port *port, int rxq_id,
			int used_count, int free_count)
{
	/* Decrement the number of used descriptors and increment count
	 * increment the number of free descriptors.
	 */
	u32 val = used_count | (free_count << MVPP2_RXQ_NUM_NEW_OFFSET);

	mvpp2_write(port->priv, MVPP2_RXQ_STATUS_UPDATE_REG(rxq_id), val);
}

/* Get pointer to next RX descriptor to be processed by SW */
static inline struct mvpp2_rx_desc *
mvpp2_rxq_next_desc_get(struct mvpp2_rx_queue *rxq)
{
	int rx_desc = rxq->next_desc_to_proc;

	rxq->next_desc_to_proc = MVPP2_QUEUE_NEXT_DESC(rxq, rx_desc);
	prefetch(rxq->descs + rxq->next_desc_to_proc);
	return rxq->descs + rx_desc;
}

/* Set rx queue offset */
static void mvpp2_rxq_offset_set(struct mvpp2_port *port,
				 int prxq, int offset)
{
	u32 val;

	/* Convert offset from bytes to units of 32 bytes */
	offset = offset >> 5;

	val = mvpp2_read(port->priv, MVPP2_RXQ_CONFIG_REG(prxq));
	val &= ~MVPP2_RXQ_PACKET_OFFSET_MASK;

	/* Offset is in */
	val |= ((offset << MVPP2_RXQ_PACKET_OFFSET_OFFS) &
		    MVPP2_RXQ_PACKET_OFFSET_MASK);

	mvpp2_write(port->priv, MVPP2_RXQ_CONFIG_REG(prxq), val);
}

/* Tx descriptors helper methods */

/* Get pointer to next Tx descriptor to be processed (send) by HW */
static struct mvpp2_tx_desc *
mvpp2_txq_next_desc_get(struct mvpp2_tx_queue *txq)
{
	int tx_desc = txq->next_desc_to_proc;

	txq->next_desc_to_proc = MVPP2_QUEUE_NEXT_DESC(txq, tx_desc);
	return txq->descs + tx_desc;
}

/* Update HW with number of aggregated Tx descriptors to be sent
 *
 * Called only from mvpp2_tx(), so migration is disabled, using
 * smp_processor_id() is OK.
 */
static void mvpp2_aggr_txq_pend_desc_add(struct mvpp2_port *port, int pending)
{
	int cpu = smp_processor_id();

	mvpp2_tx_done_guard_timer_set(port, cpu);

	/* aggregated access - relevant TXQ number is written in TX desc */
	mvpp2_thread_write(port->priv,
			   mvpp2_cpu_to_thread(port->priv, cpu),
			   MVPP2_AGGR_TXQ_UPDATE_REG, pending);
}

/* Check if there are enough free descriptors in aggregated txq.
 * If not, update the number of occupied descriptors and repeat the check.
 *
 * Called only from mvpp2_tx(), so migration is disabled, using
 * smp_processor_id() is OK.
 */
static int mvpp2_aggr_desc_num_check(struct mvpp2_port *port,
				     struct mvpp2_tx_queue *aggr_txq, int num)
{
	if ((aggr_txq->count + num) > MVPP2_AGGR_TXQ_SIZE) {
		/* Update number of occupied aggregated Tx descriptors */
		unsigned int thread =
			mvpp2_cpu_to_thread(port->priv, smp_processor_id());
		u32 val = mvpp2_read_relaxed(port->priv,
					     MVPP2_AGGR_TXQ_STATUS_REG(thread));

		aggr_txq->count = val & MVPP2_AGGR_TXQ_PENDING_MASK;

		if ((aggr_txq->count + num) > MVPP2_AGGR_TXQ_SIZE)
			return -ENOMEM;
	}
	return 0;
}

/* Reserved Tx descriptors allocation request
 *
 * Called only from mvpp2_txq_reserved_desc_num_proc(), itself called
 * only by mvpp2_tx(), so migration is disabled, using
 * smp_processor_id() is OK.
 */
static int mvpp2_txq_alloc_reserved_desc(struct mvpp2_port *port,
					 struct mvpp2_tx_queue *txq, int num)
{
	unsigned int thread = mvpp2_cpu_to_thread(port->priv, smp_processor_id());
	struct mvpp2 *priv = port->priv;
	u32 val;

	val = (txq->id << MVPP2_TXQ_RSVD_REQ_Q_OFFSET) | num;
	mvpp2_thread_write_relaxed(priv, thread, MVPP2_TXQ_RSVD_REQ_REG, val);

	val = mvpp2_thread_read_relaxed(priv, thread, MVPP2_TXQ_RSVD_RSLT_REG);

	return val & MVPP2_TXQ_RSVD_RSLT_MASK;
}

/* Check if there are enough reserved descriptors for transmission.
 * If not, request chunk of reserved descriptors and check again.
 */
static int mvpp2_txq_reserved_desc_num_proc(struct mvpp2_port *port,
					    struct mvpp2_tx_queue *txq,
					    struct mvpp2_txq_pcpu *txq_pcpu,
					    int num)
{
	unsigned int thread;
	int req, desc_count;
	struct mvpp2_txq_pcpu *txq_pcpu_aux;

	if (txq_pcpu->reserved_num >= num)
		return 0;

	/* Not enough descriptors reserved! Update the reserved descriptor
	 * count and check again.
	 */
	if (num <= MAX_SKB_FRAGS) {
		req = MVPP2_CPU_DESC_CHUNK;
	} else {
		/* Compute total of used descriptors */
		desc_count = 0;
		for (thread = 0; thread < port->priv->nthreads; thread++) {
			txq_pcpu_aux = per_cpu_ptr(txq->pcpu, thread);
			desc_count += txq_pcpu_aux->reserved_num;
		}
		req = max(MVPP2_CPU_DESC_CHUNK, num - txq_pcpu->reserved_num);
		/* Check the reservation is possible */
		if ((desc_count + req) > txq->size)
			return -ENOMEM;
	}

	txq_pcpu->reserved_num += mvpp2_txq_alloc_reserved_desc(port, txq, req);

	/* Check the resulting reservation is enough */
	if (txq_pcpu->reserved_num < num)
		return -ENOMEM;
	return 0;
}

/* Release the last allocated Tx descriptor. Useful to handle DMA
 * mapping failures in the Tx path.
 */
static void mvpp2_txq_desc_put(struct mvpp2_tx_queue *txq)
{
	if (txq->next_desc_to_proc == 0)
		txq->next_desc_to_proc = txq->last_desc - 1;
	else
		txq->next_desc_to_proc--;
}

/* Set Tx descriptors fields relevant for CSUM calculation */
static u32 mvpp2_txq_desc_csum(int l3_offs, __be16 l3_proto,
			       int ip_hdr_len, int l4_proto)
{
	u32 command;

	/* fields: L3_offset, IP_hdrlen, L3_type, G_IPv4_chk,
	 * G_L4_chk, L4_type required only for checksum calculation
	 */
	command = (l3_offs << MVPP2_TXD_L3_OFF_SHIFT);
	command |= (ip_hdr_len << MVPP2_TXD_IP_HLEN_SHIFT);
	command |= MVPP2_TXD_IP_CSUM_DISABLE;

	if (l3_proto == htons(ETH_P_IP)) {
		command &= ~MVPP2_TXD_IP_CSUM_DISABLE;	/* enable IPv4 csum */
		command &= ~MVPP2_TXD_L3_IP6;		/* enable IPv4 */
	} else {
		command |= MVPP2_TXD_L3_IP6;		/* enable IPv6 */
	}

	if (l4_proto == IPPROTO_TCP) {
		command &= ~MVPP2_TXD_L4_UDP;		/* enable TCP */
		command &= ~MVPP2_TXD_L4_CSUM_FRAG;	/* generate L4 csum */
	} else if (l4_proto == IPPROTO_UDP) {
		command |= MVPP2_TXD_L4_UDP;		/* enable UDP */
		command &= ~MVPP2_TXD_L4_CSUM_FRAG;	/* generate L4 csum */
	} else {
		command |= MVPP2_TXD_L4_CSUM_NOT;
	}

	return command;
}

/* Get number of sent descriptors and decrement counter.
 * The number of sent descriptors is returned.
 * Per-thread access
 *
 * Called only from mvpp2_txq_done(), called from mvpp2_tx()
 * (migration disabled) and from the TX completion tasklet (migration
 * disabled) so using smp_processor_id() is OK.
 */
static inline int mvpp2_txq_sent_desc_proc(struct mvpp2_port *port,
					   struct mvpp2_tx_queue *txq)
{
	u32 val;

	/* Reading status reg resets transmitted descriptor counter */
	val = mvpp2_thread_read_relaxed(port->priv,
					mvpp2_cpu_to_thread(port->priv, smp_processor_id()),
					MVPP2_TXQ_SENT_REG(txq->id));

	return (val & MVPP2_TRANSMITTED_COUNT_MASK) >>
		MVPP2_TRANSMITTED_COUNT_OFFSET;
}

/* Called through on_each_cpu(), so runs on all CPUs, with migration
 * disabled, therefore using smp_processor_id() is OK.
 */
static void mvpp2_txq_sent_counter_clear(void *arg)
{
	struct mvpp2_port *port = arg;
	int queue;

	/* If the thread isn't used, don't do anything */
	if (smp_processor_id() > port->priv->nthreads)
		return;

	for (queue = 0; queue < port->ntxqs; queue++) {
		int id = port->txqs[queue]->id;

		mvpp2_thread_read(port->priv,
				  mvpp2_cpu_to_thread(port->priv, smp_processor_id()),
				  MVPP2_TXQ_SENT_REG(id));
	}
}

/* Avoid wrong tx_done calling for netif_tx_wake at time of
 * dev-stop or linkDown processing by flag MVPP2_F_IF_TX_ON.
 * Set/clear it on each cpu.
 */
static inline bool mvpp2_tx_stopped(struct mvpp2_port *port)
{
	return !(port->flags & MVPP2_F_IF_TX_ON);
}

static void mvpp2_txqs_on(void *arg)
{
	((struct mvpp2_port *)arg)->flags |= MVPP2_F_IF_TX_ON;
}

static void mvpp2_txqs_off(void *arg)
{
	((struct mvpp2_port *)arg)->flags &= ~MVPP2_F_IF_TX_ON;
}

static void mvpp2_txqs_on_tasklet_cb(unsigned long data)
{
	/* Activated/runs on 1 cpu only (with link_status_irq)
	 * to update/guarantee TX_ON coherency on other cpus
	 */
	struct mvpp2_port *port = (struct mvpp2_port *)data;

	if (mvpp2_tx_stopped(port))
		on_each_cpu(mvpp2_txqs_off, port, 1);
	else
		on_each_cpu(mvpp2_txqs_on, port, 1);
}

static void mvpp2_txqs_on_tasklet_init(struct mvpp2_port *port)
{
	/* Init called only for port with link_status_isr */
	tasklet_init(&port->txqs_on_tasklet,
		     mvpp2_txqs_on_tasklet_cb,
		     (unsigned long)port);
}

static void mvpp2_txqs_on_tasklet_kill(struct mvpp2_port *port)
{
	if (port->txqs_on_tasklet.func)
		tasklet_kill(&port->txqs_on_tasklet);
}

/* Use mvpp2 APIs instead of netif_TX_ALL:
 *  netif_tx_start_all_queues -> mvpp2_tx_start_all_queues
 *  netif_tx_wake_all_queues  -> mvpp2_tx_wake_all_queues
 *  netif_tx_stop_all_queues  -> mvpp2_tx_stop_all_queues
 * But keep using per-queue APIs netif_tx_wake_queue,
 *  netif_tx_stop_queue and netif_tx_queue_stopped.
 */
static void mvpp2_tx_start_all_queues(struct net_device *dev)
{
	struct mvpp2_port *port = netdev_priv(dev);

	if (port->flags & MVPP22_F_IF_MUSDK)
		return;

	/* Never called from IRQ. Update all cpus directly */
	on_each_cpu(mvpp2_txqs_on, port, 1);
	netif_tx_start_all_queues(dev);
}

static void mvpp2_tx_wake_all_queues(struct net_device *dev)
{
	struct mvpp2_port *port = netdev_priv(dev);

	if (port->flags & MVPP22_F_IF_MUSDK)
		return;

	if (irqs_disabled()) {
		/* Link-status IRQ context (also ACPI).
		 * Set for THIS cpu, update other cpus over tasklet
		 */
		mvpp2_txqs_on((void *)port);
		tasklet_schedule(&port->txqs_on_tasklet);
	} else {
		on_each_cpu(mvpp2_txqs_on, port, 1);
	}
	netif_tx_wake_all_queues(dev);
}

static void mvpp2_tx_stop_all_queues(struct net_device *dev)
{
	struct mvpp2_port *port = netdev_priv(dev);

	if (port->flags & MVPP22_F_IF_MUSDK)
		return;

	if (irqs_disabled()) {
		/* IRQ context. Set for THIS, update other cpus over tasklet */
		mvpp2_txqs_off((void *)port);
		tasklet_schedule(&port->txqs_on_tasklet);
	} else {
		on_each_cpu(mvpp2_txqs_off, port, 1);
	}
	netif_tx_stop_all_queues(dev);
}

/* Set max sizes for Tx queues */
static void mvpp2_txp_max_tx_size_set(struct mvpp2_port *port)
{
	u32	val, size, mtu;
	int	txq, tx_port_num;

	mtu = port->pkt_size * 8;
	if (mtu > MVPP2_TXP_MTU_MAX)
		mtu = MVPP2_TXP_MTU_MAX;

	/* WA for wrong Token bucket update: Set MTU value = 3*real MTU value */
	mtu = 3 * mtu;

	/* Indirect access to registers */
	tx_port_num = mvpp2_egress_port(port);
	mvpp2_write(port->priv, MVPP2_TXP_SCHED_PORT_INDEX_REG, tx_port_num);

	/* Set MTU */
	val = mvpp2_read(port->priv, MVPP2_TXP_SCHED_MTU_REG);
	val &= ~MVPP2_TXP_MTU_MAX;
	val |= mtu;
	mvpp2_write(port->priv, MVPP2_TXP_SCHED_MTU_REG, val);

	/* TXP token size and all TXQs token size must be larger that MTU */
	val = mvpp2_read(port->priv, MVPP2_TXP_SCHED_TOKEN_SIZE_REG);
	size = val & MVPP2_TXP_TOKEN_SIZE_MAX;
	if (size < mtu) {
		size = mtu;
		val &= ~MVPP2_TXP_TOKEN_SIZE_MAX;
		val |= size;
		mvpp2_write(port->priv, MVPP2_TXP_SCHED_TOKEN_SIZE_REG, val);
	}

	for (txq = 0; txq < port->ntxqs; txq++) {
		val = mvpp2_read(port->priv,
				 MVPP2_TXQ_SCHED_TOKEN_SIZE_REG(txq));
		size = val & MVPP2_TXQ_TOKEN_SIZE_MAX;

		if (size < mtu) {
			size = mtu;
			val &= ~MVPP2_TXQ_TOKEN_SIZE_MAX;
			val |= size;
			mvpp2_write(port->priv,
				    MVPP2_TXQ_SCHED_TOKEN_SIZE_REG(txq),
				    val);
		}
	}
}

/* Routine set the number of non-occupied descriptors threshold that change
 * interrupt error cause polled by FW Flow Control
 */
void mvpp2_set_rxq_free_tresh(struct mvpp2_port *port,
			      struct mvpp2_rx_queue *rxq)
{
	u32 val;

	mvpp2_write(port->priv, MVPP2_RXQ_NUM_REG, rxq->id);

	val = mvpp2_read(port->priv, MVPP2_RXQ_THRESH_REG);
	val &= ~MVPP2_RXQ_NON_OCCUPIED_MASK;
	val |= MSS_THRESHOLD_STOP << MVPP2_RXQ_NON_OCCUPIED_OFFSET;
	mvpp2_write(port->priv, MVPP2_RXQ_THRESH_REG, val);
}

/* Set the number of packets that will be received before Rx interrupt
 * will be generated by HW.
 */
static void mvpp2_rx_pkts_coal_set(struct mvpp2_port *port,
				   struct mvpp2_rx_queue *rxq)
{
	unsigned int thread = mvpp2_cpu_to_thread(port->priv, get_cpu());

	if (rxq->pkts_coal > MVPP2_OCCUPIED_THRESH_MASK)
		rxq->pkts_coal = MVPP2_OCCUPIED_THRESH_MASK;

	mvpp2_thread_write(port->priv, thread, MVPP2_RXQ_NUM_REG, rxq->id);
	mvpp2_thread_write(port->priv, thread, MVPP2_RXQ_THRESH_REG,
			   rxq->pkts_coal);

	put_cpu();
}

/* Set pkts-coalescing HW with ZERO or configured VALUE
 * The same should be set for all TXQs and all for all CPUs.
 * Setting ZERO causes for immediate flush into tx-done handler.
 */
static inline void mvpp2_tx_pkts_coal_set_txqs(struct mvpp2_port *port,
					       int cpu, u32 val)
{
	struct mvpp2_tx_queue *txq;
	int queue;

	val <<= MVPP2_TXQ_THRESH_OFFSET;

	for (queue = 0; queue < port->ntxqs; queue++) {
		txq = port->txqs[queue];
		mvpp2_thread_write(port->priv, cpu, MVPP2_TXQ_NUM_REG,
				   txq->id);
		mvpp2_thread_write(port->priv, cpu, MVPP2_TXQ_THRESH_REG, val);
	}
}

static void mvpp2_tx_pkts_coal_set(struct mvpp2_port *port)
{
	struct mvpp2_tx_queue *txq = port->txqs[0];
	u32 cfg_val = txq->done_pkts_coal;
	int cpu;

	for_each_present_cpu(cpu)
		mvpp2_tx_pkts_coal_set_txqs(port, cpu, cfg_val);
}

/* Set ZERO value on on_each_cpu IRQ-context for 1 cpu only */
static void mvpp2_tx_pkts_coal_set_zero_pcpu(void *arg)
{
	struct mvpp2_port *port = arg;

	mvpp2_tx_pkts_coal_set_txqs(port, smp_processor_id(), 0);
}

static u32 mvpp2_usec_to_cycles(u32 usec, unsigned long clk_hz)
{
	u64 tmp = (u64)clk_hz * usec;

	do_div(tmp, USEC_PER_SEC);

	return tmp > U32_MAX ? U32_MAX : tmp;
}

static u32 mvpp2_cycles_to_usec(u32 cycles, unsigned long clk_hz)
{
	u64 tmp = (u64)cycles * USEC_PER_SEC;

	do_div(tmp, clk_hz);

	return tmp > U32_MAX ? U32_MAX : tmp;
}

/* Set the time delay in usec before Rx interrupt */
static void mvpp2_rx_time_coal_set(struct mvpp2_port *port,
				   struct mvpp2_rx_queue *rxq)
{
	unsigned long freq = port->priv->tclk;
	u32 val = mvpp2_usec_to_cycles(rxq->time_coal, freq);

	if (val > MVPP2_MAX_ISR_RX_THRESHOLD) {
		rxq->time_coal =
			mvpp2_cycles_to_usec(MVPP2_MAX_ISR_RX_THRESHOLD, freq);

		/* re-evaluate to get actual register value */
		val = mvpp2_usec_to_cycles(rxq->time_coal, freq);
	}

	mvpp2_write(port->priv, MVPP2_ISR_RX_THRESHOLD_REG(rxq->id), val);
}

static void mvpp2_tx_time_coal_set(struct mvpp2_port *port)
{
	unsigned long freq = port->priv->tclk;
	u32 val = mvpp2_usec_to_cycles(port->tx_time_coal, freq);

	if (val > MVPP2_MAX_ISR_TX_THRESHOLD) {
		port->tx_time_coal =
			mvpp2_cycles_to_usec(MVPP2_MAX_ISR_TX_THRESHOLD, freq);

		/* re-evaluate to get actual register value */
		val = mvpp2_usec_to_cycles(port->tx_time_coal, freq);
	}

	mvpp2_write(port->priv, MVPP2_ISR_TX_THRESHOLD_REG(port->id), val);
}

/* Free Tx queue skbuffs */
static void mvpp2_txq_bufs_free(struct mvpp2_port *port,
				struct mvpp2_tx_queue *txq,
				struct mvpp2_txq_pcpu *txq_pcpu, int num)
{
	int i;

	for (i = 0; i < num; i++) {
		struct mvpp2_txq_pcpu_buf *tx_buf =
			txq_pcpu->buffs + txq_pcpu->txq_get_index;

		if (!tx_buf->skb) {
			dma_unmap_single(port->dev->dev.parent, tx_buf->dma,
					 tx_buf->size, DMA_TO_DEVICE);
		} else if (tx_buf->skb != TSO_HEADER_MARK) {
			dma_unmap_single(port->dev->dev.parent, tx_buf->dma,
					 tx_buf->size, DMA_TO_DEVICE);
			if (static_branch_unlikely(&mvpp2_recycle_ena)) {
				mvpp2_recycle_put(port, txq_pcpu, tx_buf);
				/* sets tx_buf->skb=NULL if put to recycle */
				if (tx_buf->skb)
					dev_kfree_skb_any(tx_buf->skb);
			} else {
				dev_kfree_skb_any(tx_buf->skb);
			}
		}
		/* else: no action, tx_buf->skb always overwritten in xmit */
		mvpp2_txq_inc_get(txq_pcpu);
	}
}

static inline struct mvpp2_rx_queue *mvpp2_get_rx_queue(struct mvpp2_port *port,
							u32 cause)
{
	int queue = fls(cause) - 1;

	return port->rxqs[queue];
}

static inline struct mvpp2_tx_queue *mvpp2_get_tx_queue(struct mvpp2_port *port,
							u32 cause)
{
	int queue = fls(cause) - 1;

	return port->txqs[queue];
}

/* Handle end of transmission */
static void mvpp2_txq_done(struct mvpp2_port *port, struct mvpp2_tx_queue *txq,
			   struct mvpp2_txq_pcpu *txq_pcpu)
{
	struct netdev_queue *nq = netdev_get_tx_queue(port->dev, txq->log_id);
	int tx_done;

	if (txq_pcpu->thread != mvpp2_cpu_to_thread(port->priv, smp_processor_id()))
		netdev_err(port->dev, "wrong cpu on the end of Tx processing\n");

	tx_done = mvpp2_txq_sent_desc_proc(port, txq);
	if (!tx_done)
		return;
	mvpp2_txq_bufs_free(port, txq, txq_pcpu, tx_done);

	txq_pcpu->count -= tx_done;

	if (netif_tx_queue_stopped(nq) && !mvpp2_tx_stopped(port)) {
		/* Wake if netif_tx_queue_stopped on same txq->log_id */
		if (txq_pcpu->stopped_on_txq_id == txq->log_id &&
		    txq_pcpu->count <= txq_pcpu->wake_threshold) {
			txq_pcpu->stopped_on_txq_id = MVPP2_MAX_TXQ;
			nq = netdev_get_tx_queue(port->dev, txq->log_id);
			netif_tx_wake_queue(nq);
		}
	}
}

static unsigned int mvpp2_tx_done(struct mvpp2_port *port, u32 cause,
				  unsigned int thread)
{
	struct mvpp2_tx_queue *txq;
	struct mvpp2_txq_pcpu *txq_pcpu;
	unsigned int tx_todo = 0;

	/* Set/Restore "no-force" */
	mvpp2_tx_done_guard_force_irq(port, thread, 0);

	while (cause) {
		txq = mvpp2_get_tx_queue(port, cause);
		if (!txq)
			break;

		txq_pcpu = per_cpu_ptr(txq->pcpu, thread);

		if (txq_pcpu->count) {
			mvpp2_txq_done(port, txq, txq_pcpu);
			tx_todo += txq_pcpu->count;
		}

		cause &= ~(1 << txq->log_id);
	}
	return tx_todo;
}

/* Rx/Tx queue initialization/cleanup methods */

/* Allocate and initialize descriptors for aggr TXQ */
static int mvpp2_aggr_txq_init(struct platform_device *pdev,
			       struct mvpp2_tx_queue *aggr_txq,
			       unsigned int thread, struct mvpp2 *priv)
{
	u32 txq_dma;

	/* Allocate memory for TX descriptors */
	aggr_txq->descs = dma_alloc_coherent(&pdev->dev,
				MVPP2_AGGR_TXQ_SIZE * MVPP2_DESC_ALIGNED_SIZE,
				&aggr_txq->descs_dma, GFP_KERNEL);
	if (!aggr_txq->descs)
		return -ENOMEM;

	aggr_txq->last_desc = MVPP2_AGGR_TXQ_SIZE - 1;

	/* Aggr TXQ no reset WA */
	aggr_txq->next_desc_to_proc = mvpp2_read(priv,
						 MVPP2_AGGR_TXQ_INDEX_REG(thread));

	/* Set Tx descriptors queue starting address indirect
	 * access
	 */
	if (priv->hw_version == MVPP21)
		txq_dma = aggr_txq->descs_dma;
	else
		txq_dma = aggr_txq->descs_dma >>
			MVPP22_AGGR_TXQ_DESC_ADDR_OFFS;

	mvpp2_write(priv, MVPP2_AGGR_TXQ_DESC_ADDR_REG(thread), txq_dma);
	mvpp2_write(priv, MVPP2_AGGR_TXQ_DESC_SIZE_REG(thread),
		    MVPP2_AGGR_TXQ_SIZE);

	return 0;
}

/* Create a specified Rx queue */
static int mvpp2_rxq_init(struct mvpp2_port *port,
			  struct mvpp2_rx_queue *rxq)

{
	unsigned int thread;
	u32 rxq_dma;

	rxq->size = port->rx_ring_size;

	/* Allocate memory for RX descriptors */
	rxq->descs = dma_alloc_coherent(port->dev->dev.parent,
					rxq->size * MVPP2_DESC_ALIGNED_SIZE,
					&rxq->descs_dma, GFP_KERNEL);
	if (!rxq->descs)
		return -ENOMEM;

	rxq->last_desc = rxq->size - 1;
	rxq->rx_pending = 0;

	/* Zero occupied and non-occupied counters - direct access */
	mvpp2_write(port->priv, MVPP2_RXQ_STATUS_REG(rxq->id), 0);

	/* Set Rx descriptors queue starting address - indirect access */
	thread = mvpp2_cpu_to_thread(port->priv, get_cpu());
	mvpp2_thread_write(port->priv, thread, MVPP2_RXQ_NUM_REG, rxq->id);
	if (port->priv->hw_version == MVPP21)
		rxq_dma = rxq->descs_dma;
	else
		rxq_dma = rxq->descs_dma >> MVPP22_DESC_ADDR_OFFS;
	mvpp2_thread_write(port->priv, thread, MVPP2_RXQ_DESC_ADDR_REG, rxq_dma);
	mvpp2_thread_write(port->priv, thread, MVPP2_RXQ_DESC_SIZE_REG, rxq->size);
	mvpp2_thread_write(port->priv, thread, MVPP2_RXQ_INDEX_REG, 0);
	put_cpu();

	/* Set Offset */
	mvpp2_rxq_offset_set(port, rxq->id, NET_SKB_PAD);

	/* Set coalescing pkts and time */
	mvpp2_rx_pkts_coal_set(port, rxq);
	mvpp2_rx_time_coal_set(port, rxq);

	/* Set the number of non occupied descriptors threshold */
	mvpp2_set_rxq_free_tresh(port, rxq);

	/* Add number of descriptors ready for receiving packets */
	mvpp2_rxq_status_update(port, rxq->id, 0, rxq->size);

	return 0;
}

/* Push packets received by the RXQ to BM pool */
static void mvpp2_rxq_drop_pkts(struct mvpp2_port *port,
				struct mvpp2_rx_queue *rxq)
{
	int rx_received, i;

	rxq->rx_pending = 0;
	rx_received = mvpp2_rxq_received(port, rxq->id);
	if (!rx_received)
		return;

	for (i = 0; i < rx_received; i++) {
		struct mvpp2_rx_desc *rx_desc = mvpp2_rxq_next_desc_get(rxq);
		u32 status = mvpp2_rxdesc_status_get(port, rx_desc);
		int pool;

		pool = (status & MVPP2_RXD_BM_POOL_ID_MASK) >>
			MVPP2_RXD_BM_POOL_ID_OFFS;

		mvpp2_bm_pool_put(port, pool,
				  mvpp2_rxdesc_dma_addr_get(port, rx_desc));
	}
	mvpp2_rxq_status_update(port, rxq->id, rx_received, rx_received);
}

/* Cleanup Rx queue */
static void mvpp2_rxq_deinit(struct mvpp2_port *port,
			     struct mvpp2_rx_queue *rxq)
{
	unsigned int thread;

	mvpp2_rxq_drop_pkts(port, rxq);

	if (rxq->descs)
		dma_free_coherent(port->dev->dev.parent,
				  rxq->size * MVPP2_DESC_ALIGNED_SIZE,
				  rxq->descs,
				  rxq->descs_dma);

	rxq->descs             = NULL;
	rxq->last_desc         = 0;
	rxq->next_desc_to_proc = 0;
	rxq->descs_dma         = 0;

	/* Clear Rx descriptors queue starting address and size;
	 * free descriptor number
	 */
	mvpp2_write(port->priv, MVPP2_RXQ_STATUS_REG(rxq->id), 0);
	thread = mvpp2_cpu_to_thread(port->priv, get_cpu());
	mvpp2_thread_write(port->priv, thread, MVPP2_RXQ_NUM_REG, rxq->id);
	mvpp2_thread_write(port->priv, thread, MVPP2_RXQ_DESC_ADDR_REG, 0);
	mvpp2_thread_write(port->priv, thread, MVPP2_RXQ_DESC_SIZE_REG, 0);
	put_cpu();
}

/* Disable all rx/ingress queues, called by mvpp2_init */
static void mvpp2_rxq_disable_all(struct mvpp2 *priv)
{
	int i;
	u32 val;

	for (i = 0; i < MVPP2_RXQ_MAX_NUM; i++) {
		val = mvpp2_read(priv, MVPP2_RXQ_CONFIG_REG(i));
		val |= MVPP2_RXQ_DISABLE_MASK;
		mvpp2_write(priv, MVPP2_RXQ_CONFIG_REG(i), val);
	}
}

/* Create and initialize a Tx queue */
static int mvpp2_txq_init(struct mvpp2_port *port,
			  struct mvpp2_tx_queue *txq)
{
	u32 val;
	unsigned int thread;
	int desc, desc_per_txq, tx_port_num;
	struct mvpp2_txq_pcpu *txq_pcpu;

	txq->size = port->tx_ring_size;

	/* Allocate memory for Tx descriptors */
	txq->descs = dma_alloc_coherent(port->dev->dev.parent,
				txq->size * MVPP2_DESC_ALIGNED_SIZE,
				&txq->descs_dma, GFP_KERNEL);
	if (!txq->descs)
		return -ENOMEM;

	txq->last_desc = txq->size - 1;

	/* Set Tx descriptors queue starting address - indirect access */
	thread = mvpp2_cpu_to_thread(port->priv, get_cpu());
	mvpp2_thread_write(port->priv, thread, MVPP2_TXQ_NUM_REG, txq->id);
	mvpp2_thread_write(port->priv, thread, MVPP2_TXQ_DESC_ADDR_REG,
			   txq->descs_dma);
	mvpp2_thread_write(port->priv, thread, MVPP2_TXQ_DESC_SIZE_REG,
			   txq->size & MVPP2_TXQ_DESC_SIZE_MASK);
	mvpp2_thread_write(port->priv, thread, MVPP2_TXQ_INDEX_REG, 0);
	mvpp2_thread_write(port->priv, thread, MVPP2_TXQ_RSVD_CLR_REG,
			   txq->id << MVPP2_TXQ_RSVD_CLR_OFFSET);
	val = mvpp2_thread_read(port->priv, thread, MVPP2_TXQ_PENDING_REG);
	val &= ~MVPP2_TXQ_PENDING_MASK;
	mvpp2_thread_write(port->priv, thread, MVPP2_TXQ_PENDING_REG, val);

	/* Calculate base address in prefetch buffer. We reserve 16 descriptors
	 * for each existing TXQ.
	 * TCONTS for PON port must be continuous from 0 to MVPP2_MAX_TCONT
	 * GBE ports assumed to be continuous from 0 to MVPP2_MAX_PORTS
	 */
	desc_per_txq = 16;
	desc = (port->id * MVPP2_MAX_TXQ * desc_per_txq) +
	       (txq->log_id * desc_per_txq);

	mvpp2_thread_write(port->priv, thread, MVPP2_TXQ_PREF_BUF_REG,
			   MVPP2_PREF_BUF_PTR(desc) | MVPP2_PREF_BUF_SIZE_16 |
			   MVPP2_PREF_BUF_THRESH(desc_per_txq / 2));
	put_cpu();

	/* WRR / EJP configuration - indirect access */
	tx_port_num = mvpp2_egress_port(port);
	mvpp2_write(port->priv, MVPP2_TXP_SCHED_PORT_INDEX_REG, tx_port_num);

	val = mvpp2_read(port->priv, MVPP2_TXQ_SCHED_REFILL_REG(txq->log_id));
	val &= ~MVPP2_TXQ_REFILL_PERIOD_ALL_MASK;
	val |= MVPP2_TXQ_REFILL_PERIOD_MASK(1);
	val |= MVPP2_TXQ_REFILL_TOKENS_ALL_MASK;
	mvpp2_write(port->priv, MVPP2_TXQ_SCHED_REFILL_REG(txq->log_id), val);

	val = MVPP2_TXQ_TOKEN_SIZE_MAX;
	mvpp2_write(port->priv, MVPP2_TXQ_SCHED_TOKEN_SIZE_REG(txq->log_id),
		    val);

	for (thread = 0; thread < port->priv->nthreads; thread++) {
		txq_pcpu = per_cpu_ptr(txq->pcpu, thread);
		txq_pcpu->size = txq->size;
		txq_pcpu->buffs = kmalloc_array(txq_pcpu->size,
						sizeof(*txq_pcpu->buffs),
						GFP_KERNEL);
		if (!txq_pcpu->buffs)
			return -ENOMEM;

		txq_pcpu->count = 0;
		txq_pcpu->reserved_num = 0;
		txq_pcpu->txq_put_index = 0;
		txq_pcpu->txq_get_index = 0;
		txq_pcpu->tso_headers = NULL;

		txq_pcpu->stop_threshold = txq->size -
				MVPP2_MAX_SKB_DESCS(num_present_cpus());
		txq_pcpu->wake_threshold = txq_pcpu->stop_threshold -
						MVPP2_TX_PAUSE_HYSTERESIS;
		txq_pcpu->stopped_on_txq_id = MVPP2_MAX_TXQ;

		txq_pcpu->tso_headers =
			dma_alloc_coherent(port->dev->dev.parent,
					   txq_pcpu->size * TSO_HEADER_SIZE,
					   &txq_pcpu->tso_headers_dma,
					   GFP_KERNEL);
		if (!txq_pcpu->tso_headers)
			return -ENOMEM;
	}

	return 0;
}

/* Free allocated TXQ resources */
static void mvpp2_txq_deinit(struct mvpp2_port *port,
			     struct mvpp2_tx_queue *txq)
{
	struct mvpp2_txq_pcpu *txq_pcpu;
	unsigned int thread;

	for (thread = 0; thread < port->priv->nthreads; thread++) {
		txq_pcpu = per_cpu_ptr(txq->pcpu, thread);
		kfree(txq_pcpu->buffs);

		if (txq_pcpu->tso_headers)
			dma_free_coherent(port->dev->dev.parent,
					  txq_pcpu->size * TSO_HEADER_SIZE,
					  txq_pcpu->tso_headers,
					  txq_pcpu->tso_headers_dma);

		txq_pcpu->tso_headers = NULL;
	}

	if (txq->descs)
		dma_free_coherent(port->dev->dev.parent,
				  txq->size * MVPP2_DESC_ALIGNED_SIZE,
				  txq->descs, txq->descs_dma);

	txq->descs             = NULL;
	txq->last_desc         = 0;
	txq->next_desc_to_proc = 0;
	txq->descs_dma         = 0;

	/* Set minimum bandwidth for disabled TXQs */
	mvpp2_write(port->priv, MVPP2_TXQ_SCHED_TOKEN_CNTR_REG(txq->id), 0);

	/* Set Tx descriptors queue starting address and size */
	thread = mvpp2_cpu_to_thread(port->priv, get_cpu());
	mvpp2_thread_write(port->priv, thread, MVPP2_TXQ_NUM_REG, txq->id);
	mvpp2_thread_write(port->priv, thread, MVPP2_TXQ_DESC_ADDR_REG, 0);
	mvpp2_thread_write(port->priv, thread, MVPP2_TXQ_DESC_SIZE_REG, 0);
	put_cpu();
}

/* Cleanup Tx ports */
static void mvpp2_txq_clean(struct mvpp2_port *port, struct mvpp2_tx_queue *txq)
{
	struct mvpp2_txq_pcpu *txq_pcpu;
	int delay, pending;
	unsigned int thread = mvpp2_cpu_to_thread(port->priv, get_cpu());
	u32 val;

	mvpp2_thread_write(port->priv, thread, MVPP2_TXQ_NUM_REG, txq->id);
	val = mvpp2_thread_read(port->priv, thread, MVPP2_TXQ_PREF_BUF_REG);
	val |= MVPP2_TXQ_DRAIN_EN_MASK;
	mvpp2_thread_write(port->priv, thread, MVPP2_TXQ_PREF_BUF_REG, val);

	/* Temporarily enable egress for the port.
	 * It is required for releasing all remaining packets.
	 */
	mvpp2_egress_enable(port);

	/* The napi queue has been stopped so wait for all packets
	 * to be transmitted.
	 */
	delay = 0;
	do {
		if (delay >= MVPP2_TX_PENDING_TIMEOUT_MSEC) {
			netdev_warn(port->dev,
				    "port %d: cleaning queue %d timed out\n",
				    port->id, txq->log_id);
			break;
		}
		mdelay(1);
		delay++;

		pending = mvpp2_thread_read(port->priv, thread,
					    MVPP2_TXQ_PENDING_REG);
		pending &= MVPP2_TXQ_PENDING_MASK;
	} while (pending);

	mvpp2_egress_disable(port);

	val &= ~MVPP2_TXQ_DRAIN_EN_MASK;
	mvpp2_thread_write(port->priv, thread, MVPP2_TXQ_PREF_BUF_REG, val);
	put_cpu();

	for (thread = 0; thread < port->priv->nthreads; thread++) {
		txq_pcpu = per_cpu_ptr(txq->pcpu, thread);

		/* Release all packets */
		mvpp2_txq_bufs_free(port, txq, txq_pcpu, txq_pcpu->count);

		/* Reset queue */
		txq_pcpu->count = 0;
		txq_pcpu->txq_put_index = 0;
		txq_pcpu->txq_get_index = 0;
	}
}

/* Cleanup all Tx queues */
static void mvpp2_cleanup_txqs(struct mvpp2_port *port)
{
	struct mvpp2_tx_queue *txq;
	int queue;
	u32 val;

	val = mvpp2_read(port->priv, MVPP2_TX_PORT_FLUSH_REG);

	/* Reset Tx ports and delete Tx queues */
	val |= MVPP2_TX_PORT_FLUSH_MASK(port->id);
	mvpp2_write(port->priv, MVPP2_TX_PORT_FLUSH_REG, val);

	for (queue = 0; queue < port->ntxqs; queue++) {
		txq = port->txqs[queue];
		mvpp2_txq_clean(port, txq);
		mvpp2_txq_deinit(port, txq);
	}

	on_each_cpu(mvpp2_txq_sent_counter_clear, port, 1);

	val &= ~MVPP2_TX_PORT_FLUSH_MASK(port->id);
	mvpp2_write(port->priv, MVPP2_TX_PORT_FLUSH_REG, val);
}

/* Cleanup all Rx queues */
static void mvpp2_cleanup_rxqs(struct mvpp2_port *port)
{
	int queue;

	for (queue = 0; queue < port->nrxqs; queue++)
		mvpp2_rxq_deinit(port, port->rxqs[queue]);

	if (port->tx_fc)
		mvpp2_rxq_disable_fc(port);
}

/* Init all Rx queues for port */
static int mvpp2_setup_rxqs(struct mvpp2_port *port)
{
	int queue, err;

	for (queue = 0; queue < port->nrxqs; queue++) {
		err = mvpp2_rxq_init(port, port->rxqs[queue]);
		if (err)
			goto err_cleanup;
	}

	if (port->tx_fc)
		mvpp2_rxq_enable_fc(port);

	return 0;

err_cleanup:
	mvpp2_cleanup_rxqs(port);
	return err;
}

/* Init all tx queues for port */
static int mvpp2_setup_txqs(struct mvpp2_port *port)
{
	struct mvpp2_tx_queue *txq;
	int queue, err;

	for (queue = 0; queue < port->ntxqs; queue++) {
		txq = port->txqs[queue];
		err = mvpp2_txq_init(port, txq);
		if (err)
			goto err_cleanup;
	}

	if (port->has_tx_irqs) {
		/* Download time-coal. The pkts-coal done in start_dev */
		mvpp2_tx_time_coal_set(port);
	}

	on_each_cpu(mvpp2_txq_sent_counter_clear, port, 1);
	return 0;

err_cleanup:
	mvpp2_cleanup_txqs(port);
	return err;
}

/* The callback for per-port interrupt */
static irqreturn_t mvpp2_isr(int irq, void *dev_id)
{
	struct mvpp2_queue_vector *qv = dev_id;

	mvpp2_qvec_interrupt_disable(qv);

	napi_schedule(&qv->napi);

	return IRQ_HANDLED;
}

/* Per-port interrupt for link status changes */
static irqreturn_t mvpp2_link_status_isr(int irq, void *dev_id)
{
	struct mvpp2_port *port = (struct mvpp2_port *)dev_id;
	struct net_device *dev = port->dev;
	bool event = false, link = false;
	u32 val;

	mvpp22_gop_mask_irq(port);

	if (port->has_xlg_mac &&
	    (port->phy_interface == PHY_INTERFACE_MODE_RXAUI ||
	     port->phy_interface == PHY_INTERFACE_MODE_10GKR ||
	     port->phy_interface == PHY_INTERFACE_MODE_5GKR ||
	     port->phy_interface == PHY_INTERFACE_MODE_INTERNAL)) {
		val = readl(port->base + MVPP22_XLG_INT_STAT);
		if (val & MVPP22_XLG_INT_STAT_LINK) {
			event = true;
			val = readl(port->base + MVPP22_XLG_STATUS);
			if (val & MVPP22_XLG_STATUS_LINK_UP)
				link = true;
		}
	} else if (phy_interface_mode_is_rgmii(port->phy_interface) ||
		   phy_interface_mode_is_8023z(port->phy_interface) ||
		   port->phy_interface == PHY_INTERFACE_MODE_MII ||
		   port->phy_interface == PHY_INTERFACE_MODE_SGMII) {
		val = readl(port->base + MVPP22_GMAC_INT_STAT);
		if (val & MVPP22_GMAC_INT_STAT_LINK) {
			event = true;
			val = readl(port->base + MVPP2_GMAC_STATUS0);
			if (val & MVPP2_GMAC_STATUS0_LINK_UP)
				link = true;
		}
	}

	if (!netif_running(dev) || !event)
		goto handled;

	if (port->phylink) {
		phylink_mac_change(port->phylink, link);
		goto handled;
	}

	if (link) {
		mvpp2_interrupts_enable(port);

		mvpp2_egress_enable(port);
		mvpp2_ingress_enable(port);
		netif_carrier_on(dev);
		mvpp2_tx_wake_all_queues(dev);
	} else {
		mvpp2_tx_stop_all_queues(dev);
		netif_carrier_off(dev);
		mvpp2_ingress_disable(port);
		mvpp2_egress_disable(port);

		mvpp2_interrupts_disable(port);
	}

handled:
	mvpp22_gop_unmask_irq(port);
	return IRQ_HANDLED;
}

static void mvpp2_tx_done_timer_set(struct mvpp2_port_pcpu *port_pcpu)
{
	ktime_t interval;

	if (!port_pcpu->tx_done_timer_scheduled) {
		port_pcpu->tx_done_timer_scheduled = true;
		interval = MVPP2_TXDONE_HRTIMER_PERIOD_NS;
		hrtimer_start(&port_pcpu->tx_done_timer, interval,
			      HRTIMER_MODE_REL_PINNED);
	}
}

static void mvpp2_tx_done_proc_cb(unsigned long data)
{
	struct net_device *dev = (struct net_device *)data;
	struct mvpp2_port *port = netdev_priv(dev);
	struct mvpp2_port_pcpu *port_pcpu;
	unsigned int tx_todo, cause;

	port_pcpu = per_cpu_ptr(port->pcpu,
				mvpp2_cpu_to_thread(port->priv, smp_processor_id()));

	if (!netif_running(dev))
		return;
	port_pcpu->tx_done_timer_scheduled = false;

	/* Process all the Tx queues */
	cause = (1 << port->ntxqs) - 1;
	tx_todo = mvpp2_tx_done(port, cause,
				mvpp2_cpu_to_thread(port->priv, smp_processor_id()));

	/* Set the timer in case not all the packets were processed */
	if (tx_todo)
		mvpp2_tx_done_timer_set(port_pcpu);
}

static enum hrtimer_restart mvpp2_tx_done_timer_cb(struct hrtimer *timer)
{
	struct mvpp2_port_pcpu *port_pcpu = container_of(timer,
							 struct mvpp2_port_pcpu,
							 tx_done_timer);

	tasklet_schedule(&port_pcpu->tx_done_tasklet);

	return HRTIMER_NORESTART;
}

/* Bulk-timer could be started/restarted by XMIT, timer-cb or Tasklet.
 *  XMIT calls bulk-restart() which is CONDITIONAL (restart vs request).
 *  Timer-cb has own condition-logic, calls hrtimer_forward().
 *  Tasklet has own condition-logic, calls unconditional bulk-start().
 *  The flags scheduled::restart_req are used in the state-logic.
 */
static inline void mvpp2_bulk_timer_restart(struct mvpp2_port_pcpu *port_pcpu)
{
	if (!port_pcpu->bulk_timer_scheduled) {
		port_pcpu->bulk_timer_scheduled = true;
		hrtimer_start(&port_pcpu->bulk_timer, MVPP2_TX_BULK_TIME,
			      HRTIMER_MODE_REL_PINNED);
	} else {
		port_pcpu->bulk_timer_restart_req = true;
	}
}

static void mvpp2_bulk_timer_start(struct mvpp2_port_pcpu *port_pcpu)
{
	port_pcpu->bulk_timer_scheduled = true;
	port_pcpu->bulk_timer_restart_req = false;
	hrtimer_start(&port_pcpu->bulk_timer, MVPP2_TX_BULK_TIME,
		      HRTIMER_MODE_REL_PINNED);
}

static enum hrtimer_restart mvpp2_bulk_timer_cb(struct hrtimer *timer)
{
	/* ISR context */
	struct mvpp2_port_pcpu *port_pcpu =
		container_of(timer, struct mvpp2_port_pcpu, bulk_timer);

	if (!port_pcpu->bulk_timer_scheduled) {
		/* All pending are already flushed by xmit */
		return HRTIMER_NORESTART;
	}
	if (port_pcpu->bulk_timer_restart_req) {
		/* Not flushed but restart requested by xmit */
		port_pcpu->bulk_timer_scheduled = true;
		port_pcpu->bulk_timer_restart_req = false;
		hrtimer_forward_now(timer, MVPP2_TX_BULK_TIME);
		return HRTIMER_RESTART;
	}
	/* Expired and need the flush for pending */
	tasklet_schedule(&port_pcpu->bulk_tasklet);
	return HRTIMER_NORESTART;
}

static void mvpp2_bulk_tasklet_cb(unsigned long data)
{
	struct net_device *dev = (struct net_device *)data;
	struct mvpp2_port *port = netdev_priv(dev);
	struct mvpp2_port_pcpu *port_pcpu;
	struct mvpp2_tx_queue *aggr_txq;
	int frags;
	int cpu = smp_processor_id();

	port_pcpu = per_cpu_ptr(port->pcpu, cpu);

	if (!port_pcpu->bulk_timer_scheduled) {
		/* Flushed by xmit-softirq since timer-irq */
		return;
	}
	port_pcpu->bulk_timer_scheduled = false;
	if (port_pcpu->bulk_timer_restart_req) {
		/* Restart requested by xmit-softirq since timer-irq */
		mvpp2_bulk_timer_start(port_pcpu);
		return;
	}

	/* Full time expired. Flush pending packets here */
	aggr_txq = &port->priv->aggr_txqs[cpu];
	frags = aggr_txq->pending;
	if (!frags)
		return; /* Flushed by xmit */
	aggr_txq->pending -= frags;
	mvpp2_aggr_txq_pend_desc_add(port, frags);
}

/* Guard timer, tasklet, fixer utilities */

/* The Guard fixer, called for 2 opposite actions:
 *  Activate fix by set frame-coalescing to Zero (according to_zero_map)
 *     which forces the tx-done IRQ. Called by guard tasklet.
 *  Deactivate fixer ~ restore the coal-configration (to_zero_map=0)
 *    when/by tx-done activated.
 */
static void mvpp2_tx_done_guard_force_irq(struct mvpp2_port *port,
					  int sw_thread, u8 to_zero_map)
{
	int q;
	u32 val, coal, qmask, xor;
	struct mvpp2_port_pcpu *port_pcpu = per_cpu_ptr(port->pcpu, sw_thread);

	if (port_pcpu->txq_coal_is_zero_map == to_zero_map)
		return; /* all current & requested are already the same */

	xor = port_pcpu->txq_coal_is_zero_map ^ to_zero_map;
	/* Configuration num-of-frames coalescing is the same for all queues */
	coal = port->txqs[0]->done_pkts_coal << MVPP2_TXQ_THRESH_OFFSET;

	for (q = 0; q < port->ntxqs; q++) {
		qmask = 1 << q;
		if (!(xor & qmask))
			continue;
		if (to_zero_map & qmask)
			val = 0; /* Set ZERO forcing the Interrupt */
		else
			val = coal; /* Set/restore configured threshold */
		mvpp2_thread_write(port->priv, sw_thread,
				   MVPP2_TXQ_NUM_REG, port->txqs[q]->id);
		mvpp2_thread_write(port->priv, sw_thread,
				   MVPP2_TXQ_THRESH_REG, val);
	}
	port_pcpu->txq_coal_is_zero_map = to_zero_map;
}

static inline void mvpp2_tx_done_guard_timer_set(struct mvpp2_port *port,
						 int sw_thread)
{
	struct mvpp2_port_pcpu *port_pcpu = per_cpu_ptr(port->pcpu,
							sw_thread);

	if (!port_pcpu->guard_timer_scheduled) {
		port_pcpu->guard_timer_scheduled = true;
		hrtimer_start(&port_pcpu->tx_done_timer,
			      MVPP2_GUARD_TXDONE_HRTIMER_NS,
			      HRTIMER_MODE_REL_PINNED);
	}
}

/* Guard timer and tasklet callbacks making check logic upon flags
 *    guard_timer_scheduled, tx_done_passed,
 *    txq_coal_is_zero_map, txq_busy_suspect_map
 */
static enum hrtimer_restart mvpp2_guard_timer_cb(struct hrtimer *timer)
{
	struct mvpp2_port_pcpu *port_pcpu = container_of(timer,
			 struct mvpp2_port_pcpu, tx_done_timer);
	struct mvpp2_port *port = port_pcpu->port;
	struct mvpp2_tx_queue *txq;
	struct mvpp2_txq_pcpu *txq_pcpu;
	u8 txq_nonempty_map = 0;
	int q, cpu;
	ktime_t time;

	if (port_pcpu->tx_done_passed) {
		/* ok, tx-done was active since last checking */
		port_pcpu->tx_done_passed = false;
		time = MVPP2_GUARD_TXDONE_HRTIMER_NS; /* regular long */
		goto timer_restart;
	}

	cpu = smp_processor_id(); /* timer is per-cpu */

	for (q = 0; q < port->ntxqs; q++) {
		txq = port->txqs[q];
		txq_pcpu = per_cpu_ptr(txq->pcpu, cpu);
		if (txq_pcpu->count)
			txq_nonempty_map |= 1 << q;
	}

	if (!txq_nonempty_map || mvpp2_tx_stopped(port)) {
		/* All queues are empty, guard-timer may be stopped now
		 * It would be started again on new transmit.
		 */
		port_pcpu->guard_timer_scheduled = false;
		return HRTIMER_NORESTART;
	}

	if (port_pcpu->txq_busy_suspect_map) {
		/* Second-hit ~~ tx-done is really stalled.
		 * Activate the tasklet to fix.
		 * Keep guard_timer_scheduled=TRUE
		 */
		tasklet_schedule(&port_pcpu->tx_done_tasklet);
		return HRTIMER_NORESTART;
	}

	/* First-hit ~~ tx-done seems stalled. Schedule re-check with SHORT time
	 * bigger a bit than HW-coal-time-usec (1024=2^10 vs NSEC_PER_USEC)
	 */
	time = ktime_set(0, port->tx_time_coal << 10);
	port_pcpu->txq_busy_suspect_map |= txq_nonempty_map;

timer_restart:
	/* Keep guard_timer_scheduled=TRUE but set new expiration time */
	hrtimer_forward_now(timer, time);
	return HRTIMER_RESTART;
}

static void mvpp2_tx_done_guard_tasklet_cb(unsigned long data)
{
	struct mvpp2_port *port = (void *)data;
	struct mvpp2_port_pcpu *port_pcpu;
	int cpu;

	 /* stop_dev() has permanent setting for coal=0 */
	if (mvpp2_tx_stopped(port))
		return;

	cpu = get_cpu();
	port_pcpu = per_cpu_ptr(port->pcpu, cpu); /* tasklet is per-cpu */

	if (port_pcpu->tx_done_passed) {
		port_pcpu->tx_done_passed = false;
	} else { /* Force IRQ */
		mvpp2_tx_done_guard_force_irq(port, cpu,
					      port_pcpu->txq_busy_suspect_map);
		port_pcpu->tx_guard_cntr++;
	}
	port_pcpu->txq_busy_suspect_map = 0;

	/* guard_timer_scheduled is already TRUE, just start the timer */
	hrtimer_start(&port_pcpu->tx_done_timer,
		      MVPP2_GUARD_TXDONE_HRTIMER_NS,
		      HRTIMER_MODE_REL_PINNED);
	put_cpu();
}

static u32 mvpp2_tx_done_guard_get_stats(struct mvpp2_port *port, int cpu)
{
	return per_cpu_ptr(port->pcpu, cpu)->tx_guard_cntr;
}

static void mvpp2_tx_done_init_on_open(struct mvpp2_port *port, bool open)
{
	struct mvpp2_port_pcpu *port_pcpu;
	int cpu;

	if (port->flags & MVPP2_F_LOOPBACK)
		return;

	if (!open)
		goto close;

	/* Init tx-done tasklets and variables */
	for_each_present_cpu(cpu) {
		port_pcpu = per_cpu_ptr(port->pcpu, cpu);

		/* Timer works in tx-done or Guard mode. To eliminate per-packet
		 * mode checking each mode has own "_scheduled" flag.
		 * Set scheduled=FALSE for active mode and TRUE for inactive, so
		 * timer would never be started in inactive mode.
		 */
		if (port->has_tx_irqs) { /* guard-mode */
			port_pcpu->txq_coal_is_zero_map = 0;
			port_pcpu->txq_busy_suspect_map = 0;
			port_pcpu->tx_done_passed = false;

			/* "true" is never started */
			port_pcpu->tx_done_timer_scheduled = true;
			port_pcpu->guard_timer_scheduled = false;
			tasklet_init(&port_pcpu->tx_done_tasklet,
				     mvpp2_tx_done_guard_tasklet_cb,
				     (unsigned long)port);
		} else {
			port_pcpu->tx_done_timer_scheduled = false;
			/* "true" is never started */
			port_pcpu->guard_timer_scheduled = true;
			tasklet_init(&port_pcpu->tx_done_tasklet,
				     mvpp2_tx_done_proc_cb,
				     (unsigned long)port->dev);
		}
	}
	return;
close:
	/* Kill tx-done timers and tasklets */
	for_each_present_cpu(cpu) {
		port_pcpu = per_cpu_ptr(port->pcpu, cpu);
		/* Say "scheduled=true" is never started on XMIT */
		port_pcpu->tx_done_timer_scheduled = true;
		port_pcpu->guard_timer_scheduled = true;
		hrtimer_cancel(&port_pcpu->tx_done_timer);
		tasklet_kill(&port_pcpu->tx_done_tasklet);
	}
}

static void mvpp2_tx_done_init_on_probe(struct platform_device *pdev,
					struct mvpp2_port *port)
{
	struct mvpp2_port_pcpu *port_pcpu;
	int cpu;
	bool guard_mode = port->has_tx_irqs;

	if (port->flags & MVPP2_F_LOOPBACK)
		return;

	for_each_present_cpu(cpu) {
		port_pcpu = per_cpu_ptr(port->pcpu, cpu);
		port_pcpu->port = port;
		hrtimer_init(&port_pcpu->tx_done_timer, CLOCK_MONOTONIC,
			     HRTIMER_MODE_REL_PINNED);
		port_pcpu->tx_done_timer.function = (guard_mode) ?
				mvpp2_guard_timer_cb : mvpp2_tx_done_timer_cb;
	}
}

/* Main RX/TX processing routines */

/* Display more error info */
static void mvpp2_rx_error(struct mvpp2_port *port,
			   struct mvpp2_rx_desc *rx_desc)
{
	u32 status = mvpp2_rxdesc_status_get(port, rx_desc);
	size_t sz = mvpp2_rxdesc_size_get(port, rx_desc);
	char *err_str = NULL;

	switch (status & MVPP2_RXD_ERR_CODE_MASK) {
	case MVPP2_RXD_ERR_MAC:
		err_str = "MAC";
		break;
	case MVPP2_RXD_ERR_OVERRUN:
		err_str = "overrun";
		break;
	case MVPP2_RXD_ERR_RESOURCE:
		err_str = "resource";
		break;
	}
	if (err_str && net_ratelimit())
		netdev_dbg(port->dev,
			   "bad rx status %08x (%s error), size=%zu\n",
			   status, err_str, sz);
}

/* Handle RX checksum offload */
static void mvpp2_rx_csum(struct mvpp2_port *port, u32 status,
			  struct sk_buff *skb)
{
	if (((status & MVPP2_RXD_L3_IP4) &&
	     !(status & MVPP2_RXD_IP4_HEADER_ERR)) ||
	    (status & MVPP2_RXD_L3_IP6))
		if (((status & MVPP2_RXD_L4_UDP) ||
		     (status & MVPP2_RXD_L4_TCP)) &&
		     (status & MVPP2_RXD_L4_CSUM_OK)) {
			skb->csum = 0;
			skb->ip_summed = CHECKSUM_UNNECESSARY;
			return;
		}

	skb->ip_summed = CHECKSUM_NONE;
}

/* Allocate a new skb and add it to BM pool */
static inline int mvpp2_rx_refill(struct mvpp2_port *port,
				  struct mvpp2_bm_pool *bm_pool, int pool)
{
	dma_addr_t dma_addr = mvpp2_buf_alloc(port, bm_pool, GFP_ATOMIC);

	if (!dma_addr)
		return -ENOMEM;

	mvpp2_bm_pool_put(port, pool, dma_addr);

	return 0;
}

/* Handle tx checksum */
static u32 mvpp2_skb_tx_csum(struct mvpp2_port *port, struct sk_buff *skb)
{
	if (skb->ip_summed == CHECKSUM_PARTIAL) {
		int ip_hdr_len = 0;
		u8 l4_proto;
		__be16 l3_proto = vlan_get_protocol(skb);

		if (l3_proto == htons(ETH_P_IP)) {
			struct iphdr *ip4h = ip_hdr(skb);

			/* Calculate IPv4 checksum and L4 checksum */
			ip_hdr_len = ip4h->ihl;
			l4_proto = ip4h->protocol;
		} else if (l3_proto == htons(ETH_P_IPV6)) {
			struct ipv6hdr *ip6h = ipv6_hdr(skb);

			/* Read l4_protocol from one of IPv6 extra headers */
			if (skb_network_header_len(skb) > 0)
				ip_hdr_len = (skb_network_header_len(skb) >> 2);
			l4_proto = ip6h->nexthdr;
		} else {
			return MVPP2_TXD_L4_CSUM_NOT;
		}

		return mvpp2_txq_desc_csum(skb_network_offset(skb),
					   l3_proto, ip_hdr_len, l4_proto);
	}

	return MVPP2_TXD_L4_CSUM_NOT | MVPP2_TXD_IP_CSUM_DISABLE;
}

void mvpp2_recycle_stats(void)
{
	int cpu;
	int pl_id;
	struct mvpp2_recycle_pcpu *pcpu;

	pr_info("Recycle-stats: %d open ports (on all CP110s)\n",
		mvpp2_share.num_open_ports);
	if (!mvpp2_share.recycle_base)
		return;
	pcpu = mvpp2_share.recycle;
	for_each_online_cpu(cpu) {
		for (pl_id = 0; pl_id < MVPP2_BM_POOLS_NUM; pl_id++) {
			pr_info("| cpu[%d].pool_%d: idx=%d\n",
				cpu, pl_id, pcpu->idx[pl_id]);
		}
		pr_info("| ___[%d].skb_____idx=%d__\n",
			cpu, pcpu->idx[MVPP2_BM_POOLS_NUM]);
		pcpu++;
	}
}

static int mvpp2_recycle_open(void)
{
	int cpu, pl_id, size;
	struct mvpp2_recycle_pcpu *pcpu;
	phys_addr_t addr;

	mvpp2_share.num_open_ports++;
	wmb(); /* for num_open_ports */

	if (mvpp2_share.recycle_base)
		return 0;

	/* Allocate pool-tree */
	size = sizeof(*pcpu) * num_online_cpus() + L1_CACHE_BYTES;
	mvpp2_share.recycle_base = kzalloc(size, GFP_KERNEL);
	if (!mvpp2_share.recycle_base)
		goto err;
	/* Use Address aligned to L1_CACHE_BYTES */
	addr = (phys_addr_t)mvpp2_share.recycle_base + (L1_CACHE_BYTES - 1);
	addr &= ~(L1_CACHE_BYTES - 1);
	mvpp2_share.recycle = (void *)addr;

	pcpu = mvpp2_share.recycle;
	for_each_online_cpu(cpu) {
		for (pl_id = 0; pl_id <= MVPP2_BM_POOLS_NUM; pl_id++)
			pcpu->idx[pl_id] = -1;
		pcpu++;
	}
	return 0;
err:
	pr_err("mvpp2 error: cannot allocate recycle pool\n");
	return -ENOMEM;
}

static void mvpp2_recycle_close(void)
{
	int cpu, pl_id, i;
	struct mvpp2_recycle_pcpu *pcpu;
	struct mvpp2_recycle_pool *pool;

	mvpp2_share.num_open_ports--;
	wmb(); /* for num_open_ports */

	/* Do nothing if recycle is not used at all or in use by port/ports */
	if (mvpp2_share.num_open_ports || !mvpp2_share.recycle_base)
		return;

	/* Usable (recycle_base!=NULL), but last port gone down
	 * Let's free all accumulated buffers.
	 */
	pcpu = mvpp2_share.recycle;
	for_each_online_cpu(cpu) {
		for (pl_id = 0; pl_id <= MVPP2_BM_POOLS_NUM; pl_id++) {
			pool = &pcpu->pool[pl_id];
			for (i = 0; i <= pcpu->idx[pl_id]; i++) {
				if (!pool->pbuf[i])
					continue;
				if (pl_id < MVPP2_BM_POOLS_NUM)
					kfree(pool->pbuf[i]);
				else
					kmem_cache_free(skbuff_head_cache,
							pool->pbuf[i]);
			}
		}
		pcpu++;
	}
	kfree(mvpp2_share.recycle_base);
	mvpp2_share.recycle_base = NULL;
}

static int mvpp2_recycle_get_bm_id(struct sk_buff *skb)
{
	u32 hash;

	/* Keep checking ordering for performance */
	hash = skb_get_hash_raw(skb);
	/* Check hash */
	if (!MVPP2_RXTX_HASH_IS_OK(skb, hash))
		return -1;
	/* Check if skb could be free */
	/* Use skb->cloned but not skb_cloned(), skb_header_cloned() */
	if (skb_shared(skb) || skb->cloned)
		return -1;
	/* ipsec: sp/secpath, _skb_refdst ... */
	if (!skb_irq_freeable(skb))
		return -1;
	if (skb_shinfo(skb)->tx_flags & SKBTX_ZEROCOPY_FRAG)
		return -1;

	/* Get bm-pool-id */
	hash &= MVPP2_RXTX_HASH_BMID_MASK;
	if (hash >= MVPP2_BM_POOLS_NUM)
		return -1;

	return (int)hash;
}

static inline void mvpp2_recycle_put(struct mvpp2_port *port,
				     struct mvpp2_txq_pcpu *txq_pcpu,
				     struct mvpp2_txq_pcpu_buf *tx_buf)
{
	struct mvpp2_recycle_pcpu *pcpu;
	struct mvpp2_recycle_pool *pool;
	short int idx, pool_id;
	struct sk_buff *skb = tx_buf->skb;
	struct mvpp2_bm_pool *bm_pool;

	/* tx_buf->skb is not NULL */
	pool_id = mvpp2_recycle_get_bm_id(skb);
	if (pool_id < 0)
		return; /* non-recyclable */

	bm_pool = &port->priv->bm_pools[pool_id];
	if (skb_end_offset(skb) < (bm_pool->frag_size - MVPP2_SKB_SHINFO_SIZE))
		return; /* shrank -> non-recyclable */

	/* This skb could be destroyed. Put into recycle */
	pcpu = mvpp2_share.recycle + txq_pcpu->thread;
	idx = pcpu->idx[pool_id];
	if (idx < (MVPP2_RECYCLE_FULL - 1)) {
		pool = &pcpu->pool[pool_id];
		pool->pbuf[++idx] = skb->head; /* pre-increment */
		pcpu->idx[pool_id] = idx;
		skb->head = NULL;
	}
	idx = pcpu->idx[MVPP2_BM_POOLS_NUM];
	if (idx < (MVPP2_RECYCLE_FULL_SKB - 1)) {
		pool = &pcpu->pool[MVPP2_BM_POOLS_NUM];
		pool->pbuf[++idx] = skb;
		pcpu->idx[MVPP2_BM_POOLS_NUM] = idx;
		if (skb->head) {
			if (bm_pool->frag_size <= PAGE_SIZE)
				skb_free_frag(skb->head);
			else
				kfree(skb->head);
		}
		tx_buf->skb = NULL;
	}
}

static struct sk_buff *mvpp2_recycle_get(struct mvpp2_port *port,
					 struct mvpp2_bm_pool *bm_pool)
{
	int cpu;
	struct mvpp2_recycle_pcpu *pcpu;
	struct mvpp2_recycle_pool *pool;
	short int idx;
	void *frag;
	struct sk_buff *skb;
	dma_addr_t dma_addr;

	cpu = smp_processor_id();
	pcpu = mvpp2_share.recycle + cpu;

	/* GET bm buffer */
	idx = pcpu->idx[bm_pool->id];
	pool = &pcpu->pool[bm_pool->id];

	if (idx >= 0) {
		frag = pool->pbuf[idx];
		pcpu->idx[bm_pool->id]--; /* post-decrement */
	} else {
		/* Allocate 2 buffers, put 1, use another now */
		pcpu->idx[bm_pool->id] = 0;
		pool->pbuf[0] = mvpp2_frag_alloc(bm_pool);
		frag = NULL;
	}
	if (!frag)
		frag = mvpp2_frag_alloc(bm_pool);

	/* refill the buffer into BM */
	dma_addr = dma_map_single(port->dev->dev.parent, frag,
				  bm_pool->buf_size, DMA_FROM_DEVICE);
	if (unlikely(dma_mapping_error(port->dev->dev.parent, dma_addr))) {
		pcpu->idx[bm_pool->id]++; /* Return back to recycle */
		netdev_err(port->dev, "failed to refill BM pool-%d (%d:%p)\n",
			   bm_pool->id, pcpu->idx[bm_pool->id], frag);
		return NULL;
	}

	/* GET skb buffer */
	idx = pcpu->idx[MVPP2_BM_POOLS_NUM];
	if (idx >= 0) {
		pool = &pcpu->pool[MVPP2_BM_POOLS_NUM];
		skb = pool->pbuf[idx];
		pcpu->idx[MVPP2_BM_POOLS_NUM]--;
	} else {
		skb = kmem_cache_alloc(skbuff_head_cache, GFP_ATOMIC);
	}

	if (unlikely(!skb)) {
		dma_unmap_single(port->dev->dev.parent, dma_addr,
				 bm_pool->buf_size, DMA_FROM_DEVICE);
		mvpp2_frag_free(bm_pool, frag);
		return NULL;
	}
	mvpp2_bm_pool_put(port, bm_pool->id, dma_addr);
	return skb;
}

/* SKB and BM-buff alloc/refill like mvpp2_recycle_get but without recycle */
static inline
struct sk_buff *mvpp2_bm_refill_skb_get(struct mvpp2_port *port,
					struct mvpp2_bm_pool *bm_pool)
{
	void *frag;
	struct sk_buff *skb;
	dma_addr_t dma_addr;

	/* GET bm buffer, refill into BM */
	frag = mvpp2_frag_alloc(bm_pool);
	dma_addr = dma_map_single(port->dev->dev.parent, frag,
				  bm_pool->buf_size, DMA_FROM_DEVICE);
	if (unlikely(dma_mapping_error(port->dev->dev.parent, dma_addr))) {
		netdev_err(port->dev, "failed to refill BM pool-%d\n",
			   bm_pool->id);
		return NULL;
	}

	/* GET skb buffer */
	skb = kmem_cache_alloc(skbuff_head_cache, GFP_ATOMIC);
	if (unlikely(!skb)) {
		dma_unmap_single(port->dev->dev.parent, dma_addr,
				 bm_pool->buf_size, DMA_FROM_DEVICE);
		mvpp2_frag_free(bm_pool, frag);
		return NULL;
	}
	mvpp2_bm_pool_put(port, bm_pool->id, dma_addr);
	return skb;
}

static inline void mvpp2_skb_set_extra(struct sk_buff *skb,
				       struct napi_struct *napi,
				       u32 status,
				       u8 rxq_id,
				       struct mvpp2_bm_pool *bm_pool)
{
	u32 hash;
	enum pkt_hash_types hash_type;

	/* Improve performance and set identification for RX-TX fast-forward */
	hash = MVPP2_RXTX_HASH_GENER(skb, bm_pool->id);
	hash_type = (status & (MVPP2_RXD_L4_UDP | MVPP2_RXD_L4_TCP)) ?
		PKT_HASH_TYPE_L4 : PKT_HASH_TYPE_L3;
	skb_set_hash(skb, hash, hash_type);
	skb_mark_napi_id(skb, napi);
	skb_record_rx_queue(skb, (u16)rxq_id);
}

/* This is "fast inline" clone of __build_skb+build_skb,
 * and also with setting mv-extra information
 */
static inline
struct sk_buff *mvpp2_build_skb(void *data, unsigned int frag_size,
				struct napi_struct *napi,
				struct mvpp2_port *port,
				u32 rx_status,
				u8 rxq_id,
				struct mvpp2_bm_pool *bm_pool)
{
	struct skb_shared_info *shinfo;
	struct sk_buff *skb;
	unsigned int size = frag_size ? : ksize(data);

	if (static_branch_unlikely(&mvpp2_recycle_ena))
		skb = mvpp2_recycle_get(port, bm_pool);
	else
		skb = mvpp2_bm_refill_skb_get(port, bm_pool);
	if (unlikely(!skb))
		return NULL;

	size -= SKB_DATA_ALIGN(sizeof(struct skb_shared_info));

	memset(skb, 0, offsetof(struct sk_buff, tail));
	skb->truesize = SKB_TRUESIZE(size);
	refcount_set(&skb->users, 1);
	skb->head = data;
	skb->data = data;
	skb_reset_tail_pointer(skb);
	skb->end = skb->tail + size;
	skb->mac_header = (typeof(skb->mac_header))~0U;
	skb->transport_header = (typeof(skb->transport_header))~0U;

	/* make sure we initialize shinfo sequentially */
	shinfo = skb_shinfo(skb);
	memset(shinfo, 0, offsetof(struct skb_shared_info, dataref));
	atomic_set(&shinfo->dataref, 1);

	/* From build_skb wrapper */
	if (frag_size) {
		skb->head_frag = 1;
		if (page_is_pfmemalloc(virt_to_head_page(data)))
			skb->pfmemalloc = 1;
	}

	mvpp2_skb_set_extra(skb, napi, rx_status, rxq_id, bm_pool);

	return skb;
}

/* Main rx processing */
static int mvpp2_rx(struct mvpp2_port *port, struct napi_struct *napi,
		    int rx_todo, struct mvpp2_rx_queue *rxq)
{
	struct net_device *dev = port->dev;
	int rx_received;
	int rx_done = 0;
	u32 rcvd_pkts = 0, i = 0;
	u32 rcvd_bytes = 0;
	struct sk_buff *skb_all[64];

	if (rxq->rx_pending >= rx_todo) {
		rx_received = rx_todo;
		rxq->rx_pending -= rx_todo;
	} else {
		/* Get number of received packets and clamp the to-do */
		rx_received = mvpp2_rxq_received(port, rxq->id);
		if (rx_received < rx_todo) {
			rx_todo = rx_received;
			rxq->rx_pending = 0;
		} else {
			rxq->rx_pending = rx_received - rx_todo;
		}
	}

	while (rx_done < rx_todo) {
		struct mvpp2_rx_desc *rx_desc = mvpp2_rxq_next_desc_get(rxq);
		struct mvpp2_bm_pool *bm_pool;
		struct sk_buff *skb;
		unsigned int frag_size;
		dma_addr_t dma_addr;
		phys_addr_t phys_addr;
		u32 rx_status;
		int pool, rx_bytes;
		void *data;

		rx_done++;
		rx_status = mvpp2_rxdesc_status_get(port, rx_desc);
		rx_bytes = mvpp2_rxdesc_size_get(port, rx_desc);
		rx_bytes -= MVPP2_MH_SIZE;
		dma_addr = mvpp2_rxdesc_dma_addr_get(port, rx_desc);
		phys_addr = dma_to_phys(port->dev->dev.parent, dma_addr);
		data = (void *)phys_to_virt(phys_addr);

		pool = (rx_status & MVPP2_RXD_BM_POOL_ID_MASK) >>
			MVPP2_RXD_BM_POOL_ID_OFFS;
		bm_pool = &port->priv->bm_pools[pool];

		/* In case of an error, release the requested buffer pointer
		 * to the Buffer Manager. This request process is controlled
		 * by the hardware, and the information about the buffer is
		 * comprised by the RX descriptor.
		 */
		if (rx_status & MVPP2_RXD_ERR_SUMMARY) {
err_drop_frame:
			dev->stats.rx_errors++;
			mvpp2_rx_error(port, rx_desc);
			/* Return the buffer to the pool */
			mvpp2_bm_pool_put(port, pool, dma_addr);
			continue;
		}

		if (bm_pool->frag_size > PAGE_SIZE)
			frag_size = 0;
		else
			frag_size = bm_pool->frag_size;

		/* _sync_ for coherency (_unmap_ is asynchroneous).
		 * _sync_ should be done for the SAME size as in map/unmap.
		 * The prefetch is for CPU and should be after unmap ~ mapToCPU
		 */
		if (rx_todo == 1)
			dma_sync_single_for_cpu(dev->dev.parent, dma_addr,
						bm_pool->buf_size,
						DMA_FROM_DEVICE);
		dma_unmap_single(dev->dev.parent, dma_addr,
				 bm_pool->buf_size, DMA_FROM_DEVICE);

		prefetch(data + NET_SKB_PAD); /* packet header */

		skb = mvpp2_build_skb(data, frag_size,
				      napi, port, rx_status, rxq->id, bm_pool);
		if (!skb) {
			netdev_warn(port->dev, "skb build failed\n");
			goto err_drop_frame;
		}

		skb_reserve(skb, MVPP2_MH_SIZE + NET_SKB_PAD);
		skb_put(skb, rx_bytes);
		skb->protocol = eth_type_trans(skb, dev);
		mvpp2_rx_csum(port, rx_status, skb);

		skb_all[rcvd_pkts++] = skb;
		rcvd_bytes += rx_bytes;
	}

	while (i < rcvd_pkts)
		napi_gro_receive(napi, skb_all[i++]);

	if (rcvd_pkts) {
		struct mvpp2_pcpu_stats *stats = this_cpu_ptr(port->stats);

		u64_stats_update_begin(&stats->syncp);
		stats->rx_packets += rcvd_pkts;
		stats->rx_bytes   += rcvd_bytes;
		u64_stats_update_end(&stats->syncp);
	}

	/* Update HW Rx queue management counters with RX-done */
	mvpp2_rxq_status_update(port, rxq->id, rx_done, rx_done);

	return rx_todo;
}

static inline void
tx_desc_unmap_put(struct mvpp2_port *port, struct mvpp2_tx_queue *txq,
		  struct mvpp2_tx_desc *desc)
{
	unsigned int thread = mvpp2_cpu_to_thread(port->priv, smp_processor_id());
	struct mvpp2_txq_pcpu *txq_pcpu = per_cpu_ptr(txq->pcpu, thread);

	dma_addr_t buf_dma_addr =
		mvpp2_txdesc_dma_addr_get(port, desc);
	size_t buf_sz =
		mvpp2_txdesc_size_get(port, desc);
	if (!IS_TSO_HEADER(txq_pcpu, buf_dma_addr))
		dma_unmap_single(port->dev->dev.parent, buf_dma_addr,
				 buf_sz, DMA_TO_DEVICE);
	mvpp2_txq_desc_put(txq);
}

/* Handle tx fragmentation processing */
static int mvpp2_tx_frag_process(struct mvpp2_port *port, struct sk_buff *skb,
				 struct mvpp2_tx_queue *aggr_txq,
				 struct mvpp2_tx_queue *txq)
{
	unsigned int thread = mvpp2_cpu_to_thread(port->priv, smp_processor_id());
	struct mvpp2_txq_pcpu *txq_pcpu = per_cpu_ptr(txq->pcpu, thread);
	struct mvpp2_tx_desc *tx_desc;
	int i;
	dma_addr_t buf_dma_addr;

	for (i = 0; i < skb_shinfo(skb)->nr_frags; i++) {
		skb_frag_t *frag = &skb_shinfo(skb)->frags[i];
		void *addr = page_address(frag->page.p) + frag->page_offset;

		tx_desc = mvpp2_txq_next_desc_get(aggr_txq);
		mvpp2_txdesc_txq_set(port, tx_desc, txq->id);
		mvpp2_txdesc_size_set(port, tx_desc, frag->size);

		buf_dma_addr = dma_map_single(port->dev->dev.parent, addr,
					      skb_frag_size(frag), DMA_TO_DEVICE);
		if (dma_mapping_error(port->dev->dev.parent, buf_dma_addr)) {
			mvpp2_txq_desc_put(txq);
			goto cleanup;
		}

		mvpp2_txdesc_dma_addr_set(port, tx_desc, buf_dma_addr);

		if (i == (skb_shinfo(skb)->nr_frags - 1)) {
			/* Last descriptor */
			mvpp2_txdesc_cmd_set(port, tx_desc,
					     MVPP2_TXD_L_DESC);
			mvpp2_txq_inc_put(port, txq_pcpu, skb, tx_desc);
		} else {
			/* Descriptor in the middle: Not First, Not Last */
			mvpp2_txdesc_cmd_set(port, tx_desc, 0);
			mvpp2_txq_inc_put(port, txq_pcpu, NULL, tx_desc);
		}
	}

	return 0;
cleanup:
	/* Release all descriptors that were used to map fragments of
	 * this packet, as well as the corresponding DMA mappings
	 */
	for (i = i - 1; i >= 0; i--) {
		tx_desc = txq->descs + i;
		tx_desc_unmap_put(port, txq, tx_desc);
	}

	return -ENOMEM;
}

static inline void mvpp2_tso_put_hdr(struct sk_buff *skb,
				     struct net_device *dev,
				     struct mvpp2_tx_queue *txq,
				     struct mvpp2_tx_queue *aggr_txq,
				     struct mvpp2_txq_pcpu *txq_pcpu,
				     int hdr_sz)
{
	struct mvpp2_port *port = netdev_priv(dev);
	struct mvpp2_tx_desc *tx_desc = mvpp2_txq_next_desc_get(aggr_txq);
	dma_addr_t addr;

	mvpp2_txdesc_txq_set(port, tx_desc, txq->id);
	mvpp2_txdesc_size_set(port, tx_desc, hdr_sz);

	addr = txq_pcpu->tso_headers_dma +
	       txq_pcpu->txq_put_index * TSO_HEADER_SIZE;
	mvpp2_txdesc_dma_addr_set(port, tx_desc, addr);

	mvpp2_txdesc_cmd_set(port, tx_desc, mvpp2_skb_tx_csum(port, skb) |
					    MVPP2_TXD_F_DESC |
					    MVPP2_TXD_PADDING_DISABLE);
	mvpp2_txq_inc_put(port, txq_pcpu, TSO_HEADER_MARK, tx_desc);
}

static inline int mvpp2_tso_put_data(struct sk_buff *skb,
				     struct net_device *dev, struct tso_t *tso,
				     struct mvpp2_tx_queue *txq,
				     struct mvpp2_tx_queue *aggr_txq,
				     struct mvpp2_txq_pcpu *txq_pcpu,
				     int sz, bool left, bool last)
{
	struct mvpp2_port *port = netdev_priv(dev);
	struct mvpp2_tx_desc *tx_desc = mvpp2_txq_next_desc_get(aggr_txq);
	dma_addr_t buf_dma_addr;

	mvpp2_txdesc_txq_set(port, tx_desc, txq->id);
	mvpp2_txdesc_size_set(port, tx_desc, sz);

	buf_dma_addr = dma_map_single(dev->dev.parent, tso->data, sz,
				      DMA_TO_DEVICE);
	if (unlikely(dma_mapping_error(dev->dev.parent, buf_dma_addr))) {
		mvpp2_txq_desc_put(txq);
		return -ENOMEM;
	}

	mvpp2_txdesc_dma_addr_set(port, tx_desc, buf_dma_addr);

	if (!left) {
		mvpp2_txdesc_cmd_set(port, tx_desc, MVPP2_TXD_L_DESC);
		if (last) {
			mvpp2_txq_inc_put(port, txq_pcpu, skb, tx_desc);
			return 0;
		}
	} else {
		mvpp2_txdesc_cmd_set(port, tx_desc, 0);
	}

	mvpp2_txq_inc_put(port, txq_pcpu, NULL, tx_desc);
	return 0;
}

static int mvpp2_tx_tso(struct sk_buff *skb, struct net_device *dev,
			struct mvpp2_tx_queue *txq,
			struct mvpp2_tx_queue *aggr_txq,
			struct mvpp2_txq_pcpu *txq_pcpu)
{
	struct mvpp2_port *port = netdev_priv(dev);
	struct tso_t tso;
	int hdr_sz = skb_transport_offset(skb) + tcp_hdrlen(skb);
	int i, len, descs = tso_count_descs(skb);

	/* Check enough free-space in txq and
	 * number of available aggr/reserved descriptors
	 */
	if (((txq_pcpu->size - txq_pcpu->count) < descs) ||
	    mvpp2_aggr_desc_num_check(port, aggr_txq, descs) ||
	    mvpp2_txq_reserved_desc_num_proc(port, txq, txq_pcpu, descs))
		return 0;

	descs = 0; /* real descs <= tso_count_descs() */
	tso_start(skb, &tso);
	len = skb->len - hdr_sz;
	while (len > 0) {
		int left = min_t(int, skb_shinfo(skb)->gso_size, len);
		char *hdr = txq_pcpu->tso_headers +
			    txq_pcpu->txq_put_index * TSO_HEADER_SIZE;

		len -= left;
		descs++;

		tso_build_hdr(skb, hdr, &tso, left, len == 0);
		mvpp2_tso_put_hdr(skb, dev, txq, aggr_txq, txq_pcpu, hdr_sz);

		while (left > 0) {
			int sz = min_t(int, tso.size, left);
			left -= sz;
			descs++;

			if (mvpp2_tso_put_data(skb, dev, &tso, txq, aggr_txq,
					       txq_pcpu, sz, left, len == 0))
				goto release;
			tso_build_data(skb, &tso, sz);
		}
	}

	return descs;

release:
	for (i = descs - 1; i >= 0; i--) {
		struct mvpp2_tx_desc *tx_desc = txq->descs + i;
		tx_desc_unmap_put(port, txq, tx_desc);
	}
	return 0;
}

/* Main tx processing */
static netdev_tx_t mvpp2_tx(struct sk_buff *skb, struct net_device *dev)
{
	struct mvpp2_port *port = netdev_priv(dev);
	struct mvpp2_tx_queue *txq, *aggr_txq;
	struct mvpp2_txq_pcpu *txq_pcpu;
	struct mvpp2_tx_desc *tx_desc;
	dma_addr_t buf_dma_addr;
	unsigned long flags = 0;
	unsigned int thread;
	int frags = 0;
	u16 txq_id;
	u32 tx_cmd;

	thread = mvpp2_cpu_to_thread(port->priv, smp_processor_id());

	txq_id = skb_get_queue_mapping(skb);
	txq = port->txqs[txq_id];
	txq_pcpu = per_cpu_ptr(txq->pcpu, thread);
	aggr_txq = &port->priv->aggr_txqs[thread];

	if (test_bit(thread, &port->priv->lock_map))
		spin_lock_irqsave(&port->tx_lock[thread], flags);

	if (skb_is_gso(skb)) {
		frags = mvpp2_tx_tso(skb, dev, txq, aggr_txq, txq_pcpu);
		goto out;
	}
	frags = skb_shinfo(skb)->nr_frags + 1;

	/* Check enough free-space in txq and
	 * number of available aggr/reserved descriptors
	 */
	if (((txq_pcpu->size - txq_pcpu->count) < frags) ||
	    mvpp2_aggr_desc_num_check(port, aggr_txq, frags) ||
	    mvpp2_txq_reserved_desc_num_proc(port, txq, txq_pcpu, frags)) {
		frags = 0;
		goto out;
	}

	/* Get a descriptor for the first part of the packet */
	tx_desc = mvpp2_txq_next_desc_get(aggr_txq);
	mvpp2_txdesc_txq_set(port, tx_desc, txq->id);
	mvpp2_txdesc_size_set(port, tx_desc, skb_headlen(skb));

	buf_dma_addr = dma_map_single(dev->dev.parent, skb->data,
				      skb_headlen(skb), DMA_TO_DEVICE);
	if (unlikely(dma_mapping_error(dev->dev.parent, buf_dma_addr))) {
		mvpp2_txq_desc_put(txq);
		frags = 0;
		goto out;
	}

	mvpp2_txdesc_dma_addr_set(port, tx_desc, buf_dma_addr);

	tx_cmd = mvpp2_skb_tx_csum(port, skb);

	if (frags == 1) {
		/* First and Last descriptor */
		tx_cmd |= MVPP2_TXD_F_DESC | MVPP2_TXD_L_DESC;
		mvpp2_txdesc_cmd_set(port, tx_desc, tx_cmd);
		mvpp2_txq_inc_put(port, txq_pcpu, skb, tx_desc);
	} else {
		/* First but not Last */
		tx_cmd |= MVPP2_TXD_F_DESC | MVPP2_TXD_PADDING_DISABLE;
		mvpp2_txdesc_cmd_set(port, tx_desc, tx_cmd);
		mvpp2_txq_inc_put(port, txq_pcpu, NULL, tx_desc);

		/* Continue with other skb fragments */
		if (mvpp2_tx_frag_process(port, skb, aggr_txq, txq)) {
			tx_desc_unmap_put(port, txq, tx_desc);
			frags = 0;
		}
	}

out:
	if (frags > 0) {
		struct mvpp2_pcpu_stats *stats = per_cpu_ptr(port->stats, thread);
		struct mvpp2_port_pcpu *port_pcpu = this_cpu_ptr(port->pcpu);
		struct netdev_queue *nq;
		bool deferred_tx;

		txq_pcpu->reserved_num -= frags;
		txq_pcpu->count += frags;
		aggr_txq->count += frags;

		/* Enable transmit; RX-to-TX may be deferred with Bulk-timer */
		deferred_tx = (frags == 1) &&
			MVPP2_RXTX_HASH_IS_OK_TX(skb, skb_get_hash_raw(skb)) &&
			(aggr_txq->pending < min(MVPP2_TX_BULK_MAX_PACKETS,
					       (int)(txq->done_pkts_coal / 2)));

		if (deferred_tx) {
			aggr_txq->pending += frags;
			mvpp2_bulk_timer_restart(port_pcpu);
		} else {
			port_pcpu->bulk_timer_scheduled = false;
			port_pcpu->bulk_timer_restart_req = false;
			frags += aggr_txq->pending;
			aggr_txq->pending = 0;
			mvpp2_aggr_txq_pend_desc_add(port, frags);
		}

		if (unlikely(txq_pcpu->count >= txq_pcpu->stop_threshold)) {
			nq = netdev_get_tx_queue(dev, txq_id);
			/* txq_id may differ from thread/cpu and come from more
			 * than one txq_pcpu. Save only the first for wakeup.
			 */
			if (unlikely(!netif_tx_queue_stopped(nq))) {
				txq_pcpu->stopped_on_txq_id = txq_id;
				netif_tx_stop_queue(nq);
			}
		}
		u64_stats_update_begin(&stats->syncp);
		stats->tx_packets++;
		stats->tx_bytes += skb->len;
		u64_stats_update_end(&stats->syncp);
	} else {
		dev->stats.tx_dropped++;
		dev_kfree_skb_any(skb);
	}

	/* Finalize TX processing */
	if (!port->has_tx_irqs && txq_pcpu->count >= txq->done_pkts_coal)
		mvpp2_txq_done(port, txq, txq_pcpu);

	/* Set the timer in case not all frags were processed */
	if (!port->has_tx_irqs && txq_pcpu->count <= frags &&
	    txq_pcpu->count > 0) {
		struct mvpp2_port_pcpu *port_pcpu = per_cpu_ptr(port->pcpu, thread);

		mvpp2_tx_done_timer_set(port_pcpu);
	}

	if (test_bit(thread, &port->priv->lock_map))
		spin_unlock_irqrestore(&port->tx_lock[thread], flags);

	return NETDEV_TX_OK;
}

static int mvpp2_poll(struct napi_struct *napi, int budget)
{
	u32 cause_rx_tx, cause_rx, cause_tx;
	int rx_done = 0;
	struct mvpp2_port *port = netdev_priv(napi->dev);
	struct mvpp2_queue_vector *qv;

	qv = container_of(napi, struct mvpp2_queue_vector, napi);

	/* Rx/Tx cause register
	 *
	 * Bits 0-15: each bit indicates received packets on the Rx queue
	 * (bit 0 is for Rx queue 0).
	 *
	 * Bits 16-23: each bit indicates transmitted packets on the Tx queue
	 * (bit 16 is for Tx queue 0).
	 *
	 * Each CPU has its own Rx/Tx cause register
	 */
	cause_rx_tx = mvpp2_thread_read_relaxed(port->priv, qv->sw_thread_id,
						MVPP2_ISR_RX_TX_CAUSE_REG(port->id));

	if (port->has_tx_irqs) {
		cause_tx = cause_rx_tx & MVPP2_CAUSE_TXQ_OCCUP_DESC_ALL_MASK;
		if (cause_tx) {
			per_cpu_ptr(port->pcpu,
				    qv->sw_thread_id)->tx_done_passed =	true;
			cause_tx >>= MVPP2_CAUSE_TXQ_OCCUP_DESC_ALL_OFFSET;
			mvpp2_tx_done(port, cause_tx, qv->sw_thread_id);
		}
	}

	/* Process RX packets */
	cause_rx = cause_rx_tx &
		   MVPP2_CAUSE_RXQ_OCCUP_DESC_ALL_MASK(mvpp21_variant);
	cause_rx <<= qv->first_rxq;
	cause_rx |= qv->pending_cause_rx;
	while (cause_rx && budget > 0) {
		int count;
		struct mvpp2_rx_queue *rxq;

		rxq = mvpp2_get_rx_queue(port, cause_rx);
		if (!rxq)
			break;

		count = mvpp2_rx(port, napi, budget, rxq);
		rx_done += count;
		budget -= count;
		if (budget > 0) {
			/* Clear the bit associated to this Rx queue
			 * so that next iteration will continue from
			 * the next Rx queue.
			 */
			cause_rx &= ~(1 << rxq->logic_rxq);
		}
	}

	if (budget > 0) {
		cause_rx = 0;
		napi_complete_done(napi, rx_done);

		mvpp2_qvec_interrupt_enable(qv);
	}
	qv->pending_cause_rx = cause_rx;
	return rx_done;
}

static void mvpp22_mode_reconfigure(struct mvpp2_port *port)
{
	u32 ctrl3;

	/* comphy reconfiguration */
	mvpp22_comphy_init(port);

	/* gop reconfiguration */
	mvpp22_gop_init(port);

	if  (port->phy_interface == PHY_INTERFACE_MODE_INTERNAL)
		return;

	if (port->has_xlg_mac) {
		ctrl3 = readl(port->base + MVPP22_XLG_CTRL3_REG);
		ctrl3 &= ~MVPP22_XLG_CTRL3_MACMODESELECT_MASK;

		if (port->phy_interface == PHY_INTERFACE_MODE_RXAUI ||
		    port->phy_interface == PHY_INTERFACE_MODE_10GKR ||
		    port->phy_interface == PHY_INTERFACE_MODE_5GKR)
			ctrl3 |= MVPP22_XLG_CTRL3_MACMODESELECT_10G;
		else
			ctrl3 |= MVPP22_XLG_CTRL3_MACMODESELECT_GMAC;

		writel(ctrl3, port->base + MVPP22_XLG_CTRL3_REG);
	}

	if (port->has_xlg_mac &&
	    (port->phy_interface == PHY_INTERFACE_MODE_RXAUI ||
	     port->phy_interface == PHY_INTERFACE_MODE_10GKR ||
	     port->phy_interface == PHY_INTERFACE_MODE_5GKR))
		mvpp2_xlg_max_rx_size_set(port);
	else
		mvpp2_gmac_max_rx_size_set(port);
}

/* Set hw internals when starting port */
static void mvpp2_start_dev(struct mvpp2_port *port)
{
	int i;

	mvpp2_txp_max_tx_size_set(port);

	/* stop_dev() sets Coal to ZERO. Care to restore it now */
	if (port->has_tx_irqs)
		mvpp2_tx_pkts_coal_set(port);

	for (i = 0; i < port->nqvecs; i++)
		napi_enable(&port->qvecs[i].napi);

	/* Enable interrupts on all threads */
	mvpp2_interrupts_enable(port);

	if (port->priv->hw_version != MVPP21)
		mvpp22_mode_reconfigure(port);

	if (port->phylink) {
		phylink_start(port->phylink);
	} else {
		/* Phylink isn't used as of now for ACPI, so the MAC has to be
		 * configured manually when the interface is started. This will
		 * be removed as soon as the phylink ACPI support lands in.
		 */
		struct phylink_link_state state = {
			.interface = port->phy_interface,
		};
		mvpp2_mac_config(&port->phylink_config, MLO_AN_INBAND, &state);
		mvpp2_mac_link_up(&port->phylink_config, MLO_AN_INBAND,
				  port->phy_interface, NULL);
	}

	mvpp2_tx_start_all_queues(port->dev);
}

/* Set hw internals when stopping port */
static void mvpp2_stop_dev(struct mvpp2_port *port)
{
	int i;

	/* Stop-dev called by ifconfig but also by ethtool-features.
	 * Under active traffic the BM/RX and TX PP2-HW could be non-empty.
	 * Stop asap new packets ariving from both RX and TX directions,
	 * but do NOT disable egress free/send-out and interrupts tx-done,
	 * yeild and msleep this context for gracefull finishing.
	 * Flush all tx-done by forcing pkts-coal to ZERO
	 */
	mvpp2_tx_stop_all_queues(port->dev);
	mvpp2_ingress_disable(port);
	if (port->has_tx_irqs)
		on_each_cpu(mvpp2_tx_pkts_coal_set_zero_pcpu, port, 1);

	msleep(40);
	mvpp2_egress_disable(port);

	/* Disable interrupts on all threads */
	mvpp2_interrupts_disable(port);

	for (i = 0; i < port->nqvecs; i++)
		napi_disable(&port->qvecs[i].napi);

	if (port->phylink)
		phylink_stop(port->phylink);
	phy_power_off(port->comphy);
}

static int mvpp2_check_ringparam_valid(struct net_device *dev,
				       struct ethtool_ringparam *ring)
{
	u16 new_rx_pending = ring->rx_pending;
	u16 new_tx_pending = ring->tx_pending;

	if (ring->rx_pending == 0 || ring->tx_pending == 0)
		return -EINVAL;

	if (ring->rx_pending > MVPP2_MAX_RXD_MAX)
		new_rx_pending = MVPP2_MAX_RXD_MAX;
	else if (!IS_ALIGNED(ring->rx_pending, 16))
		new_rx_pending = ALIGN(ring->rx_pending, 16);

	if (ring->tx_pending > MVPP2_MAX_TXD_MAX)
		new_tx_pending = MVPP2_MAX_TXD_MAX;
	else if (!IS_ALIGNED(ring->tx_pending, 32))
		new_tx_pending = ALIGN(ring->tx_pending, 32);

	if (new_tx_pending < MVPP2_MIN_TXD(num_present_cpus()))
		new_tx_pending = MVPP2_MIN_TXD(num_present_cpus());

	if (ring->rx_pending != new_rx_pending) {
		netdev_info(dev, "illegal Rx ring size value %d, round to %d\n",
			    ring->rx_pending, new_rx_pending);
		ring->rx_pending = new_rx_pending;
	}

	if (ring->tx_pending != new_tx_pending) {
		netdev_info(dev, "illegal Tx ring size value %d, round to %d\n",
			    ring->tx_pending, new_tx_pending);
		ring->tx_pending = new_tx_pending;
	}

	return 0;
}

static void mvpp21_get_mac_address(struct mvpp2_port *port, unsigned char *addr)
{
	u32 mac_addr_l, mac_addr_m, mac_addr_h;

	mac_addr_l = readl(port->base + MVPP2_GMAC_CTRL_1_REG);
	mac_addr_m = readl(port->priv->lms_base + MVPP2_SRC_ADDR_MIDDLE);
	mac_addr_h = readl(port->priv->lms_base + MVPP2_SRC_ADDR_HIGH);
	addr[0] = (mac_addr_h >> 24) & 0xFF;
	addr[1] = (mac_addr_h >> 16) & 0xFF;
	addr[2] = (mac_addr_h >> 8) & 0xFF;
	addr[3] = mac_addr_h & 0xFF;
	addr[4] = mac_addr_m & 0xFF;
	addr[5] = (mac_addr_l >> MVPP2_GMAC_SA_LOW_OFFS) & 0xFF;
}

static int mvpp2_irqs_init(struct mvpp2_port *port)
{
	int err, i;

	for (i = 0; i < port->nqvecs; i++) {
		struct mvpp2_queue_vector *qv = port->qvecs + i;

		if (qv->type == MVPP2_QUEUE_VECTOR_PRIVATE) {
			qv->mask = kzalloc(cpumask_size(), GFP_KERNEL);
			if (!qv->mask) {
				err = -ENOMEM;
				goto err;
			}

			irq_set_status_flags(qv->irq, IRQ_NO_BALANCING);
		}

		err = request_irq(qv->irq, mvpp2_isr, 0, port->dev->name, qv);
		if (err)
			goto err;

		if (qv->type == MVPP2_QUEUE_VECTOR_PRIVATE) {
			unsigned int cpu;

			for_each_present_cpu(cpu) {
				if (mvpp2_cpu_to_thread(port->priv, cpu) ==
				    qv->sw_thread_id)
					cpumask_set_cpu(cpu, qv->mask);
			}

			irq_set_affinity_hint(qv->irq, qv->mask);
		}
	}

	return 0;
err:
	for (i = 0; i < port->nqvecs; i++) {
		struct mvpp2_queue_vector *qv = port->qvecs + i;

		irq_set_affinity_hint(qv->irq, NULL);
		kfree(qv->mask);
		qv->mask = NULL;
		free_irq(qv->irq, qv);
	}

	return err;
}

static void mvpp2_irqs_deinit(struct mvpp2_port *port)
{
	int i;

	for (i = 0; i < port->nqvecs; i++) {
		struct mvpp2_queue_vector *qv = port->qvecs + i;

		irq_set_affinity_hint(qv->irq, NULL);
		kfree(qv->mask);
		qv->mask = NULL;
		irq_clear_status_flags(qv->irq, IRQ_NO_BALANCING);
		free_irq(qv->irq, qv);
	}
}

static bool mvpp22_rss_is_supported(struct mvpp2_port *port)
{
	return (queue_mode == MVPP2_QDIST_MULTI_MODE) &&
		!(port->flags & MVPP2_F_LOOPBACK) &&
		!(port->flags & MVPP22_F_IF_MUSDK);
}

static int mvpp2_open(struct net_device *dev)
{
	struct mvpp2_port *port = netdev_priv(dev);
	struct mvpp2 *priv = port->priv;
	struct mvpp2_port_pcpu *port_pcpu;
	unsigned char mac_bcast[ETH_ALEN] = {
			0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
	bool valid = false;
	int err, cpu;

	err = mvpp2_prs_mac_da_accept(port, mac_bcast, true);
	if (err) {
		netdev_err(dev, "mvpp2_prs_mac_da_accept BC failed\n");
		return err;
	}

	if (port->flags & MVPP22_F_IF_MUSDK)
		goto skip_musdk_parser;

	err = mvpp2_prs_mac_da_accept(port, dev->dev_addr, true);
	if (err) {
		netdev_err(dev, "mvpp2_prs_mac_da_accept own addr failed\n");
		return err;
	}
	err = mvpp2_prs_tag_mode_set(port->priv, port->id, MVPP2_TAG_TYPE_MH);
	if (err) {
		netdev_err(dev, "mvpp2_prs_tag_mode_set failed\n");
		return err;
	}
	err = mvpp2_prs_def_flow(port);
	if (err) {
		netdev_err(dev, "mvpp2_prs_def_flow failed\n");
		return err;
	}

skip_musdk_parser:
	/* Allocate the Rx/Tx queues */
	err = mvpp2_setup_rxqs(port);
	if (err) {
		netdev_err(port->dev, "cannot allocate Rx queues\n");
		return err;
	}

	err = mvpp2_setup_txqs(port);
	if (err) {
		netdev_err(port->dev, "cannot allocate Tx queues\n");
		goto err_cleanup_rxqs;
	}

	/* Recycle buffer pool for performance optimization */
	mvpp2_recycle_open();

	err = mvpp2_irqs_init(port);
	if (err) {
		netdev_err(port->dev, "cannot init IRQs\n");
		goto err_cleanup_txqs;
	}

	/* Phylink isn't supported yet in ACPI mode */
	if (port->of_node) {
		err = phylink_of_phy_connect(port->phylink, port->of_node, 0);
		if (err) {
			netdev_err(port->dev, "could not attach PHY (%d)\n",
				   err);
			goto err_free_irq;
		}

		valid = true;
	}

	if (priv->hw_version != MVPP21 && port->link_irq &&
	    (!port->phylink || !port->has_phy)) {
		mvpp2_txqs_on_tasklet_init(port);
		err = request_irq(port->link_irq, mvpp2_link_status_isr, 0,
				  dev->name, port);
		if (err) {
			netdev_err(port->dev, "cannot request link IRQ %d\n",
				   port->link_irq);
			goto err_free_irq;
		}

		mvpp22_gop_setup_irq(port);

		/* In default link is down */
		netif_carrier_off(port->dev);

		valid = true;
	} else {
		port->link_irq = 0;
	}

	if (!valid) {
		netdev_err(port->dev,
			   "invalid configuration: no dt or link IRQ");
		err = -ENOENT;
		goto err_free_irq;
	}

	/* Init bulk-transmit timer */
	for_each_present_cpu(cpu) {
		port_pcpu = per_cpu_ptr(port->pcpu, cpu);
		port_pcpu->bulk_timer_scheduled = false;
		port_pcpu->bulk_timer_restart_req = false;
	}

	/* Unmask interrupts on all CPUs */
	on_each_cpu(mvpp2_interrupts_unmask, port, 1);
	mvpp2_shared_interrupt_mask_unmask(port, false);

	mvpp2_tx_done_init_on_open(port, true);

	mvpp2_start_dev(port);

	/* Start hardware statistics gathering */
	queue_delayed_work(priv->stats_queue, &port->stats_work,
			   MVPP2_MIB_COUNTERS_STATS_DELAY);

	return 0;

err_free_irq:
	mvpp2_irqs_deinit(port);
err_cleanup_txqs:
	mvpp2_cleanup_txqs(port);
err_cleanup_rxqs:
	mvpp2_cleanup_rxqs(port);
	return err;
}

static int mvpp2_stop(struct net_device *dev)
{
	struct mvpp2_port *port = netdev_priv(dev);
	struct mvpp2_port_pcpu *port_pcpu;
	unsigned int thread;
	int cpu;

	mvpp2_stop_dev(port);

	/* Mask interrupts on all threads */
	on_each_cpu(mvpp2_interrupts_mask, port, 1);
	mvpp2_shared_interrupt_mask_unmask(port, true);

	if (port->phylink)
		phylink_disconnect_phy(port->phylink);
	if (port->link_irq)
		free_irq(port->link_irq, port);

	mvpp2_irqs_deinit(port);
	if (!port->has_tx_irqs) {
		for (thread = 0; thread < port->priv->nthreads; thread++) {
			port_pcpu = per_cpu_ptr(port->pcpu, thread);

			hrtimer_cancel(&port_pcpu->tx_done_timer);
			port_pcpu->tx_done_timer_scheduled = false;
			tasklet_kill(&port_pcpu->tx_done_tasklet);
		}
	}
	/* Cancel bulk tasklet and timer */
	for_each_present_cpu(cpu) {
		port_pcpu = per_cpu_ptr(port->pcpu, cpu);
		hrtimer_cancel(&port_pcpu->bulk_timer);
		tasklet_kill(&port_pcpu->bulk_tasklet);
	}
	mvpp2_tx_done_init_on_open(port, false);
	mvpp2_txqs_on_tasklet_kill(port);
	mvpp2_cleanup_rxqs(port);
	mvpp2_cleanup_txqs(port);

	cancel_delayed_work_sync(&port->stats_work);

	mvpp2_recycle_close();

	return 0;
}

static int mvpp2_prs_mac_da_accept_list(struct mvpp2_port *port,
					struct netdev_hw_addr_list *list)
{
	struct netdev_hw_addr *ha;
	int ret;

	netdev_hw_addr_list_for_each(ha, list) {
		ret = mvpp2_prs_mac_da_accept(port, ha->addr, true);
		if (ret)
			return ret;
	}

	return 0;
}

static void mvpp2_set_rx_promisc(struct mvpp2_port *port, bool enable)
{
	if (!enable && (port->dev->features & NETIF_F_HW_VLAN_CTAG_FILTER))
		mvpp2_prs_vid_enable_filtering(port);
	else
		mvpp2_prs_vid_disable_filtering(port);

	mvpp2_prs_mac_promisc_set(port->priv, port->id,
				  MVPP2_PRS_L2_UNI_CAST, enable);

	mvpp2_prs_mac_promisc_set(port->priv, port->id,
				  MVPP2_PRS_L2_MULTI_CAST, enable);
}

static void mvpp2_set_rx_mode(struct net_device *dev)
{
	struct mvpp2_port *port = netdev_priv(dev);

	/* Clear the whole UC and MC list */
	mvpp2_prs_mac_del_all(port);

	if (dev->flags & IFF_PROMISC) {
		mvpp2_set_rx_promisc(port, true);
		return;
	}

	mvpp2_set_rx_promisc(port, false);

	if (netdev_uc_count(dev) > MVPP2_PRS_MAC_UC_FILT_MAX ||
	    mvpp2_prs_mac_da_accept_list(port, &dev->uc))
		mvpp2_prs_mac_promisc_set(port->priv, port->id,
					  MVPP2_PRS_L2_UNI_CAST, true);

	if (dev->flags & IFF_ALLMULTI) {
		mvpp2_prs_mac_promisc_set(port->priv, port->id,
					  MVPP2_PRS_L2_MULTI_CAST, true);
		return;
	}

	if (netdev_mc_count(dev) > MVPP2_PRS_MAC_MC_FILT_MAX ||
	    mvpp2_prs_mac_da_accept_list(port, &dev->mc))
		mvpp2_prs_mac_promisc_set(port->priv, port->id,
					  MVPP2_PRS_L2_MULTI_CAST, true);
}

static int mvpp2_set_mac_address(struct net_device *dev, void *p)
{
	const struct sockaddr *addr = p;
	int err;

	if (!is_valid_ether_addr(addr->sa_data))
		return -EADDRNOTAVAIL;

	err = mvpp2_prs_update_mac_da(dev, addr->sa_data);
	if (err) {
		/* Reconfigure parser accept the original MAC address */
		mvpp2_prs_update_mac_da(dev, dev->dev_addr);
		netdev_err(dev, "failed to change MAC address\n");
	}
	return err;
}

static int mvpp2_change_mtu(struct net_device *dev, int mtu)
{
	struct mvpp2_port *port = netdev_priv(dev);
	int err;

	if (port->flags & MVPP22_F_IF_MUSDK) {
		netdev_err(dev, "MTU cannot be modified in MUSDK mode\n");
		return -EPERM;
	}

	if (!netif_running(dev)) {
		err = mvpp2_bm_update_mtu(dev, mtu);
		if (!err) {
			port->pkt_size =  MVPP2_RX_PKT_SIZE(mtu);
			return 0;
		}

		/* Reconfigure BM to the original MTU */
		err = mvpp2_bm_update_mtu(dev, dev->mtu);
		if (err)
			goto log_error;
	}

	mvpp2_stop_dev(port);

	err = mvpp2_bm_update_mtu(dev, mtu);
	if (!err) {
		port->pkt_size =  MVPP2_RX_PKT_SIZE(mtu);
		goto out_start;
	}

	/* Reconfigure BM to the original MTU */
	err = mvpp2_bm_update_mtu(dev, dev->mtu);
	if (err)
		goto log_error;

out_start:
	mvpp2_start_dev(port);
	mvpp2_egress_enable(port);
	mvpp2_ingress_enable(port);

	return 0;
log_error:
	netdev_err(dev, "failed to change MTU\n");
	return err;
}

static void
mvpp2_get_stats64(struct net_device *dev, struct rtnl_link_stats64 *stats)
{
	struct mvpp2_port *port = netdev_priv(dev);
	unsigned int start;
	unsigned int cpu;

	for_each_possible_cpu(cpu) {
		struct mvpp2_pcpu_stats *cpu_stats;
		u64 rx_packets;
		u64 rx_bytes;
		u64 tx_packets;
		u64 tx_bytes;

		cpu_stats = per_cpu_ptr(port->stats, cpu);
		do {
			start = u64_stats_fetch_begin_irq(&cpu_stats->syncp);
			rx_packets = cpu_stats->rx_packets;
			rx_bytes   = cpu_stats->rx_bytes;
			tx_packets = cpu_stats->tx_packets;
			tx_bytes   = cpu_stats->tx_bytes;
		} while (u64_stats_fetch_retry_irq(&cpu_stats->syncp, start));

		stats->rx_packets += rx_packets;
		stats->rx_bytes   += rx_bytes;
		stats->tx_packets += tx_packets;
		stats->tx_bytes   += tx_bytes;
	}

	stats->rx_errors	= dev->stats.rx_errors;
	stats->rx_dropped	= dev->stats.rx_dropped;
	stats->tx_dropped	= dev->stats.tx_dropped;
	stats->collisions	= dev->stats.collisions;
}

static int mvpp2_ioctl(struct net_device *dev, struct ifreq *ifr, int cmd)
{
	struct mvpp2_port *port = netdev_priv(dev);

	if (!port->phylink)
		return -ENOTSUPP;

	return phylink_mii_ioctl(port->phylink, ifr, cmd);
}

static int mvpp2_vlan_rx_add_vid(struct net_device *dev, __be16 proto, u16 vid)
{
	struct mvpp2_port *port = netdev_priv(dev);
	int ret;

	ret = mvpp2_prs_vid_entry_add(port, vid);
	if (ret)
		netdev_err(dev, "rx-vlan-filter offloading cannot accept more than %d VIDs per port\n",
			   MVPP2_PRS_VLAN_FILT_MAX - 1);
	return ret;
}

static int mvpp2_vlan_rx_kill_vid(struct net_device *dev, __be16 proto, u16 vid)
{
	struct mvpp2_port *port = netdev_priv(dev);

	mvpp2_prs_vid_entry_remove(port, vid);
	return 0;
}

static int mvpp2_set_features(struct net_device *dev,
			      netdev_features_t features)
{
	netdev_features_t changed = dev->features ^ features;
	struct mvpp2_port *port = netdev_priv(dev);

	if (changed & NETIF_F_HW_VLAN_CTAG_FILTER) {
		if (features & NETIF_F_HW_VLAN_CTAG_FILTER) {
			mvpp2_prs_vid_enable_filtering(port);
		} else {
			/* Invalidate all registered VID filters for this
			 * port
			 */
			mvpp2_prs_vid_remove_all(port);

			mvpp2_prs_vid_disable_filtering(port);
		}
	}

	if (changed & NETIF_F_RXHASH) {
		if (features & NETIF_F_RXHASH)
			mvpp22_rss_enable(port);
		else
			mvpp22_rss_disable(port);
	}

	return 0;
}

/* Ethtool methods */

static int mvpp2_ethtool_nway_reset(struct net_device *dev)
{
	struct mvpp2_port *port = netdev_priv(dev);

	if (!port->phylink)
		return -ENOTSUPP;

	return phylink_ethtool_nway_reset(port->phylink);
}

/* Set interrupt coalescing for ethtools */
static int mvpp2_ethtool_set_coalesce(struct net_device *dev,
				      struct ethtool_coalesce *c)
{
	struct mvpp2_port *port = netdev_priv(dev);
	struct mvpp2_tx_queue *txq;
	int queue;

	for (queue = 0; queue < port->nrxqs; queue++) {
		struct mvpp2_rx_queue *rxq = port->rxqs[queue];

		rxq->time_coal = c->rx_coalesce_usecs;
		rxq->pkts_coal = c->rx_max_coalesced_frames;
		mvpp2_rx_pkts_coal_set(port, rxq);
		mvpp2_rx_time_coal_set(port, rxq);
	}

	/* Set TX time and pkts coalescing configuration */
	if (port->has_tx_irqs)
		port->tx_time_coal = c->tx_coalesce_usecs;

	for (queue = 0; queue < port->ntxqs; queue++) {
		txq = port->txqs[queue];
		txq->done_pkts_coal = c->tx_max_coalesced_frames;
		if (port->has_tx_irqs &&
		    txq->done_pkts_coal > MVPP2_TXQ_THRESH_MASK)
			txq->done_pkts_coal = MVPP2_TXQ_THRESH_MASK;
	}

	if (port->has_tx_irqs) {
		/* Download configured values into MVPP2 HW */
		mvpp2_tx_time_coal_set(port);
		mvpp2_tx_pkts_coal_set(port);
	}

	return 0;
}

/* get coalescing for ethtools */
static int mvpp2_ethtool_get_coalesce(struct net_device *dev,
				      struct ethtool_coalesce *c)
{
	struct mvpp2_port *port = netdev_priv(dev);

	c->rx_coalesce_usecs       = port->rxqs[0]->time_coal;
	c->rx_max_coalesced_frames = port->rxqs[0]->pkts_coal;
	c->tx_max_coalesced_frames = port->txqs[0]->done_pkts_coal;
	c->tx_coalesce_usecs       = port->tx_time_coal;
	return 0;
}

static void mvpp2_ethtool_get_drvinfo(struct net_device *dev,
				      struct ethtool_drvinfo *drvinfo)
{
	struct mvpp2_port *port = netdev_priv(dev);

	strlcpy(drvinfo->driver, MVPP2_DRIVER_NAME,
		sizeof(drvinfo->driver));
	strlcpy(drvinfo->version, MVPP2_DRIVER_VERSION,
		sizeof(drvinfo->version));
	strlcpy(drvinfo->bus_info, dev_name(&dev->dev),
		sizeof(drvinfo->bus_info));
	drvinfo->n_priv_flags = (port->priv->hw_version == MVPP21) ?
			0 : ARRAY_SIZE(mvpp22_priv_flags_strings);
}

static void mvpp2_ethtool_get_ringparam(struct net_device *dev,
					struct ethtool_ringparam *ring)
{
	struct mvpp2_port *port = netdev_priv(dev);

	ring->rx_max_pending = MVPP2_MAX_RXD_MAX;
	ring->tx_max_pending = MVPP2_MAX_TXD_MAX;
	ring->rx_pending = port->rx_ring_size;
	ring->tx_pending = port->tx_ring_size;
}

static int mvpp2_ethtool_set_ringparam(struct net_device *dev,
				       struct ethtool_ringparam *ring)
{
	struct mvpp2_port *port = netdev_priv(dev);
	u16 prev_rx_ring_size = port->rx_ring_size;
	u16 prev_tx_ring_size = port->tx_ring_size;
	int err;

	err = mvpp2_check_ringparam_valid(dev, ring);
	if (err)
		return err;

	if (ring->rx_pending < MSS_THRESHOLD_START && port->tx_fc) {
		netdev_warn(dev, "TX FC disabled. Ring size is less than %d\n",
			    MSS_THRESHOLD_START);
		port->tx_fc = false;
		mvpp2_rxq_disable_fc(port);
		if (port->priv->hw_version == MVPP23)
			mvpp23_rx_fifo_fc_en(port->priv, port->id, false);
	}

	if (!netif_running(dev)) {
		port->rx_ring_size = ring->rx_pending;
		port->tx_ring_size = ring->tx_pending;
		return 0;
	}

	/* The interface is running, so we have to force a
	 * reallocation of the queues
	 */
	mvpp2_stop_dev(port);
	mvpp2_cleanup_rxqs(port);
	mvpp2_cleanup_txqs(port);

	port->rx_ring_size = ring->rx_pending;
	port->tx_ring_size = ring->tx_pending;

	err = mvpp2_setup_rxqs(port);
	if (err) {
		/* Reallocate Rx queues with the original ring size */
		port->rx_ring_size = prev_rx_ring_size;
		ring->rx_pending = prev_rx_ring_size;
		err = mvpp2_setup_rxqs(port);
		if (err)
			goto err_out;
	}
	err = mvpp2_setup_txqs(port);
	if (err) {
		/* Reallocate Tx queues with the original ring size */
		port->tx_ring_size = prev_tx_ring_size;
		ring->tx_pending = prev_tx_ring_size;
		err = mvpp2_setup_txqs(port);
		if (err)
			goto err_clean_rxqs;
	}

	mvpp2_start_dev(port);
	mvpp2_egress_enable(port);
	mvpp2_ingress_enable(port);

	return 0;

err_clean_rxqs:
	mvpp2_cleanup_rxqs(port);
err_out:
	netdev_err(dev, "failed to change ring parameters");
	return err;
}

static void mvpp2_ethtool_get_pause_param(struct net_device *dev,
					  struct ethtool_pauseparam *pause)
{
	struct mvpp2_port *port = netdev_priv(dev);

	if (!port->phylink)
		return;

	phylink_ethtool_get_pauseparam(port->phylink, pause);
}

static void mvpp2_reconfigure_fc(struct mvpp2_port *port)
{
	struct mvpp2_bm_pool **pools_pcpu = port->priv->pools_pcpu;
	int cpu;

	if (recycle) {
		for_each_present_cpu(cpu)
			mvpp2_bm_pool_update_fc(port, pools_pcpu[cpu],
						port->tx_fc);
		if (port->pool_long->type == MVPP2_BM_JUMBO)
			mvpp2_bm_pool_update_fc(port,
						port->pool_long, port->tx_fc);
		else
			mvpp2_bm_pool_update_fc(port,
						port->pool_short, port->tx_fc);
	} else {
		mvpp2_bm_pool_update_fc(port, port->pool_long, port->tx_fc);
		mvpp2_bm_pool_update_fc(port, port->pool_short, port->tx_fc);
	}
	if (port->priv->hw_version == MVPP23)
		mvpp23_rx_fifo_fc_en(port->priv, port->id, port->tx_fc);
}

static int mvpp2_ethtool_set_pause_param(struct net_device *dev,
					 struct ethtool_pauseparam *pause)
{
	struct mvpp2_port *port = netdev_priv(dev);

	if (pause->tx_pause && port->priv->global_tx_fc &&
	    bm_underrun_protect) {
		if (port->rx_ring_size < MSS_THRESHOLD_START) {
			netdev_err(dev, "TX FC cannot be supported.");
			netdev_err(dev, "Ring size is less than %d\n",
				   MSS_THRESHOLD_START);
			return -EINVAL;
		}

		port->tx_fc = true;
		mvpp2_rxq_enable_fc(port);
		mvpp2_reconfigure_fc(port);
	} else if (port->priv->global_tx_fc) {
		port->tx_fc = false;
		mvpp2_rxq_disable_fc(port);
		mvpp2_reconfigure_fc(port);
	}

	if (!port->phylink)
		return -ENOTSUPP;

	return phylink_ethtool_set_pauseparam(port->phylink, pause);
}

static int mvpp2_ethtool_get_link_ksettings(struct net_device *dev,
					    struct ethtool_link_ksettings *cmd)
{
	struct mvpp2_port *port = netdev_priv(dev);

	if (!port->phylink)
		return -ENOTSUPP;

	return phylink_ethtool_ksettings_get(port->phylink, cmd);
}

static int mvpp2_ethtool_set_link_ksettings(struct net_device *dev,
					    const struct ethtool_link_ksettings *cmd)
{
	struct mvpp2_port *port = netdev_priv(dev);

	if (!port->phylink)
		return -ENOTSUPP;

	return phylink_ethtool_ksettings_set(port->phylink, cmd);
}

static int mvpp2_ethtool_get_rxnfc(struct net_device *dev,
				   struct ethtool_rxnfc *info, u32 *rules)
{
	struct mvpp2_port *port = netdev_priv(dev);
	int ret = 0;

	if (!mvpp22_rss_is_supported(port))
		return -EOPNOTSUPP;

	switch (info->cmd) {
	case ETHTOOL_GRXFH:
		ret = mvpp2_ethtool_rxfh_get(port, info);
		break;
	case ETHTOOL_GRXRINGS:
		info->data = port->nrxqs;
		break;
	default:
		return -ENOTSUPP;
	}

	return ret;
}

static int mvpp2_ethtool_set_rxnfc(struct net_device *dev,
				   struct ethtool_rxnfc *info)
{
	struct mvpp2_port *port = netdev_priv(dev);
	int ret = 0;

	if (!mvpp22_rss_is_supported(port))
		return -EOPNOTSUPP;

	switch (info->cmd) {
	case ETHTOOL_SRXFH:
		ret = mvpp2_ethtool_rxfh_set(port, info);
		break;
	default:
		return -EOPNOTSUPP;
	}
	return ret;
}

static u32 mvpp2_ethtool_get_rxfh_indir_size(struct net_device *dev)
{
	struct mvpp2_port *port = netdev_priv(dev);

	return mvpp22_rss_is_supported(port) ? MVPP22_RSS_TABLE_ENTRIES : 0;
}

static int mvpp2_ethtool_get_rxfh(struct net_device *dev, u32 *indir, u8 *key,
				  u8 *hfunc)
{
	struct mvpp2_port *port = netdev_priv(dev);

	if (!mvpp22_rss_is_supported(port))
		return -EOPNOTSUPP;

	if (indir)
		memcpy(indir, port->indir,
		       ARRAY_SIZE(port->indir) * sizeof(port->indir[0]));

	if (hfunc)
		*hfunc = ETH_RSS_HASH_CRC32;

	return 0;
}

static int mvpp2_ethtool_set_rxfh(struct net_device *dev, const u32 *indir,
				  const u8 *key, const u8 hfunc)
{
	struct mvpp2_port *port = netdev_priv(dev);

	if (!mvpp22_rss_is_supported(port))
		return -EOPNOTSUPP;

	if (hfunc != ETH_RSS_HASH_NO_CHANGE && hfunc != ETH_RSS_HASH_CRC32)
		return -EOPNOTSUPP;

	if (key)
		return -EOPNOTSUPP;

	if (indir) {
		memcpy(port->indir, indir,
		       ARRAY_SIZE(port->indir) * sizeof(port->indir[0]));
		mvpp22_rss_fill_table(port, port->id);
	}

	return 0;
}

static u32 mvpp22_get_priv_flags(struct net_device *dev)
{
	struct mvpp2_port *port = netdev_priv(dev);
	u32 priv_flags = 0;

	if (port->flags & MVPP22_F_IF_MUSDK)
		priv_flags |= MVPP22_F_IF_MUSDK_PRIV;
	return priv_flags;
}

static int mvpp2_port_musdk_cfg(struct net_device *dev, bool ena)
{
	struct mvpp2_port_us_cfg {
		unsigned int nqvecs;
		unsigned int nrxqs;
		unsigned int ntxqs;
		int mtu;
		bool rxhash_en;
		u8 rss_en;
	} *us;

	struct mvpp2_port *port = netdev_priv(dev);
	int rxq;

	if (ena) {
		/* Disable Queues and IntVec allocations for MUSDK,
		 * but save original values.
		 */
		us = kzalloc(sizeof(*us), GFP_KERNEL);
		if (!us)
			return -ENOMEM;
		port->us_cfg = (void *)us;
		us->nqvecs = port->nqvecs;
		us->nrxqs  = port->nrxqs;
		us->ntxqs = port->ntxqs;
		us->mtu = dev->mtu;
		us->rxhash_en = !!(dev->hw_features & NETIF_F_RXHASH);

		port->nqvecs = 0;
		port->nrxqs  = 0;
		port->ntxqs  = 0;
		if (us->rxhash_en) {
			dev->hw_features &= ~NETIF_F_RXHASH;
			netdev_update_features(dev);
		}
	} else {
		/* Back to Kernel mode */
		us = port->us_cfg;
		port->nqvecs = us->nqvecs;
		port->nrxqs  = us->nrxqs;
		port->ntxqs  = us->ntxqs;
		if (us->rxhash_en) {
			dev->hw_features |= NETIF_F_RXHASH;
			netdev_update_features(dev);
		}
		kfree(us);
		port->us_cfg = NULL;

		/* Restore RxQ/pool association */
		for (rxq = 0; rxq < port->nrxqs; rxq++) {
			mvpp2_rxq_long_pool_set(port, rxq, port->pool_long->id);
			mvpp2_rxq_short_pool_set(port, rxq,
						 port->pool_short->id);
		}
	}
	return 0;
}

static int mvpp2_port_musdk_set(struct net_device *dev, bool ena)
{
	struct mvpp2_port *port = netdev_priv(dev);
	bool running = netif_running(dev);
	int err;

	/* This procedure is called by ethtool change or by Module-remove.
	 * For "remove" do anything only if we are in musdk-mode
	 * and toggling back to Kernel-mode is really required.
	 */
	if (!ena && !port->us_cfg)
		return 0;

	if (running)
		mvpp2_stop(dev);

	if (ena) {
		err = mvpp2_port_musdk_cfg(dev, ena);
		port->flags |= MVPP22_F_IF_MUSDK;
	} else {
		err = mvpp2_port_musdk_cfg(dev, ena);
		port->flags &= ~MVPP22_F_IF_MUSDK;
	}

	if (err) {
		netdev_err(dev, "musdk set=%d: error=%d\n", ena, err);
		if (err)
			return err;
		/* print Error message but continue */
	}

	if (running)
		mvpp2_open(dev);

	return 0;
}

static int mvpp22_set_priv_flags(struct net_device *dev, u32 priv_flags)
{
	struct mvpp2_port *port = netdev_priv(dev);
	bool f_old, f_new;
	int err = 0;

	if (recycle && (priv_flags & MVPP22_F_IF_MUSDK_PRIV)) {
		WARN(1, "Fail to enable MUSDK. KS recycling feature enabled.");
		return -EOPNOTSUPP;
	}

	f_old = port->flags & MVPP22_F_IF_MUSDK;
	f_new = priv_flags & MVPP22_F_IF_MUSDK_PRIV;
	if (f_old != f_new)
		err = mvpp2_port_musdk_set(dev, f_new);

	return err;
}

/* Device ops */

static const struct net_device_ops mvpp2_netdev_ops = {
	.ndo_open		= mvpp2_open,
	.ndo_stop		= mvpp2_stop,
	.ndo_start_xmit		= mvpp2_tx,
	.ndo_set_rx_mode	= mvpp2_set_rx_mode,
	.ndo_set_mac_address	= mvpp2_set_mac_address,
	.ndo_change_mtu		= mvpp2_change_mtu,
	.ndo_get_stats64	= mvpp2_get_stats64,
	.ndo_do_ioctl		= mvpp2_ioctl,
	.ndo_vlan_rx_add_vid	= mvpp2_vlan_rx_add_vid,
	.ndo_vlan_rx_kill_vid	= mvpp2_vlan_rx_kill_vid,
	.ndo_set_features	= mvpp2_set_features,
};

static const struct ethtool_ops mvpp2_eth_tool_ops = {
	.nway_reset		= mvpp2_ethtool_nway_reset,
	.get_link		= ethtool_op_get_link,
	.set_coalesce		= mvpp2_ethtool_set_coalesce,
	.get_coalesce		= mvpp2_ethtool_get_coalesce,
	.get_drvinfo		= mvpp2_ethtool_get_drvinfo,
	.get_ringparam		= mvpp2_ethtool_get_ringparam,
	.set_ringparam		= mvpp2_ethtool_set_ringparam,
	.get_strings		= mvpp2_ethtool_get_strings,
	.get_ethtool_stats	= mvpp2_ethtool_get_stats,
	.get_sset_count		= mvpp2_ethtool_get_sset_count,
	.get_pauseparam		= mvpp2_ethtool_get_pause_param,
	.set_pauseparam		= mvpp2_ethtool_set_pause_param,
	.get_link_ksettings	= mvpp2_ethtool_get_link_ksettings,
	.set_link_ksettings	= mvpp2_ethtool_set_link_ksettings,
	.get_rxnfc		= mvpp2_ethtool_get_rxnfc,
	.set_rxnfc		= mvpp2_ethtool_set_rxnfc,
	.get_rxfh_indir_size	= mvpp2_ethtool_get_rxfh_indir_size,
	.get_rxfh		= mvpp2_ethtool_get_rxfh,
	.set_rxfh		= mvpp2_ethtool_set_rxfh,
	.get_priv_flags		= mvpp22_get_priv_flags,
	.set_priv_flags		= mvpp22_set_priv_flags,
};

/* Used for PPv2.1, or PPv2.2 with the old Device Tree binding that
 * had a single IRQ defined per-port.
 */
static int mvpp2_simple_queue_vectors_init(struct mvpp2_port *port,
					   struct device_node *port_node)
{
	struct mvpp2_queue_vector *v = &port->qvecs[0];

	v->first_rxq = 0;
	v->nrxqs = port->nrxqs;
	v->type = MVPP2_QUEUE_VECTOR_SHARED;
	v->sw_thread_id = 0;
	v->sw_thread_mask = *cpumask_bits(cpu_online_mask);
	v->port = port;
	v->irq = irq_of_parse_and_map(port_node, 0);
	if (v->irq <= 0)
		return -EINVAL;
	netif_napi_add(port->dev, &v->napi, mvpp2_poll,
		       NAPI_POLL_WEIGHT);

	port->nqvecs = 1;

	return 0;
}

static int mvpp2_multi_queue_vectors_init(struct mvpp2_port *port,
					  struct device_node *port_node)
{
	struct mvpp2 *priv = port->priv;
	struct mvpp2_queue_vector *v;
	int i, ret;

	switch (queue_mode) {
	case MVPP2_QDIST_SINGLE_MODE:
		port->nqvecs = priv->nthreads + 1;
		break;
	case MVPP2_QDIST_MULTI_MODE:
		port->nqvecs = priv->nthreads;
		break;
	}

	for (i = 0; i < port->nqvecs; i++) {
		char irqname[16];

		v = port->qvecs + i;

		v->port = port;
		v->type = MVPP2_QUEUE_VECTOR_PRIVATE;
		v->sw_thread_id = i;
		v->sw_thread_mask = BIT(i);

		if (port->flags & MVPP2_F_DT_COMPAT)
			snprintf(irqname, sizeof(irqname), "tx-cpu%d", i);
		else
			snprintf(irqname, sizeof(irqname), "hif%d", i);

		if (queue_mode == MVPP2_QDIST_MULTI_MODE) {
			v->first_rxq = i;
			v->nrxqs = 1;
		} else if (queue_mode == MVPP2_QDIST_SINGLE_MODE &&
			   i == (port->nqvecs - 1)) {
			v->first_rxq = 0;
			v->nrxqs = port->nrxqs;
			v->type = MVPP2_QUEUE_VECTOR_SHARED;

			if (port->flags & MVPP2_F_DT_COMPAT)
				strncpy(irqname, "rx-shared", sizeof(irqname));
		}

		if (port_node)
			v->irq = of_irq_get_byname(port_node, irqname);
		else
			v->irq = fwnode_irq_get(port->fwnode, i);
		if (v->irq <= 0) {
			ret = -EINVAL;
			goto err;
		}

		netif_napi_add(port->dev, &v->napi, mvpp2_poll,
			       NAPI_POLL_WEIGHT);
	}

	return 0;

err:
	for (i = 0; i < port->nqvecs; i++)
		irq_dispose_mapping(port->qvecs[i].irq);
	return ret;
}

static int mvpp2_queue_vectors_init(struct mvpp2_port *port,
				    struct device_node *port_node)
{
	if (port->has_tx_irqs)
		return mvpp2_multi_queue_vectors_init(port, port_node);
	else
		return mvpp2_simple_queue_vectors_init(port, port_node);
}

static void mvpp2_queue_vectors_deinit(struct mvpp2_port *port)
{
	int i;

	for (i = 0; i < port->nqvecs; i++)
		irq_dispose_mapping(port->qvecs[i].irq);
}

/* Configure Rx queue group interrupt for this port */
static void mvpp2_rx_irqs_setup(struct mvpp2_port *port)
{
	struct mvpp2 *priv = port->priv;
	u32 val;
	int i;

	if (priv->hw_version == MVPP21) {
		mvpp2_write(priv, MVPP21_ISR_RXQ_GROUP_REG(port->id),
			    port->nrxqs);
		return;
	}

	/* Handle the more complicated PPv2.2 and PPv2.3 case */
	for (i = 0; i < port->nqvecs; i++) {
		struct mvpp2_queue_vector *qv = port->qvecs + i;

		if (!qv->nrxqs)
			continue;

		val = qv->sw_thread_id;
		val |= port->id << MVPP22_ISR_RXQ_GROUP_INDEX_GROUP_OFFSET;
		mvpp2_write(priv, MVPP22_ISR_RXQ_GROUP_INDEX_REG, val);

		val = qv->first_rxq;
		val |= qv->nrxqs << MVPP22_ISR_RXQ_SUB_GROUP_SIZE_OFFSET;
		mvpp2_write(priv, MVPP22_ISR_RXQ_SUB_GROUP_CONFIG_REG, val);
	}
}

/* Initialize port HW */
static int mvpp2_port_init(struct mvpp2_port *port)
{
	struct device *dev = port->dev->dev.parent;
	struct mvpp2 *priv = port->priv;
	struct mvpp2_txq_pcpu *txq_pcpu;
	unsigned int thread;
	int queue, err;

	/* Checks for hardware constraints */
	if (port->first_rxq + port->nrxqs >
	    MVPP2_MAX_PORTS * priv->max_port_rxqs)
		return -EINVAL;

	if (port->nrxqs > priv->max_port_rxqs || port->ntxqs > MVPP2_MAX_TXQ)
		return -EINVAL;

	/* Disable port */
	mvpp2_egress_disable(port);
	mvpp2_port_disable(port);

	port->tx_time_coal = MVPP2_TXDONE_COAL_USEC;

	port->txqs = devm_kcalloc(dev, port->ntxqs, sizeof(*port->txqs),
				  GFP_KERNEL);
	if (!port->txqs)
		return -ENOMEM;

	/* Associate physical Tx queues to this port and initialize.
	 * The mapping is predefined.
	 */
	for (queue = 0; queue < port->ntxqs; queue++) {
		int queue_phy_id = mvpp2_txq_phys(port->id, queue);
		struct mvpp2_tx_queue *txq;

		txq = devm_kzalloc(dev, sizeof(*txq), GFP_KERNEL);
		if (!txq) {
			err = -ENOMEM;
			goto err_free_percpu;
		}

		txq->pcpu = alloc_percpu(struct mvpp2_txq_pcpu);
		if (!txq->pcpu) {
			err = -ENOMEM;
			goto err_free_percpu;
		}

		txq->id = queue_phy_id;
		txq->log_id = queue;
		txq->done_pkts_coal = MVPP2_TXDONE_COAL_PKTS_THRESH;
		for (thread = 0; thread < priv->nthreads; thread++) {
			txq_pcpu = per_cpu_ptr(txq->pcpu, thread);
			txq_pcpu->thread = thread;
		}

		port->txqs[queue] = txq;
	}

	port->rxqs = devm_kcalloc(dev, port->nrxqs, sizeof(*port->rxqs),
				  GFP_KERNEL);
	if (!port->rxqs) {
		err = -ENOMEM;
		goto err_free_percpu;
	}

	/* Allocate and initialize Rx queue for this port */
	for (queue = 0; queue < port->nrxqs; queue++) {
		struct mvpp2_rx_queue *rxq;

		/* Map physical Rx queue to port's logical Rx queue */
		rxq = devm_kzalloc(dev, sizeof(*rxq), GFP_KERNEL);
		if (!rxq) {
			err = -ENOMEM;
			goto err_free_percpu;
		}
		/* Map this Rx queue to a physical queue */
		rxq->id = port->first_rxq + queue;
		rxq->port = port->id;
		rxq->logic_rxq = (u8)queue;

		port->rxqs[queue] = rxq;
	}

	mvpp2_rx_irqs_setup(port);

	/* Create Rx descriptor rings */
	for (queue = 0; queue < port->nrxqs; queue++) {
		struct mvpp2_rx_queue *rxq = port->rxqs[queue];

		rxq->size = port->rx_ring_size;
		rxq->pkts_coal = MVPP2_RX_COAL_PKTS;
		rxq->time_coal = MVPP2_RX_COAL_USEC;
	}

	mvpp2_ingress_disable(port);

	/* Port default configuration */
	mvpp2_defaults_set(port);

	/* Port's classifier configuration */
	mvpp2_cls_oversize_rxq_set(port);
	mvpp2_cls_port_config(port);

	if (mvpp22_rss_is_supported(port))
		mvpp22_rss_port_init(port);

	/* Provide an initial Rx packet size */
	port->pkt_size = MVPP2_RX_PKT_SIZE(port->dev->mtu);

	/* Initialize pools for swf */
	if (recycle)
		err = mvpp2_swf_bm_pool_pcpu_init(port);
	else
		err = mvpp2_swf_bm_pool_init(port);
	if (err)
		goto err_free_percpu;

	return 0;

err_free_percpu:
	for (queue = 0; queue < port->ntxqs; queue++) {
		if (!port->txqs[queue])
			continue;
		free_percpu(port->txqs[queue]->pcpu);
	}
	return err;
}

static bool mvpp22_port_has_legacy_tx_irqs(struct device_node *port_node,
					   unsigned long *flags)
{
	char *irqs[5] = { "rx-shared", "tx-cpu0", "tx-cpu1", "tx-cpu2",
			  "tx-cpu3" };
	int i;

	for (i = 0; i < 5; i++)
		if (of_property_match_string(port_node, "interrupt-names",
					     irqs[i]) < 0)
			return false;

	*flags |= MVPP2_F_DT_COMPAT;
	return true;
}

/* Checks if the port dt description has the required Tx interrupts:
 * - PPv2.1: there are no such interrupts.
 * - PPv2.2 and PPv2.3:
 *   - The old DTs have: "rx-shared", "tx-cpuX" with X in [0...3]
 *   - The new ones have: "hifX" with X in [0..8]
 *
 * All those variants are supported to keep the backward compatibility.
 */
static bool mvpp2_port_has_irqs(struct mvpp2 *priv,
				struct device_node *port_node,
				unsigned long *flags)
{
	char name[5];
	int i;

	/* ACPI */
	if (!port_node)
		return true;

	if (priv->hw_version == MVPP21)
		return false;

	if (mvpp22_port_has_legacy_tx_irqs(port_node, flags))
		return true;

	for (i = 0; i < MVPP2_MAX_THREADS; i++) {
		snprintf(name, 5, "hif%d", i);
		if (of_property_match_string(port_node, "interrupt-names",
					     name) < 0)
			return false;
	}

	return true;
}

static void mvpp2_port_copy_mac_addr(struct net_device *dev, struct mvpp2 *priv,
				     struct fwnode_handle *fwnode,
				     char **mac_from)
{
	struct mvpp2_port *port = netdev_priv(dev);
	char hw_mac_addr[ETH_ALEN] = {0};
	char fw_mac_addr[ETH_ALEN];

	if (fwnode_get_mac_address(fwnode, fw_mac_addr, ETH_ALEN)) {
		*mac_from = "firmware node";
		ether_addr_copy(dev->dev_addr, fw_mac_addr);
		return;
	}

	if (priv->hw_version == MVPP21) {
		mvpp21_get_mac_address(port, hw_mac_addr);
		if (is_valid_ether_addr(hw_mac_addr)) {
			*mac_from = "hardware";
			ether_addr_copy(dev->dev_addr, hw_mac_addr);
			return;
		}
	}

	*mac_from = "random";
	eth_hw_addr_random(dev);
}

static struct mvpp2_port *mvpp2_phylink_to_port(struct phylink_config *config)
{
	return container_of(config, struct mvpp2_port, phylink_config);
}

static void mvpp2_phylink_validate(struct phylink_config *config,
				   unsigned long *supported,
				   struct phylink_link_state *state)
{
	struct net_device *dev = to_net_dev(config->dev);
	struct mvpp2_port *port = netdev_priv(dev);
	__ETHTOOL_DECLARE_LINK_MODE_MASK(mask) = { 0, };

	/* Invalid combinations */
	switch (state->interface) {
	case PHY_INTERFACE_MODE_10GKR:
	case PHY_INTERFACE_MODE_5GKR:
	case PHY_INTERFACE_MODE_INTERNAL:
		if (!port->has_xlg_mac)
			goto empty_set;
		break;
	case PHY_INTERFACE_MODE_RXAUI:
		if (port->gop_id != 0)
			goto empty_set;
		break;
	case PHY_INTERFACE_MODE_MII:
		if (port->gop_id == 2)
			goto empty_set;
		/* Fall-through */
	case PHY_INTERFACE_MODE_GMII:
	case PHY_INTERFACE_MODE_RGMII:
	case PHY_INTERFACE_MODE_RGMII_ID:
	case PHY_INTERFACE_MODE_RGMII_RXID:
	case PHY_INTERFACE_MODE_RGMII_TXID:
		if (port->priv->hw_version != MVPP21 && port->gop_id == 0)
			goto empty_set;
		break;
	default:
		break;
	}

	phylink_set(mask, Autoneg);
	phylink_set_port_modes(mask);

	switch (state->interface) {
	case PHY_INTERFACE_MODE_10GKR:
	case PHY_INTERFACE_MODE_RXAUI:
	case PHY_INTERFACE_MODE_NA:
	case PHY_INTERFACE_MODE_INTERNAL:
		if (port->has_xlg_mac) {
			phylink_set(mask, 10000baseT_Full);
			phylink_set(mask, 10000baseCR_Full);
			phylink_set(mask, 10000baseSR_Full);
			phylink_set(mask, 10000baseLR_Full);
			phylink_set(mask, 10000baseLRM_Full);
			phylink_set(mask, 10000baseER_Full);
			phylink_set(mask, 10000baseKR_Full);
		}
		/* Fall-through */
	case PHY_INTERFACE_MODE_5GKR:
		if (port->has_xlg_mac)
			phylink_set(mask, 5000baseT_Full);
		/* Fall-through */
	case PHY_INTERFACE_MODE_2500BASET:
		phylink_set(mask, 2500baseT_Full);
		/* Fall-through */
	case PHY_INTERFACE_MODE_GMII:
	case PHY_INTERFACE_MODE_RGMII:
	case PHY_INTERFACE_MODE_RGMII_ID:
	case PHY_INTERFACE_MODE_RGMII_RXID:
	case PHY_INTERFACE_MODE_RGMII_TXID:
	case PHY_INTERFACE_MODE_SGMII:
		phylink_set(mask, 1000baseT_Full);
		/* Fall-through */
	case PHY_INTERFACE_MODE_MII:
		phylink_set(mask, 10baseT_Half);
		phylink_set(mask, 10baseT_Full);
		phylink_set(mask, 100baseT_Half);
		phylink_set(mask, 100baseT_Full);
		break;
	case PHY_INTERFACE_MODE_1000BASEX:
	case PHY_INTERFACE_MODE_2500BASEX:
		phylink_set(mask, 1000baseX_Full);
		phylink_set(mask, 2500baseX_Full);
		break;
	default:
		goto empty_set;
	}

	bitmap_and(supported, supported, mask, __ETHTOOL_LINK_MODE_MASK_NBITS);
	bitmap_and(state->advertising, state->advertising, mask,
		   __ETHTOOL_LINK_MODE_MASK_NBITS);
	return;

empty_set:
	bitmap_zero(supported, __ETHTOOL_LINK_MODE_MASK_NBITS);
}

static void mvpp22_xlg_link_state(struct mvpp2_port *port,
				  struct phylink_link_state *state)
{
	u32 val;

	if (state->interface == PHY_INTERFACE_MODE_5GKR)
		state->speed = SPEED_5000;
	else
		state->speed = SPEED_10000;

	state->duplex = 1;
	state->an_complete = 1;

	val = readl(port->base + MVPP22_XLG_STATUS);
	state->link = !!(val & MVPP22_XLG_STATUS_LINK_UP);

	state->pause = 0;
	val = readl(port->base + MVPP22_XLG_CTRL0_REG);
	if (val & MVPP22_XLG_CTRL0_TX_FLOW_CTRL_EN)
		state->pause |= MLO_PAUSE_TX;
	if (val & MVPP22_XLG_CTRL0_RX_FLOW_CTRL_EN)
		state->pause |= MLO_PAUSE_RX;
}

static void mvpp2_gmac_link_state(struct mvpp2_port *port,
				  struct phylink_link_state *state)
{
	u32 val;

	val = readl(port->base + MVPP2_GMAC_STATUS0);

	state->an_complete = !!(val & MVPP2_GMAC_STATUS0_AN_COMPLETE);
	state->link = !!(val & MVPP2_GMAC_STATUS0_LINK_UP);
	state->duplex = !!(val & MVPP2_GMAC_STATUS0_FULL_DUPLEX);

	switch (port->phy_interface) {
	case PHY_INTERFACE_MODE_1000BASEX:
		state->speed = SPEED_1000;
		break;
	case PHY_INTERFACE_MODE_2500BASEX:
	case PHY_INTERFACE_MODE_2500BASET:
		state->speed = SPEED_2500;
		break;
	default:
		if (val & MVPP2_GMAC_STATUS0_GMII_SPEED)
			state->speed = SPEED_1000;
		else if (val & MVPP2_GMAC_STATUS0_MII_SPEED)
			state->speed = SPEED_100;
		else
			state->speed = SPEED_10;
	}

	state->pause = 0;
	if (val & MVPP2_GMAC_STATUS0_RX_PAUSE)
		state->pause |= MLO_PAUSE_RX;
	if (val & MVPP2_GMAC_STATUS0_TX_PAUSE)
		state->pause |= MLO_PAUSE_TX;
}

static int mvpp2_phylink_mac_link_state(struct phylink_config *config,
					struct phylink_link_state *state)
{
	struct net_device *dev = to_net_dev(config->dev);
	struct mvpp2_port *port = netdev_priv(dev);

	if (port->has_xlg_mac) {
		u32 mode = readl(port->base + MVPP22_XLG_CTRL3_REG);
		mode &= MVPP22_XLG_CTRL3_MACMODESELECT_MASK;

		if (mode == MVPP22_XLG_CTRL3_MACMODESELECT_10G) {
			mvpp22_xlg_link_state(port, state);
			return 1;
		}
	}

	mvpp2_gmac_link_state(port, state);
	return 1;
}

static void mvpp2_mac_an_restart(struct phylink_config *config)
{
	struct net_device *dev = to_net_dev(config->dev);
	struct mvpp2_port *port = netdev_priv(dev);
	u32 val = readl(port->base + MVPP2_GMAC_AUTONEG_CONFIG);

	writel(val | MVPP2_GMAC_IN_BAND_RESTART_AN,
	       port->base + MVPP2_GMAC_AUTONEG_CONFIG);
	writel(val & ~MVPP2_GMAC_IN_BAND_RESTART_AN,
	       port->base + MVPP2_GMAC_AUTONEG_CONFIG);
}

static void mvpp2_xlg_config(struct mvpp2_port *port, unsigned int mode,
			     const struct phylink_link_state *state)
{
	u32 ctrl0, ctrl4;

	ctrl0 = readl(port->base + MVPP22_XLG_CTRL0_REG);
	ctrl4 = readl(port->base + MVPP22_XLG_CTRL4_REG);

	ctrl0 |= MVPP22_XLG_CTRL0_MAC_RESET_DIS;

	if (state->pause & MLO_PAUSE_TX)
		ctrl0 |= MVPP22_XLG_CTRL0_TX_FLOW_CTRL_EN;
	else
		ctrl0 &= ~MVPP22_XLG_CTRL0_TX_FLOW_CTRL_EN;

	if (state->pause & MLO_PAUSE_RX)
		ctrl0 |= MVPP22_XLG_CTRL0_RX_FLOW_CTRL_EN;
	else
		ctrl0 &= ~MVPP22_XLG_CTRL0_RX_FLOW_CTRL_EN;

	ctrl4 &= ~MVPP22_XLG_CTRL4_MACMODSELECT_GMAC;

	if (state->interface == PHY_INTERFACE_MODE_RXAUI)
		ctrl4 |= MVPP22_XLG_CTRL4_USE_XPCS;

	writel(ctrl0, port->base + MVPP22_XLG_CTRL0_REG);
	writel(ctrl4, port->base + MVPP22_XLG_CTRL4_REG);
}

static void mvpp2_gmac_config(struct mvpp2_port *port, unsigned int mode,
			      const struct phylink_link_state *state)
{
	u32 old_an, an;
	u32 old_ctrl0, ctrl0;
	u32 old_ctrl2, ctrl2;
	u32 old_ctrl4, ctrl4;

	old_an = an = readl(port->base + MVPP2_GMAC_AUTONEG_CONFIG);
	old_ctrl0 = ctrl0 = readl(port->base + MVPP2_GMAC_CTRL_0_REG);
	old_ctrl2 = ctrl2 = readl(port->base + MVPP2_GMAC_CTRL_2_REG);
	old_ctrl4 = ctrl4 = readl(port->base + MVPP22_GMAC_CTRL_4_REG);

	an &= ~(MVPP2_GMAC_CONFIG_MII_SPEED | MVPP2_GMAC_CONFIG_GMII_SPEED |
		MVPP2_GMAC_AN_SPEED_EN | MVPP2_GMAC_FC_ADV_EN |
		MVPP2_GMAC_FC_ADV_ASM_EN | MVPP2_GMAC_FLOW_CTRL_AUTONEG |
		MVPP2_GMAC_CONFIG_FULL_DUPLEX | MVPP2_GMAC_AN_DUPLEX_EN |
		MVPP2_GMAC_IN_BAND_AUTONEG | MVPP2_GMAC_IN_BAND_AUTONEG_BYPASS);
	ctrl0 &= ~MVPP2_GMAC_PORT_TYPE_MASK;
	ctrl2 &= ~(MVPP2_GMAC_INBAND_AN_MASK | MVPP2_GMAC_PORT_RESET_MASK |
		   MVPP2_GMAC_PCS_ENABLE_MASK);
	ctrl4 &= ~(MVPP22_CTRL4_RX_FC_EN | MVPP22_CTRL4_TX_FC_EN);

	/* Configure port type */
	if (phy_interface_mode_is_8023z(state->interface)) {
		ctrl2 |= MVPP2_GMAC_PCS_ENABLE_MASK;
		ctrl4 &= ~MVPP22_CTRL4_EXT_PIN_GMII_SEL;
		ctrl4 |= MVPP22_CTRL4_SYNC_BYPASS_DIS |
			 MVPP22_CTRL4_DP_CLK_SEL |
			 MVPP22_CTRL4_QSGMII_BYPASS_ACTIVE;
	} else if (state->interface == PHY_INTERFACE_MODE_SGMII ||
		   state->interface == PHY_INTERFACE_MODE_2500BASET) {
		ctrl2 |= MVPP2_GMAC_PCS_ENABLE_MASK | MVPP2_GMAC_INBAND_AN_MASK;
		ctrl4 &= ~MVPP22_CTRL4_EXT_PIN_GMII_SEL;
		ctrl4 |= MVPP22_CTRL4_SYNC_BYPASS_DIS |
			 MVPP22_CTRL4_DP_CLK_SEL |
			 MVPP22_CTRL4_QSGMII_BYPASS_ACTIVE;
	} else if ((phy_interface_mode_is_rgmii(state->interface)) ||
		   (state->interface == PHY_INTERFACE_MODE_MII)) {
		ctrl4 &= ~MVPP22_CTRL4_DP_CLK_SEL;
		ctrl4 |= MVPP22_CTRL4_EXT_PIN_GMII_SEL |
			 MVPP22_CTRL4_SYNC_BYPASS_DIS |
			 MVPP22_CTRL4_QSGMII_BYPASS_ACTIVE;
	}

	/* Configure advertisement bits */
	if (phylink_test(state->advertising, Pause))
		an |= MVPP2_GMAC_FC_ADV_EN;
	if (phylink_test(state->advertising, Asym_Pause))
		an |= MVPP2_GMAC_FC_ADV_ASM_EN;

	ctrl4 &= ~(MVPP22_CTRL4_RX_FC_EN | MVPP22_CTRL4_TX_FC_EN);

	/* Configure negotiation style */
	if (!phylink_autoneg_inband(mode)) {
		/* Phy or fixed speed - no in-band AN */
		if (state->duplex)
			an |= MVPP2_GMAC_CONFIG_FULL_DUPLEX;

		if (state->speed == SPEED_1000 || state->speed == SPEED_2500)
			an |= MVPP2_GMAC_CONFIG_GMII_SPEED;
		else if (state->speed == SPEED_100)
			an |= MVPP2_GMAC_CONFIG_MII_SPEED;

	} else if (state->interface == PHY_INTERFACE_MODE_SGMII) {
		/* SGMII in-band mode receives the speed and duplex from
		 * the PHY. Flow control information is not received.
		 */
		an &= ~(MVPP2_GMAC_FORCE_LINK_DOWN |
			MVPP2_GMAC_FORCE_LINK_PASS);
		an |= MVPP2_GMAC_IN_BAND_AUTONEG |
		      MVPP2_GMAC_AN_SPEED_EN |
		      MVPP2_GMAC_AN_DUPLEX_EN;

	} else if (phy_interface_mode_is_8023z(state->interface) ||
		   state->interface == PHY_INTERFACE_MODE_2500BASET) {
		/* 1000BaseX and 2500BaseX ports cannot negotiate speed nor can
		 * they negotiate duplex: they are always operating with a fixed
		 * speed of 1000/2500Mbps in full duplex, so force 1000/2500
		 * speed and full duplex here.
		 */
		ctrl0 |= MVPP2_GMAC_PORT_TYPE_MASK;
		an &= ~(MVPP2_GMAC_FORCE_LINK_DOWN |
			MVPP2_GMAC_FORCE_LINK_PASS);
		an |= MVPP2_GMAC_IN_BAND_AUTONEG |
		      MVPP2_GMAC_IN_BAND_AUTONEG_BYPASS |
		      MVPP2_GMAC_CONFIG_GMII_SPEED |
		      MVPP2_GMAC_CONFIG_FULL_DUPLEX;

		if (state->pause & MLO_PAUSE_AN && state->an_enabled)
			an |= MVPP2_GMAC_FLOW_CTRL_AUTONEG;
	}

	if (state->pause & MLO_PAUSE_TX)
		ctrl4 |= MVPP22_CTRL4_TX_FC_EN;
	if (state->pause & MLO_PAUSE_RX)
		ctrl4 |= MVPP22_CTRL4_RX_FC_EN;

/* Some fields of the auto-negotiation register require the port to be down when
 * their value is updated.
 */
#define MVPP2_GMAC_AN_PORT_DOWN_MASK	\
		(MVPP2_GMAC_IN_BAND_AUTONEG | \
		 MVPP2_GMAC_IN_BAND_AUTONEG_BYPASS | \
		 MVPP2_GMAC_CONFIG_MII_SPEED | MVPP2_GMAC_CONFIG_GMII_SPEED | \
		 MVPP2_GMAC_AN_SPEED_EN | MVPP2_GMAC_CONFIG_FULL_DUPLEX | \
		 MVPP2_GMAC_AN_DUPLEX_EN)

	if ((old_ctrl0 ^ ctrl0) & MVPP2_GMAC_PORT_TYPE_MASK ||
	    (old_ctrl2 ^ ctrl2) & MVPP2_GMAC_INBAND_AN_MASK ||
	    (old_an ^ an) & MVPP2_GMAC_AN_PORT_DOWN_MASK) {
		/* Force link down */
		old_an &= ~MVPP2_GMAC_FORCE_LINK_PASS;
		old_an |= MVPP2_GMAC_FORCE_LINK_DOWN;
		writel(old_an, port->base + MVPP2_GMAC_AUTONEG_CONFIG);

		/* Set the GMAC in a reset state - do this in a way that
		 * ensures we clear it below.
		 */
		old_ctrl2 |= MVPP2_GMAC_PORT_RESET_MASK;
		writel(old_ctrl2, port->base + MVPP2_GMAC_CTRL_2_REG);
	}

	if (old_ctrl0 != ctrl0)
		writel(ctrl0, port->base + MVPP2_GMAC_CTRL_0_REG);
	if (old_ctrl2 != ctrl2)
		writel(ctrl2, port->base + MVPP2_GMAC_CTRL_2_REG);
	if (old_ctrl4 != ctrl4)
		writel(ctrl4, port->base + MVPP22_GMAC_CTRL_4_REG);
	if (old_an != an)
		writel(an, port->base + MVPP2_GMAC_AUTONEG_CONFIG);

	if (old_ctrl2 & MVPP2_GMAC_PORT_RESET_MASK) {
		while (readl(port->base + MVPP2_GMAC_CTRL_2_REG) &
		       MVPP2_GMAC_PORT_RESET_MASK)
			continue;
	}
}

static void mvpp2_mac_config(struct phylink_config *config, unsigned int mode,
			     const struct phylink_link_state *state)
{
	struct mvpp2_port *port = mvpp2_phylink_to_port(config);
	struct net_device *dev = port->dev;
	bool change_interface = port->phy_interface != state->interface;

	/* Check for invalid configuration */
	switch (state->interface) {
	case PHY_INTERFACE_MODE_10GKR:
	case PHY_INTERFACE_MODE_5GKR:
		if (!port->has_xlg_mac) {
			netdev_err(dev, "Invalid mode %s on %s\n",
				   phy_modes(port->phy_interface), dev->name);
			return;
		}
		break;
	case PHY_INTERFACE_MODE_RXAUI:
		if (port->id != 0) {
			netdev_err(dev, "Invalid mode %s on %s\n",
				   phy_modes(port->phy_interface), dev->name);
			return;
		}
	default:
		break;
	};

	if (port->priv->hw_version != MVPP21 && change_interface) {
		/* Make sure the port is disabled when reconfiguring the mode */
		mvpp2_tx_stop_all_queues(port->dev);
		mvpp2_port_disable(port);

		mvpp22_gop_mask_irq(port);

		port->phy_interface = state->interface;

		/* Reconfigure the serdes lanes */
		phy_power_off(port->comphy);
		mvpp22_mode_reconfigure(port);

		mvpp2_tx_wake_all_queues(dev);
		mvpp2_port_enable(port);
	}

	/* mac (re)configuration */
	if (state->interface == PHY_INTERFACE_MODE_RXAUI ||
	    state->interface == PHY_INTERFACE_MODE_10GKR ||
	    state->interface == PHY_INTERFACE_MODE_5GKR) {
		mvpp2_xlg_config(port, mode, state);
	} else {
		mvpp2_gmac_config(port, mode, state);
		mvpp2_gmac_tx_fifo_configure(port);
	}

	if (port->priv->hw_version == MVPP21 && port->flags & MVPP2_F_LOOPBACK)
		mvpp2_port_loopback_set(port, state);

	if (port->priv->hw_version != MVPP21 && change_interface)
		mvpp22_gop_unmask_irq(port);
}

static void mvpp2_mac_link_up(struct phylink_config *config, unsigned int mode,
			      phy_interface_t interface, struct phy_device *phy)
{
	struct mvpp2_port *port = mvpp2_phylink_to_port(config);
	u32 val;

	if (!phylink_autoneg_inband(mode) &&
	    interface != PHY_INTERFACE_MODE_RXAUI &&
	    interface != PHY_INTERFACE_MODE_10GKR &&
	    interface != PHY_INTERFACE_MODE_5GKR) {
		val = readl(port->base + MVPP2_GMAC_AUTONEG_CONFIG);
		val &= ~MVPP2_GMAC_FORCE_LINK_DOWN;
		val |= MVPP2_GMAC_FORCE_LINK_PASS;
		writel(val, port->base + MVPP2_GMAC_AUTONEG_CONFIG);
	}

	mvpp2_port_enable(port);

	mvpp2_egress_enable(port);
	mvpp2_ingress_enable(port);
	mvpp2_tx_wake_all_queues(port->dev);
}

static void mvpp2_mac_link_down(struct phylink_config *config, unsigned int mode,
				phy_interface_t interface)
{
	struct mvpp2_port *port = mvpp2_phylink_to_port(config);
	u32 val;

	if (!phylink_autoneg_inband(mode) &&
	    interface != PHY_INTERFACE_MODE_RXAUI &&
	    interface != PHY_INTERFACE_MODE_10GKR &&
	    interface != PHY_INTERFACE_MODE_5GKR) {
		val = readl(port->base + MVPP2_GMAC_AUTONEG_CONFIG);
		val &= ~MVPP2_GMAC_FORCE_LINK_PASS;
		val |= MVPP2_GMAC_FORCE_LINK_DOWN;
		writel(val, port->base + MVPP2_GMAC_AUTONEG_CONFIG);
	}

	mvpp2_tx_stop_all_queues(port->dev);
	mvpp2_egress_disable(port);
	mvpp2_ingress_disable(port);

	mvpp2_port_disable(port);
}

static const struct phylink_mac_ops mvpp2_phylink_ops = {
	.validate = mvpp2_phylink_validate,
	.mac_link_state = mvpp2_phylink_mac_link_state,
	.mac_an_restart = mvpp2_mac_an_restart,
	.mac_config = mvpp2_mac_config,
	.mac_link_up = mvpp2_mac_link_up,
	.mac_link_down = mvpp2_mac_link_down,
};

/* DSA notifier */
static void mvpp2_dsa_port_register(struct net_device *dev)
{
	struct mvpp2_port *port = netdev_priv(dev);
	struct mvpp2 *priv = port->priv;
	u32 reg;

	/* For switch port enable non-extended DSA tags and make sure
	 * the extended DSA tag usage is disabled as those
	 * two options cannot coexist.
	 */
	reg = mvpp2_read(priv, MVPP2_MH_REG(port->id));
	reg &= ~MVPP2_DSA_EXTENDED;
	reg |= MVPP2_DSA_NON_EXTENDED;
	mvpp2_write(priv, MVPP2_MH_REG(port->id), reg);
}

static int mvpp2_dsa_notifier(struct notifier_block *unused,
			      unsigned long event, void *ptr)
{
	struct dsa_notifier_register_info *info = ptr;

	if (event == DSA_PORT_REGISTER)
		mvpp2_dsa_port_register(info->master);

	return NOTIFY_DONE;
}

/* Ports initialization */
static int mvpp2_port_probe(struct platform_device *pdev,
			    struct fwnode_handle *port_fwnode,
			    struct mvpp2 *priv)
{
	struct phy *comphy = NULL;
	struct mvpp2_port *port;
	struct mvpp2_port_pcpu *port_pcpu;
	struct device_node *port_node = to_of_node(port_fwnode);
	struct net_device *dev;
	struct resource *res;
	struct phylink *phylink;
	char *mac_from = "";
	unsigned int ntxqs, nrxqs;
	unsigned long flags = 0;
	bool has_tx_irqs;
	dma_addr_t p;
	u32 id;
	int features;
	int phy_mode;
	int err, i;
	int cpu;

	has_tx_irqs = mvpp2_port_has_irqs(priv, port_node, &flags);
	if (!has_tx_irqs && queue_mode == MVPP2_QDIST_MULTI_MODE) {
		dev_err(&pdev->dev,
			"not enough IRQs to support multi queue mode\n");
		return -EINVAL;
	}

	ntxqs = MVPP2_MAX_TXQ;
	if (priv->hw_version != MVPP21 && queue_mode ==
	    MVPP2_QDIST_SINGLE_MODE) {
		nrxqs = 1;
	} else {
		/* According to the PPv2.2 datasheet and our experiments on
		 * PPv2.1, RX queues have an allocation granularity of 4 (when
		 * more than a single one on PPv2.2).
		 * Round up to nearest multiple of 4.
		 */
		nrxqs = (num_possible_cpus() + 3) & ~0x3;
		if (nrxqs > MVPP2_PORT_MAX_RXQ)
			nrxqs = MVPP2_PORT_MAX_RXQ;
	}

	dev = alloc_etherdev_mqs(sizeof(*port), ntxqs, nrxqs);
	if (!dev)
		return -ENOMEM;

	/* XPS mapping queues to 0..N cpus (may be less than ntxqs) */
	for_each_online_cpu(cpu)
		netif_set_xps_queue(dev, cpumask_of(cpu), cpu);

	phy_mode = fwnode_get_phy_mode(port_fwnode);
	if (phy_mode < 0) {
		dev_err(&pdev->dev, "incorrect phy mode\n");
		err = phy_mode;
		goto err_free_netdev;
	}

	if (port_node) {
		comphy = devm_of_phy_get(&pdev->dev, port_node, NULL);
		if (IS_ERR(comphy)) {
			if (PTR_ERR(comphy) == -EPROBE_DEFER) {
				err = -EPROBE_DEFER;
				goto err_free_netdev;
			}
			comphy = NULL;
		}
	}

	if (fwnode_property_read_u32(port_fwnode, "port-id", &id)) {
		err = -EINVAL;
		dev_err(&pdev->dev, "missing port-id value\n");
		goto err_free_netdev;
	}

	dev->tx_queue_len = MVPP2_MAX_TXD_MAX;
	dev->watchdog_timeo = 5 * HZ;
	dev->netdev_ops = &mvpp2_netdev_ops;
	dev->ethtool_ops = &mvpp2_eth_tool_ops;

	port = netdev_priv(dev);
	port->dev = dev;
	port->fwnode = port_fwnode;
	port->has_phy = !!of_find_property(port_node, "phy", NULL);
	if (port->has_phy && phy_mode == PHY_INTERFACE_MODE_INTERNAL) {
		err = -EINVAL;
		dev_err(&pdev->dev, "internal mode doesn't work with phy\n");
		goto err_free_netdev;
	}

	port->ntxqs = ntxqs;
	port->nrxqs = nrxqs;
	port->priv = priv;
	port->has_tx_irqs = has_tx_irqs;
	port->flags = flags;

	err = mvpp2_queue_vectors_init(port, port_node);
	if (err)
		goto err_free_netdev;

	if (port_node)
		port->link_irq = of_irq_get_byname(port_node, "link");
	else
		port->link_irq = fwnode_irq_get(port_fwnode, port->nqvecs + 1);
	if (port->link_irq == -EPROBE_DEFER) {
		err = -EPROBE_DEFER;
		goto err_deinit_qvecs;
	}
	if (port->link_irq <= 0)
		/* the link irq is optional */
		port->link_irq = 0;

	if (fwnode_property_read_bool(port_fwnode, "marvell,loopback"))
		port->flags |= MVPP2_F_LOOPBACK;

	port->id = id;
	if (priv->hw_version == MVPP21)
		port->first_rxq = port->id * port->nrxqs;
	else
		port->first_rxq = port->id * priv->max_port_rxqs;

	port->of_node = port_node;
	port->phy_interface = phy_mode;
	port->comphy = comphy;

	if ((port->id == 0 && port->priv->hw_version != MVPP21) ||
	    (port->id == 1 && port->priv->hw_version == MVPP23))
		port->has_xlg_mac = true;

	if (priv->hw_version == MVPP21) {
		res = platform_get_resource(pdev, IORESOURCE_MEM, 2 + id);
		port->base = devm_ioremap_resource(&pdev->dev, res);
		if (IS_ERR(port->base)) {
			err = PTR_ERR(port->base);
			goto err_free_irq;
		}

		port->stats_base = port->priv->lms_base +
				   MVPP21_MIB_COUNTERS_OFFSET +
				   port->gop_id * MVPP21_MIB_COUNTERS_PORT_SZ;
	} else {
		if (fwnode_property_read_u32(port_fwnode, "gop-port-id",
					     &port->gop_id)) {
			err = -EINVAL;
			dev_err(&pdev->dev, "missing gop-port-id value\n");
			goto err_deinit_qvecs;
		}

		port->base = priv->iface_base + MVPP22_GMAC_BASE(port->gop_id);
		port->stats_base = port->priv->iface_base +
				   MVPP22_MIB_COUNTERS_OFFSET +
				   port->gop_id * MVPP22_MIB_COUNTERS_PORT_SZ;
	}

	/* Alloc per-cpu and ethtool stats */
	port->stats = netdev_alloc_pcpu_stats(struct mvpp2_pcpu_stats);
	if (!port->stats) {
		err = -ENOMEM;
		goto err_free_irq;
	}

	p = (dma_addr_t)devm_kcalloc(&pdev->dev,
				     ARRAY_SIZE(mvpp2_ethtool_regs) +
				     L1_CACHE_BYTES,
				     sizeof(u64), GFP_KERNEL);
	if (!p) {
		err = -ENOMEM;
		goto err_free_stats;
	}
	p = (p + ~CACHE_LINE_MASK) & CACHE_LINE_MASK;
	port->ethtool_stats = (void *)p;

	mutex_init(&port->gather_stats_lock);
	INIT_DELAYED_WORK(&port->stats_work, mvpp2_gather_hw_statistics);

	mvpp2_port_copy_mac_addr(dev, priv, port_fwnode, &mac_from);

	port->tx_ring_size = MVPP2_MAX_TXD_DFLT;
	port->rx_ring_size = MVPP2_MAX_RXD_DFLT;
	SET_NETDEV_DEV(dev, &pdev->dev);

	err = mvpp2_port_init(port);
	if (err < 0) {
		dev_err(&pdev->dev, "failed to init port %d\n", id);
		goto err_free_stats;
	}

	mvpp2_port_periodic_xon_disable(port);

	mvpp2_port_reset(port);

	port->pcpu = alloc_percpu(struct mvpp2_port_pcpu);
	if (!port->pcpu) {
		err = -ENOMEM;
		goto err_free_txq_pcpu;
	}

	/* Init tx-done/guard timer and tasklet */
	 mvpp2_tx_done_init_on_probe(pdev, port);

	/* Init bulk timer and tasklet */
	for_each_present_cpu(cpu) {
		port_pcpu = per_cpu_ptr(port->pcpu, cpu);
		hrtimer_init(&port_pcpu->bulk_timer,
			     CLOCK_MONOTONIC, HRTIMER_MODE_REL_PINNED);
		port_pcpu->bulk_timer.function = mvpp2_bulk_timer_cb;
		tasklet_init(&port_pcpu->bulk_tasklet,
			     mvpp2_bulk_tasklet_cb, (unsigned long)dev);
	}

	features = NETIF_F_SG | NETIF_F_IP_CSUM | NETIF_F_IPV6_CSUM |
		   NETIF_F_TSO;
	dev->features = features | NETIF_F_RXCSUM;
	dev->hw_features |= features | NETIF_F_RXCSUM | NETIF_F_GRO |
			    NETIF_F_HW_VLAN_CTAG_FILTER;

	if (mvpp22_rss_is_supported(port))
		dev->hw_features |= NETIF_F_RXHASH;

	if (port->pool_long->id == MVPP2_BM_JUMBO && port->id != 0) {
		dev->features &= ~(NETIF_F_IP_CSUM | NETIF_F_IPV6_CSUM);
		dev->hw_features &= ~(NETIF_F_IP_CSUM | NETIF_F_IPV6_CSUM);
	}

	dev->vlan_features |= features;
	dev->gso_max_segs = MVPP2_MAX_TSO_SEGS;
	dev->priv_flags |= IFF_UNICAST_FLT;

	/* MTU range: 68 - 9704 */
	dev->min_mtu = ETH_MIN_MTU;
	/* 9704 == 9728 - 24 (no rounding for MTU but for frag_size) */
	dev->max_mtu = MVPP2_BM_JUMBO_PKT_SIZE - MVPP2_MTU_OVERHEAD_SIZE;
	dev->dev.of_node = port_node;

	/* Phylink isn't used w/ ACPI as of now */
	if (port_node) {
		port->phylink_config.dev = &dev->dev;
		port->phylink_config.type = PHYLINK_NETDEV;

		phylink = phylink_create(&port->phylink_config, port_fwnode,
					 phy_mode, &mvpp2_phylink_ops);
		if (IS_ERR(phylink)) {
			err = PTR_ERR(phylink);
			goto err_free_port_pcpu;
		}
		port->phylink = phylink;
	} else {
		port->phylink = NULL;
	}

	err = register_netdev(dev);
	if (err < 0) {
		dev_err(&pdev->dev, "failed to register netdev\n");
		goto err_phylink;
	}
	netdev_info(dev, "Using %s mac address %pM\n", mac_from, dev->dev_addr);

	priv->port_list[priv->port_count++] = port;

	/* Port may be configured by Uboot to transmit IDLE, so a remote side
	 * feels the link as UP. Stop TX in same way as in mvpp2_open/stop.
	 */
	if (port->of_node && port->phylink) {
		if (rtnl_is_locked()) {
			if (!phylink_of_phy_connect(port->phylink,
						    port->of_node, 0))
				phylink_disconnect_phy(port->phylink);
		} else {
			rtnl_lock();
			if (!phylink_of_phy_connect(port->phylink,
						    port->of_node, 0))
				phylink_disconnect_phy(port->phylink);
			rtnl_unlock();
		}
	}

	/* Init TX locks and bm locks */
	for (i = 0; i < MVPP2_MAX_THREADS; i++) {
		spin_lock_init(&port->bm_lock[i]);
		spin_lock_init(&port->tx_lock[i]);
	}

	/* Register DSA notifier */
	port->dsa_notifier.notifier_call = mvpp2_dsa_notifier;
	err = register_dsa_notifier(&port->dsa_notifier);
	if (err) {
		dev_err(&pdev->dev, "failed to register DSA notifier\n");
		goto err_phylink;
	}

	return 0;

err_phylink:
	if (port->phylink)
		phylink_destroy(port->phylink);
err_free_port_pcpu:
	free_percpu(port->pcpu);
err_free_txq_pcpu:
	for (i = 0; i < port->ntxqs; i++)
		free_percpu(port->txqs[i]->pcpu);
err_free_stats:
	free_percpu(port->stats);
err_free_irq:
	if (port->link_irq)
		irq_dispose_mapping(port->link_irq);
err_deinit_qvecs:
	mvpp2_queue_vectors_deinit(port);
err_free_netdev:
	free_netdev(dev);
	return err;
}

/* Ports removal routine */
static void mvpp2_port_remove(struct mvpp2_port *port)
{
	int i;

	mvpp2_port_musdk_set(port->dev, false);
	kfree(port->dbgfs_port_flow_entry);
	unregister_netdev(port->dev);
	unregister_dsa_notifier(&port->dsa_notifier);
	if (port->phylink)
		phylink_destroy(port->phylink);
	free_percpu(port->pcpu);
	free_percpu(port->stats);
	for (i = 0; i < port->ntxqs; i++)
		free_percpu(port->txqs[i]->pcpu);
	mvpp2_queue_vectors_deinit(port);
	if (port->link_irq)
		irq_dispose_mapping(port->link_irq);
	free_netdev(port->dev);
}

/* Initialize decoding windows */
static void mvpp2_conf_mbus_windows(const struct mbus_dram_target_info *dram,
				    struct mvpp2 *priv)
{
	u32 win_enable;
	int i;

	for (i = 0; i < 6; i++) {
		mvpp2_write(priv, MVPP2_WIN_BASE(i), 0);
		mvpp2_write(priv, MVPP2_WIN_SIZE(i), 0);

		if (i < 4)
			mvpp2_write(priv, MVPP2_WIN_REMAP(i), 0);
	}

	win_enable = 0;

	for (i = 0; i < dram->num_cs; i++) {
		const struct mbus_dram_window *cs = dram->cs + i;

		mvpp2_write(priv, MVPP2_WIN_BASE(i),
			    (cs->base & 0xffff0000) | (cs->mbus_attr << 8) |
			    dram->mbus_dram_target_id);

		mvpp2_write(priv, MVPP2_WIN_SIZE(i),
			    (cs->size - 1) & 0xffff0000);

		win_enable |= (1 << i);
	}

	mvpp2_write(priv, MVPP2_BASE_ADDR_ENABLE, win_enable);
}

/* Initialize Rx FIFO's */
static void mvpp2_rx_fifo_init(struct mvpp2 *priv)
{
	int port;

	for (port = 0; port < MVPP2_MAX_PORTS; port++) {
		mvpp2_write(priv, MVPP2_RX_DATA_FIFO_SIZE_REG(port),
			    MVPP2_RX_FIFO_PORT_DATA_SIZE_4KB);
		mvpp2_write(priv, MVPP2_RX_ATTR_FIFO_SIZE_REG(port),
			    MVPP2_RX_FIFO_PORT_ATTR_SIZE_4KB);
	}

	mvpp2_write(priv, MVPP2_RX_MIN_PKT_SIZE_REG,
		    MVPP2_RX_FIFO_PORT_MIN_PKT);
	mvpp2_write(priv, MVPP2_RX_FIFO_INIT_REG, 0x1);
}

static void mvpp22_rx_fifo_set_hw(struct mvpp2 *priv, int port, int data_size)
{
	int attr_size = MVPP2_RX_FIFO_PORT_ATTR_SIZE(data_size);

	mvpp2_write(priv, MVPP2_RX_DATA_FIFO_SIZE_REG(port), data_size);
	mvpp2_write(priv, MVPP2_RX_ATTR_FIFO_SIZE_REG(port), attr_size);
}

/* Initialize TX FIFO's: the total FIFO size is 48kB on PPv2.2 and PPv2.3.
 * 4kB fixed space must be assigned for the loopback port.
 * Redistribute remaining avialable 44kB space among all active ports.
 * Guarantee minimum 32kB for 10G port and 8kB for port 1, capable of 2.5G
 * SGMII link.
 */
static void mvpp22_rx_fifo_init(struct mvpp2 *priv)
{
	int port, size;
	unsigned long port_map;
	int remaining_ports_count;
	int size_remainder;

	/* The loopback requires fixed 4kB of the FIFO space assignment. */
	mvpp22_rx_fifo_set_hw(priv, MVPP2_LOOPBACK_PORT_INDEX,
			      MVPP2_RX_FIFO_PORT_DATA_SIZE_4KB);
	port_map = priv->port_map & ~BIT(MVPP2_LOOPBACK_PORT_INDEX);

	/* Set RX FIFO size to 0 for inactive ports. */
	for_each_clear_bit(port, &port_map, MVPP2_LOOPBACK_PORT_INDEX)
		mvpp22_rx_fifo_set_hw(priv, port, 0);

	/* Assign remaining RX FIFO space among all active ports. */
	size_remainder = MVPP2_RX_FIFO_PORT_DATA_SIZE_44KB;
	remaining_ports_count = hweight_long(port_map);

	for_each_set_bit(port, &port_map, MVPP2_LOOPBACK_PORT_INDEX) {
		if (remaining_ports_count == 1)
			size = size_remainder;
		else if (port == 0)
			size = max(size_remainder / remaining_ports_count,
				   MVPP2_RX_FIFO_PORT_DATA_SIZE_32KB);
		else if (port == 1)
			size = max(size_remainder / remaining_ports_count,
				   MVPP2_RX_FIFO_PORT_DATA_SIZE_8KB);
		else
			size = size_remainder / remaining_ports_count;

		size_remainder -= size;
		remaining_ports_count--;

		mvpp22_rx_fifo_set_hw(priv, port, size);
	}

	mvpp2_write(priv, MVPP2_RX_MIN_PKT_SIZE_REG,
		    MVPP2_RX_FIFO_PORT_MIN_PKT);
	mvpp2_write(priv, MVPP2_RX_FIFO_INIT_REG, 0x1);
}

/* Configure Rx FIFO Flow control thresholds */
static void mvpp23_rx_fifo_fc_set_tresh(struct mvpp2 *priv)
{
	int port, val;

	/* Port 0: maximum speed -10Gb/s port
	 *	   required by spec RX FIFO threshold 9KB
	 * Port 1: maximum speed -5Gb/s port
	 *	   required by spec RX FIFO threshold 4KB
	 * Port 2: maximum speed -1Gb/s port
	 *	   required by spec RX FIFO threshold 2KB
	 */

	/* Without loopback port */
	for (port = 0; port < (MVPP2_MAX_PORTS - 1); port++) {
		if (port == 0) {
			val = (MVPP23_PORT0_FIFO_TRSH / MVPP2_RX_FC_TRSH_UNIT)
				<< MVPP2_RX_FC_TRSH_OFFS;
			val &= MVPP2_RX_FC_TRSH_MASK;
			mvpp2_write(priv, MVPP2_RX_FC_REG(port), val);
		} else if (port == 1) {
			val = (MVPP23_PORT1_FIFO_TRSH / MVPP2_RX_FC_TRSH_UNIT)
				<< MVPP2_RX_FC_TRSH_OFFS;
			val &= MVPP2_RX_FC_TRSH_MASK;
			mvpp2_write(priv, MVPP2_RX_FC_REG(port), val);
		} else {
			val = (MVPP23_PORT2_FIFO_TRSH / MVPP2_RX_FC_TRSH_UNIT)
				<< MVPP2_RX_FC_TRSH_OFFS;
			val &= MVPP2_RX_FC_TRSH_MASK;
			mvpp2_write(priv, MVPP2_RX_FC_REG(port), val);
		}
	}
}

/* Configure Rx FIFO Flow control thresholds */
void mvpp23_rx_fifo_fc_en(struct mvpp2 *priv, int port, bool en)
{
	int val;

	val = mvpp2_read(priv, MVPP2_RX_FC_REG(port));

	if (en)
		val |= MVPP2_RX_FC_EN;
	else
		val &= ~MVPP2_RX_FC_EN;

	mvpp2_write(priv, MVPP2_RX_FC_REG(port), val);
}

static void mvpp22_tx_fifo_set_hw(struct mvpp2 *priv, int port, int size)
{
	int threshold = MVPP2_TX_FIFO_THRESHOLD(size);

	mvpp2_write(priv, MVPP22_TX_FIFO_SIZE_REG(port), size);
	mvpp2_write(priv, MVPP22_TX_FIFO_THRESH_REG(port), threshold);
}

/* Initialize TX FIFO's: the total FIFO size is 19kB on PPv2.2 and PPv2.3.
 * 1kB fixed space must be assigned for the loopback port.
 * Redistribute remaining avialable 18kB space among all active ports.
 * The 10G interface should use 10kB (which is maximum possible size
 * per single port).
 */
static void mvpp22_tx_fifo_init_default(struct mvpp2 *priv)
{
	int port, size;
	unsigned long port_map;
	int remaining_ports_count;
	int size_remainder;

	/* The loopback requires fixed 1kB of the FIFO space assignment. */
	mvpp22_tx_fifo_set_hw(priv, MVPP2_LOOPBACK_PORT_INDEX,
			      MVPP22_TX_FIFO_DATA_SIZE_1KB);
	port_map = priv->port_map & ~BIT(MVPP2_LOOPBACK_PORT_INDEX);

	/* Set TX FIFO size to 0 for inactive ports. */
	for_each_clear_bit(port, &port_map, MVPP2_LOOPBACK_PORT_INDEX)
		mvpp22_tx_fifo_set_hw(priv, port, 0);

	/* Assign remaining TX FIFO space among all active ports. */
	size_remainder = MVPP22_TX_FIFO_DATA_SIZE_18KB;
	remaining_ports_count = hweight_long(port_map);

	for_each_set_bit(port, &port_map, MVPP2_LOOPBACK_PORT_INDEX) {
		if (remaining_ports_count == 1)
			size = min(size_remainder,
				   MVPP22_TX_FIFO_DATA_SIZE_10KB);
		else if (port == 0)
			size = MVPP22_TX_FIFO_DATA_SIZE_10KB;
		else
			size = size_remainder / remaining_ports_count;

		size_remainder -= size;
		remaining_ports_count--;

		mvpp22_tx_fifo_set_hw(priv, port, size);
	}
}

static void mvpp22_tx_fifo_init_param(struct platform_device *pdev,
				      struct mvpp2 *priv)
{
	unsigned long port_map;
	int size_remainder;
	int port, size;

	/* The loopback requires fixed 1kB of the FIFO space assignment. */
	mvpp22_tx_fifo_set_hw(priv, MVPP2_LOOPBACK_PORT_INDEX,
			      MVPP22_TX_FIFO_DATA_SIZE_1KB);
	port_map = priv->port_map & ~BIT(MVPP2_LOOPBACK_PORT_INDEX);

	/* Set TX FIFO size to 0 for inactive ports. */
	for_each_clear_bit(port, &port_map, MVPP2_LOOPBACK_PORT_INDEX) {
		mvpp22_tx_fifo_set_hw(priv, port, 0);
		if (MVPP22_TX_FIFO_EXTRA_PARAM_SIZE(port, tx_fifo_map))
			goto error;
	}

	/* The physical port requires minimum 3kB */
	for_each_set_bit(port, &port_map, MVPP2_LOOPBACK_PORT_INDEX) {
		size = MVPP22_TX_FIFO_EXTRA_PARAM_SIZE(port, tx_fifo_map);
		if (size < MVPP22_TX_FIFO_DATA_SIZE_MIN ||
		    size > MVPP22_TX_FIFO_DATA_SIZE_MAX)
			goto error;
	}

	/* Assign remaining TX FIFO space among all active ports. */
	size_remainder = MVPP22_TX_FIFO_DATA_SIZE_18KB;
	for (port = 0; port < MVPP2_LOOPBACK_PORT_INDEX; port++) {
		size = MVPP22_TX_FIFO_EXTRA_PARAM_SIZE(port, tx_fifo_map);
		if (!size)
			continue;
		size_remainder -= size;
		mvpp22_tx_fifo_set_hw(priv, port, size);
	}

	if (size_remainder)
		goto error;

	return;

error:
	dev_warn(&pdev->dev, "Fail to set TX FIFO from module_param, fallback to default\n");
	mvpp22_tx_fifo_init_default(priv);
}

static void mvpp2_axi_init(struct mvpp2 *priv)
{
	u32 val, rdval, wrval;

	mvpp2_write(priv, MVPP22_BM_ADDR_HIGH_RLS_REG, 0x0);

	/* AXI Bridge Configuration */

	rdval = MVPP22_AXI_CODE_CACHE_RD_CACHE
		<< MVPP22_AXI_ATTR_CACHE_OFFS;
	rdval |= MVPP22_AXI_CODE_DOMAIN_OUTER_DOM
		<< MVPP22_AXI_ATTR_DOMAIN_OFFS;

	wrval = MVPP22_AXI_CODE_CACHE_WR_CACHE
		<< MVPP22_AXI_ATTR_CACHE_OFFS;
	wrval |= MVPP22_AXI_CODE_DOMAIN_OUTER_DOM
		<< MVPP22_AXI_ATTR_DOMAIN_OFFS;

	/* BM */
	mvpp2_write(priv, MVPP22_AXI_BM_WR_ATTR_REG, wrval);
	mvpp2_write(priv, MVPP22_AXI_BM_RD_ATTR_REG, rdval);

	/* Descriptors */
	mvpp2_write(priv, MVPP22_AXI_AGGRQ_DESCR_RD_ATTR_REG, rdval);
	mvpp2_write(priv, MVPP22_AXI_TXQ_DESCR_WR_ATTR_REG, wrval);
	mvpp2_write(priv, MVPP22_AXI_TXQ_DESCR_RD_ATTR_REG, rdval);
	mvpp2_write(priv, MVPP22_AXI_RXQ_DESCR_WR_ATTR_REG, wrval);

	/* Buffer Data */
	/* Force TX FIFO transactions priority on the AXI QOS bus */
	if (tx_fifo_protection)
		rdval |= MVPP22_AXI_TX_DATA_RD_QOS_ATTRIBUTE;

	mvpp2_write(priv, MVPP22_AXI_TX_DATA_RD_ATTR_REG, rdval);
	mvpp2_write(priv, MVPP22_AXI_RX_DATA_WR_ATTR_REG, wrval);

	val = MVPP22_AXI_CODE_CACHE_NON_CACHE
		<< MVPP22_AXI_CODE_CACHE_OFFS;
	val |= MVPP22_AXI_CODE_DOMAIN_SYSTEM
		<< MVPP22_AXI_CODE_DOMAIN_OFFS;
	mvpp2_write(priv, MVPP22_AXI_RD_NORMAL_CODE_REG, val);
	mvpp2_write(priv, MVPP22_AXI_WR_NORMAL_CODE_REG, val);

	val = MVPP22_AXI_CODE_CACHE_RD_CACHE
		<< MVPP22_AXI_CODE_CACHE_OFFS;
	val |= MVPP22_AXI_CODE_DOMAIN_OUTER_DOM
		<< MVPP22_AXI_CODE_DOMAIN_OFFS;

	mvpp2_write(priv, MVPP22_AXI_RD_SNOOP_CODE_REG, val);

	val = MVPP22_AXI_CODE_CACHE_WR_CACHE
		<< MVPP22_AXI_CODE_CACHE_OFFS;
	val |= MVPP22_AXI_CODE_DOMAIN_OUTER_DOM
		<< MVPP22_AXI_CODE_DOMAIN_OFFS;

	mvpp2_write(priv, MVPP22_AXI_WR_SNOOP_CODE_REG, val);
}

/* Initialize network controller common part HW */
static int mvpp2_init(struct platform_device *pdev, struct mvpp2 *priv)
{
	const struct mbus_dram_target_info *dram_target_info;
	int err, i;
	u32 val;
	dma_addr_t p;

	/* MBUS windows configuration */
	dram_target_info = mv_mbus_dram_info();
	if (dram_target_info)
		mvpp2_conf_mbus_windows(dram_target_info, priv);

	if (priv->hw_version != MVPP21)
		mvpp2_axi_init(priv);

	/* Disable HW PHY polling */
	if (priv->hw_version == MVPP21) {
		val = readl(priv->lms_base + MVPP2_PHY_AN_CFG0_REG);
		val |= MVPP2_PHY_AN_STOP_SMI0_MASK;
		writel(val, priv->lms_base + MVPP2_PHY_AN_CFG0_REG);
	} else {
		val = readl(priv->iface_base + MVPP22_SMI_MISC_CFG_REG);
		val &= ~MVPP22_SMI_POLLING_EN;
		writel(val, priv->iface_base + MVPP22_SMI_MISC_CFG_REG);
	}

	/* Allocate and initialize aggregated TXQs
	 * The aggr_txqs[per-cpu] entry should be aligned onto cache.
	 * So allocate more than needed and round-up the pointer.
	 */
	val = sizeof(*priv->aggr_txqs) * MVPP2_MAX_THREADS + L1_CACHE_BYTES;
	p = (dma_addr_t)devm_kzalloc(&pdev->dev, val, GFP_KERNEL);
	if (!p)
		return -ENOMEM;
	p = (p + ~CACHE_LINE_MASK) & CACHE_LINE_MASK;
	priv->aggr_txqs = (struct mvpp2_tx_queue *)p;

	for (i = 0; i < MVPP2_MAX_THREADS; i++) {
		priv->aggr_txqs[i].id = i;
		priv->aggr_txqs[i].size = MVPP2_AGGR_TXQ_SIZE;
		err = mvpp2_aggr_txq_init(pdev, &priv->aggr_txqs[i], i, priv);
		if (err < 0)
			return err;
	}

	/* Fifo Init */
	if (priv->hw_version == MVPP21) {
		mvpp2_rx_fifo_init(priv);
	} else {
		mvpp22_rx_fifo_init(priv);
		if (tx_fifo_map)
			mvpp22_tx_fifo_init_param(pdev, priv);
		else
			mvpp22_tx_fifo_init_default(priv);
		if (priv->hw_version == MVPP23)
			mvpp23_rx_fifo_fc_set_tresh(priv);
	}

	if (priv->hw_version == MVPP21)
		writel(MVPP2_EXT_GLOBAL_CTRL_DEFAULT,
		       priv->lms_base + MVPP2_MNG_EXTENDED_GLOBAL_CTRL_REG);

	/* Allow cache snoop when transmiting packets */
	mvpp2_write(priv, MVPP2_TX_SNOOP_REG, 0x1);

	/* Buffer Manager initialization */
	err = mvpp2_bm_init(pdev, priv);
	if (err < 0)
		return err;

	/* Parser default initialization */
	err = mvpp2_prs_default_init(pdev, priv);
	if (err < 0)
		return err;

	/* Classifier default initialization */
	mvpp2_cls_init(priv);

	/* Disable all ingress queues */
	mvpp2_rxq_disable_all(priv);

	return 0;
}

static int mvpp2_get_sram(struct platform_device *pdev,
			  struct mvpp2 *priv)
{
	struct device_node *dn = pdev->dev.of_node;
	struct resource *res;

	if (has_acpi_companion(&pdev->dev)) {
		res = platform_get_resource(pdev, IORESOURCE_MEM, 2);
		if (!res) {
			dev_warn(&pdev->dev, "ACPI is too old, TX FC disabled\n");
			return 0;
		}
		priv->cm3_base = devm_ioremap_resource(&pdev->dev, res);
		if (IS_ERR(priv->cm3_base))
			return PTR_ERR(priv->cm3_base);
	} else {
		priv->sram_pool = of_gen_pool_get(dn, "cm3-mem", 0);
		if (!priv->sram_pool) {
			dev_warn(&pdev->dev, "DT is too old, TX FC disabled\n");
			return 0;
		}
		priv->cm3_base = (void __iomem *)gen_pool_alloc(priv->sram_pool,
								MSS_SRAM_SIZE);
		if (!priv->cm3_base)
			return -ENOMEM;
	}
	return 0;
}

static int mvpp2_probe(struct platform_device *pdev)
{
	const struct acpi_device_id *acpi_id;
	struct fwnode_handle *fwnode = pdev->dev.fwnode;
	struct fwnode_handle *port_fwnode;
	struct mvpp2 *priv;
	struct resource *res;
	void __iomem *base;
	int i, shared;
	int err;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	if (has_acpi_companion(&pdev->dev)) {
		acpi_id = acpi_match_device(pdev->dev.driver->acpi_match_table,
					    &pdev->dev);
		priv->hw_version = (unsigned long)acpi_id->driver_data;
	} else {
		priv->hw_version =
			(unsigned long)of_device_get_match_data(&pdev->dev);
	}

	/* multi queue mode isn't supported on PPV2.1, fallback to single
	 * mode
	 */
	if (priv->hw_version == MVPP21)
		queue_mode = MVPP2_QDIST_SINGLE_MODE;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(base))
		return PTR_ERR(base);

	if (priv->hw_version == MVPP21) {
		res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
		priv->lms_base = devm_ioremap_resource(&pdev->dev, res);
		if (IS_ERR(priv->lms_base))
			return PTR_ERR(priv->lms_base);
	} else {
		res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
		if (has_acpi_companion(&pdev->dev)) {
			/* In case the MDIO memory region is declared in
			 * the ACPI, it can already appear as 'in-use'
			 * in the OS. Because it is overlapped by second
			 * region of the network controller, make
			 * sure it is released, before requesting it again.
			 * The care is taken by mvpp2 driver to avoid
			 * concurrent access to this memory region.
			 */
			release_resource(res);
		}
		priv->iface_base = devm_ioremap_resource(&pdev->dev, res);
		if (IS_ERR(priv->iface_base))
			return PTR_ERR(priv->iface_base);

		/* Map CM3 SRAM */
		err = mvpp2_get_sram(pdev, priv);
		if (err)
			dev_warn(&pdev->dev, "Fail to alloc CM3 SRAM\n");

		/* Enable global Flow Control only if hanler to SRAM not NULL */
		if (priv->cm3_base)
			priv->global_tx_fc = true;
	}

	if (priv->hw_version != MVPP21 && dev_of_node(&pdev->dev)) {
		priv->sysctrl_base =
			syscon_regmap_lookup_by_phandle(pdev->dev.of_node,
							"marvell,system-controller");
		if (IS_ERR(priv->sysctrl_base))
			/* The system controller regmap is optional for dt
			 * compatibility reasons. When not provided, the
			 * configuration of the GoP relies on the
			 * firmware/bootloader.
			 */
			priv->sysctrl_base = NULL;
	}

	priv->nthreads = min_t(unsigned int, num_present_cpus(),
			       MVPP2_MAX_THREADS);

	shared = num_present_cpus() - priv->nthreads;
	if (shared > 0)
		bitmap_fill(&priv->lock_map,
			    min_t(int, shared, MVPP2_MAX_THREADS));

	for (i = 0; i < MVPP2_MAX_THREADS; i++) {
		u32 addr_space_sz;

		addr_space_sz = (priv->hw_version == MVPP21 ?
				 MVPP21_ADDR_SPACE_SZ : MVPP22_ADDR_SPACE_SZ);
		priv->swth_base[i] = base + i * addr_space_sz;
	}

	if (dev_of_node(&pdev->dev)) {
		priv->pp_clk = devm_clk_get(&pdev->dev, "pp_clk");
		if (IS_ERR(priv->pp_clk)) {
			err = PTR_ERR(priv->pp_clk);
			goto err_cm3;
		}
		err = clk_prepare_enable(priv->pp_clk);
		if (err < 0)
			goto err_cm3;

		priv->gop_clk = devm_clk_get(&pdev->dev, "gop_clk");
		if (IS_ERR(priv->gop_clk)) {
			err = PTR_ERR(priv->gop_clk);
			goto err_pp_clk;
		}
		err = clk_prepare_enable(priv->gop_clk);
		if (err < 0)
			goto err_pp_clk;

		if (priv->hw_version != MVPP21) {
			priv->mg_clk = devm_clk_get(&pdev->dev, "mg_clk");
			if (IS_ERR(priv->mg_clk)) {
				err = PTR_ERR(priv->mg_clk);
				goto err_gop_clk;
			}

			err = clk_prepare_enable(priv->mg_clk);
			if (err < 0)
				goto err_gop_clk;

			priv->mg_core_clk = devm_clk_get(&pdev->dev, "mg_core_clk");
			if (IS_ERR(priv->mg_core_clk)) {
				priv->mg_core_clk = NULL;
			} else {
				err = clk_prepare_enable(priv->mg_core_clk);
				if (err < 0)
					goto err_mg_clk;
			}
		}

		priv->axi_clk = devm_clk_get(&pdev->dev, "axi_clk");
		if (IS_ERR(priv->axi_clk)) {
			err = PTR_ERR(priv->axi_clk);
			if (err == -EPROBE_DEFER)
				goto err_mg_core_clk;
			priv->axi_clk = NULL;
		} else {
			err = clk_prepare_enable(priv->axi_clk);
			if (err < 0)
				goto err_mg_core_clk;
		}

		/* Get system's tclk rate */
		priv->tclk = clk_get_rate(priv->pp_clk);
	} else if (device_property_read_u32(&pdev->dev, "clock-frequency",
					    &priv->tclk)) {
		dev_err(&pdev->dev, "missing clock-frequency value\n");
		return -EINVAL;
	}

	if (priv->hw_version != MVPP21) {
		if (mvpp2_read(priv, MVPP2_VER_ID_REG) == MVPP2_VER_PP23)
			priv->hw_version = MVPP23;
	}

	if (priv->hw_version == MVPP21)
		priv->max_port_rxqs = 8;
	else
		priv->max_port_rxqs = 32;

	priv->custom_dma_mask = false;
	if (priv->hw_version != MVPP21) {
		/* If dma_mask points to coherent_dma_mask, setting both will
		 * override the value of the other. This is problematic as the
		 * PPv2 driver uses a 32-bit-mask for coherent accesses (txq,
		 * rxq, bm) and a 40-bit mask for all other accesses.
		 */
		if (pdev->dev.dma_mask == &pdev->dev.coherent_dma_mask) {
			pdev->dev.dma_mask =
				kzalloc(sizeof(*pdev->dev.dma_mask),
					GFP_KERNEL);
			if (!pdev->dev.dma_mask) {
				err = -ENOMEM;
				goto err_mg_clk;
			}

			priv->custom_dma_mask = true;
		}

		err = dma_set_mask(&pdev->dev, MVPP2_DESC_DMA_MASK);
		if (err)
			goto err_dma_mask;

		/* Sadly, the BM pools all share the same register to
		 * store the high 32 bits of their address. So they
		 * must all have the same high 32 bits, which forces
		 * us to restrict coherent memory to DMA_BIT_MASK(32).
		 */
		err = dma_set_coherent_mask(&pdev->dev, DMA_BIT_MASK(32));
		if (err)
			goto err_dma_mask;
	}

	/* Assign the reserved memory region to the device for DMA allocations,
	 * if a memory-region phandle is found.
	 */
	if (dev_of_node(&pdev->dev))
		of_reserved_mem_device_init_by_idx(&pdev->dev,
						   pdev->dev.of_node, 0);

	/* Configure branch prediction switch */
	if (priv->hw_version == MVPP21)
		static_branch_enable(&mvpp21_variant);
	if (recycle) {
		dev_info(&pdev->dev,
			 "kernel space packet recycling feature enabled\n");
		static_branch_enable(&mvpp2_recycle_ena);
	}
	/* else - keep the DEFINE_STATIC_KEY_FALSE */

	/* Map DTS-active ports. Should be done before FIFO mvpp2_init */
	fwnode_for_each_available_child_node(fwnode, port_fwnode) {
		if (!fwnode_property_read_u32(port_fwnode, "port-id", &i))
			priv->port_map |= BIT(i);
	}

	/* Init mss lock */
	spin_lock_init(&priv->mss_spinlock);

	/* Initialize network controller */
	err = mvpp2_init(pdev, priv);
	if (err < 0) {
		dev_err(&pdev->dev, "failed to initialize controller\n");
		goto err_axi_clk;
	}

	/* Initialize ports */
	fwnode_for_each_available_child_node(fwnode, port_fwnode) {
		err = mvpp2_port_probe(pdev, port_fwnode, priv);
		if (err < 0)
			goto err_port_probe;
	}

	if (priv->port_count == 0) {
		dev_err(&pdev->dev, "no ports enabled\n");
		err = -ENODEV;
		goto err_axi_clk;
	}

	/* Statistics must be gathered regularly because some of them (like
	 * packets counters) are 32-bit registers and could overflow quite
	 * quickly. For instance, a 10Gb link used at full bandwidth with the
	 * smallest packets (64B) will overflow a 32-bit counter in less than
	 * 30 seconds. Then, use a workqueue to fill 64-bit counters.
	 */
	snprintf(priv->queue_name, sizeof(priv->queue_name),
		 "stats-wq-%s%s", netdev_name(priv->port_list[0]->dev),
		 priv->port_count > 1 ? "+" : "");
	priv->stats_queue = create_singlethread_workqueue(priv->queue_name);
	if (!priv->stats_queue) {
		err = -ENOMEM;
		goto err_port_probe;
	}

	if (priv->global_tx_fc && priv->hw_version != MVPP21) {
		err = mvpp2_enable_global_fc(priv);
		if (err)
			dev_warn(&pdev->dev, "CM3 firmware not running, TX FC disabled\n");
	}

	mvpp2_dbgfs_init(priv, pdev->name);

	platform_set_drvdata(pdev, priv);
	return 0;

err_port_probe:
	i = 0;
	fwnode_for_each_available_child_node(fwnode, port_fwnode) {
		if (priv->port_list[i])
			mvpp2_port_remove(priv->port_list[i]);
		i++;
	}
err_dma_mask:
	if (priv->custom_dma_mask) {
		kfree(pdev->dev.dma_mask);
		pdev->dev.dma_mask = &pdev->dev.coherent_dma_mask;
	}
err_axi_clk:
	clk_disable_unprepare(priv->axi_clk);

err_mg_core_clk:
	if (priv->hw_version != MVPP21)
		clk_disable_unprepare(priv->mg_core_clk);
err_mg_clk:
	if (priv->hw_version != MVPP21)
		clk_disable_unprepare(priv->mg_clk);
err_gop_clk:
	clk_disable_unprepare(priv->gop_clk);
err_pp_clk:
	clk_disable_unprepare(priv->pp_clk);
err_cm3:
	if (!has_acpi_companion(&pdev->dev) && priv->cm3_base)
		gen_pool_free(priv->sram_pool, (unsigned long)priv->cm3_base,
			      MSS_SRAM_SIZE);

	return err;
}

static int mvpp2_remove(struct platform_device *pdev)
{
	struct mvpp2 *priv = platform_get_drvdata(pdev);
	struct fwnode_handle *fwnode = pdev->dev.fwnode;
	struct fwnode_handle *port_fwnode;
	int i = 0;

	mvpp2_dbgfs_cleanup(priv);

	flush_workqueue(priv->stats_queue);
	destroy_workqueue(priv->stats_queue);

	fwnode_for_each_available_child_node(fwnode, port_fwnode) {
		if (priv->port_list[i]) {
			mutex_destroy(&priv->port_list[i]->gather_stats_lock);
			mvpp2_port_remove(priv->port_list[i]);
		}
		i++;
	}

	for (i = 0; i < MVPP2_BM_POOLS_NUM; i++) {
		struct mvpp2_bm_pool *bm_pool = &priv->bm_pools[i];

		mvpp2_bm_pool_destroy(pdev, priv, bm_pool);
	}

	for (i = 0; i < MVPP2_MAX_THREADS; i++) {
		struct mvpp2_tx_queue *aggr_txq = &priv->aggr_txqs[i];

		dma_free_coherent(&pdev->dev,
				  MVPP2_AGGR_TXQ_SIZE * MVPP2_DESC_ALIGNED_SIZE,
				  aggr_txq->descs,
				  aggr_txq->descs_dma);
	}

	if (priv->custom_dma_mask) {
		kfree(pdev->dev.dma_mask);
		pdev->dev.dma_mask = &pdev->dev.coherent_dma_mask;
	}

	if (!has_acpi_companion(&pdev->dev)) {
		gen_pool_free(priv->sram_pool, (unsigned long)priv->cm3_base,
			      MSS_SRAM_SIZE);
		gen_pool_destroy(priv->sram_pool);
	}

	if (is_acpi_node(port_fwnode))
		return 0;

	clk_disable_unprepare(priv->axi_clk);
	clk_disable_unprepare(priv->mg_core_clk);
	clk_disable_unprepare(priv->mg_clk);
	clk_disable_unprepare(priv->pp_clk);
	clk_disable_unprepare(priv->gop_clk);

	return 0;
}

static const struct of_device_id mvpp2_match[] = {
	{
		.compatible = "marvell,armada-375-pp2",
		.data = (void *)MVPP21,
	},
	{
		.compatible = "marvell,armada-7k-pp22",
		.data = (void *)MVPP22,
	},
	{ }
};
MODULE_DEVICE_TABLE(of, mvpp2_match);

static const struct acpi_device_id mvpp2_acpi_match[] = {
	{ "MRVL0110", MVPP22 },
	{ },
};
MODULE_DEVICE_TABLE(acpi, mvpp2_acpi_match);

static struct platform_driver mvpp2_driver = {
	.probe = mvpp2_probe,
	.remove = mvpp2_remove,
	.driver = {
		.name = MVPP2_DRIVER_NAME,
		.of_match_table = mvpp2_match,
		.acpi_match_table = ACPI_PTR(mvpp2_acpi_match),
	},
};

module_platform_driver(mvpp2_driver);

MODULE_DESCRIPTION("Marvell PPv2 Ethernet Driver - www.marvell.com");
MODULE_AUTHOR("Marcin Wojtas <mw@semihalf.com>");
MODULE_LICENSE("GPL v2");
