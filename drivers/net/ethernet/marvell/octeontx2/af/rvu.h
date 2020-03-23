/* SPDX-License-Identifier: GPL-2.0
 * Marvell OcteonTx2 RVU Admin Function driver
 *
 * Copyright (C) 2018 Marvell International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef RVU_H
#define RVU_H

#include <linux/pci.h>
#include "rvu_struct.h"
#include "common.h"
#include "mbox.h"
#include "npc.h"
#include "rvu_validation.h"

/* PCI device IDs */
#define	PCI_DEVID_OCTEONTX2_RVU_AF		0xA065

/* Subsystem Device ID */
#define PCI_SUBSYS_DEVID_98XX                  0xB100
#define PCI_SUBSYS_DEVID_96XX                  0xB200

/* PCI BAR nos */
#define	PCI_AF_REG_BAR_NUM			0
#define	PCI_PF_REG_BAR_NUM			2
#define	PCI_MBOX_BAR_NUM			4

#define NAME_SIZE				32

/* PF_FUNC */
#define RVU_PFVF_PF_SHIFT	10
#define RVU_PFVF_PF_MASK	0x3F
#define RVU_PFVF_FUNC_SHIFT	0
#define RVU_PFVF_FUNC_MASK	0x3FF

/* CONFIG_DEBUG_FS */
#ifdef CONFIG_DEBUG_FS
struct dump_ctx {
	int	lf;
	int	id;
	bool	all;
};

struct cpt_dump_ctx {
	char    e_type[NAME_SIZE];
};

struct rvu_debugfs {
	struct dentry *root;
	struct dentry *npa;
	struct dentry *cgx_root;
	struct dentry *cgx;
	struct dentry *lmac;
	struct dentry *nix;
	struct dentry *npc;
	struct dentry *sso;
	struct dentry *sso_hwgrp;
	struct dentry *sso_hws;
	struct dentry *cpt;
	struct dump_ctx npa_aura_ctx;
	struct dump_ctx npa_pool_ctx;
	struct dump_ctx nix_cq_ctx;
	struct dump_ctx nix_rq_ctx;
	struct dump_ctx nix_sq_ctx;
	struct cpt_dump_ctx cpt_ctx;
	int npa_qsize_id;
	int nix_qsize_id;
};
#endif /* CONFIG_DEBUG_FS */

struct rvu_work {
	struct	work_struct work;
	struct	rvu *rvu;
	int num_msgs;
	int up_num_msgs;
};

struct rsrc_bmap {
	unsigned long *bmap;	/* Pointer to resource bitmap */
	u16  max;		/* Max resource id or count */
};

struct rvu_block {
	struct rsrc_bmap	lf;
	struct admin_queue	*aq; /* NIX/NPA AQ */
	u16  *fn_map; /* LF to pcifunc mapping */
	bool multislot;
	bool implemented;
	u8   addr;  /* RVU_BLOCK_ADDR_E */
	u8   type;  /* RVU_BLOCK_TYPE_E */
	u8   lfshift;
	u64  lookup_reg;
	u64  pf_lfcnt_reg;
	u64  vf_lfcnt_reg;
	u64  lfcfg_reg;
	u64  msixcfg_reg;
	u64  lfreset_reg;
	unsigned char name[NAME_SIZE];
};

struct nix_mcast {
	struct qmem	*mce_ctx;
	struct qmem	*mcast_buf;
	int		replay_pkind;
	int		next_free_mce;
	struct mutex	mce_lock; /* Serialize MCE updates */
};

struct nix_mce_list {
	struct hlist_head	head;
	int			count;
	int			max;
};

/* list of known and supported fields in packet header and
 * fields present in key structure.
 */
enum key_fields {
	NPC_CHAN = NPC_HEADER_FIELDS_MAX, /* Valid when Rx */
	NPC_PF_FUNC, /* Valid when Tx */
	NPC_ERRLEV,
	NPC_ERRCODE,
	NPC_LXMB,
	NPC_LA,
	NPC_LB,
	NPC_LC,
	NPC_LD,
	NPC_LE,
	NPC_LF,
	NPC_LG,
	NPC_LH,
	/* ether type for untagged frame */
	NPC_ETYPE_ETHER,
	/* ether type for single tagged frame */
	NPC_ETYPE_TAG1,
	/* ether type for double tagged frame */
	NPC_ETYPE_TAG2,
	/* outer vlan tci for single tagged frame */
	NPC_VLAN_TAG1,
	/* outer vlan tci for double tagged frame */
	NPC_VLAN_TAG2,
	/* other header fields programmed to extract but not of our interest */
	NPC_UNKNOWN,
	NPC_KEY_FIELDS_MAX,
};

/* layer meta data to uniquely identify a packet header field */
struct npc_layer_mdata {
	u8 lid;
	u8 ltype;
	u8 hdr;
	u8 key;
	u8 len;
};

/* Structure to represent a field present in the
 * generated key. A key field may present anywhere and can
 * be of any size in the generated key. Once this structure
 * is populated for fields of interest then field's presence
 * and location (if present) can be known.
 */
struct npc_key_field {
	/* Masks where all set bits indicate position
	 * of a field in the key
	 */
	u64 kw_mask[NPC_MAX_KWS_IN_KEY];
	/* Number of words in the key a field spans. If a field is
	 * of 16 bytes and key offset is 4 then the field will use
	 * 4 bytes in KW0, 8 bytes in KW1 and 4 bytes in KW2 and
	 * nr_kws will be 3(KW0, KW1 and KW2).
	 */
	int nr_kws;
	/* used by packet header fields */
	struct npc_layer_mdata layer_mdata;
};

struct npc_mcam {
	struct rsrc_bmap counters;
	struct mutex	lock;	/* MCAM entries and counters update lock */
	unsigned long	*bmap;		/* bitmap, 0 => bmap_entries */
	unsigned long	*bmap_reverse;	/* Reverse bitmap, bmap_entries => 0 */
	u16	bmap_entries;	/* Number of unreserved MCAM entries */
	u16	bmap_fcnt;	/* MCAM entries free count */
	u16	*entry2pfvf_map;
	u16	*entry2cntr_map;
	u16	*cntr2pfvf_map;
	u16	*cntr_refcnt;
	u8	keysize;	/* MCAM keysize 112/224/448 bits */
	u8	banks;		/* Number of MCAM banks */
	u8	banks_per_entry;/* Number of keywords in key */
	u16	banksize;	/* Number of MCAM entries in each bank */
	u16	total_entries;	/* Total number of MCAM entries */
	u16	nixlf_offset;	/* Offset of nixlf rsvd uncast entries */
	u16	pf_offset;	/* Offset of PF's rsvd bcast, promisc entries */
	u16	lprio_count;
	u16	lprio_start;
	u16	hprio_count;
	u16	hprio_end;
	u16	rx_miss_act_cntr; /* Counter for RX MISS action */
	/* fields present in the generated key */
	struct npc_key_field	tx_key_fields[NPC_KEY_FIELDS_MAX];
	struct npc_key_field	rx_key_fields[NPC_KEY_FIELDS_MAX];
	u64	tx_features;
	u64	rx_features;
	struct list_head mcam_rules;
};

/* KPU profile adapter structure which is used to translate between built-in and
 * firmware KPU profile structures.
 */
struct npc_kpu_profile_adapter {
	const char		*name;
	u64			version;
	struct npc_lt_def_cfg	*lt_def;
	struct npc_kpu_profile_action	*ikpu; /* array[pkinds] */
	struct npc_kpu_profile	*kpu; /* array[kpus] */
	struct npc_mcam_kex	*mkex;
	bool			custom; /* true if loadable profile used */
	size_t			pkinds;
	size_t			kpus;
};

struct sso_rsrc {
	u8      sso_hws;
	u16     sso_hwgrps;
	u16     sso_xaq_num_works;
	u16     sso_xaq_buf_size;
	u16     sso_iue;
	u64	iaq_rsvd;
	u64	iaq_max;
	u64	taq_rsvd;
	u64	taq_max;
	struct rsrc_bmap pfvf_ident;
};

/* Structure for per RVU func info ie PF/VF */
struct rvu_pfvf {
	bool		npalf; /* Only one NPALF per RVU_FUNC */
	bool		nixlf; /* Only one NIXLF per RVU_FUNC */
	u16		sso;
	u16		ssow;
	u16		cptlfs;
	u16		timlfs;
	u8		cgx_lmac;
	u8		sso_uniq_ident;

	/* Block LF's MSIX vector info */
	struct rsrc_bmap msix;      /* Bitmap for MSIX vector alloc */
#define MSIX_BLKLF(blkaddr, lf) (((blkaddr) << 8) | ((lf) & 0xFF))
	u16		 *msix_lfmap; /* Vector to block LF mapping */

	/* NPA contexts */
	struct qmem	*aura_ctx;
	struct qmem	*pool_ctx;
	struct qmem	*npa_qints_ctx;
	unsigned long	*aura_bmap;
	unsigned long	*pool_bmap;

	/* NIX contexts */
	struct qmem	*rq_ctx;
	struct qmem	*sq_ctx;
	struct qmem	*cq_ctx;
	struct qmem	*rss_ctx;
	struct qmem	*cq_ints_ctx;
	struct qmem	*nix_qints_ctx;
	unsigned long	*sq_bmap;
	unsigned long	*rq_bmap;
	unsigned long	*cq_bmap;

	u16		rx_chan_base;
	u16		tx_chan_base;
	u8              rx_chan_cnt; /* total number of RX channels */
	u8              tx_chan_cnt; /* total number of TX channels */
	u16		maxlen;
	u16		minlen;

	bool		hw_rx_tstamp_en; /* Is rx_tstamp enabled */
	u8		pf_set_vf_cfg;
	u8		mac_addr[ETH_ALEN]; /* MAC address of this PF/VF */
	u8		default_mac[ETH_ALEN]; /* MAC address from FWdata */

	/* Broadcast pkt replication info */
	u16			bcast_mce_idx;
	struct nix_mce_list	bcast_mce_list;

	/* For resource limits */
	struct pci_dev	*pdev;
	struct kobject	*limits_kobj;

	struct rvu_npc_mcam_rule *def_rule;

	bool	cgx_in_use; /* this PF/VF using CGX? */
	int	cgx_users;  /* number of cgx users - used only by PFs */
};

struct nix_txsch {
	struct rsrc_bmap schq;
	u8   lvl;
#define NIX_TXSCHQ_FREE		      BIT_ULL(1)
#define NIX_TXSCHQ_CFG_DONE	      BIT_ULL(0)
#define TXSCH_MAP_FUNC(__pfvf_map)    ((__pfvf_map) & 0xFFFF)
#define TXSCH_MAP_FLAGS(__pfvf_map)   ((__pfvf_map) >> 16)
#define TXSCH_MAP(__func, __flags)    (((__func) & 0xFFFF) | ((__flags) << 16))
#define TXSCH_SET_FLAG(__pfvf_map, flag)    ((__pfvf_map) | ((flag) << 16))
	u32  *pfvf_map;
};

struct nix_mark_format {
	u8 total;
	u8 in_use;
	u32 *cfg;
};

struct npc_pkind {
	struct rsrc_bmap rsrc;
	u32	*pfchan_map;
};

struct nix_flowkey {
#define NIX_FLOW_KEY_ALG_MAX 32
	u32 flowkey[NIX_FLOW_KEY_ALG_MAX];
	int in_use;
};

struct nix_lso {
	u8 total;
	u8 in_use;
};

struct nix_txvlan {
#define NIX_TX_VTAG_DEF_MAX 0x400
	struct rsrc_bmap rsrc;
	u16 *entry2pfvf_map;
	struct mutex rsrc_lock; /* Serialize resource alloc/free */
};

struct nix_hw {
	struct nix_txsch txsch[NIX_TXSCH_LVL_CNT]; /* Tx schedulers */
	struct nix_mcast mcast;
	struct nix_flowkey flowkey;
	struct nix_mark_format mark_format;
	struct nix_lso lso;
	struct nix_txvlan txvlan;
	void   *tx_stall;
};

/* RVU block's capabilities or functionality,
 * which vary by silicon version/skew.
 */
struct hw_cap {
	/* Transmit side supported functionality */
	u8	nix_tx_aggr_lvl; /* Tx link's traffic aggregation level */
	u16	nix_txsch_per_cgx_lmac; /* Max Q's transmitting to CGX LMAC */
	u16	nix_txsch_per_lbk_lmac; /* Max Q's transmitting to LBK LMAC */
	u16	nix_txsch_per_sdp_lmac; /* Max Q's transmitting to SDP LMAC */
	bool	nix_fixed_txschq_mapping; /* Schq mapping fixed or flexible */
	bool	nix_shaping;		 /* Is shaping and coloring supported */
	bool	nix_tx_link_bp;		 /* Can link backpressure TL queues ? */
	bool	nix_rx_multicast;	 /* Rx packet replication support */
};

struct rvu_hwinfo {
	u8	total_pfs;   /* MAX RVU PFs HW supports */
	u16	total_vfs;   /* Max RVU VFs HW supports */
	u16	max_vfs_per_pf; /* Max VFs that can be attached to a PF */
	u8	cgx;
	u8	lmac_per_cgx;
	u8	cgx_links;
	u8	lbk_links;
	u8	sdp_links;
	u8	npc_kpus;          /* No of parser units */

	struct hw_cap    cap;
	struct rvu_block block[BLK_COUNT]; /* Block info */
	struct nix_hw    *nix0;
	struct npc_pkind pkind;
	struct npc_mcam  mcam;
	struct sso_rsrc  sso;
};

struct mbox_wq_info {
	struct otx2_mbox mbox;
	struct rvu_work *mbox_wrk;

	struct otx2_mbox mbox_up;
	struct rvu_work *mbox_wrk_up;

	struct workqueue_struct *mbox_wq;
};

struct rvu_fwdata {
#define RVU_FWDATA_HEADER_MAGIC	0xCFDA	/*Custom Firmware Data*/
#define RVU_FWDATA_VERSION	0x0001
	u32 header_magic;
	u32 version;		/* version id */

	/* MAC address */
#define PF_MACNUM_MAX	32
#define VF_MACNUM_MAX	256
	u64 pf_macs[PF_MACNUM_MAX];
	u64 vf_macs[VF_MACNUM_MAX];
	u64 sclk;
	u64 rclk;
	u64 mcam_addr;
	u64 mcam_sz;
	u64 msixtr_base;
#define FWDATA_RESERVED_MEM 1023
	u64 reserved[FWDATA_RESERVED_MEM];
	/* Do not add new fields below this line */
#define CGX_MAX         4
#define CGX_LMACS_MAX   4
	struct cgx_lmac_fwdata_s cgx_fw_data[CGX_MAX][CGX_LMACS_MAX];
};

struct ptp;

struct rvu {
	void __iomem		*afreg_base;
	void __iomem		*pfreg_base;
	struct pci_dev		*pdev;
	struct device		*dev;
	struct rvu_hwinfo       *hw;
	struct rvu_pfvf		*pf;
	struct rvu_pfvf		*hwvf;
	struct rvu_limits	pf_limits;
	struct mutex		rsrc_lock; /* Serialize resource alloc/free */
	int			vfs; /* Number of VFs attached to RVU */

	/* Mbox */
	struct mbox_wq_info	afpf_wq_info;
	struct mbox_wq_info	afvf_wq_info;

	/* PF FLR */
	struct rvu_work		*flr_wrk;
	struct workqueue_struct *flr_wq;
	struct mutex		flr_lock; /* Serialize FLRs */

	/* MSI-X */
	u16			num_vec;
	char			*irq_name;
	bool			*irq_allocated;
	dma_addr_t		msix_base_iova;
	u64			msixtr_base_phy;/* Register reset value */

	/* CGX */
#define PF_CGXMAP_BASE		1 /* PF 0 is reserved for RVU PF */
	u8			cgx_mapped_pfs;
	u8			cgx_cnt_max;	 /* CGX port count max */
	u8			*pf2cgxlmac_map; /* pf to cgx_lmac map */
	u16			*cgxlmac2pf_map; /* bitmap of mapped pfs for
						  * every cgx lmac port
						  */
	unsigned long		pf_notify_bmap; /* Flags for PF notification */
	void			**cgx_idmap; /* cgx id to cgx data map table */
	struct			work_struct cgx_evh_work;
	struct			workqueue_struct *cgx_evh_wq;
	spinlock_t		cgx_evq_lock; /* cgx event queue lock */
	struct list_head	cgx_evq_head; /* cgx event queue head */
	struct mutex		cgx_cfg_lock; /* serialize cgx configuration */

	char mkex_pfl_name[MKEX_NAME_LEN]; /* Configured MKEX profile name */
	char kpu_pfl_name[KPU_NAME_LEN]; /* Configured KPU profile name */

	/* Firmware data */
	struct rvu_fwdata	*fwdata;
	void			*kpu_fwdata;
	size_t			kpu_fwdata_sz;

	/* NPC KPU data */
	struct npc_kpu_profile_adapter kpu;

	struct ptp		*ptp;

/* CONFIG_DEBUG_FS */
#ifdef CONFIG_DEBUG_FS
	struct rvu_debugfs	rvu_dbg;
#endif /* CONFIG_DEBUG_FS */
};

static inline void rvu_write64(struct rvu *rvu, u64 block, u64 offset, u64 val)
{
	writeq(val, rvu->afreg_base + ((block << 28) | offset));
}

static inline u64 rvu_read64(struct rvu *rvu, u64 block, u64 offset)
{
	return readq(rvu->afreg_base + ((block << 28) | offset));
}

static inline void rvupf_write64(struct rvu *rvu, u64 offset, u64 val)
{
	writeq(val, rvu->pfreg_base + offset);
}

static inline u64 rvupf_read64(struct rvu *rvu, u64 offset)
{
	return readq(rvu->pfreg_base + offset);
}

/* Silicon revisions */

static inline bool is_rvu_post_96xx_C0(struct rvu *rvu)
{
	struct pci_dev *pdev = rvu->pdev;

	return (pdev->revision == 0x08) || (pdev->revision == 0x30);
}

static inline bool is_rvu_96xx_A0(struct rvu *rvu)
{
	struct pci_dev *pdev = rvu->pdev;

	return (pdev->revision == 0x00);
}

static inline bool is_rvu_96xx_B0(struct rvu *rvu)
{
	struct pci_dev *pdev = rvu->pdev;

	return (pdev->revision == 0x00) || (pdev->revision == 0x01);
}

static inline bool is_rvu_95xx_B0(struct rvu *rvu)
{
	struct pci_dev *pdev = rvu->pdev;

	return (pdev->revision == 0x14);
}

static inline bool is_rvu_95xx_A0(struct rvu *rvu)
{
	struct pci_dev *pdev = rvu->pdev;

	return (pdev->revision == 0x10) || (pdev->revision == 0x11);
}

static inline bool is_cgx_mapped_to_nix(unsigned short id, u8 cgx_id)
{
	return !(cgx_id && !(id == PCI_SUBSYS_DEVID_96XX ||
			     id == PCI_SUBSYS_DEVID_98XX));
}

/* Function Prototypes
 * RVU
 */
static inline int is_afvf(u16 pcifunc)
{
	return !(pcifunc & ~RVU_PFVF_FUNC_MASK);
}

static inline bool is_rvu_fwdata_valid(struct rvu *rvu)
{
	return (rvu->fwdata->header_magic == RVU_FWDATA_HEADER_MAGIC) &&
		(rvu->fwdata->version == RVU_FWDATA_VERSION);
}

int rvu_alloc_bitmap(struct rsrc_bmap *rsrc);
int rvu_alloc_rsrc(struct rsrc_bmap *rsrc);
void rvu_free_rsrc(struct rsrc_bmap *rsrc, int id);
int rvu_rsrc_free_count(struct rsrc_bmap *rsrc);
int rvu_alloc_rsrc_contig(struct rsrc_bmap *rsrc, int nrsrc);
bool rvu_rsrc_check_contig(struct rsrc_bmap *rsrc, int nrsrc);
u16 rvu_get_rsrc_mapcount(struct rvu_pfvf *pfvf, int blktype);
int rvu_get_pf(u16 pcifunc);
struct rvu_pfvf *rvu_get_pfvf(struct rvu *rvu, int pcifunc);
void rvu_get_pf_numvfs(struct rvu *rvu, int pf, int *numvfs, int *hwvf);
bool is_block_implemented(struct rvu_hwinfo *hw, int blkaddr);
bool is_pffunc_map_valid(struct rvu *rvu, u16 pcifunc, int blktype);
int rvu_get_lf(struct rvu *rvu, struct rvu_block *block, u16 pcifunc, u16 slot);
int rvu_lf_reset(struct rvu *rvu, struct rvu_block *block, int lf);
int rvu_get_blkaddr(struct rvu *rvu, int blktype, u16 pcifunc);
int rvu_poll_reg(struct rvu *rvu, u64 block, u64 offset, u64 mask, bool zero);
u16 rvu_get_rsrc_mapcount(struct rvu_pfvf *pfvf, int blkid);
int rvu_get_num_lbk_chans(void);
int rvu_ndc_sync(struct rvu *rvu, int lfblkid, int lfidx, u64 lfoffset);

/* RVU HW reg validation */
enum regmap_block {
	TXSCHQ_HWREGMAP = 0,
	MAX_HWREGMAP,
};

bool rvu_check_valid_reg(int regmap, int regblk, u64 reg);

/* NPA/NIX AQ APIs */
int rvu_aq_alloc(struct rvu *rvu, struct admin_queue **ad_queue,
		 int qsize, int inst_size, int res_size);
void rvu_aq_free(struct rvu *rvu, struct admin_queue *aq);

/* CGX APIs */
static inline bool is_pf_cgxmapped(struct rvu *rvu, u8 pf)
{
	return (pf >= PF_CGXMAP_BASE && pf <= rvu->cgx_mapped_pfs);
}

static inline void rvu_get_cgx_lmac_id(u8 map, u8 *cgx_id, u8 *lmac_id)
{
	*cgx_id = (map >> 4) & 0xF;
	*lmac_id = (map & 0xF);
}

#define M(_name, _id, fn_name, req, rsp)				\
int rvu_mbox_handler_ ## fn_name(struct rvu *, struct req *, struct rsp *);
MBOX_MESSAGES
#undef M

int rvu_cgx_init(struct rvu *rvu);
int rvu_cgx_exit(struct rvu *rvu);
void *rvu_cgx_pdata(u8 cgx_id, struct rvu *rvu);
int rvu_cgx_config_rxtx(struct rvu *rvu, u16 pcifunc, bool start);
void rvu_cgx_enadis_rx_bp(struct rvu *rvu, int pf, bool enable);
void rvu_cgx_enadis_higig2(struct rvu *rvu, int pf, bool enable);
bool rvu_cgx_is_higig2_enabled(struct rvu *rvu, int pf);
void rvu_cgx_disable_dmac_entries(struct rvu *rvu, u16 pcifunc);
int rvu_cgx_start_stop_io(struct rvu *rvu, u16 pcifunc, bool start);
int rvu_cgx_nix_cuml_stats(struct rvu *rvu, void *cgxd, int lmac_id, int index,
			   int rxtxflag, u64 *stat);
bool is_cgx_config_permitted(struct rvu *rvu, u16 pcifunc);

/* SSO APIs */
int rvu_sso_init(struct rvu *rvu);
void rvu_sso_freemem(struct rvu *rvu);
int rvu_sso_register_interrupts(struct rvu *rvu);
void rvu_sso_unregister_interrupts(struct rvu *rvu);
int rvu_sso_lf_teardown(struct rvu *rvu, u16 pcifunc, int lf, int slot_id);
int rvu_ssow_lf_teardown(struct rvu *rvu, u16 pcifunc, int lf, int slot_id);
void rvu_sso_hwgrp_config_thresh(struct rvu *rvu, int blkaddr, int lf);

/* NPA APIs */
int rvu_npa_init(struct rvu *rvu);
void rvu_npa_freemem(struct rvu *rvu);
void rvu_npa_lf_teardown(struct rvu *rvu, u16 pcifunc, int npalf);
int rvu_npa_aq_enq_inst(struct rvu *rvu, struct npa_aq_enq_req *req,
			struct npa_aq_enq_rsp *rsp);
int rvu_npa_register_interrupts(struct rvu *rvu);
void rvu_npa_unregister_interrupts(struct rvu *rvu);

/* NIX APIs */
bool is_nixlf_attached(struct rvu *rvu, u16 pcifunc);
int rvu_nix_init(struct rvu *rvu);
int rvu_nix_reserve_mark_format(struct rvu *rvu, struct nix_hw *nix_hw,
				int blkaddr, u32 cfg);
void rvu_nix_freemem(struct rvu *rvu);
int rvu_get_nixlf_count(struct rvu *rvu);
void rvu_nix_lf_teardown(struct rvu *rvu, u16 pcifunc, int blkaddr, int npalf);
int nix_get_nixlf(struct rvu *rvu, u16 pcifunc, int *nixlf, int *nix_blkaddr);
int rvu_nix_register_interrupts(struct rvu *rvu);
void rvu_nix_unregister_interrupts(struct rvu *rvu);
void rvu_nix_reset_mac(struct rvu_pfvf *pfvf, int pcifunc);
bool rvu_nix_is_ptp_tx_enabled(struct rvu *rvu, u16 pcifunc);

/* NPC APIs */
int rvu_npc_init(struct rvu *rvu);
void rvu_npc_freemem(struct rvu *rvu);
int rvu_npc_get_pkind(struct rvu *rvu, u16 pf);
void rvu_npc_set_pkind(struct rvu *rvu, int pkind, struct rvu_pfvf *pfvf);
int npc_config_ts_kpuaction(struct rvu *rvu, int pf, u16 pcifunc, bool en);
void rvu_npc_install_ucast_entry(struct rvu *rvu, u16 pcifunc,
				 int nixlf, u64 chan, u8 *mac_addr);
void rvu_npc_install_promisc_entry(struct rvu *rvu, u16 pcifunc,
				   int nixlf, u64 chan, bool allmulti);
void rvu_npc_disable_promisc_entry(struct rvu *rvu, u16 pcifunc, int nixlf);
void rvu_npc_enable_promisc_entry(struct rvu *rvu, u16 pcifunc, int nixlf);
void rvu_npc_install_bcast_match_entry(struct rvu *rvu, u16 pcifunc,
				       int nixlf, u64 chan);
void rvu_npc_disable_bcast_entry(struct rvu *rvu, u16 pcifunc);
void rvu_npc_disable_mcam_entries(struct rvu *rvu, u16 pcifunc, int nixlf);
void rvu_npc_free_mcam_entries(struct rvu *rvu, u16 pcifunc, int nixlf);
void rvu_npc_disable_default_entries(struct rvu *rvu, u16 pcifunc, int nixlf);
void rvu_npc_enable_default_entries(struct rvu *rvu, u16 pcifunc, int nixlf);
void rvu_npc_update_flowkey_alg_idx(struct rvu *rvu, u16 pcifunc, int nixlf,
				    int group, int alg_idx, int mcam_index);
void rvu_npc_get_mcam_entry_alloc_info(struct rvu *rvu, u16 pcifunc,
				       int blkaddr, int *alloc_cnt,
				       int *enable_cnt);
void rvu_npc_get_mcam_counter_alloc_info(struct rvu *rvu, u16 pcifunc,
					 int blkaddr, int *alloc_cnt,
					 int *enable_cnt);
int npc_flow_steering_init(struct rvu *rvu, int blkaddr);
const char *npc_get_field_name(u8 hdr);
bool rvu_npc_write_default_rule(struct rvu *rvu, int blkaddr, int nixlf,
				u16 pcifunc, u8 intf, struct mcam_entry *entry,
				int *entry_index);
int npc_mcam_verify_channel(struct rvu *rvu, u16 pcifunc, u8 intf, u16 channel);
int npc_get_bank(struct npc_mcam *mcam, int index);
void npc_mcam_enable_flows(struct rvu *rvu, u16 target);
void npc_enable_mcam_entry(struct rvu *rvu, struct npc_mcam *mcam,
			   int blkaddr, int index, bool enable);
void npc_read_mcam_entry(struct rvu *rvu, struct npc_mcam *mcam,
			 int blkaddr, u16 src,
			 struct mcam_entry *entry, u8 *intf, u8 *ena);

/* CPT APIs */
int rvu_cpt_init(struct rvu *rvu);
int rvu_cpt_register_interrupts(struct rvu *rvu);
void rvu_cpt_unregister_interrupts(struct rvu *rvu);

/* TIM APIs */
int rvu_tim_init(struct rvu *rvu);
int rvu_tim_lf_teardown(struct rvu *rvu, u16 pcifunc, int lf, int slot);

/* SDP APIs */
int rvu_sdp_init(struct rvu *rvu);
bool is_sdp_pf(u16 pcifunc);

/* CONFIG_DEBUG_FS*/
#ifdef CONFIG_DEBUG_FS
void rvu_dbg_init(struct rvu *rvu);
void rvu_dbg_exit(struct rvu *rvu);
#else
static inline void rvu_dbg_init(struct rvu *rvu) {}
static inline void rvu_dbg_exit(struct rvu *rvu) {}
#endif /* CONFIG_DEBUG_FS*/

/* HW workarounds/fixes */
#include "npc.h"
void rvu_nix_txsch_lock(struct nix_hw *nix_hw);
void rvu_nix_txsch_unlock(struct nix_hw *nix_hw);
void rvu_nix_update_link_credits(struct rvu *rvu, int blkaddr,
				 int link, u64 ncredits);

void rvu_nix_update_sq_smq_mapping(struct rvu *rvu, int blkaddr, int nixlf,
				   u16 sq, u16 smq);
void rvu_nix_txsch_config_changed(struct nix_hw *nix_hw);
ssize_t rvu_nix_get_tx_stall_counters(struct rvu *rvu,
				      char __user *buffer, loff_t *ppos);
int rvu_nix_fixes_init(struct rvu *rvu, struct nix_hw *nix_hw, int blkaddr);
void rvu_nix_fixes_exit(struct rvu *rvu, struct nix_hw *nix_hw);
int rvu_tim_lookup_rsrc(struct rvu *rvu, struct rvu_block *block,
			u16 pcifunc, int slot);
int rvu_npc_get_tx_nibble_cfg(struct rvu *rvu, u64 nibble_ena);
bool is_parse_nibble_config_valid(struct rvu *rvu,
				  struct npc_mcam_kex *mcam_kex);
int rvu_npc_set_parse_mode(struct rvu *rvu, u16 pcifunc, u64 mode, u8 dir,
			   u64 pkind);
#endif /* RVU_H */
