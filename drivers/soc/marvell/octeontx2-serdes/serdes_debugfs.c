// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2019 Marvell International Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifdef CONFIG_DEBUG_FS

#include <linux/arm-smccc.h>
#include <linux/debugfs.h>
#include <linux/fs.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/string.h>
#include <linux/time.h>
#include <linux/uaccess.h>

MODULE_AUTHOR("Marvell International Ltd.");
MODULE_DESCRIPTION("Serdes diagnostic commands for OcteonTX2");
MODULE_LICENSE("GPL v2");

#define ARM_SMC_SVC_UID			0xc200ff01

#define OCTEONTX_SERDES_DBG_GET_MEM	0xc2000d04
#define OCTEONTX_SERDES_DBG_GET_EYE	0xc2000d05
#define OCTEONTX_SERDES_DBG_GET_CONF	0xc2000d06
#define OCTEONTX_SERDES_DBG_PRBS	0xc2000d07

/* This is expected OcteonTX response for SVC UID command */
static const int octeontx_svc_uuid[] = {
	0x6ff498cf,
	0x5a4e9cfa,
	0x2f2a3aa4,
	0x5945b105,
};

#define MAX_LMAC_PER_CGX		4

#define OCTEONTX_SMC_PENDING		0x1

#define SERDES_SETTINGS_SIZE		0x1000
enum qlm_type {
	QLM_GSERC_TYPE,
	QLM_GSERR_TYPE,
	QLM_GSERN_TYPE,
};

struct dentry *pserdes_root;

enum cgx_prbs_cmd {
	CGX_PRBS_START_CMD = 1,
	CGX_PRBS_STOP_CMD,
	CGX_PRBS_GET_DATA_CMD
};

struct cgx_prbs_errors {
	u64 err;
	u64 phy_host;
	u64 phy_line;
};

struct cgx_prbs_data {
	u64 num_lanes;
	struct cgx_prbs_errors errors[MAX_LMAC_PER_CGX];
};

struct prbs_status {
	struct list_head list;
	int qlm;
	long start_time;
	struct prbs_status *next;
};

struct eye_data {
	int width;
	int height;
	u32 data[64][128];
	enum qlm_type type;
};

static struct {
	int qlm;
	int lane;
	struct eye_data *res;
} eye_cmd_data;

static struct {
	int qlm;
	int lane;
	char *res;
} serdes_cmd_data;

static struct {
	int qlm;
	struct prbs_status status_list;
	struct cgx_prbs_data *res;
} prbs_cmd_data;

static int serdes_dbg_lane_parse(const char __user *buffer,
				 size_t count, int *qlm, int *lane)
{
	char *cmd_buf, *cmd_buf_tmp, *subtoken;
	int ec;

	cmd_buf = memdup_user(buffer, count);
	if (IS_ERR(cmd_buf))
		return -ENOMEM;

	cmd_buf[count] = '\0';

	cmd_buf_tmp = strchr(cmd_buf, '\n');
	if (cmd_buf_tmp) {
		*cmd_buf_tmp = '\0';
		count = cmd_buf_tmp - cmd_buf + 1;
	}

	cmd_buf_tmp = cmd_buf;
	subtoken = strsep(&cmd_buf, " ");
	ec = subtoken ? kstrtoint(subtoken, 10, qlm) : -EINVAL;

	if (ec < 0) {
		kfree(cmd_buf_tmp);
		return ec;
	}

	subtoken = strsep(&cmd_buf, " ");
	ec = subtoken ? kstrtoint(subtoken, 10, lane) : -EINVAL;

	kfree(cmd_buf_tmp);
	return ec;
}

static ssize_t serdes_dbg_eye_write_op(struct file *filp,
				       const char __user *buffer,
				       size_t count, loff_t *ppos)
{
	struct arm_smccc_res res;
	int ec;

	ec = serdes_dbg_lane_parse(buffer, count, &eye_cmd_data.qlm,
				   &eye_cmd_data.lane);
	if (ec < 0) {
		pr_info("Usage: echo <qlm> <lane> > eye\n");
		return ec;
	}

	do {
		arm_smccc_smc(OCTEONTX_SERDES_DBG_GET_EYE, eye_cmd_data.qlm,
			      eye_cmd_data.lane, 0, 0, 0, 0, 0, &res);
	} while (res.a0 == OCTEONTX_SMC_PENDING);
	if (res.a0 != SMCCC_RET_SUCCESS) {
		pr_info("CGX eye capture failed.\n");
		return -EIO;
	}

	return count;
}

static int serdes_dbg_eye_read_op(struct seq_file *s, void *unused)
{
	int v, t, v_height;
	int errors_tr_ones, errors_nt_ones, errors_tr_zeros, errors_nt_zeros;

	if (eye_cmd_data.res->type != QLM_GSERN_TYPE) {
		seq_puts(s, "Currently only GSERN type of QLM is supported.\n");
		return 0;
	}

	seq_printf(s, "V  T  %-20s %-20s %-20s %-20s\n", "TRANS_ONE_ECNT",
		   "NON_TRANS_ONE_ECNT", "TRANS_ZEROS_ECNT",
		   "NON_TRANS_ZEROS_ECNT");

	v_height = (eye_cmd_data.res->height + 1) / 2;

	for (t = 0; t < eye_cmd_data.res->width; t++) {
		for (v = 0; v < v_height; v++) {
			errors_nt_ones =
				eye_cmd_data.res->data[v_height-v-1][t];
			errors_tr_ones =
				eye_cmd_data.res->data[v_height-v-1][t+64];
			errors_nt_zeros =
				eye_cmd_data.res->data[v_height+v-1][t];
			errors_tr_zeros =
				eye_cmd_data.res->data[v_height+v-1][t+64];

			seq_printf(s, "%02x %02x %020x %020x %020x %020x\n",
				   v, t, errors_tr_ones, errors_nt_ones,
				   errors_tr_zeros, errors_nt_zeros);
		}
	}

	return 0;
}

static int serdes_dbg_open_eye(struct inode *inode, struct file *file)
{
	return single_open(file, serdes_dbg_eye_read_op, inode->i_private);
}

static const struct file_operations serdes_dbg_eye_fops = {
	.owner		= THIS_MODULE,
	.open		= serdes_dbg_open_eye,
	.read		= seq_read,
	.write		= serdes_dbg_eye_write_op,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static ssize_t serdes_dbg_settings_write_op(struct file *filp,
					    const char __user *buffer,
					    size_t count, loff_t *ppos)
{
	struct arm_smccc_res res;
	int ec;

	ec = serdes_dbg_lane_parse(buffer, count, &serdes_cmd_data.qlm,
				   &serdes_cmd_data.lane);
	if (ec < 0) {
		pr_info("Usage: echo <qlm> <lane> > serdes\n");
		return ec;
	}

	arm_smccc_smc(OCTEONTX_SERDES_DBG_GET_CONF, serdes_cmd_data.qlm,
		      serdes_cmd_data.lane, 0, 0, 0, 0, 0, &res);
	if (res.a0 != SMCCC_RET_SUCCESS) {
		pr_info("CGX serdes display command failed.\n");
		return -EIO;
	}

	return count;
}

static int serdes_dbg_settings_read_op(struct seq_file *s, void *unused)
{
	serdes_cmd_data.res[SERDES_SETTINGS_SIZE - 1] = '\0';

	seq_printf(s, "%s", serdes_cmd_data.res);

	return 0;
}

static int serdes_dbg_open_settings(struct inode *inode, struct file *file)
{
	return single_open(file, serdes_dbg_settings_read_op, inode->i_private);
}

static const struct file_operations serdes_dbg_settings_fops = {
	.owner		= THIS_MODULE,
	.open		= serdes_dbg_open_settings,
	.read		= seq_read,
	.write		= serdes_dbg_settings_write_op,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int serdes_dbg_prbs_lane_parse(const char __user *buffer,
				      size_t count, int *qlm,
				      enum cgx_prbs_cmd *cmd, int *mode)
{
	char *cmd_buf, *cmd_buf_tmp, *subtoken;
	int ec;

	cmd_buf = memdup_user(buffer, count);
	if (IS_ERR(cmd_buf))
		return -ENOMEM;

	cmd_buf[count] = '\0';

	cmd_buf_tmp = strchr(cmd_buf, '\n');
	if (cmd_buf_tmp) {
		*cmd_buf_tmp = '\0';
		count = cmd_buf_tmp - cmd_buf + 1;
	}

	cmd_buf_tmp = cmd_buf;
	subtoken = strsep(&cmd_buf, " ");
	ec = subtoken ? kstrtoint(subtoken, 10, qlm) : -EINVAL;

	if (ec < 0) {
		kfree(cmd_buf_tmp);
		return ec;
	}

	subtoken = strsep(&cmd_buf, " ");
	if (subtoken == NULL) {
		*cmd = CGX_PRBS_GET_DATA_CMD;
	} else {
		if (!strcmp(subtoken, "start")) {
			*cmd = CGX_PRBS_START_CMD;
			subtoken = strsep(&cmd_buf, " ");
			ec = subtoken ? kstrtoint(subtoken, 10, mode) :
					-EINVAL;
		} else if (!strcmp(subtoken, "stop")) {
			*cmd = CGX_PRBS_STOP_CMD;
		} else {
			ec = -EINVAL;
		}
	}

	kfree(cmd_buf_tmp);
	return ec;
}

static ssize_t serdes_dbg_prbs_write_op(struct file *filp,
					const char __user *buffer,
					size_t count, loff_t *ppos)
{
	struct prbs_status *status = NULL;
	struct arm_smccc_res res;
	enum cgx_prbs_cmd cmd;
	int mode;
	int qlm;
	int ec;

	ec = serdes_dbg_prbs_lane_parse(buffer, count, &prbs_cmd_data.qlm,
					&cmd, &mode);
	if (ec < 0) {
		pr_info("Usage: echo <qlm> [{start <mode>|stop}] > prbs\n");
		return ec;
	}

	qlm = prbs_cmd_data.qlm;

	switch (cmd) {
	case CGX_PRBS_START_CMD:
		arm_smccc_smc(OCTEONTX_SERDES_DBG_PRBS, cmd,
			      prbs_cmd_data.qlm, mode, 0, 0, 0, 0, &res);

		list_for_each_entry(status,
				    &prbs_cmd_data.status_list.list,
				    list) {
			if (status->qlm == qlm)
				break;
		}

		/*
		 * If status is head of the list, status for specific
		 * qlm doesn't exist
		 */
		if (&status->list == &prbs_cmd_data.status_list.list)
			status = NULL;

		if (res.a0 != SMCCC_RET_SUCCESS) {
			if (status != NULL) {
				list_del(&status->list);
				kfree(status);
			}
			pr_info("CGX prbs start command failed.\n");
			return -EIO;
		}

		if (status == NULL) {
			status = kmalloc(sizeof(struct prbs_status),
					 GFP_KERNEL);
			if (status == NULL)
				return -ENOMEM;
			status->qlm = qlm;
			list_add(&status->list,
				 &prbs_cmd_data.status_list.list);
		}
		status->start_time = get_seconds();
		pr_info("CGX PRBS-%d start on QLM %d.\n", mode, qlm);
		break;

	case CGX_PRBS_STOP_CMD:
		arm_smccc_smc(OCTEONTX_SERDES_DBG_PRBS, cmd,
			      prbs_cmd_data.qlm, 0, 0, 0, 0, 0, &res);
		if (res.a0 != SMCCC_RET_SUCCESS) {
			pr_info("CGX prbs stop command failed.\n");
			return -EIO;
		}
		list_for_each_entry(status,
				    &prbs_cmd_data.status_list.list,
				    list) {
			if (status->qlm == qlm) {
				list_del(&status->list);
				kfree(status);
				break;
			}
		}
		pr_info("CGX PRBS stop on QLM %d.\n", qlm);
		break;

	default:
		pr_info("CGX PRBS set QLM %d to read.\n", qlm);
		break;
	}

	return count;
}

static int serdes_dbg_prbs_read_op(struct seq_file *s, void *unused)
{
	struct prbs_status *status = NULL;
	struct cgx_prbs_errors *errors;
	struct arm_smccc_res res;
	long time = -1;
	int num_lanes;
	int lane;
	int qlm;

	qlm = prbs_cmd_data.qlm;

	list_for_each_entry(status,
			    &prbs_cmd_data.status_list.list,
			    list) {
		if (status->qlm == qlm) {
			time = status->start_time;
			break;
		}
	}

	if (time == -1) {
		seq_printf(s, "CGX PRBS not started for qlm %d.\n", qlm);
		return 0;
	}
	time = get_seconds() - time;

	arm_smccc_smc(OCTEONTX_SERDES_DBG_PRBS, CGX_PRBS_GET_DATA_CMD,
		      qlm, 0, 0, 0, 0, 0, &res);

	if (res.a0 != SMCCC_RET_SUCCESS) {
		seq_printf(s, "CGX prbs get command failed for qlm %d.\n", qlm);
		return 0;
	}

	errors = prbs_cmd_data.res->errors;
	num_lanes = prbs_cmd_data.res->num_lanes;
	if (num_lanes > MAX_LMAC_PER_CGX) {
		seq_printf(s, "ATF returned status for %d lanes.\n", num_lanes);
		seq_printf(s, "Kernel support only %d lanes.\n",
			   MAX_LMAC_PER_CGX);
		num_lanes = MAX_LMAC_PER_CGX;
	}

	for (lane = 0; lane < num_lanes; lane++) {
		seq_printf(s, "Time: %ld seconds QLM%d.Lane%d: errors: ",
			   time, qlm, lane);
		if (errors[lane].err != -1)
			seq_printf(s, "%lld", errors[lane].err);
		else
			seq_puts(s, "No lock");

		if (errors[lane].phy_host != -2) {
			seq_puts(s, ", PHY Host errors: ");
			if (errors[lane].phy_host != -1)
				seq_printf(s, "%lld", errors[lane].phy_host);
			else
				seq_puts(s, "No lock");
		}

		if (errors[lane].phy_line != -2) {
			seq_puts(s, ", PHY Line errors: ");
			if (errors[lane].phy_line != -1)
				seq_printf(s, "%lld", errors[lane].phy_line);
			else
				seq_puts(s, "No lock");
		}
		seq_puts(s, "\n");
	}

	return 0;
}

static int serdes_dbg_open_prbs(struct inode *inode, struct file *file)
{
	return single_open(file, serdes_dbg_prbs_read_op, inode->i_private);
}

static const struct file_operations serdes_dbg_prbs_fops = {
	.owner		= THIS_MODULE,
	.open		= serdes_dbg_open_prbs,
	.read		= seq_read,
	.write		= serdes_dbg_prbs_write_op,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int serdes_dbg_setup_debugfs(void)
{
	struct dentry *pfile;

	pserdes_root = debugfs_create_dir("octeontx2_serdes", NULL);

	pfile = debugfs_create_file("eye", 0644, pserdes_root, NULL,
				    &serdes_dbg_eye_fops);
	if (!pfile)
		goto create_failed;

	pfile = debugfs_create_file("settings", 0644, pserdes_root, NULL,
				    &serdes_dbg_settings_fops);
	if (!pfile)
		goto create_failed;

	pfile = debugfs_create_file("prbs", 0644, pserdes_root, NULL,
				    &serdes_dbg_prbs_fops);
	if (!pfile)
		goto create_failed;

	return 0;

create_failed:
	pr_err("Failed to create debugfs dir/file for serdes\n");
	debugfs_remove_recursive(pserdes_root);
	return 1;
}

static int serdes_dbg_init(void)
{
	struct arm_smccc_res res;
	int ec;

	/*
	 * Compare response for standard SVC_UID commandi with OcteonTX UUID.
	 * Continue only if it is OcteonTX.
	 */
	arm_smccc_smc(ARM_SMC_SVC_UID, 0, 0, 0, 0, 0, 0, 0, &res);
	if (res.a0 != octeontx_svc_uuid[0] || res.a1 != octeontx_svc_uuid[1] ||
	    res.a2 != octeontx_svc_uuid[2] || res.a3 != octeontx_svc_uuid[3]) {
		pr_info("UIID SVC doesn't match OcteonTX. No serdes cmds.\n");
		return 0;
	}

	arm_smccc_smc(OCTEONTX_SERDES_DBG_GET_MEM, 0, 0, 0, 0, 0, 0, 0, &res);
	if (res.a0 == SMCCC_RET_NOT_SUPPORTED) {
		pr_info("Firmware doesn't support serdes diagnostic cmds.\n");
		return 0;
	}

	if (res.a0 != SMCCC_RET_SUCCESS)
		goto serdes_mem_init_failed;

	eye_cmd_data.res = ioremap_wc(res.a1, sizeof(struct eye_data));
	if (!eye_cmd_data.res)
		goto serdes_mem_init_failed;

	serdes_cmd_data.res = ioremap_wc(res.a2, SERDES_SETTINGS_SIZE);
	if (!serdes_cmd_data.res)
		goto serdes_mem_init_failed;

	prbs_cmd_data.res = ioremap_wc(res.a3, sizeof(struct cgx_prbs_data));
	if (!prbs_cmd_data.res)
		goto serdes_mem_init_failed;

	ec = serdes_dbg_setup_debugfs();
	if (ec)
		goto serdes_debugfs_failed;

	INIT_LIST_HEAD(&prbs_cmd_data.status_list.list);

	return 0;

serdes_mem_init_failed:
	pr_err("Failed to obtain shared memory for serdes debug commands\n");

serdes_debugfs_failed:
	if (eye_cmd_data.res)
		iounmap(eye_cmd_data.res);

	if (serdes_cmd_data.res)
		iounmap(serdes_cmd_data.res);

	if (prbs_cmd_data.res)
		iounmap(prbs_cmd_data.res);

	return 0;
}

static void serdes_dbg_exit(void)
{
	struct prbs_status *status, *n;

	debugfs_remove_recursive(pserdes_root);

	if (eye_cmd_data.res)
		iounmap(eye_cmd_data.res);

	if (serdes_cmd_data.res)
		iounmap(serdes_cmd_data.res);

	if (prbs_cmd_data.res)
		iounmap(prbs_cmd_data.res);

	list_for_each_entry_safe(status, n,
				 &prbs_cmd_data.status_list.list,
				 list) {
		kfree(status);
	}
}

module_init(serdes_dbg_init);
module_exit(serdes_dbg_exit);
#endif /* CONFIG_DEBUG_FS */

