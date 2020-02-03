# SPDX-License-Identifier: GPL-2.0
#
# Makefile for Cavium's Thunder ethernet device
#

obj-$(CONFIG_THUNDER_NIC_RGX) += thunder_xcv.o
obj-$(CONFIG_THUNDER_NIC_BGX) += thunder_bgx.o
obj-$(CONFIG_THUNDER_NIC_PF) += thunder_nic.o
obj-$(CONFIG_THUNDER_NIC_VF) += thunder_nicvf.o

thunder_nic-y := nic_main.o
thunder_nicvf-y := nicvf_main.o nicvf_queues.o
thunder_nicvf-y += nicvf_ethtool.o
