/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __ASM_IRQ_H
#define __ASM_IRQ_H

#ifndef __ASSEMBLER__

#include <asm-generic/irq.h>

/* Platforms with multiple SR-IOV capable PCI devices will
 * need large number of MSIX vectors, hence keep this number
 * fairly high.
 */
#ifdef CONFIG_PCI_MSI
#undef  NR_IRQS
#define NR_IRQS        65536
#endif

struct pt_regs;

static inline int nr_legacy_irqs(void)
{
	return 0;
}

#endif /* !__ASSEMBLER__ */
#endif
