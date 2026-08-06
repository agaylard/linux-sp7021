/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __MACH_SUNPLUS_PLATSMP_H
#define __MACH_SUNPLUS_PLATSMP_H

#ifdef CONFIG_HOTPLUG_CPU
void sc_smp_cpu_die(unsigned int cpu);
#endif

#endif
