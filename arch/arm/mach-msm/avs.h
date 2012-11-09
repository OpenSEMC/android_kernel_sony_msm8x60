/*
 * Copyright (c) 2009,2012, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef AVS_H
#define AVS_H

#define VOLTAGE_MIN  1000 /* mV */
#define VOLTAGE_MAX  1250
#define VOLTAGE_STEP 25

int __init avs_init(int (*set_vdd)(int), u32 freq_cnt, u32 freq_idx);
void __exit avs_exit(void);

int avs_adjust_freq(u32 freq_index, int begin);

/* Routines exported from avs_hw.S */
#ifdef CONFIG_MSM_CPU_AVS
u32 avs_test_delays(void);
#else
static inline u32 avs_test_delays(void)
{ return 0; }
#endif

#ifdef CONFIG_MSM_AVS_HW
u32 avs_get_avscsr(void);
void avs_set_avscsr(u32 avscsr);
u32 avs_get_avsdscr(void);
void avs_set_avsdscr(u32 avsdscr);
void avs_disable(int cpu);
void avs_enable(int cpu, u32 avsdscr);
#else
static inline u32 avs_get_avscsr(void)
{ return 0; }
static inline void avs_set_avscsr(u32 avscsr) {}
static inline u32 avs_get_avsdscr(void)
{ return 0; }
static inline void avs_set_avsdscr(u32 avsdscr) {}
static inline void avs_disable(int cpu) {}
static inline void avs_enable(int cpu, u32 avsdscr) {}
#endif

#define AVS_DISABLE(cpu) avs_disable(cpu)
#define AVS_ENABLE(cpu, x) avs_enable(cpu, x)

#endif