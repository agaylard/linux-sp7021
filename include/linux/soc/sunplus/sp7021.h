/* SPDX-License-Identifier: (GPL-2.0-only OR BSD-2-Clause) */
/*
 * Copyright (C) Sunplus Technology Co., Ltd.
 */
#ifndef __LINUX_SOC_SUNPLUS_SP7021_H
#define __LINUX_SOC_SUNPLUS_SP7021_H

#include <linux/bits.h>

/*
 * SP7021 Moon-format register write.
 *
 * SP7021 control registers use a mask-write format: bits [31:16] are
 * write-enable masks and bits [15:0] are the values to write. Only bits
 * whose corresponding write-enable bit is set are actually modified.
 *
 * MOON_REG_WRITE(mask, val)  - write val to the bits selected by mask
 * MOON_REG_SET(bit)          - set a single bit by position (integer)
 * MOON_REG_CLR(bit)          - clear a single bit by position (integer)
 * MOON_REG_FIELD_SET(bits)   - set one or more bits by bitmask
 * MOON_REG_FIELD_CLR(bits)   - clear one or more bits by bitmask
 */
#define MOON_REG_MASK_SHIFT		16
#define MOON_REG_WRITE(mask, val)	(((mask) << MOON_REG_MASK_SHIFT) | ((val) & (mask)))
#define MOON_REG_SET(bit)		MOON_REG_WRITE(BIT(bit), BIT(bit))
#define MOON_REG_CLR(bit)		MOON_REG_WRITE(BIT(bit), 0)
#define MOON_REG_FIELD_SET(bits)	MOON_REG_WRITE(bits, bits)
#define MOON_REG_FIELD_CLR(bits)	MOON_REG_WRITE(bits, 0)

#endif /* __LINUX_SOC_SUNPLUS_SP7021_H */
