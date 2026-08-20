// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Sunplus SP7021 IOP (8051 I/O Processor) driver.
 *
 * The IOP is an 8051 microcontroller that manages GPIO in low-power states,
 * RTC wake-up, and cooperates with the PMC during system power management.
 * Firmware is loaded via the Linux firmware loader (request_firmware).
 */

#include <linux/delay.h>
#include <linux/soc/sunplus/sp7021.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/memremap.h>
#include <asm/cacheflush.h>
#include <asm/outercache.h>
#include <linux/firmware.h>
#include <linux/printk.h>
#include <linux/reboot.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/uaccess.h>

#include "sunplus-iop-regs.h"

#define NORMAL_CODE_MAX_SIZE	0x10000	/* 64 KB */
#define STANDBY_CODE_MAX_SIZE	0x4000	/* 16 KB */

#define IOP_FW_NORMAL	"sunplus/8051-normal.bin"
#define IOP_FW_STANDBY	"sunplus/8051-standby.bin"

/* Global state */
static bool iop_code_mode;	/* 0=normal, 1=standby */
static bool iop_wake_in;
static unsigned long SP_IOP_RESERVE_BASE;
static unsigned long SP_IOP_RESERVE_SIZE;
static unsigned long B_REG_MOON0;
static unsigned long B_REG_MOON1;
static unsigned int RECEIVE_CODE_SIZE;
static unsigned char NormalCode[NORMAL_CODE_MAX_SIZE];
static unsigned char StandbyCode[STANDBY_CODE_MAX_SIZE];

/* G0.1  mo_clken0  — Clock Enable Register #0 (0x9C000004) */
#define MOON0_CLKEN0		((void __iomem *)(B_REG_MOON0 + 4))
#define  CLKEN0_IOP_BIT		4

/* G0.21 mo_reset0  — Hardware Reset Control Register #0 (0x9C000054) */
#define MOON0_RESET0		((void __iomem *)(B_REG_MOON0 + 4 * 21))
#define  RESET0_IOP_BIT		4

/* G1.1/G1.2 — Group 1 software-configure registers used by IOP */
#define MOON1_CLKEN0		((void __iomem *)(B_REG_MOON1 + 4))
#define MOON1_CLKEN1		((void __iomem *)(B_REG_MOON1 + 8))

#define IOP_READY	0x4
#define RISC_READY	0x8

struct sp_iop {
	struct miscdevice mdev;
	struct mutex lock;
	struct device *dev;
	void __iomem *iop_regs;
	void __iomem *moon0_regs;
	void __iomem *moon1_regs;
	void __iomem *qctl_regs;
	void __iomem *pmc_regs;
	void *standby_fw;
	size_t standby_fw_size;
};

static struct sp_iop *g_iop;

/* -------------------------------------------------------------------------
 * HAL: firmware loading and IOP control
 * -------------------------------------------------------------------------
 */
static void iop_load_and_start(void __iomem *iopbase,
				const unsigned char *code, size_t size)
{
	struct regs_iop_t *r = (struct regs_iop_t *)iopbase;
	unsigned long phys = SP_IOP_RESERVE_BASE;
	void __iomem *sram;
	u32 reg;

	sram = memremap(phys, size, MEMREMAP_WB);
	if (!sram) {
		pr_err("sp_iop: failed to map firmware SRAM\n");
		return;
	}
	memset(sram, 0, size);
	memcpy(sram, code, size);
	/*
	 * Flush the 8051 firmware from the ARM caches to main RAM.
	 * The 8051 is an AXI bus master that reads physical RAM directly;
	 * it bypasses the ARM's L1/L2 caches entirely.
	 */
	__cpuc_flush_dcache_area(sram, size);          /* L1 D-cache → PoU */
	outer_flush_range(phys, phys + size);           /* L2 (outer) → PoC */
	memunmap(sram);

	writel(MOON_REG_SET(CLKEN0_IOP_BIT), MOON0_CLKEN0);

	/* Assert IOP reset (bit 0 = 1), clear boot-ROM select (bit 15 = 0) */
	reg = readl(&r->iop_control);
	reg |= 0x01;
	reg &= ~0x8000;
	writel(reg, &r->iop_control);

	/* Disable watchdog-event reset of IOP (bit 9 = 1) */
	reg = readl(&r->iop_control);
	reg |= 0x0200;
	writel(reg, &r->iop_control);

	/* Set firmware base address in IOP registers */
	writel(phys & 0xffff, &r->iop_base_adr_l);
	writel(phys >> 16,    &r->iop_base_adr_h);

	/* Release IOP reset → 8051 starts executing */
	reg = readl(&r->iop_control);
	reg &= ~0x01;
	writel(reg, &r->iop_control);
}


static void iop_load_normal_code(void __iomem *iopbase)
{
	iop_load_and_start(iopbase, NormalCode, RECEIVE_CODE_SIZE);
	iop_code_mode = 0;
}

static void iop_load_standby_code(void __iomem *iopbase)
{
	iop_load_and_start(iopbase, StandbyCode, RECEIVE_CODE_SIZE);
	iop_code_mode = 1;
}

static void iop_normalmode(void __iomem *iopbase)
{
	if (g_iop && g_iop->standby_fw) /* normal fw loaded the same way */
		iop_load_and_start(iopbase, NormalCode, RECEIVE_CODE_SIZE);
	iop_code_mode = 0;
}

static void iop_standbymode(void __iomem *iopbase)
{
	if (g_iop && g_iop->standby_fw)
		iop_load_and_start(iopbase, g_iop->standby_fw,
				   g_iop->standby_fw_size);
	iop_code_mode = 1;
}

static void iop_get_data(void __iomem *iopbase)
{
	struct regs_iop_t *r = (struct regs_iop_t *)iopbase;

	pr_debug("sp_iop: data0=%x data1=%x data2=%x data3=%x data4=%x data5=%x\n",
		 readl(&r->iop_data0), readl(&r->iop_data1),
		 readl(&r->iop_data2), readl(&r->iop_data3),
		 readl(&r->iop_data4), readl(&r->iop_data5));
	pr_debug("sp_iop: data6=%x data7=%x data8=%x data9=%x data10=%x data11=%x\n",
		 readl(&r->iop_data6), readl(&r->iop_data7),
		 readl(&r->iop_data8), readl(&r->iop_data9),
		 readl(&r->iop_data10), readl(&r->iop_data11));
}

static void iop_set_data(void __iomem *iopbase, unsigned int num, unsigned int value)
{
	struct regs_iop_t *r = (struct regs_iop_t *)iopbase;

	switch (num) {
	case '0': case 0:  writel(value, &r->iop_data0);  break;
	case '1': case 1:  writel(value, &r->iop_data1);  break;
	case '2': case 2:  writel(value, &r->iop_data2);  break;
	case '3': case 3:  writel(value, &r->iop_data3);  break;
	case '4': case 4:  writel(value, &r->iop_data4);  break;
	case '5': case 5:  writel(value, &r->iop_data5);  break;
	case '6': case 6:  writel(value, &r->iop_data6);  break;
	case '7': case 7:  writel(value, &r->iop_data7);  break;
	case '8': case 8:  writel(value, &r->iop_data8);  break;
	case '9': case 9:  writel(value, &r->iop_data9);  break;
	case 'A': case 10: writel(value, &r->iop_data10); break;
	case 'B': case 11: writel(value, &r->iop_data11); break;
	}
}

static void iop_pmc_setup(void __iomem *iopbase, void __iomem *pmcbase)
{
	struct regs_iop_t *r = (struct regs_iop_t *)iopbase;
	struct regs_iop_pmc_t *pmc = (struct regs_iop_pmc_t *)pmcbase;
	u32 reg;

	writel(MOON_REG_SET(CLKEN0_IOP_BIT), MOON0_CLKEN0);

	/* Assert reset (bit 0=1), clear boot-ROM (bit 15=0) */
	reg = readl(&r->iop_control);
	reg |= 0x01;
	reg &= ~0x8000;
	writel(reg, &r->iop_control);

	writel(0x00010001, &pmc->PMC_TIMER);
	reg = readl(&pmc->PMC_CTRL);
	reg |= 0x23;
	writel(reg, &pmc->PMC_CTRL);
	writel(0x55aa00ff, &pmc->XTAL27M_PASSWORD_I);
	writel(0x00ff55aa, &pmc->XTAL27M_PASSWORD_II);
	writel(0xaa00ff55, &pmc->XTAL32K_PASSWORD_I);
	writel(0xff55aa00, &pmc->XTAL32K_PASSWORD_II);
	writel(0xaaff0055, &pmc->CLK27M_PASSWORD_I);
	writel(0x5500aaff, &pmc->CLK27M_PASSWORD_II);
	writel(0x01000100, &pmc->PMC_TIMER2);

	/* Send reset pulse to IOP */
	writel(MOON_REG_SET(RESET0_IOP_BIT), MOON0_RESET0);
	writel(MOON_REG_CLR(RESET0_IOP_BIT), MOON0_RESET0);

	/*
	 * Enable IOP clocks in MOON1. The original write 0x00ff0085 used a
	 * broad mask that also CLEARED bits 1,3,4,5,6 — disabling secondary
	 * CPU clocks and causing RCU stalls. Use individual bit masks to only
	 * SET bits 7, 2, 0 without touching other bits.
	 */
	writel(MOON_REG_WRITE(BIT(7)|BIT(2)|BIT(0), BIT(7)|BIT(2)|BIT(0)), MOON1_CLKEN0);
	writel(MOON_REG_SET(11), MOON1_CLKEN1);

	/* Disable watchdog-event reset (bit 9 = 1) */
	reg = readl(&r->iop_control);
	reg |= 0x0200;
	writel(reg, &r->iop_control);

	writel(SP_IOP_RESERVE_BASE & 0xffff, &r->iop_base_adr_l);
	writel(SP_IOP_RESERVE_BASE >> 16,    &r->iop_base_adr_h);

	/* Release IOP reset → 8051 starts with standby firmware */
	reg = readl(&r->iop_control);
	reg &= ~0x01;
	writel(reg, &r->iop_control);
}

static void iop_handshake_and_powerdown(void __iomem *iopbase)
{
	struct regs_iop_t *r = (struct regs_iop_t *)iopbase;
	unsigned int timeout;

	pr_info("sp_iop: waiting for IOP_READY\n");
	timeout = 500000;
	while ((readl(&r->iop_data2) & IOP_READY) != IOP_READY) {
		if (!--timeout) {
			pr_warn("sp_iop: timeout waiting for IOP_READY\n");
			return;
		}
		cpu_relax();
	}
	pr_info("sp_iop: iop answered IOP_READY, sending RISC_READY\n");
	writel(RISC_READY, &r->iop_data2);
	writel(0x00, &r->iop_data5);
	writel(0x60, &r->iop_data6);

	timeout = 500000;
	while (readl(&r->iop_data7) != 0xaaaa) {
		if (!--timeout) {
			pr_warn("sp_iop: timeout waiting for IOP ack\n");
			return;
		}
		cpu_relax();
	}

	pr_info("sp_iop: handshake complete — sending powerdown\n");
	writel(0xdd, &r->iop_data1); /* command 8051 to enter ultra-low-power */
}

static void iop_shutdown(void __iomem *iopbase, void __iomem *pmcbase)
{
	iop_pmc_setup(iopbase, pmcbase);
	mdelay(50);
	iop_handshake_and_powerdown(iopbase);

	/* Handshake timed out — the 8051 didn't cut power. Give it 500ms
	 * (in case it's still executing S3_Mode), then force a reboot. */
	mdelay(500);
	pr_warn("sp_iop: poweroff handshake failed, forcing reboot\n");
	machine_restart(NULL);
}

static void iop_S1mode(void __iomem *iopbase)
{
	struct regs_iop_t *r = (struct regs_iop_t *)iopbase;

	while ((readl(&r->iop_data2) & IOP_READY) != IOP_READY)
		cpu_relax();

	writel(RISC_READY, &r->iop_data2);
	writel(0x00, &r->iop_data5);
	writel(0x60, &r->iop_data6);

	while (readl(&r->iop_data7) != 0xaaaa)
		cpu_relax();

	writel(0xee, &r->iop_data1); /* command 8051 to enter S1 mode */
}

static void sp_iop_poweroff(void);

static int sp_iop_load_firmware(struct sp_iop *iop)
{
	const struct firmware *fw, *standby_fw;
	int ret;

	ret = request_firmware(&fw, IOP_FW_NORMAL, iop->dev);
	if (ret) {
		dev_err(iop->dev, "8051 normal firmware (%s) not found\n",
			IOP_FW_NORMAL);
		return ret;
	}

	iop_load_and_start(iop->iop_regs, fw->data, fw->size);
	iop_code_mode = 0;
	release_firmware(fw);

	kfree(iop->standby_fw);
	iop->standby_fw = NULL;
	ret = request_firmware(&standby_fw, IOP_FW_STANDBY, iop->dev);
	if (ret) {
		dev_warn(iop->dev, "standby firmware not found, poweroff unavailable\n");
	} else {
		iop->standby_fw = kmemdup(standby_fw->data, standby_fw->size,
					  GFP_KERNEL);
		iop->standby_fw_size = iop->standby_fw ? standby_fw->size : 0;
		release_firmware(standby_fw);
	}

	pm_power_off = sp_iop_poweroff;

	dev_info(iop->dev, "IOP (8051) started, SRAM at 0x%lx (%lu KB)%s\n",
		 SP_IOP_RESERVE_BASE, SP_IOP_RESERVE_SIZE / 1024,
		 iop->standby_fw ? ", standby cached" : ", NO standby fw");
	return 0;
}

/* -------------------------------------------------------------------------
 * sysfs: normalcode / standbycode (binary, firmware upload)
 * -------------------------------------------------------------------------
 */
static ssize_t normalcode_read(struct file *filp, struct kobject *kobj,
				const struct bin_attribute *attr,
				char *buf, loff_t offset, size_t count)
{
	void *sram = memremap(SP_IOP_RESERVE_BASE, NORMAL_CODE_MAX_SIZE, MEMREMAP_WB);

	if (!sram)
		return -ENOMEM;
	memcpy(buf, sram + offset, count);
	memunmap(sram);
	return count;
}

static ssize_t normalcode_write(struct file *filp, struct kobject *kobj,
				 const struct bin_attribute *attr,
				 char *buf, loff_t offset, size_t count)
{
	if (offset != RECEIVE_CODE_SIZE) {
		RECEIVE_CODE_SIZE = 0;
		return -EINVAL;
	}
	memcpy(NormalCode + RECEIVE_CODE_SIZE, buf, count);
	RECEIVE_CODE_SIZE += count;

	if (RECEIVE_CODE_SIZE == NORMAL_CODE_MAX_SIZE) {
		iop_load_normal_code(g_iop->iop_regs);
		RECEIVE_CODE_SIZE = 0;
	}
	return count;
}

static ssize_t standbycode_read(struct file *filp, struct kobject *kobj,
				 const struct bin_attribute *attr,
				 char *buf, loff_t offset, size_t count)
{
	void *sram = memremap(SP_IOP_RESERVE_BASE, STANDBY_CODE_MAX_SIZE, MEMREMAP_WB);

	if (!sram)
		return -ENOMEM;
	memcpy(buf, sram + offset, count);
	memunmap(sram);
	return count;
}

static ssize_t standbycode_write(struct file *filp, struct kobject *kobj,
				  const struct bin_attribute *attr,
				  char *buf, loff_t offset, size_t count)
{
	if (offset != RECEIVE_CODE_SIZE) {
		RECEIVE_CODE_SIZE = 0;
		return -EINVAL;
	}
	memcpy(StandbyCode + RECEIVE_CODE_SIZE, buf, count);
	RECEIVE_CODE_SIZE += count;

	if (RECEIVE_CODE_SIZE == STANDBY_CODE_MAX_SIZE) {
		iop_load_standby_code(g_iop->iop_regs);
		RECEIVE_CODE_SIZE = 0;
	}
	return count;
}

static BIN_ATTR_RW(normalcode, NORMAL_CODE_MAX_SIZE);
static BIN_ATTR_RW(standbycode, STANDBY_CODE_MAX_SIZE);

/* -------------------------------------------------------------------------
 * sysfs: mode, wakein, getdata, setdata, S1mode
 * -------------------------------------------------------------------------
 */
static ssize_t mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "%d\n", iop_code_mode);
}

static ssize_t mode_store(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	if (buf[0] == '0')
		iop_normalmode(g_iop->iop_regs);
	else if (buf[0] == '1')
		iop_standbymode(g_iop->iop_regs);
	else
		return -EINVAL;
	return count;
}

static ssize_t wakein_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "%d\n", iop_wake_in);
}

static ssize_t wakein_store(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t count)
{
	u32 reg;

	if (buf[0] == '0') {
		reg = readl(MOON1_CLKEN1);
		reg = 0x08000000;
		writel(reg, MOON1_CLKEN1);
		iop_wake_in = 0;
	} else if (buf[0] == '1') {
		reg = readl(MOON1_CLKEN1);
		reg |= 0x08000800;
		writel(reg, MOON1_CLKEN1);
		iop_wake_in = 1;
	} else {
		return -EINVAL;
	}
	return count;
}

static ssize_t getdata_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	iop_get_data(g_iop->iop_regs);
	return 0;
}

static ssize_t getdata_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	return count;
}

static ssize_t setdata_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return 0;
}

static ssize_t setdata_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	unsigned long val;
	unsigned int num, value;
	int ret;

	num = buf[0];
	ret = kstrtoul(buf + 2, 16, &val);
	if (ret)
		return ret;
	value = val;
	iop_set_data(g_iop->iop_regs, num, value);
	return count;
}

static ssize_t S1mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	iop_standbymode(g_iop->iop_regs);
	mdelay(10);
	iop_S1mode(g_iop->iop_regs);
	return 0;
}

static ssize_t S1mode_store(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t count)
{
	return count;
}

static ssize_t start_store(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t count)
{
	int ret = sp_iop_load_firmware(g_iop);

	return ret ? ret : count;
}

static DEVICE_ATTR_WO(start);
static DEVICE_ATTR_RW(mode);
static DEVICE_ATTR_RW(wakein);
static DEVICE_ATTR_RW(getdata);
static DEVICE_ATTR_RW(setdata);
static DEVICE_ATTR_RW(S1mode);

static const struct bin_attribute *const iop_bin_attrs[] = {
	&bin_attr_normalcode,
	&bin_attr_standbycode,
	NULL,
};

static struct attribute *iop_attrs[] = {
	&dev_attr_start.attr,
	&dev_attr_mode.attr,
	&dev_attr_wakein.attr,
	&dev_attr_getdata.attr,
	&dev_attr_setdata.attr,
	&dev_attr_S1mode.attr,
	NULL,
};

static const struct attribute_group iop_attr_group = {
	.attrs     = iop_attrs,
	.bin_attrs = iop_bin_attrs,
};

/* -------------------------------------------------------------------------
 * /dev/sp_iop miscdevice
 * -------------------------------------------------------------------------
 */
static int sp_iop_open(struct inode *inode, struct file *f)	  { return 0; }
static int sp_iop_release(struct inode *inode, struct file *f) { return 0; }

static ssize_t sp_iop_write(struct file *f, const char __user *ubuf,
			     size_t len, loff_t *off)
{
	u8 buf[3];
	unsigned int num, value;

	if (len < 3 || copy_from_user(buf, ubuf, 3))
		return -EFAULT;

	num   = buf[0];
	value = ((u32)buf[1] << 8) | buf[2];
	iop_set_data(g_iop->iop_regs, num, value);
	return len;
}

static const struct file_operations sp_iop_fops = {
	.owner   = THIS_MODULE,
	.open    = sp_iop_open,
	.write   = sp_iop_write,
	.release = sp_iop_release,
};

/* -------------------------------------------------------------------------
 * Shutdown / poweroff
 * -------------------------------------------------------------------------
 */
static void sp_iop_poweroff(void)
{
	if (!g_iop)
		return;
	pr_info("sp_iop: invoking poweroff\n");
	iop_standbymode(g_iop->iop_regs);
	pr_info("sp_iop: standby firmware loaded\n");
	iop_shutdown(g_iop->iop_regs, g_iop->pmc_regs);
}

/* -------------------------------------------------------------------------
 * Platform driver
 * -------------------------------------------------------------------------
 */
static int sp_iop_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *memnp;
	struct resource mem_res;
	struct resource *r;
	struct sp_iop *iop;
	int ret;

	iop = devm_kzalloc(dev, sizeof(*iop), GFP_KERNEL);
	if (!iop)
		return -ENOMEM;

	mutex_init(&iop->lock);

	iop->iop_regs = devm_platform_ioremap_resource_byname(pdev, "iop");
	if (IS_ERR(iop->iop_regs))
		return dev_err_probe(dev, PTR_ERR(iop->iop_regs), "failed to map iop regs\n");

	/*
	 * MOON0 and MOON1 are shared with clkc and pctl — use devm_ioremap
	 * without request_mem_region to avoid a spurious EBUSY conflict.
	 */
	r = platform_get_resource_byname(pdev, IORESOURCE_MEM, "iop_moon0");
	if (!r)
		return dev_err_probe(dev, -ENODEV, "no iop_moon0 resource\n");
	iop->moon0_regs = devm_ioremap(dev, r->start, resource_size(r));
	if (!iop->moon0_regs)
		return dev_err_probe(dev, -ENOMEM, "failed to map moon0 regs\n");

	r = platform_get_resource_byname(pdev, IORESOURCE_MEM, "iop_moon1");
	if (!r)
		return dev_err_probe(dev, -ENODEV, "no iop_moon1 resource\n");
	iop->moon1_regs = devm_ioremap(dev, r->start, resource_size(r));
	if (!iop->moon1_regs)
		return dev_err_probe(dev, -ENOMEM, "failed to map moon1 regs\n");

	iop->qctl_regs = devm_platform_ioremap_resource_byname(pdev, "iop_qctl");
	if (IS_ERR(iop->qctl_regs))
		return dev_err_probe(dev, PTR_ERR(iop->qctl_regs), "failed to map qctl regs\n");

	iop->pmc_regs = devm_platform_ioremap_resource_byname(pdev, "iop_pmc");
	if (IS_ERR(iop->pmc_regs))
		return dev_err_probe(dev, PTR_ERR(iop->pmc_regs), "failed to map pmc regs\n");

	B_REG_MOON0 = (unsigned long)iop->moon0_regs;
	B_REG_MOON1 = (unsigned long)iop->moon1_regs;

	memnp = of_parse_phandle(dev->of_node, "memory-region", 0);
	if (!memnp)
		return dev_err_probe(dev, -EINVAL, "no memory-region node\n");

	ret = of_address_to_resource(memnp, 0, &mem_res);
	of_node_put(memnp);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get memory-region resource\n");

	SP_IOP_RESERVE_BASE = mem_res.start;
	SP_IOP_RESERVE_SIZE = resource_size(&mem_res);

	iop->dev = dev;
	platform_set_drvdata(pdev, iop);
	g_iop = iop;

	iop->mdev.name  = "sp_iop";
	iop->mdev.minor = MISC_DYNAMIC_MINOR;
	iop->mdev.fops  = &sp_iop_fops;
	ret = misc_register(&iop->mdev);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register miscdevice\n");

	ret = sysfs_create_group(&dev->kobj, &iop_attr_group);
	if (ret)
		dev_warn(dev, "failed to create sysfs group: %d\n", ret);

	dev_info(dev, "SRAM at 0x%lx (%lu KB)\n",
		 SP_IOP_RESERVE_BASE, SP_IOP_RESERVE_SIZE / 1024);
	return 0;
}

static void sp_iop_remove(struct platform_device *pdev)
{
	struct sp_iop *iop = platform_get_drvdata(pdev);

	kfree(iop->standby_fw);
	iop->standby_fw = NULL;

	sysfs_remove_group(&pdev->dev.kobj, &iop_attr_group);
	misc_deregister(&iop->mdev);
	if (pm_power_off == sp_iop_poweroff)
		pm_power_off = NULL;
	g_iop = NULL;
}

static const struct of_device_id sp_iop_of_match[] = {
	{ .compatible = "sunplus,sp7021-iop" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, sp_iop_of_match);

static struct platform_driver sp_iop_driver = {
	.probe    = sp_iop_probe,
	.remove   = sp_iop_remove,
	.driver = {
		.name           = "sp7021-iop",
		.of_match_table = sp_iop_of_match,
	},
};
module_platform_driver(sp_iop_driver);

MODULE_AUTHOR("Sunplus Technology");
MODULE_DESCRIPTION("Sunplus SP7021 IOP (8051) driver");
MODULE_LICENSE("GPL");
