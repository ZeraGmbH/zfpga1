#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/fcntl.h>
#include <linux/slab.h>
#include <linux/signal.h>
#include <linux/list.h>
#include <linux/io.h>
#include <linux/cdev.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>

#include "zfpga1.h"

/* If you want debugging uncomment: */
#define DEBUG 1

static struct class *zfpga_class;

static const struct of_device_id zfpga_of_match[] = {
  { .compatible = "zera,zfpga-1", },
  {}
};
MODULE_DEVICE_TABLE(of, zfpga_of_match);

#define MAX_DSP_COUNT 4
#define MAX_NODE_COUNT 2+MAX_DSP_COUNT /* boot+reg+n*dsp */

enum node_types {
	NODE_TYPE_BOOT = 0,
	NODE_TYPE_REG,
	NODE_TYPE_DSP,

	NODE_TYPE_COUNT
};

/* global flags singleton:
   * Assumption: We have one fpga hardware and therefore boot device in our
   * system. Data type is arch specific for atomic bit access */
static volatile unsigned long global_flags = 0;

#define FLAG_GLOBAL_FPGA_BOOT_DEVICE_FOUND 0 /* ensure assumption above */
#define FLAG_GLOBAL_FPGA_CONFIGURED 1        /* fpga booted/configured state */

/* node type specific data definitions */
struct boot_data {
	int gpio_done;
	int gpio_reset;
};

struct reg_data {
	struct fasync_struct *aqueue;
};

struct dsp_data {
	u32 dsp_magic_id;             /* dsp type identification */
	struct fasync_struct *aqueue;
};

/* device node data: further information on device nodes / devices below */
struct zfpga_node_data {
	const char *nodename;
	u8 nodetype;
	void __iomem *base;           /* io mem region address */
	resource_size_t size;         /* io mem region size */
	struct cdev cdev;             /* hook to our instance in fop callbacks */
	struct device *device;        /* in case we want to add entries to sysfs */
	struct platform_device *pdev; /* don't forget our grandparent */
	volatile unsigned long flags; /* data type arch specific for atomic bit access */
	unsigned int irq;             /* > 0 -> an interrupt handler was requested */
    union node_specific {
		struct boot_data boot;
		struct reg_data reg;
		struct dsp_data dsp;
	} node_specifc_data;
};

/* per node flags */
#define FLAG_OPEN        (1<<0)	/* avoid file opened  more than once */
#define FLAG_DSP_RUNNING (1<<1)	/* keep track of dsp running state */

/* device data
 * Note: each devicetree node compatible to "zera,zfpga-1" creates a device
 * and multiple devices can be created. Although confusing this can be
 * neccessary: e.g fsl-weim driver forces us to create one device per
 * chip-select line configured. A device can contain a free number of
 * device nodes - these create and handle userpace IO in /dev. */
struct zfpga_dev_data {
	struct zfpga_node_data nodes[MAX_NODE_COUNT];
	unsigned int count_nodes;
	dev_t first_char_node;
};

/* ---------------------- internal constants/structs  ---------------------- */
/* fpga addresses for dsp access */
#define FPGA_ADDR_DSP_SPI 0
#define FPGA_ADDR_DSP_SERIAL 4
#define FPGA_ADDR_DSP_CTRL 8
#define FPGA_ADDR_DSP_STAT 12
#define FPGA_ADDR_DSP_CFG 16
#define FPGA_ADDR_DSP_VERSION 20
#define FPGA_ADDR_DSP_MAGICID 24

/* fpga dsp ctrl bits */
#define FPGA_DSP_CTRL_BIT_RESET (1<<7)
#define FPGA_DSP_CTRL_BIT_IRQ2 (1<<1)

/* fpga dsp cfg bits */
#define FPGA_DSP_CFG_BIT_IRQ_ENABLE (1<<0)

/* fpga dsp status bits */
#define FPGA_DSP_STAT_IRQ (1<<0)
#define FPGA_DSP_STAT_TIMEOUT_IRQ  (1<<1)

/* dsp 'magic' identifiation values */
#define ADSP_21262_1_MAGIC 0xAA55BB44
#define ADSP_21362_1_MAGIC 0xAA55CC33

/* dsp internal data memory space */
#define ADSP_DATA_MEM_BASE_21262   0x82800
#define ADSP_DATA_MEM_TOP_21262    0x87FFF

#define ADSP_DATA_MEM_BASE_21262_1 0xE0800
#define ADSP_DATA_MEM_TOP_21262_1  0xE3FFF

#define ADSP_DATA_MEM_BASE_21362_2 0x98180
#define ADSP_DATA_MEM_TOP_21362_2  0x9FFFF

/* dsp commands for initialization of dsp internal serial interface
   and dma, which work as host interface port emulation */
#define DSP_CMD_READ  0x80000001
#define DSP_CMD_WRITE 0x00000001

struct dsp_bootheader {
	u32 tag;
	u32 count;
	u32 address;
};

/* dsp boot block tags */
enum dsp_boot_block_tagg {
	DSP_BOOT_BLOCK_FINALINIT = 0,
	DSP_BOOT_BLOCK_ZERO_LDATA,
	DSP_BOOT_BLOCK_ZERO_L48,
	DSP_BOOT_BLOCK_INIT_L16,
	DSP_BOOT_BLOCK_INIT_L32,
	DSP_BOOT_BLOCK_INIT_L48,
	DSP_BOOT_BLOCK_INIT_L64,
	DSP_BOOT_BLOCK_ZERO_EXT8,
	DSP_BOOT_BLOCK_ZERO_EXT16,

	DSP_BOOT_BLOCK_COUNT
};

/* ---------------------- common fops helper ---------------------- */
static void* fops_check_and_alloc_kmem(
	const struct zfpga_node_data *node_data,
	size_t count, loff_t *offset)
{
	size_t len32;
	int ret;

	switch (node_data->nodetype) {
		case NODE_TYPE_BOOT: /* Not booting twice incidentally */
			if (test_bit(FLAG_GLOBAL_FPGA_CONFIGURED, &global_flags)) {
				dev_info(&node_data->pdev->dev, "%s failed (fpga already configured) for %s!\n",
					__func__, node_data->nodename);
				return ERR_PTR(-ENODEV);
			}
			break;
		case NODE_TYPE_REG:
			/* Note: fpga is memory mapped 1:1. Therefore limits are taken from
			 * devicetree settings */
			if ( (*offset < 0) || /* is that possible ?? */
					((*offset + count) > node_data->size) ||
					(count & 3)) {
				dev_info(&node_data->pdev->dev, "%s failed (fpga address out of limits) for %s!\n",
					__func__, node_data->nodename);
				return ERR_PTR(-EFAULT); /* bad adress */
			}
			break;
		case NODE_TYPE_DSP:
			len32 = count >> 2;
			ret = 0;
			/* Note: dsp is NOT memory mapped 1:1. An offset in our device node
			 * is transferred to dsp by fpga. Therefore memory limits depend on
			 * dsp type connected */
			if ( node_data->node_specifc_data.dsp.dsp_magic_id == ADSP_21262_1_MAGIC) {
				if ( (*offset < ADSP_DATA_MEM_BASE_21262) ||
						((*offset + len32) > ADSP_DATA_MEM_TOP_21262) ||
						(count & 3)) {
					ret = -EFAULT; /* bad adress */
				}
			}
			else
			{
				if ( (((*offset < ADSP_DATA_MEM_BASE_21262_1) || ((*offset + len32) > ADSP_DATA_MEM_TOP_21262_1)) &&
					   ((*offset < ADSP_DATA_MEM_BASE_21362_2) || ((*offset + len32) > ADSP_DATA_MEM_TOP_21362_2))) || (count & 3)) {
					ret = -EFAULT; /* bad adress */;
				}
			}
			if(ret) {
				dev_info(&node_data->pdev->dev, "%s failed (dsp address out of limits) for %s!\n",
					__func__, node_data->nodename);
				return ERR_PTR(ret);
			}
			break;
	}
	return kmalloc(count, GFP_KERNEL);
}

/* ---------------------- common fops ---------------------- */
static int fo_open(struct inode *inode, struct file *file)
{
	struct zfpga_node_data *node_data =
		container_of(inode->i_cdev, struct zfpga_node_data, cdev);
#ifdef DEBUG
	dev_info(&node_data->pdev->dev, "%s for %s entered\n",
		__func__, node_data->nodename);
#endif
	/* device file must not be opened more than once */
	if (test_bit(FLAG_OPEN, &node_data->flags)) {
		dev_info(&node_data->pdev->dev, "%s already opened!\n", node_data->nodename);
		return -EBUSY;
	}
	else {
		set_bit(FLAG_OPEN, &node_data->flags);
	}
	/* reg and dsp devices require a configured FPGA */
	if(node_data->nodetype == NODE_TYPE_REG ||
		node_data->nodetype == NODE_TYPE_DSP) {
		if (!test_bit(FLAG_GLOBAL_FPGA_CONFIGURED, &global_flags)) {
			dev_info(&node_data->pdev->dev, "opening %s requires configured FPGA!\n",
				node_data->nodename);
			return -ENODEV;
		}
	}
	/* dsp devices need identification to determine memory limits */
	if(node_data->nodetype == NODE_TYPE_DSP) {
		node_data->node_specifc_data.dsp.dsp_magic_id = 
			ioread32(node_data->base + FPGA_ADDR_DSP_MAGICID);
		if (node_data->node_specifc_data.dsp.dsp_magic_id != ADSP_21262_1_MAGIC &&
			node_data->node_specifc_data.dsp.dsp_magic_id != ADSP_21362_1_MAGIC) {
			dev_info(&node_data->pdev->dev, "unknown dsp magic id 0x%08X read for %s!\n",
				node_data->node_specifc_data.dsp.dsp_magic_id, node_data->nodename);
			return -ENODEV;
		}
	}
	file->private_data = node_data;
#ifdef DEBUG
	dev_info(&node_data->pdev->dev, "device %s opened\n", node_data->nodename);
#endif
	return 0;
}

static int fo_release(struct inode *inode, struct file *file)
{
	struct zfpga_node_data *node_data = file->private_data;
#ifdef DEBUG
	dev_info(&node_data->pdev->dev, "%s for %s entered\n",
		__func__, node_data->nodename);
#endif
	clear_bit(FLAG_OPEN, &node_data->flags);
	file->private_data = NULL;
#ifdef DEBUG
	dev_info(&node_data->pdev->dev, "device %s closed\n", node_data->nodename);
#endif
	return 0;
}

static ssize_t fo_read (struct file *file, char *buf, size_t count, loff_t *offset)
{
	void* kbuff;
	u32 *source32, *dest32;
	size_t transaction_no, transaction_count;
	struct zfpga_node_data *node_data = file->private_data;

#ifdef DEBUG
	dev_info(&node_data->pdev->dev, "%s offset: 0x%llx, length: 0x%zx for %s\n",
		__func__, *offset, count, node_data->nodename);
#endif
	kbuff = fops_check_and_alloc_kmem(node_data, count, offset);
	if (IS_ERR(kbuff)) {
		dev_info(&node_data->pdev->dev, "%s failed to alloc kmem for %s (%li)!\n",
			__func__, node_data->nodename, PTR_ERR(kbuff));
		return PTR_ERR(kbuff);
	}
	switch (node_data->nodetype) {
		case NODE_TYPE_REG:
			/* reg-device reads data 32bitwise mapped 1:1 */
			source32 = node_data->base + *offset;
			dest32 = kbuff;
			transaction_count = count>>2;
			for (transaction_no=0; transaction_no<transaction_count; transaction_no++) {
				*dest32 = ioread32(source32);
				source32++;
				dest32++;
			}
			break;
		case NODE_TYPE_DSP:
			/* dsp-device reads data 32bitwise from single fixed address in fpga */
			source32 = node_data->base + FPGA_ADDR_DSP_SERIAL;
			dest32 = kbuff;
			transaction_count = count>>2;
			/* serial interface and dma initialization */
			iowrite32(DSP_CMD_READ, node_data->base + FPGA_ADDR_DSP_SPI);
			iowrite32(*offset, node_data->base + FPGA_ADDR_DSP_SPI);
			iowrite32(transaction_count, node_data->base + FPGA_ADDR_DSP_SPI);
			/*udelay(100); give the dsp 100 uS for initialzing serial and dma */
			for (transaction_no=0; transaction_no<transaction_count; transaction_no++) {
				*dest32 = ioread32(source32);
				dest32++;
			}
			break;
	}
	if (copy_to_user(buf, kbuff, count)) {
		dev_info(&node_data->pdev->dev, "%s copy_to_user failed for %s\n",
			__func__, node_data->nodename);
		kfree(kbuff);
		return -EFAULT;
	}
	kfree(kbuff);
	return count;
}

static ssize_t fo_write (struct file *file, const char *buf, size_t count, loff_t *offset)
{
	void *kbuff;
	u8 *source8, *dest8;
	u32 *source32, *dest32;
	size_t transaction_no, transaction_count;
	struct zfpga_node_data *node_data = file->private_data;

#ifdef DEBUG
	dev_info(&node_data->pdev->dev, "%s offset: 0x%llx, length: 0x%zx for %s\n",
		__func__, *offset, count, node_data->nodename);
#endif
	kbuff = fops_check_and_alloc_kmem(node_data, count, offset);
	if (IS_ERR(kbuff)) {
		dev_info(&node_data->pdev->dev,	"%s failed to check/alloc kmem for %s (%li)!\n",
			__func__, node_data->nodename,
			PTR_ERR(kbuff));
		return PTR_ERR(kbuff);
	}
	if ( copy_from_user(kbuff, buf, count)) {
		dev_info(&node_data->pdev->dev,	"%s copy_from_user failed for %s\n",
			__func__, node_data->nodename);
		kfree(kbuff);
		return -EFAULT;
	}
	switch (node_data->nodetype) {
		case NODE_TYPE_BOOT:
			/* boot-device writes data bytewise to single fixed address - later
			 * fpga versions accept 16bitwise data but ot be compatible we
			 * transfer bytewise */
			source8 = kbuff;
			dest8 = node_data->base;
			transaction_count = count;
			for(transaction_no=0; transaction_no<transaction_count; transaction_no++) {
				iowrite8(*source8, dest8);
				source8++;
			}
			break;
		case NODE_TYPE_REG:
			/* reg-device writes data 32bitwise mapped 1:1 */
			source32 = kbuff;
			dest32 = node_data->base + *offset;
			transaction_count = count>>2;
			for (transaction_no=0; transaction_no<transaction_count; transaction_no++) {
				iowrite32(*source32, dest32);
				source32++;
				dest32++;
			}
			break;
		case NODE_TYPE_DSP:
			/* dsp-device writes data 32bitwise to single fixed address in fpga */
			source32 = kbuff;
			dest32 = node_data->base + FPGA_ADDR_DSP_SERIAL;
			transaction_count = count>>2;
			/* serial interface and dma initialization */
			iowrite32(DSP_CMD_WRITE, node_data->base + FPGA_ADDR_DSP_SPI);
			iowrite32(*offset, node_data->base + FPGA_ADDR_DSP_SPI);
			iowrite32(transaction_count, node_data->base + FPGA_ADDR_DSP_SPI);
			/*udelay(100); give the dsp 100 uS for initialzing serial and dma */
			for (transaction_no=0; transaction_no<transaction_count; transaction_no++) {
				iowrite32(*source32++, dest32);
				source32++;
			}
			break;
	}
	kfree(kbuff);
#ifdef DEBUG
	dev_info(&node_data->pdev->dev, "%s 0x%lx bytes written for %s\n",
		__func__, (unsigned long)(count), node_data->nodename);
#endif
	return count;
}

static loff_t fo_lseek(struct file *file, loff_t offset, int origin)
{
#ifdef DEBUG
	struct zfpga_node_data *node_data = file->private_data;
	dev_info(&node_data->pdev->dev, "%s adress 0x%llx for %s\n",
		__func__, offset, node_data->nodename);
#endif
	file->f_pos = offset; /* only absolut positioning */
	return offset;
}

static int fo_fasync (int fd, struct file *file, int mode)
{
	struct zfpga_node_data *node_data = file->private_data;
	struct fasync_struct** async_callback = NULL;
#ifdef DEBUG
	dev_info(&node_data->pdev->dev, "%s entered for %s\n",
		__func__, node_data->nodename);
#endif
	switch (node_data->nodetype)
	{
		case NODE_TYPE_REG:
			async_callback = &node_data->node_specifc_data.reg.aqueue;
			break;
		case NODE_TYPE_DSP:
			async_callback = &node_data->node_specifc_data.dsp.aqueue;
			break;
	}
	if(async_callback) {
		return (fasync_helper(fd, file, mode, async_callback));
	}
	else {
		return -EFAULT;
	}
}

/* ---------------------- ioctl helper methods ---------------------- */
int fpga_reset(const struct zfpga_node_data *node_data)
{
#ifdef DEBUG
	dev_info(&node_data->pdev->dev, "%s entered for %s\n",
		__func__, node_data->nodename);
#endif
	if (node_data->nodetype == NODE_TYPE_BOOT) {
		gpio_set_value(node_data->node_specifc_data.boot.gpio_reset, 1);
		udelay(10);
		gpio_set_value(node_data->node_specifc_data.boot.gpio_reset, 1);
		/* resetting causes unconfigured state */
		clear_bit(FLAG_GLOBAL_FPGA_CONFIGURED, &global_flags);
		return 0;
	}
	dev_info(&node_data->pdev->dev, "%s was called for %s which is not a boot node!\n",
		__func__, node_data->nodename);
	return -EFAULT;
}

static int dsp_reset(struct zfpga_node_data *node_data)
{
	void* adr = node_data->base + FPGA_ADDR_DSP_CTRL;
#ifdef DEBUG
	dev_info(&node_data->pdev->dev, "%s entered for %s\n",
		__func__, node_data->nodename);
#endif
	iowrite32(ioread32(adr) | FPGA_DSP_CTRL_BIT_RESET, adr);
	udelay(400); /* dsp requires 4096 CLKIN (25MHz) cycles after deasserting reset */
	clear_bit(FLAG_DSP_RUNNING, &node_data->flags);
	return 0;
}

static unsigned long dsp_get_boot_block_len(struct dsp_bootheader *h)
{
	unsigned long nr = h->count;
	switch (h->tag) {
		case DSP_BOOT_BLOCK_FINALINIT:	return 0x600; /* fix length */
		case DSP_BOOT_BLOCK_ZERO_LDATA:	return 0;
		case DSP_BOOT_BLOCK_ZERO_L48:	return 0;
		case DSP_BOOT_BLOCK_INIT_L16:	return (nr << 1);
		case DSP_BOOT_BLOCK_INIT_L32:	return (nr << 2);
		case DSP_BOOT_BLOCK_INIT_L48:	return (((nr + 1) & 0xFFFFFFFE) * 6);
		case DSP_BOOT_BLOCK_INIT_L64:	return (nr << 3);
		case DSP_BOOT_BLOCK_ZERO_EXT8:	return 0;
		case DSP_BOOT_BLOCK_ZERO_EXT16:	return 0;
	}
	return nr;
}

/* ioctl's arg for dsp booting contains dsp's boot file. This is organized
 * as sequence of blocks of different types see 
 * "VisualDSP++ / Loader and Utilities Manual" for more details */
static int dsp_boot(struct zfpga_node_data *node_data, unsigned long arg)
{
	unsigned long blocklen, transaction_no;
	u32 *data;
	void *kmem;
	struct dsp_bootheader act_bootheader;
	char *user_data = (char*) arg;

	dsp_reset(node_data);
	/* first data block to load is the dsp´s bootstrap loader
	 * this block has no header */
	act_bootheader.tag = DSP_BOOT_BLOCK_INIT_L48;
	act_bootheader.count = 0x100; /* with fix length */
	act_bootheader.address = 0x40000; /* and fix adress */
#ifdef DEBUG
	dev_info(&node_data->pdev->dev, "%s entering boot loop for %s\n",
		__func__, node_data->nodename);
#endif
	for (;;) {
		if (act_bootheader.tag >= DSP_BOOT_BLOCK_COUNT) {
			dev_info(&node_data->pdev->dev, "%s invalid tag %u found for %s!\n",
				__func__, act_bootheader.tag, node_data->nodename);
			return -EFAULT;
		}
		blocklen = dsp_get_boot_block_len(&act_bootheader);
#ifdef DEBUG
		dev_info(&node_data->pdev->dev, "%s boot block: type %u length %lu start 0x%x\n",
			__func__, act_bootheader.tag, blocklen, act_bootheader.address);
#endif
		/* is there data for current block header to send ? */
		if (blocklen > 0) {
			kmem = kmalloc(blocklen,GFP_KERNEL);
			if (kmem == NULL) {
				dev_info(&node_data->pdev->dev, "%s memory allocation for boot block data failed for %s\n",
					__func__, node_data->nodename);
				return -ENOMEM;
			}
			if (copy_from_user(kmem, user_data, blocklen)) {
				dev_info(&node_data->pdev->dev, "%s copy_from_user for boot block data failed for %s\n",
					__func__, node_data->nodename);
				kfree(kmem);
				return -EFAULT;
			}
			user_data += blocklen;
			/* write all the data of the desired block to the FPGA_ADDR_DSP_SPI */
			data = kmem;
			for (transaction_no=0; transaction_no<(blocklen>>2); transaction_no++,data++)
				iowrite32(*data, node_data->base + FPGA_ADDR_DSP_SPI);
			kfree(kmem);
			/* for each boot block wait at least 10 mS */
			msleep(10);
		}
#ifdef DEBUG
		dev_info(&node_data->pdev->dev, "%s boot datablock type %u done for %s\n",
			__func__, act_bootheader.tag, node_data->nodename);
#endif
		if (act_bootheader.tag == DSP_BOOT_BLOCK_FINALINIT) {
			break; /* we have finished */
		}
		/* read next boot header */
		if (copy_from_user(&act_bootheader, user_data, sizeof(act_bootheader))) {
			dev_info(&node_data->pdev->dev, "%s copy_from_user for boot block header failed for %s\n",
				__func__, node_data->nodename);
			return -EFAULT;
		}
		user_data += sizeof(act_bootheader);
		/* send boot header */
		iowrite32(act_bootheader.tag, node_data->base + FPGA_ADDR_DSP_SPI);
		iowrite32(act_bootheader.count, node_data->base + FPGA_ADDR_DSP_SPI);
		iowrite32(act_bootheader.address, node_data->base + FPGA_ADDR_DSP_SPI);
		/* in case of ZeroX tag Headers there must be a delay of 2mS + act_bootheader.count * 100nS */
		switch (act_bootheader.tag) {
			case DSP_BOOT_BLOCK_FINALINIT: break;
			case DSP_BOOT_BLOCK_INIT_L16: break;
			case DSP_BOOT_BLOCK_INIT_L32: break;
			case DSP_BOOT_BLOCK_INIT_L48: break;
			case  DSP_BOOT_BLOCK_INIT_L64: break;
			default: msleep(20); /* should be 2ms + count*100nS, count is max 1.5*2^16 */
		}
	}
#ifdef DEBUG
	dev_info(&node_data->pdev->dev, "%s booting successful for %s\n",
		__func__, node_data->nodename);
#endif
	set_bit(FLAG_DSP_RUNNING, &node_data->flags);
	return 0;
}

static int dsp_int_generate(const struct zfpga_node_data *node_data)
{
	void* adr = node_data->base + FPGA_ADDR_DSP_CTRL;
#ifdef DEBUG
	dev_info(&node_data->pdev->dev, "%s entered for %s\n",
		__func__, node_data->nodename);
#endif
	iowrite32(ioread32(adr) | FPGA_DSP_CTRL_BIT_IRQ2, adr);
	return 0;
}

static int dsp_int_enable(const struct zfpga_node_data *node_data)
{
	void* adr = node_data->base + FPGA_ADDR_DSP_CFG;
#ifdef DEBUG
	dev_info(&node_data->pdev->dev, "%s entered for %s\n",
		__func__, node_data->nodename);
#endif
	iowrite32(ioread32(adr) | FPGA_DSP_CFG_BIT_IRQ_ENABLE, adr);
	return 0;
}

static int dsp_int_disable(const struct zfpga_node_data *node_data)
{
	void* adr = node_data->base + FPGA_ADDR_DSP_CFG;
#ifdef DEBUG
	dev_info(&node_data->pdev->dev, "%s entered for %s\n",
		__func__, node_data->nodename);
#endif
	iowrite32(ioread32(adr) & (~FPGA_DSP_CFG_BIT_IRQ_ENABLE), adr);
	return 0;
}

static int dsp_read_io(const struct zfpga_node_data *node_data, unsigned long arg)
{	// reads the register[num] with num in arg
	void* adr = node_data->base + (arg << 2);
#ifdef DEBUG
	dev_info(&node_data->pdev->dev, "%s entered for %s\n",
		__func__, node_data->nodename);
#endif
	return ioread32(adr);
}

/* ---------------------- ioctls ---------------------- */
static long fo_ioctl_boot (struct file *file, unsigned int cmd, unsigned long arg)
{
	struct zfpga_node_data *node_data = file->private_data;
#ifdef DEBUG
	dev_info(&node_data->pdev->dev, "%s entered for %s, cmd: 0x%x, arg: %lx\n",
		__func__, node_data->nodename, cmd, arg);
#endif
	switch (cmd) {
		case FPGA_RESET: return(fpga_reset(node_data));
		default:
			dev_info(&node_data->pdev->dev, "%s cmd: 0x%x invalid for %s\n",
				__func__, cmd, node_data->nodename);
			return(-EINVAL);
	}
}

long fo_ioctl_dsp(struct file *file, unsigned int cmd, unsigned long arg)
{
#ifdef DEBUG
	struct zfpga_node_data *node_data = file->private_data;
	dev_info(&node_data->pdev->dev, "%s entered for %s cmd: 0x%x, arg: %lx\n",
		__func__, node_data->nodename, cmd, arg);
#endif
	switch (cmd) {
		case DSP_RESET: return(dsp_reset(node_data));
		case DSP_BOOT: return(dsp_boot(node_data, arg));
		case DSP_INT_REQ: return(dsp_int_generate(node_data));
		case DSP_INT_ENABLE: return(dsp_int_enable(node_data));
		case DSP_INT_DISABLE: return(dsp_int_disable(node_data));
		case IO_READ: return(dsp_read_io(node_data, arg));
		default:
			dev_info(&node_data->pdev->dev, "%s cmd: 0x%x invalid for %s\n",
				__func__, cmd, node_data->nodename);
			return(-EINVAL);
	}
}

/* ---------------------- fops array ---------------------- */
static const struct file_operations fops_arr[NODE_TYPE_COUNT] = {
	[NODE_TYPE_BOOT] = {
		.owner = THIS_MODULE,
		.open = fo_open,
		.release = fo_release,
		.llseek = no_llseek,
		.write = fo_write,
		.compat_ioctl = fo_ioctl_boot
	},
	[NODE_TYPE_REG] = {
		.owner = THIS_MODULE,
		.open = fo_open,
		.release = fo_release,
		.llseek = fo_lseek,
		.read = fo_read,
		.write = fo_write,
		.fasync = fo_fasync
	},
	[NODE_TYPE_DSP] = {
		.owner = THIS_MODULE,
		.open = fo_open,
		.release = fo_release,
		.llseek = fo_lseek,
		.read = fo_read,
		.write = fo_write,
		.unlocked_ioctl = fo_ioctl_dsp,
		.fasync = fo_fasync,
	},
};

/* ---------------------- create char devices ---------------------- */
static int create_char_devices(struct platform_device *pdev, struct zfpga_dev_data *zfpga)
{
	unsigned int node_added_count;
	unsigned int inode;
	int ret = 0;

	if (zfpga->count_nodes) {
		if ((ret = alloc_chrdev_region (
					&zfpga->first_char_node,
					0,
					zfpga->count_nodes,
					FPGADEV_NAME))) {
			dev_info(&pdev->dev, "can't alloc chardev region for %s\n",
				pdev->dev.of_node->full_name);
			goto exit;
		}
#ifdef DEBUG
        dev_info(&pdev->dev, "chardev region created for %s\n",
			pdev->dev.of_node->full_name);
#endif
		for (node_added_count=0; node_added_count<zfpga->count_nodes; node_added_count++) {
			cdev_init(
				&zfpga->nodes[node_added_count].cdev,
				&fops_arr[zfpga->nodes[node_added_count].nodetype]);
			zfpga->nodes[node_added_count].cdev.owner = THIS_MODULE;
			if ((ret = cdev_add(
							&zfpga->nodes[node_added_count].cdev,
							zfpga->first_char_node+node_added_count, 1))) {
				dev_info(&pdev->dev, "cdev_add failed for %s\n",
					zfpga->nodes[node_added_count].nodename);
				goto exit_cleanup_nodes;
			}
#ifdef DEBUG
			dev_info(&pdev->dev, "cdev_add succeeded for %s\n",
				zfpga->nodes[node_added_count].nodename);
#endif
		}
		for (node_added_count=0; node_added_count<zfpga->count_nodes; node_added_count++) {
			zfpga->nodes[node_added_count].device = device_create(
				zfpga_class,
				&pdev->dev,
				zfpga->first_char_node+node_added_count,
				NULL,
				"%s%s",
				FPGADEV_NAME,
				zfpga->nodes[node_added_count].nodename);
            if(IS_ERR(zfpga->nodes[node_added_count].device)) {
				ret = PTR_ERR(zfpga->nodes[node_added_count].device);
				dev_info(&pdev->dev, "device_create failed for %s\n",
					zfpga->nodes[node_added_count].nodename);
				goto exit_cleanup_devices;
			}
#ifdef DEBUG
			dev_info(&pdev->dev, "device_create succeded for %s\n",
				zfpga->nodes[node_added_count].nodename);
#endif
		}
	}
	return 0;

exit_cleanup_devices:
	for (inode=0; inode<node_added_count; inode++) {
#ifdef DEBUG
		dev_info(&pdev->dev, "exit_cleanup_devices for %s\n",
			zfpga->nodes[inode].nodename);
#endif
		device_destroy(zfpga_class, zfpga->first_char_node+inode);
		zfpga->nodes[inode].device = NULL;
	}
	/* all cdevs were created here */
	node_added_count = zfpga->count_nodes;

exit_cleanup_nodes:
	for (inode=0; inode<node_added_count; inode++) {
#ifdef DEBUG
		dev_info(&pdev->dev, "exit_cleanup_nodes for %s\n",
			zfpga->nodes[inode].nodename);
#endif
		cdev_del(&zfpga->nodes[inode].cdev);
	}
exit:
	return ret;
}

/* ---------------------- free device's interrupt ---------------------- */
void devm_free_interrupts(
	struct platform_device *pdev, struct zfpga_dev_data *zfpga)
{
	unsigned int inode;

	for (inode=0; inode<zfpga->count_nodes; inode++) {
		if(zfpga->nodes[inode].irq) {
#ifdef DEBUG
			dev_info(&pdev->dev, "free interrupt %u for %s\n",
				zfpga->nodes[inode].irq, zfpga->nodes[inode].nodename);
#endif
			free_irq(zfpga->nodes[inode].irq, &zfpga->nodes[inode]);
			zfpga->nodes[inode].irq = 0;
		}
	}
}

/* ---------------------- gpio helper ---------------------- */
static int create_gpio(struct zfpga_node_data *znode, struct device_node *dtnode,
	unsigned long flags, const char *name, int *gpio_var)
{
	int gpio;
	int ret = 0;

	if(gpio_is_valid(gpio = of_get_named_gpio(dtnode, name, 0)) &&
		!devm_gpio_request_one(
			&znode->pdev->dev,
			gpio,
			flags,
			name)) {
		*gpio_var = gpio;
#ifdef DEBUG
		dev_info(&znode->pdev->dev, "gpio '%s' request ok for %s\n",
			name, znode->nodename);
#endif
	}
	else {
		dev_info(&znode->pdev->dev, "gpio '%s' request failed for %s!\n",
			name, znode->nodename);
		ret = -EINVAL;
	}
	return ret;
}

/* ---------------------- interrupt handler ---------------------- */
static irqreturn_t boot_isr (int irq_nr, void *dev_id)
{
	struct zfpga_node_data *znode = dev_id;
	irqreturn_t ret = IRQ_NONE;

#ifdef DEBUG
	pr_info("%s entered int %u for %s!\n",
		__func__, irq_nr, znode->nodename);
#endif
	if (znode->nodetype == NODE_TYPE_BOOT) {
		if (gpio_get_value(znode->node_specifc_data.boot.gpio_done)) {
			set_bit(FLAG_GLOBAL_FPGA_CONFIGURED, &global_flags);
		}
		else {
			clear_bit(FLAG_GLOBAL_FPGA_CONFIGURED, &global_flags);
		}
		ret = IRQ_HANDLED;
	}
	else {
		pr_info("%s: irq for non boot device %s received ?!\n", 
			__func__, znode->nodename);
	}
	return ret;
}

static irqreturn_t reg_isr (int irq_nr, void *dev_id)
{
	struct zfpga_node_data *znode = dev_id;
	irqreturn_t ret = IRQ_NONE;

#ifdef DEBUG
	pr_info("%s entered int %u for %s!\n",
		__func__, irq_nr, znode->nodename);
#endif
	if (znode->nodetype == NODE_TYPE_REG) {
		/* TODO ? */
	}
	else {
		pr_info("%s: irq for non reg device %s received ?!\n", 
			__func__, znode->nodename);
	}
	return ret;
}

static irqreturn_t dsp_isr(int irq_nr, void *dev_id)
{
	struct zfpga_node_data *znode = dev_id;
	u32 status;
	void* adr = znode->base + FPGA_ADDR_DSP_STAT;
	irqreturn_t ret = IRQ_NONE;

#ifdef DEBUG
	pr_info("%s entered int %u for %s\n", __func__, irq_nr, znode->nodename);
#endif

	if (znode->nodetype == NODE_TYPE_DSP) {
		status = ioread32(adr);
#ifdef DEBUG
		pr_info("irq status 0x%08x read for %s\n", status, znode->nodename);
#endif
		if (status & FPGA_DSP_STAT_IRQ) {
			/* acknowledge irq + fasync */
#ifdef DEBUG
			pr_info("irq reset for %s\n", znode->nodename);
#endif
			iowrite32( FPGA_DSP_STAT_IRQ, adr); /* ack irq */
			if (znode->node_specifc_data.dsp.aqueue) {
				kill_fasync(&znode->node_specifc_data.dsp.aqueue, SIGIO, POLL_IN);
			}
			ret = IRQ_HANDLED;
		}
		if (status & FPGA_DSP_STAT_TIMEOUT_IRQ) {
			/* acknowledge timeout irq - no further action */
#ifdef DEBUG
			pr_info("timeout irq reset for %s\n", znode->nodename);
#endif // DEBUG
			iowrite32(FPGA_DSP_STAT_TIMEOUT_IRQ, adr); /* quit irq */
			ret = IRQ_HANDLED;
		}
	}
	else {
		pr_info("%s: irq for non dsp device %s received ?!\n", 
			__func__, znode->nodename);
	}
	return ret;
}

/* ---------------------- apply devicetree settings ----------------------
 * - setup zfpga_dev_data *zfpga
 * - ioremap
 * - interrupts
 */
static int check_dt_settings(struct platform_device *pdev, struct zfpga_dev_data *zfpga)
{
	struct resource res;
	u32 nodetype;
	unsigned int irq;
	irq_handler_t irqhandler;
	struct zfpga_node_data *curr_node_data;
	int ret = 0;
	struct device_node *child_node_dt = NULL;

	while ((child_node_dt = of_get_next_child(pdev->dev.of_node, child_node_dt)) != NULL) {
		/* avoid buffer overflow */
		if (zfpga->count_nodes > MAX_NODE_COUNT) {
			dev_info(&pdev->dev, "maximum node count %u reached in %s!\n",
				MAX_NODE_COUNT,
				pdev->dev.of_node->full_name);
			goto exit;
		}
		curr_node_data = &zfpga->nodes[zfpga->count_nodes];

		curr_node_data->pdev = pdev;
		/* node's name */
		if((ret = of_property_read_string(
				child_node_dt,
				"nodename",
				&curr_node_data->nodename))) {
			dev_info(&pdev->dev, "missing/incorrect entry 'nodename' for %s!\n",
				child_node_dt->full_name);
			goto exit;
		}
#ifdef DEBUG
		dev_info(&pdev->dev, "entry 'nodename = %s' found in %s\n",
				curr_node_data->nodename,
				child_node_dt->full_name);
#endif
		/* node's type */
		if((ret = of_property_read_u32(child_node_dt, "nodetype", &nodetype))) {
			dev_info(&pdev->dev, "missing/incorrect entry 'nodetype' in %s!\n",
				child_node_dt->full_name);
			goto exit;
		}
#ifdef DEBUG
		dev_info(&pdev->dev, "entry 'nodetype = %u' found in %s\n",
				nodetype,
				child_node_dt->full_name);
#endif
		if(nodetype >= NODE_TYPE_COUNT) {
			dev_info(&pdev->dev, "entry 'nodetype' out of limits in %s!\n",
				child_node_dt->full_name);
			goto exit;
		}
		if(nodetype == NODE_TYPE_BOOT)
		{
			/* There must be only one boot device in the system */
			if (test_bit(FLAG_GLOBAL_FPGA_BOOT_DEVICE_FOUND, &global_flags)) {
				dev_info(&pdev->dev, "%s tries to set up a second boot device!\n",
					curr_node_data->nodename);
				ret = -EINVAL;
				goto exit;
			}
			else {
				ret = fpga_reset(&zfpga->nodes[zfpga->count_nodes]);
				if (ret) {
					dev_info(&pdev->dev, "zfpga: unable to reset FPGA!\n");
					goto exit;
				}
				set_bit(FLAG_GLOBAL_FPGA_BOOT_DEVICE_FOUND, &global_flags);
			}
		}
		curr_node_data->nodetype = (u8)nodetype;
		/* setup node's memory region */
		ret = of_address_to_resource(pdev->dev.of_node, zfpga->count_nodes, &res);
		if (ret) {
			dev_info(&pdev->dev, "can't get memory limits - 'reg' properly set in %s?\n",
				child_node_dt->full_name);
			goto exit;
		}
		curr_node_data->base = devm_ioremap_resource(&pdev->dev, &res);
		if (IS_ERR(curr_node_data->base)) {
			dev_info(&pdev->dev, "can't remap in %s\n",
				child_node_dt->full_name);
			ret = PTR_ERR(curr_node_data->base);
			goto exit;
		}
		curr_node_data->size = resource_size(&res);
#ifdef DEBUG
		dev_info(&pdev->dev, "memory region remapped for %s to %p size 0x%08X\n",
				curr_node_data->nodename,
				curr_node_data->base,
				curr_node_data->size);
#endif
		/* device specific entires */
	    irqhandler = NULL;
		switch (curr_node_data->nodetype) {
			case NODE_TYPE_BOOT:
			    irqhandler = boot_isr;
				ret = create_gpio(
					&zfpga->nodes[zfpga->count_nodes],
					child_node_dt,
					GPIOF_DIR_OUT,
					"gpio-reset",
					&curr_node_data->node_specifc_data.boot.gpio_reset);
				if(ret) {
					goto exit;
				}
				ret = create_gpio(
					&zfpga->nodes[zfpga->count_nodes],
					child_node_dt,
					GPIOF_DIR_IN,
					"gpio-done",
					&curr_node_data->node_specifc_data.boot.gpio_done);
				if(ret) {
					goto exit;
				}
				break;
			case NODE_TYPE_REG:
			    irqhandler = reg_isr;
				break;
			case NODE_TYPE_DSP:
			    irqhandler = dsp_isr;
				break;
		}
		irq = irq_of_parse_and_map(child_node_dt, 0);
		if (irq && irqhandler) {
			ret = request_irq(
				irq,
				irqhandler,
				IRQF_SHARED,
				curr_node_data->nodename,
				curr_node_data);
			if (ret < 0) {
				dev_info(&pdev->dev, "failed to request interrupt for %s\n",
					curr_node_data->nodename);
				goto exit;
			}
#ifdef DEBUG
			dev_info(&pdev->dev, "interrupt %u requested for %s\n",
				irq, curr_node_data->nodename);
#endif
			curr_node_data->irq = irq;
		}
		/* next node */
		zfpga->count_nodes++;
	}
	return 0;

exit:
	/* remove interrupt handlers (other resources are managed)  */
	devm_free_interrupts(pdev, zfpga);
	return ret;
}

/* ---------------------- kernel module interface ---------------------- */
static int zfpga_probe(struct platform_device *pdev)
{
	const struct of_device_id *match;
	struct zfpga_dev_data *zfpga;
	int ret = 0;

#ifdef DEBUG
	dev_info(&pdev->dev, "%s called\n", __func__);
#endif
	match = of_match_device(zfpga_of_match, &pdev->dev);
	if (!match) {
		ret = -EINVAL;
		goto exit;
	}
	zfpga = devm_kzalloc(&pdev->dev, sizeof(*zfpga), GFP_KERNEL);
	if (!zfpga) {
		dev_info(&pdev->dev, "can't alloc device data\n");
		ret = -ENOMEM;
		goto exit;
	}
	platform_set_drvdata(pdev, zfpga);
	if((ret = check_dt_settings(pdev, zfpga))) {
		goto exit_cleanup_drvdata;
	}
	if((ret = create_char_devices(pdev, zfpga))) {
		goto exit_cleanup_drvdata;
	}
	return 0;

exit_cleanup_drvdata:
	platform_set_drvdata(pdev, NULL);
exit:
	return ret;
}

static int zfpga_remove(struct platform_device *pdev)
{
	struct zfpga_dev_data *zfpga;
	unsigned int inode;
#ifdef DEBUG
	dev_info(&pdev->dev, "%s entered\n", __func__);
#endif
	zfpga = platform_get_drvdata(pdev);
	if (!IS_ERR(zfpga) && zfpga->count_nodes) {
		devm_free_interrupts(pdev, zfpga);
#ifdef DEBUG
		dev_info(&pdev->dev, "%s removing device nodes\n", __func__);
#endif
		for (inode=0; inode<zfpga->count_nodes; inode++) {
#ifdef DEBUG
			dev_info(&pdev->dev, "device_destroy for %s\n",
				zfpga->nodes[inode].nodename);
#endif
			device_destroy(zfpga_class, zfpga->first_char_node+inode);
			zfpga->nodes[inode].device = NULL;
#ifdef DEBUG
			dev_info(&pdev->dev, "cdev_del for %s\n",
				zfpga->nodes[inode].nodename);
#endif
			cdev_del(&zfpga->nodes[inode].cdev);
		}
#ifdef DEBUG
		dev_info(&pdev->dev, "calling unregister_chrdev_region\n");
#endif
		unregister_chrdev_region(zfpga->first_char_node, zfpga->count_nodes);
	}
	platform_set_drvdata(pdev, NULL);

	return 0;
}

static struct platform_driver zfpga_platform_driver = {
	.probe 		= zfpga_probe,
	.remove		= zfpga_remove,
	.driver		= {
		.name	= FPGADEV_NAME,
		.owner 	= THIS_MODULE,
		.of_match_table = zfpga_of_match,
	},
};

static int __init zfpga_init(void)
{
	int res;

	zfpga_class = class_create(THIS_MODULE, ZERACLASS_NAME);
	if (IS_ERR(zfpga_class)) {
		pr_err("zfpga: unable to create class!\n");
		res = PTR_ERR(zfpga_class);
		goto exit;
	}
#ifdef DEBUG
	pr_info( "zfpga: class created\n");
#endif
	res = platform_driver_register(&zfpga_platform_driver);
	if (res) {
		pr_info( "zfpga: unable to register platform driver!\n");
		goto exit;
	}
#ifdef DEBUG
	pr_info( "zfpga: platform driver registered\n");
#endif
exit:
	return res;
}

static void __exit zfpga_exit(void)
{
	platform_driver_unregister(&zfpga_platform_driver);
	class_destroy(zfpga_class);
}

MODULE_DESCRIPTION("ZERA FPGA Type 1 kernel module");
MODULE_AUTHOR("Peter Lohmer (p.lohmer@zera.de)");
MODULE_AUTHOR("Andreas Mueller (a.mueller@zera.de)");
MODULE_LICENSE("GPL");
MODULE_SUPPORTED_DEVICE("ZERA zFPGA1");

module_init(zfpga_init);
module_exit(zfpga_exit);
