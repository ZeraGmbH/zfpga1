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

enum en_node_types {
	NODE_TYPE_BOOT = 0,
	NODE_TYPE_REG,
	NODE_TYPE_DSP,

	NODE_TYPE_COUNT
};

/* Flags singleton:
   * Assumption: We have one fpga hardware in our system.
   * data type is arch specific for atomic bit access */
static unsigned long global_flags = 0;

#define FLAG_GLOBAL_FPGA_BOOT_DEVICE_FOUND 0  /* used to ensure assumption above */
#define FLAG_GLOBAL_FPGA_CONFIGURED 1         /* booted ? */

struct zfpga_node_data {
	const char *nodename;
	u8 nodetype;
	void __iomem *base;
	resource_size_t size;
	struct cdev cdev;
	struct device *device; /* keep it just in case we want to add entries to sysfs later */
	struct platform_device *pdev;
	struct fasync_struct *async_queue; /* for used interrupt */

	/* per device flags - data type is arch specific for atomic bit access */
	unsigned long flags;
	/* dsp type identification */
	u32 dsp_magic_id;
};

/* per device flags */
#define FLAG_OPEN       (1<<0)	/* avoid file opened  more than once */
#define FLAG_RUNNING    (1<<1)

struct zfpga_dev_data {
	struct zfpga_node_data nodes[MAX_NODE_COUNT];
	unsigned int count_nodes;
	dev_t first_char_node;
};

/* ---------------------- internal constants  ---------------------- */
/* dsp 'magic' identifiation ids */
#define adsp_21262_1_magic 0xAA55BB44
#define adsp_21362_1_magic 0xAA55CC33

/* addresses for dsp access */
#define SPI 0
#define SERIAL 4
#define DSPCTRL 8
#define DSPSTAT 12
#define DSPCFG 16
#define VERSIONNR 20
#define MAGICID 24

/* dsp internal data memory space */
#define DSPDataMemBase21262 0x82800
#define DSPDataMemTop21262 0x87FFF

#define DSPDataMemBase21362_1 0xE0800
#define DSPDataMemTop21362_1 0xE3FFF

#define DSPDataMemBase21362_2 0x98180
#define DSPDataMemTop21362_2 0x9FFFF

/* commands for initialization of dsp internal serial interface
   and dma, which work as host interface port emulation */
#define DSPREAD 0x80000001
#define DSPWRITE 0x00000001

/* ---------------------- device specific helpers ---------------------- */
int fpga_reset(struct zfpga_node_data *node_data)
{
#ifdef DEBUG
	dev_info(&node_data->pdev->dev, "%s entered\n", __func__);
#endif
	/*gpio_set_value(zFPGA_platform_data.gpio_reset, 1);
	udelay(10);
	gpio_set_value(zFPGA_platform_data.gpio_reset, 0);*/
	/* resetting causes unconfigured state */
	clear_bit(FLAG_GLOBAL_FPGA_CONFIGURED, &global_flags);
	return 0;
}

/* ---------------------- common fops helper ---------------------- */
static void* check_and_alloc_kmem(const struct zfpga_node_data *node_data, size_t count, loff_t *offset)
{
	size_t len32;
	int ret;

	switch(node_data->nodetype) {
		case NODE_TYPE_BOOT: /* Not booting twice incidentally */
			if (test_bit(FLAG_GLOBAL_FPGA_CONFIGURED, &global_flags)) {
				dev_info(
					&node_data->pdev->dev,
					"%s failed (fpga already configured) for %s!\n",
					__func__, node_data->nodename);
				return ERR_PTR(-ENODEV);
			}
			break;
		case NODE_TYPE_REG:
			/* Note: fpga is memory mapped 1:1. Therefore limits are takene from
			 * devicetree settings */
			if ( (*offset < 0) || /* is that possible ?? */
					((*offset + count) > node_data->size) ||
					(count & 3)) {
				dev_info(
					&node_data->pdev->dev,
					"%s failed (fpga address out of limits) for %s!\n",
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
			if ( node_data->dsp_magic_id == adsp_21262_1_magic) {
				if ( (*offset < DSPDataMemBase21262) ||
						((*offset + len32) > DSPDataMemTop21262) ||
						(count & 3))
					ret = -EFAULT; /* bad adress */
			}
			else
			{
				if ( (((*offset < DSPDataMemBase21362_1) || ((*offset + len32) > DSPDataMemTop21362_1)) &&
					   ((*offset < DSPDataMemBase21362_2) || ((*offset + len32) > DSPDataMemTop21362_2))) || (count & 3))
					ret = -EFAULT; /* bad adress */;
			}
			if(ret) {
				dev_info(
					&node_data->pdev->dev,
					"%s failed (dsp address out of limits) for %s!\n",
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
	dev_info(&node_data->pdev->dev, "%s for %s entered\n", __func__, node_data->nodename);
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
			dev_info(
				&node_data->pdev->dev,
				"opening %s requires configured FPGA!\n",
				node_data->nodename);
			return -ENODEV;
		}
	}
	/* dsp devices need identification to determine memory limits */
	if(node_data->nodetype == NODE_TYPE_DSP) {
		node_data->dsp_magic_id = ioread32(node_data->base + MAGICID);
		if (node_data->dsp_magic_id != adsp_21262_1_magic &&
			node_data->dsp_magic_id != adsp_21362_1_magic) {
			dev_info(
				&node_data->pdev->dev,
				"unknown dsp magic id 0x%08X read for %s!\n",
				node_data->dsp_magic_id, node_data->nodename);
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
	dev_info(&node_data->pdev->dev, "%s for %s entered\n", __func__, node_data->nodename);
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
	dev_info(
		&node_data->pdev->dev,
		"%s offset: 0x%llx, length: 0x%zx for %s\n",
		__func__, *offset, count, node_data->nodename);
#endif
	kbuff = check_and_alloc_kmem(node_data, count, offset);
	if (IS_ERR(kbuff)) {
		dev_info(
			&node_data->pdev->dev,
			"%s failed to alloc kmem for %s (%li)!\n",
			__func__, node_data->nodename, PTR_ERR(kbuff));
		return PTR_ERR(kbuff);
	}
	switch(node_data->nodetype) {
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
			source32 = node_data->base + SERIAL;
			dest32 = kbuff;
			transaction_count = count>>2;
			/* serial interface and dma initialization */
			iowrite32(DSPREAD, node_data->base + SPI);
			iowrite32(*offset, node_data->base + SPI);
			iowrite32(transaction_count, node_data->base + SPI);
			/*udelay(100); give the dsp 100 uS for initialzing serial and dma */
			for (transaction_no=0; transaction_no<transaction_count; transaction_no++) {
				*dest32 = ioread32(source32);
				dest32++;
			}
			break;
	}
	if (copy_to_user(buf, kbuff, count)) {
		dev_info(
			&node_data->pdev->dev,
			"%s copy_to_user failed for %s\n",
			__func__,
			node_data->nodename);
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
	dev_info(
		&node_data->pdev->dev,
		"%s offset: 0x%llx, length: 0x%zx for %s\n",
		__func__, *offset, count, node_data->nodename);
#endif
	kbuff = check_and_alloc_kmem(node_data, count, offset);
	if (IS_ERR(kbuff)) {
		dev_info(
			&node_data->pdev->dev,
			"%s failed to check/alloc kmem for %s (%li)!\n",
			__func__, node_data->nodename,
			PTR_ERR(kbuff));
		return PTR_ERR(kbuff);
	}
	if ( copy_from_user(kbuff, buf, count)) {
		dev_info(
			&node_data->pdev->dev,
			"%s copy_from_user failed for %s\n",
			__func__, node_data->nodename);
		kfree(kbuff);
		return -EFAULT;
	}
	switch(node_data->nodetype) {
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
			dest32 = node_data->base + SERIAL;
			transaction_count = count>>2;
			/* serial interface and dma initialization */
			iowrite32(DSPWRITE, node_data->base + SPI);
			iowrite32(*offset, node_data->base + SPI);
			iowrite32(transaction_count, node_data->base + SPI);
			/*udelay(100); give the dsp 100 uS for initialzing serial and dma */
			for (transaction_no=0; transaction_no<transaction_count; transaction_no++) {
				iowrite32(*source32++, dest32);
				source32++;
			}
			break;
	}
	kfree(kbuff);
#ifdef DEBUG
	dev_info(
		&node_data->pdev->dev,
		"%s 0x%lx bytes written for %s\n",
		__func__, (unsigned long)(count), node_data->nodename);
#endif
	return count;
}

static loff_t fo_lseek(struct file *file, loff_t offset, int origin)
{
#ifdef DEBUG
	struct zfpga_node_data *node_data = file->private_data;
	dev_info(
		&node_data->pdev->dev,
		"%s adress 0x%llx for %s\n",
		__func__, offset, node_data->nodename);
#endif
	file->f_pos = offset; /* only absolut positioning */
	return offset;
}

static int fo_fasync (int fd, struct file *file, int mode)
{
	struct zfpga_node_data *node_data = file->private_data;
#ifdef DEBUG
	dev_info(
		&node_data->pdev->dev,
		"%s entered for %s\n",
		__func__, node_data->nodename);
#endif
	return (fasync_helper(fd, file, mode, &node_data->async_queue));
}

/* ---------------------- boot device fops ---------------------- */
static long fo_ioctl_boot (struct file *file, unsigned int cmd, unsigned long arg)
{
#ifdef DEBUG
	struct zfpga_node_data *node_data = file->private_data;
	dev_info(
		&node_data->pdev->dev,
		"%s entered for %s, cmd: 0x%x, arg: %lx\n",
		__func__, node_data->nodename, cmd, arg);
#endif
	switch ( cmd ) {
		case FPGA_RESET: return(fpga_reset(node_data));
		default:
			dev_info(
				&node_data->pdev->dev,
				"%s cmd: 0x%x invalid for %s\n",
				__func__, cmd, node_data->nodename);
			return(-EINVAL);
	}
}

/* ---------------------- dsp device specific fops ----------------------- */
long fo_ioctl_dsp(struct file *file, unsigned int cmd, unsigned long arg)
{
#ifdef DEBUG
	struct zfpga_node_data *node_data = file->private_data;
	dev_info(
		&node_data->pdev->dev,
		"%s entered for %s cmd: 0x%x, arg: %lx\n",
		__func__, node_data->nodename, cmd, arg);
#endif
	switch ( cmd ) {
		/*case DSP_RESET: return(reset_dsp(file));
		case DSP_BOOT: return(boot_dsp(file, arg));
		case DSP_INT_REQ: return(int_dsp(file));
		case DSP_INT_ENABLE: return(int_dsp_enable(file));
		case DSP_INT_DISABLE: return(int_dsp_disable(file));
		case IO_READ: return(read_io(file,arg));*/
		default:
			dev_info(
				&node_data->pdev->dev,
				"%s cmd: 0x%x invalid for %s\n",
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
			dev_info(&pdev->dev, "can't alloc chardev region for %s\n", pdev->dev.of_node->full_name);
			goto exit;
		}
#ifdef DEBUG
        dev_info(&pdev->dev, "chardev region created for %s\n", pdev->dev.of_node->full_name);
#endif
		for (node_added_count=0; node_added_count<zfpga->count_nodes; node_added_count++) {
			cdev_init(&zfpga->nodes[node_added_count].cdev, &fops_arr[zfpga->nodes[node_added_count].nodetype]);
			zfpga->nodes[node_added_count].cdev.owner = THIS_MODULE;
			if ((ret = cdev_add(
							&zfpga->nodes[node_added_count].cdev,
							zfpga->first_char_node+node_added_count, 1))) {
				dev_info(&pdev->dev, "cdev_add failed for %s\n", zfpga->nodes[node_added_count].nodename);
				goto exit_cleanup_nodes;
			}
#ifdef DEBUG
			dev_info(&pdev->dev, "cdev_add succeeded for %s\n", zfpga->nodes[node_added_count].nodename);
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
				dev_info(&pdev->dev, "device_create failed for %s\n", zfpga->nodes[node_added_count].nodename);
				goto exit_cleanup_devices;
			}
#ifdef DEBUG
			dev_info(&pdev->dev, "device_create succeded for %s\n", zfpga->nodes[node_added_count].nodename);
#endif
		}
	}
	return 0;

exit_cleanup_devices:
	for (inode=0; inode<node_added_count; inode++) {
#ifdef DEBUG
		dev_info(&pdev->dev, "exit_cleanup_devices for %s\n", zfpga->nodes[inode].nodename);
#endif
		device_destroy(zfpga_class, zfpga->first_char_node+inode);
		zfpga->nodes[inode].device = NULL;
	}
	/* all cdevs were created here */
	node_added_count = zfpga->count_nodes;

exit_cleanup_nodes:
	for (inode=0; inode<node_added_count; inode++) {
#ifdef DEBUG
		dev_info(&pdev->dev, "exit_cleanup_nodes for %s\n", zfpga->nodes[inode].nodename);
#endif
		cdev_del(&zfpga->nodes[inode].cdev);
	}
exit:
	return ret;
}

/* ---------------------- apply devicetree settings ----------------------
 * - setup zfpga_dev_data *zfpga
 * - ioremap
 * - interrupts TODO
 */
static int check_dt_settings(struct platform_device *pdev, struct zfpga_dev_data *zfpga)
{
	struct resource res;
	u32 nodetype;
	int ret = 0;
	struct device_node *child_node = NULL;

	while ((child_node = of_get_next_child(pdev->dev.of_node, child_node)) != NULL) {
		/* avoid buffer overflow */
		if (zfpga->count_nodes > MAX_NODE_COUNT) {
			dev_info(&pdev->dev, "maximum node count %u reached in %s!\n",
				MAX_NODE_COUNT,
				pdev->dev.of_node->full_name);
			goto exit;
		}
		zfpga->nodes[zfpga->count_nodes].pdev = pdev;
		/* node's name */
		if((ret = of_property_read_string(
				child_node,
				"nodename",
				&zfpga->nodes[zfpga->count_nodes].nodename))) {
			dev_info(&pdev->dev, "missing/incorrect entry 'nodename' for %s!\n",
				child_node->full_name);
			goto exit;
		}
#ifdef DEBUG
		dev_info(&pdev->dev, "entry 'nodename = %s' found in %s\n",
				zfpga->nodes[zfpga->count_nodes].nodename,
				child_node->full_name);
#endif
		/* node's type */
		if((ret = of_property_read_u32(child_node, "nodetype", &nodetype))) {
			dev_info(&pdev->dev, "missing/incorrect entry 'nodetype' in %s!\n",
				child_node->full_name);
			goto exit;
		}
#ifdef DEBUG
		dev_info(&pdev->dev, "entry 'nodetype = %u' found in %s\n",
				nodetype,
				child_node->full_name);
#endif
		if(nodetype >= NODE_TYPE_COUNT) {
			dev_info(&pdev->dev, "entry 'nodetype' out of limits in %s!\n",
				child_node->full_name);
			goto exit;
		}
		if(nodetype == NODE_TYPE_BOOT)
		{
			/* There must be only one boot device in the system */
			if (test_bit(FLAG_GLOBAL_FPGA_BOOT_DEVICE_FOUND, &global_flags)) {
				dev_info(&pdev->dev, "%s tries to set up a second boot device!\n",
					child_node->full_name);
				ret = -EINVAL;
				goto exit;
			}
			else {
				ret = fpga_reset(&zfpga->nodes[zfpga->count_nodes]);
				if (ret) {
					pr_info( "zfpga: unable to reset FPGA!\n");
					goto exit;
				}
				set_bit(FLAG_GLOBAL_FPGA_BOOT_DEVICE_FOUND, &global_flags);
			}
		}
		zfpga->nodes[zfpga->count_nodes].nodetype = (u8)nodetype;
		/* setup node's memory region */
		ret = of_address_to_resource(pdev->dev.of_node, zfpga->count_nodes, &res);
		if (ret) {
			dev_info(&pdev->dev, "can't get memory limits - 'reg' properly set in %s?\n",
				child_node->full_name);
			goto exit;
		}
		zfpga->nodes[zfpga->count_nodes].base = devm_ioremap_resource(&pdev->dev, &res);
		if (IS_ERR(zfpga->nodes[zfpga->count_nodes].base)) {
			dev_info(&pdev->dev, "can't remap in %s\n",
				child_node->full_name);
			ret = PTR_ERR(zfpga->nodes[zfpga->count_nodes].base);
			goto exit;
		}
		zfpga->nodes[zfpga->count_nodes].size = resource_size(&res);
#ifdef DEBUG
		dev_info(&pdev->dev, "memory region remapped for %s to %p size 0x%08X\n",
				zfpga->nodes[zfpga->count_nodes].nodename,
				zfpga->nodes[zfpga->count_nodes].base,
				zfpga->nodes[zfpga->count_nodes].size);
#endif
		/* next node */
		zfpga->count_nodes++;
	}
	return 0;

exit:
	return ret;
}

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
	dev_info(&pdev->dev, "%s called\n", __func__);
#endif
	zfpga = platform_get_drvdata(pdev);
	if (!IS_ERR(zfpga) && zfpga->count_nodes) {
#ifdef DEBUG
		dev_info(&pdev->dev, "%s removing char devices\n", __func__);
#endif
		for (inode=0; inode<zfpga->count_nodes; inode++) {
#ifdef DEBUG
			dev_info(&pdev->dev, "device_destroy for %s\n", zfpga->nodes[inode].nodename);
#endif
			device_destroy(zfpga_class, zfpga->first_char_node+inode);
			zfpga->nodes[inode].device = NULL;
#ifdef DEBUG
			dev_info(&pdev->dev, "cdev_del for %s\n", zfpga->nodes[inode].nodename);
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
