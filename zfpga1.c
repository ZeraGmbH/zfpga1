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
	NOTE_TYPE_REG,
	NODE_TYPE_DSP,

	NODE_TYPE_COUNT
};

struct zfpga_node_data {
	const char *nodename;
	u8 nodetype;
	void __iomem *base;
	/* TODO iomem size */
	struct cdev cdev;
	struct device *device; /* keep it just in case we want to add entries to sysfs later */
	struct platform_device *pdev;
	struct fasync_struct *async_queue; /* for used interrupt */

	/* type specifics - not all devices make use of the following */
	/* TODO these have to be made thread/interrupt safe !!!!!!!! */
	u16 bootcount;
	u16 flags;
	int configured;		/* !! not part of flags because set in irq !! */
};

#define FLAG_OPEN       (1<<0)	/* avoid file opened  more than once */
#define FLAG_RUNNING    (1<<1)

struct zfpga_dev_data {
	struct zfpga_node_data nodes[MAX_NODE_COUNT];
	unsigned int count_nodes;
	dev_t first_char_node;
};

/* ---------------------- common fops ---------------------- */
static int common_fop_open(struct inode *inode, struct file *file)
{
	struct zfpga_node_data *node_data =
		container_of(inode->i_cdev, struct zfpga_node_data, cdev);
#ifdef DEBUG
	dev_info(&node_data->pdev->dev, "common_fop_open for %s entered\n", node_data->nodename);
#endif
	if (node_data->flags & FLAG_OPEN) {
		dev_info(&node_data->pdev->dev, "%s already opened!\n", node_data->nodename);
		return -EBUSY;
	}
	else {
		node_data->flags |= FLAG_OPEN;
	}
	file->private_data = node_data;
#ifdef DEBUG
	dev_info(&node_data->pdev->dev, "device %s opened\n", node_data->nodename);
#endif
	return 0;
}

static int common_fop_release(struct inode *inode, struct file *file)
{
	struct zfpga_node_data *node_data = file->private_data;
#ifdef DEBUG
	dev_info(&node_data->pdev->dev, "common_fop_release for %s entered\n", node_data->nodename);
#endif
	node_data->flags &= ~FLAG_OPEN;
	file->private_data = NULL;
#ifdef DEBUG
	dev_info(&node_data->pdev->dev, "device %s closed\n", node_data->nodename);
#endif
	return 0;
}

static loff_t common_fop_lseek(struct file *file, loff_t offset, int origin)
{
#ifdef DEBUG
	struct zfpga_node_data *node_data = file->private_data;
	dev_info(
		&node_data->pdev->dev,
		"lseek to adress 0x%lx for %s\n",
		(long)offset, node_data->nodename);
#endif
	file->f_pos = offset; /* only absolut positioning */
	return offset;
}

static int common_fop_fasync (int fd, struct file *file, int mode)
{
	struct zfpga_node_data *node_data = file->private_data;
#ifdef DEBUG
	dev_info(&node_data->pdev->dev, "%s: fasync entered\n", node_data->nodename);
#endif
	return (fasync_helper(fd, file, mode, &node_data->async_queue));
}

/* ---------------------- boot device fops ---------------------- */
static ssize_t boot_fop_write (struct file *file, const char *buf, size_t count,loff_t *offset)
{
	struct zfpga_node_data *node_data = file->private_data;
#ifdef DEBUG
	dev_info(&node_data->pdev->dev, "fpga boot write entered for %s\n", node_data->nodename);
#endif
/*	char* tmp;
	unsigned long adr;
	unsigned long len;
	if (zFPGA_device_stat.configured) {
		dev_info(
			&node_data->pdev->dev,
			"fboot_fop_write failed (fpga already configured) for %s\n",
			node_data->nodename);
		return -ENODEV;
	}
	tmp = kmalloc(count,GFP_KERNEL);
	if (tmp == NULL) {
		dev_info(
			&node_data->pdev->dev,
			"boot_fop_write kernel memory allocation failed for %s\n",
			node_data->nodename);
		return -ENOMEM;
	}

	// copy user space data to kernel space
	if ( copy_from_user(tmp,buf,count)) {
		dev_info(
			&node_data->pdev->dev,
			"boot_fop_write copy_from_user failed for %s\n",
			node_data->nodename);
		kfree(tmp);
		return -EFAULT;
	}

	adr = node_data->base_adr;
	len = count;

	while (len--)
		writeb(*(buf++), adr);

	zFPGA_device_stat.fpgabootcount += count;

	kfree(tmp);
*/
#ifdef DEBUG
	dev_info(
		&node_data->pdev->dev,
		"boot_fop_write 0x%lx bytes written for %s\n",
		(unsigned long)(count), node_data->nodename);
#endif
	return count; /* fpga boot data written sucessfully */
}

static long boot_fop_ioctl (struct file *file,unsigned int cmd, unsigned long arg)
{
#ifdef DEBUG
	struct zfpga_node_data *node_data = file->private_data;
	dev_info(
		&node_data->pdev->dev,
		"boot_fop_ioctl entered for %s, cmd: 0x%x, arg: %lx\n",
		node_data->nodename, cmd, arg);
#endif
	switch ( cmd ) {
		/*case FPGA_RESET: return(reset_FPGA());*/
		default:
			dev_info(
				&node_data->pdev->dev,
				"boot_fop_ioctl, cmd: 0x%x invalid for %s\n",
				cmd, node_data->nodename);
			return(-EINVAL);
	}
}

/* ---------------------- register device specific fops ---------------------- */
static ssize_t reg_fop_read (struct file *file, char *buf, size_t count,loff_t *offset)
{
	struct zfpga_node_data *node_data = file->private_data;
#ifdef DEBUG
	dev_info(&node_data->pdev->dev,"reg_fop_read entered for %s\n", node_data->nodename);
#endif
/*	unsigned long adr;
	unsigned long *dest;
	unsigned long *start;
	unsigned long data;
	unsigned long len;
	int i;
	len = count >> 2;

	if ( (*offset < FPGARegMemBase) || ((*offset + count) > (FPGARegMemBase+FPGARegMemSize)) || (count & 3))
	{
		dev_info(
			&node_data->pdev->dev,
			"reg_fop_read address fault for %s\n",
			node_data->nodename);
		return -EFAULT; // bad adress
	}
	dest = kmalloc(count,GFP_KERNEL);
	if (dest == NULL) {
		dev_info(
			&node_data->pdev->dev,
			"reg_fop_read kernel memory allocation failed for %s\n",
			node_data->nodename);
		return -ENOMEM;
	}

	adr = node_data->base_adr + *offset;
	start =dest;

#ifdef DEBUG
	dev_info(
		&node_data->pdev->dev,
		"reg_fop_read start adress: 0x%lx\ for %sn",
		adr, node_data->nodename);
#endif

	for (i = 0; i < len; i++, start++, adr+=4)
	{
		data = ioread32(adr);
#ifdef DEBUG
		dev_info(
			&node_data->pdev->dev,
			"reg_fop_read 0x%lx\n",
			data);
#endif
		*start = data;
	}
	if (copy_to_user(buf,(void*)dest,count)) {
		dev_info(
			&node_data->pdev->dev,
			"reg_fop_read copy_to_user failed for %s\n",
			node_data->nodename);
		kfree(dest);
		return -EFAULT;
	}
	kfree(dest);*/
#ifdef DEBUG
	dev_info(
		&node_data->pdev->dev,
		"reg_fop_read 0x%lx bytes read for %s\n",
		(unsigned long)(count),
		node_data->nodename);
#endif
	return count; /* fpga reg data read sucessfully */

}

static ssize_t reg_fop_write (struct file *file, const char *buf, size_t count,loff_t *offset)
{
	struct zfpga_node_data *node_data = file->private_data;
#ifdef DEBUG
	dev_info(
		&node_data->pdev->dev,
		"reg_fop_write entered for %s\n",
		node_data->nodename);
#endif
/*	unsigned long adr;
	unsigned long data;
	unsigned long *dp;
	unsigned long *source;
	unsigned long len;
	int i;

	len = count >> 2;
	if ( (*offset < FPGARegMemBase) || ((*offset + count) > (FPGARegMemBase+FPGARegMemSize)) || (count & 3))
	{
		dev_infoor(
			&node_data->pdev->dev,
			"reg_fop_write address fault for %s\n",
			node_data->nodename);
		return -EFAULT; // bad adress
	}
	dp = kmalloc(count,GFP_KERNEL);
	if (dp == NULL) {
		dev_infoor(
			&node_data->pdev->dev,
			"reg_fop_write kernel memory allocation failed for %s\n",
			node_data->nodename);
		return -ENOMEM;
	}
	source = dp;
	// copy user space data to kernel space
	if ( copy_from_user(dp,buf,count)) {
		dev_infoor(
			&node_data->pdev->dev,
			"reg_fop_write copy_from_user failed for %s\n",
			node_data->nodename);
		kfree(dp);
		return -EFAULT;
	}
	dp = source;
	adr = node_data->base_adr + *offset;
	for (i = 0; i < len; i++, source++, adr+=4)
	{
		data = *source;
#ifdef DEBUG
		dev_info(
			&node_data->pdev->dev,
			"reg_fop_write  data 0x%lx to adr 0x%lx for %s\n",
			data, adr, node_data->nodename);
#endif
		iowrite32(data, adr);
	}
	kfree(dp);*/
#ifdef DEBUG
	dev_info(
		&node_data->pdev->dev,
		"reg_fop_write 0x%lx bytes written for %s\n",
		(unsigned long)(count), node_data->nodename);
#endif
	return count; /* fpga reg data written sucessfully */
}

/* ---------------------- dsp device specific fops ----------------------- */
ssize_t dsp_fop_read (struct file *file, char *buf, size_t count,loff_t *offset)
{
	struct zfpga_node_data *node_data = file->private_data;
#ifdef DEBUG
	dev_info(
		&node_data->pdev->dev,
		"dsp_fop_read entered startadr: 0x%lx, length: 0x%lx for %s\n",
		(unsigned long)(*offset),(unsigned long) count, node_data->nodename);
#endif
/*	unsigned long adr, len, data;
	unsigned long *dest;
	int i;
	char* tmp;

	len = count >> 2;
	if ( test_adrspace(node_data->id, len, count, offset) <  0) {
		dev_info(
			&node_data->pdev->dev,
			"dsp_fop_read adress fault for %s\n",
			node_data->nodename);
		return -EFAULT; // bad adress
	}
	tmp = kmalloc(count,GFP_KERNEL);
	if (tmp == NULL) {
		dev_info(
			&node_data->pdev->dev,
			"dsp_fop_read adress memory allocation failed for %s\n",
			node_data->nodename);
		return -ENOMEM;
	}
	adr = node_data->base_adr;
	dest = (unsigned long*) tmp;
	// serial interface and dma initialization
	iowrite32(DSPREAD, adr + SPI);
	iowrite32((unsigned long)*offset,adr + SPI);
	iowrite32(len, adr + SPI);
	//udelay(100); give the dsp 100 uS for initialzing serial and dma
	for (i = 0;i < len;i++,dest++) {
		data = ioread32(adr + SERIAL);
		*dest = data;
#ifdef DEBUG
		dev_info(
			&node_data->pdev->dev,
			"dsp_fop_read data 0x%lx for %s\n",
			data, node_data->nodename);
#endif
	}
	if (copy_to_user(buf,(void*)tmp,count)) {
		dev_info(
			&node_data->pdev->dev,
			"dsp_fop_read copy_to_user failed for %s\n",
			node_data->nodename);
		kfree(tmp);
		return -EFAULT;
	}
	kfree(tmp);*/
#ifdef DEBUG
	dev_info(
		&node_data->pdev->dev,
		"dsp_fop_read data 0x%lx bytes read for %s\n",
		(unsigned long)(count), node_data->nodename);
#endif
	return count; /* data read from dsp sucessfully */
}

ssize_t dsp_fop_write (struct file *file, const char *buf, size_t count,loff_t *offset)
{
	struct zfpga_node_data *node_data = file->private_data;
#ifdef DEBUG
	dev_info(
		&node_data->pdev->dev,
		"dsp_fop_write entered startadr: 0x%lx, length: 0x%lx for %s\n",
		(unsigned long)(*offset),(unsigned long) count, node_data->nodename);
#endif
/*	unsigned long adr, len, data;
	unsigned long *source;
	int i;
	char* tmp;

	len = count >> 2;
	if ( test_adrspace(node_data->id, len, count, offset) <  0) {
		dev_info(
			&node_data->pdev->dev,
			"dsp_fop_write adress fault for %s\n",
			node_data->nodename);
		return -EFAULT; // bad adress
	}
	tmp = kmalloc(count,GFP_KERNEL);
	if (tmp == NULL) {
		dev_info(
			&node_data->pdev->dev,
			"dsp_fop_write adress memory allocation failed for %s\n",
			node_data->nodename);
		return -ENOMEM;
	}
	// copy user space data to kernel space
	if ( copy_from_user(tmp,buf,count)) {
		dev_info(
			&node_data->pdev->dev,
			"dsp_fop_write copy_from_user failed for %s\n",
			node_data->nodename);
		kfree(tmp);
		return -EFAULT;
	}

	adr = node_data->base_adr;
	source = (unsigned long*) tmp;
	// serial interface and dma initialization
	iowrite32(DSPWRITE, adr + SPI);
	iowrite32((unsigned long)*offset, adr + SPI);
	iowrite32(len, adr + SPI);

	/// udelay(100); give the dsp 100 uS for initialzing serial and dma

	// write the data now
	for (i = 0;i < len;i++,source++) {
		data = *source;
#ifdef DEBUG
		dev_info(
			&node_data->pdev->dev,
			"dsp_fop_write data 0x%lx for %s\n",
			data, node_data->nodename);
#endif
		iowrite32(temp, adr + SERIAL);
	}
	kfree(tmp);*/
#ifdef DEBUG
	dev_info(
		&node_data->pdev->dev,
		"dsp_fop_read data 0x%lx bytes written for %s\n",
		(unsigned long)(count), node_data->nodename);
#endif
	return count; /* data written to dsp sucessfully */
}

long dsp_fop_ioctl( struct file *file, unsigned int cmd, unsigned long arg)
{
#ifdef DEBUG
	struct zfpga_node_data *node_data = file->private_data;
	dev_info(&node_data->pdev->dev, "dsp ioctl, dev: %s cmd: 0x%x, arg: %lx\n", node_data->nodename, cmd, arg);
#endif
	switch ( cmd ) {
		/*case ADSP_RESET: return(reset_dsp(file));
		case ADSP_BOOT: return(boot_dsp(file, arg));
		case ADSP_INT_REQ: return(int_dsp(file));
		case ADSP_INT_ENABLE: return(int_dsp_enable(file));
		case ADSP_INT_DISABLE: return(int_dsp_disable(file));
		case IO_READ: return(read_io(file,arg));*/
		default:
#ifdef DEBUG
			dev_info(&node_data->pdev->dev, "dsp ioctl, dev: %s cmd: 0x%x invalid\n", node_data->nodename, cmd);
#endif
			return(-EINVAL);
	}
}

/* ---------------------- fops array ---------------------- */
static const struct file_operations fops_arr[NODE_TYPE_COUNT] = {
	[NODE_TYPE_BOOT] = {
		.owner = THIS_MODULE,
		.open = common_fop_open,
		.release = common_fop_release,
		.llseek = no_llseek,
		.write = boot_fop_write,
		.compat_ioctl = boot_fop_ioctl
	},
	[NOTE_TYPE_REG] = {
		.owner = THIS_MODULE,
		.open = common_fop_open,
		.release = common_fop_release,
		.llseek = common_fop_lseek,
		.read = reg_fop_read,
		.write = reg_fop_write,
		.fasync = common_fop_fasync
	},
	[NODE_TYPE_DSP] = {
		.owner = THIS_MODULE,
		.open = common_fop_open,
		.release = common_fop_release,
		.llseek = common_fop_lseek,
		.read = dsp_fop_read,
		.write = dsp_fop_write,
		.unlocked_ioctl = dsp_fop_ioctl,
		.fasync = common_fop_fasync,
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
static int parse_of(struct platform_device *pdev, struct zfpga_dev_data *zfpga)
{
	struct resource res;
	u8 nodetype;
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
		if((ret = of_property_read_u8(child_node, "nodetype", &nodetype))) {
			dev_info(&pdev->dev, "missing/incorrect entry 'nodetype' in %s!\n",
				child_node->full_name);
			goto exit;
		}
		if(nodetype >= NODE_TYPE_COUNT) {
			dev_info(&pdev->dev, "entry 'nodetype' out of limits in %s!\n",
				child_node->full_name);
			goto exit;
		}
		zfpga->nodes[zfpga->count_nodes].nodetype = nodetype;
#ifdef DEBUG
		dev_info(&pdev->dev, "entry 'nodetype = %u' found in %s\n",
				nodetype,
				child_node->full_name);
#endif
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
#ifdef DEBUG
		dev_info(&pdev->dev, "memory region remapped for %s to %p\n",
				zfpga->nodes[zfpga->count_nodes].nodename,
				zfpga->nodes[zfpga->count_nodes].base);
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
	dev_info(&pdev->dev, "zfpga_probe called\n");
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
	if((ret = parse_of(pdev, zfpga))) {
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
	dev_info(&pdev->dev, "zfpga_remove called\n");
#endif
	zfpga = platform_get_drvdata(pdev);
	if (!IS_ERR(zfpga) && zfpga->count_nodes) {
#ifdef DEBUG
		dev_info(&pdev->dev, "zfpga_remove: removing char devices\n");
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
		pr_err( "zfpga: unable to register platform driver!\n");
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
