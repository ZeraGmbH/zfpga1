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

static const struct of_device_id zfpga_of_match[] = {
  { .compatible = "zera,zfpga-1", },
  {}
};
MODULE_DEVICE_TABLE(of, zfpga_of_match);


#define MAX_NODE_COUNT 8
#define MAX_NODE_TYPES 2

struct zfpga_node_data {
	const char *nodename;
	u32 nodetype;
	void __iomem *base;
};

struct zfpga_dev_data {
	struct zfpga_node_data sub_device[MAX_NODE_COUNT];
	unsigned int count_nodes;
};

static int zfpga_probe(struct platform_device *pdev)
{
	const struct of_device_id *match;
	struct zfpga_dev_data *zfpga;
	struct resource res;
	struct device_node *child_node;
	int ret;
	u32 nodetype;

#ifdef DEBUG
	dev_info(&pdev->dev, "zfpga_probe called\n");
#endif // DEBUG

	ret = 0;
	match = of_match_device(zfpga_of_match, &pdev->dev);
	if (!match) {
		ret = -EINVAL;
		goto exit;
	}

	/* alloc driver data and set to device */
	zfpga = devm_kzalloc(&pdev->dev, sizeof(*zfpga), GFP_KERNEL);
	if (!zfpga) {
		dev_err(&pdev->dev, "can't alloc device data\n");
		ret = -ENOMEM;
		goto exit;
	}
	platform_set_drvdata(pdev, zfpga);

	/* apply device node's information (dsp/boot/reg) */
	child_node = NULL;
	while ((child_node = of_get_next_child(pdev->dev.of_node, child_node)) != NULL) {
		/* avoid buffer overflow */
		if (zfpga->count_nodes > MAX_NODE_COUNT) {
			dev_err(&pdev->dev, "maximum node count %u reached in %s!\n",
				MAX_NODE_COUNT,
				pdev->dev.of_node->full_name);
			goto exit_cleanup_drvdata;
		}
		/* node's name */
		if((ret = of_property_read_string(
				child_node,
				"nodename",
				&zfpga->sub_device[zfpga->count_nodes].nodename))) {
			dev_err(&pdev->dev, "missing/incorrect entry 'nodename' for %s!\n",
				child_node->full_name);
			goto exit_cleanup_drvdata;
		}
#ifdef DEBUG
		dev_info(&pdev->dev, "entry 'nodename = %s' found in %s\n",
				zfpga->sub_device[zfpga->count_nodes].nodename,
		child_node->full_name);
#endif // DEBUG
		/* node's type */
		if((ret = of_property_read_u32(child_node, "nodetype", &nodetype))) {
			dev_err(&pdev->dev, "missing/incorrect entry 'nodetype' in %s!\n",
				child_node->full_name);
			goto exit_cleanup_drvdata;
		}
		if(nodetype > MAX_NODE_TYPES) {
			dev_err(&pdev->dev, "entry 'nodetype' out of limits in %s!\n",
				child_node->full_name);
			goto exit_cleanup_drvdata;
		}
		zfpga->sub_device[zfpga->count_nodes].nodetype = nodetype;
#ifdef DEBUG
		dev_info(&pdev->dev, "entry 'nodetype = %u' found in %s\n",
				nodetype,
		child_node->full_name);
#endif // DEBUG
		/* setup node's memory region */
		ret = of_address_to_resource(pdev->dev.of_node, zfpga->count_nodes, &res);
		if (ret) {
			dev_err(&pdev->dev, "can't get memory limits - 'reg' properly set in %s?\n",
				child_node->full_name);
			goto exit_cleanup_drvdata;
		}
		zfpga->sub_device[zfpga->count_nodes].base = devm_ioremap_resource(&pdev->dev, &res);
		if (IS_ERR(zfpga->sub_device[zfpga->count_nodes].base)) {
			dev_err(&pdev->dev, "can't remap in %s\n",
				child_node->full_name);
			ret = PTR_ERR(zfpga->sub_device[zfpga->count_nodes].base);
			goto exit_cleanup_drvdata;
		}
		/* next node */
		zfpga->count_nodes++;
	}
	return 0;

exit_cleanup_drvdata:
	platform_set_drvdata(pdev, NULL);
exit:
	return ret;
}


static int zfpga_remove(struct platform_device *pdev)
{
#ifdef DEBUG
	dev_info(&pdev->dev, "zfpga_remove called\n");
#endif // DEBUG

	platform_set_drvdata(pdev, NULL);

	return 0;
}

static struct class *zfpga_class;

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
	class_destroy(zfpga_class);
	platform_driver_unregister(&zfpga_platform_driver);
}

MODULE_DESCRIPTION("ZERA FPGA Type 1 kernel module");
MODULE_AUTHOR("Peter Lohmer (p.lohmer@zera.de)");
MODULE_AUTHOR("Andreas Mueller (a.mueller@zera.de)");
MODULE_LICENSE("GPL");
MODULE_SUPPORTED_DEVICE("ZERA zFPGA1");

module_init(zfpga_init);
module_exit(zfpga_exit);
