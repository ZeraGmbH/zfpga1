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

#include "zfpga1.h"

/* If you want debugging uncomment: */
#define DEBUG 1

static const struct of_device_id zfpga_of_match[] = {
  { .compatible = "zera,zfpga-1", },
  {}
};
MODULE_DEVICE_TABLE(of, zfpga_of_match);


static int zfpga_probe(struct platform_device *pdev)
{
	const struct of_device_id *match;

	match = of_match_device(zfpga_of_match, &pdev->dev);
	if (!match)
		return -EINVAL;

#ifdef DEBUG
	pr_info( "zfpga_probe called\n");
#endif // DEBUG
	return 0;
}


static int zfpga_remove(struct platform_device *pdev)
{
#ifdef DEBUG
	pr_info( "zfpga_remove called\n");
#endif // DEBUG
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
module_platform_driver(zfpga_platform_driver);


MODULE_DESCRIPTION("ZERA FPGA Type 1 kernel module");
MODULE_AUTHOR("Peter Lohmer (p.lohmer@zera.de)");
MODULE_AUTHOR("Andreas Mueller (a.mueller@zera.de)");
MODULE_LICENSE("GPL");
MODULE_SUPPORTED_DEVICE("ZERA zFPGA1");
