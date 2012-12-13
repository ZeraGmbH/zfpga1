#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <asm/system.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/fcntl.h>
#include <linux/slab.h>
#include <linux/signal.h>
#include <linux/list.h>
#include <linux/ioport.h>
#include <linux/cdev.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <plat/gpmc.h>
#include <asm/io.h>
#include <asm/uaccess.h>


#include "zfpga1.h"

/* If you want debugging uncomment: */
//#define DEBUG 1

/* and please forgive me....it´s my first kernel driver... */


#define FPGA_BUFFER_SIZE PAGE_SIZE


static struct resource zFPGA_resources[] = {
	[bootMem] = {
		.name		= RESOURCE_NAME_BOOT,
		.flags		= IORESOURCE_MEM,
	},
	[regMem] = {
		.name		= RESOURCE_NAME_REG,
		.flags		= IORESOURCE_MEM,
	},
	[hipMem] = {
		.name		= RESOURCE_NAME_HIP,
		.flags		= IORESOURCE_MEM,
	},
	[sysIrq] = {
		.name		= RESOURCE_NAME_IRQSYS,
		.flags		= IORESOURCE_IRQ,
	},
	[doneIrq] = {
		.name		= RESOURCE_NAME_IRQDONE,
		.flags		= IORESOURCE_IRQ,
	},
};


static struct zFPGA_platform_config zFPGA_platform_data = {
	.cs 		= GPMC_CS,
	.gpio_irq	= GPIO_IRQ_PIN,
	.gpio_reset	= GPIO_RESET_PIN,
	.gpio_done	= GPIO_DONE_PIN,
};


static struct platform_device zFPGA_platform_device = {
	.name		= FPGADEV_NAME,
	.id		= -1,	/* we only have 1 device -> -1 */
	.num_resources	= ARRAY_SIZE(zFPGA_resources),
	.resource	= zFPGA_resources,
	.dev		= {
		.platform_data = &zFPGA_platform_data, /* we set platform data here, perhaps we want to use this in probe function */
	},
};


static int zFPGA_probe(struct platform_device *pdev);
static int zFPGA_remove(struct platform_device *pdev);

static struct platform_driver zFPGA_platform_driver = {
	.driver		= {
		.name	= FPGADEV_NAME,
		.owner 	= THIS_MODULE,
	},
	.probe 		= zFPGA_probe,
	.remove		= __devexit_p (zFPGA_remove),
};



static struct class *zeraIOClass; /* the class to create, we will attach our devices to */
struct driver_data *driver_data;


static struct fpga_device_data zFPGA_device_data[] = {
	[boot] = {
		.devname	= "boot",
		.devnr		= 0,
		.usecount	= 0,
		.async_queue	= NULL,
	},
	[reg] = {
		.devname	= "reg",
		.devnr		= 1,
		.usecount	= 0,
		.async_queue	= NULL,
	},
	[dsp1] = {
		.devname	= "dsp1",
		.devnr		= 2,
		.usecount	= 0,
		.async_queue	= NULL,
	},
	[dsp2] = {
		.devname	= "dsp2",
		.devnr		= 3,
		.usecount	= 0,
		.async_queue	= NULL,
	},
	[dsp3] = {
		.devname	= "dsp3",
		.devnr		= 4,
		.usecount	= 0,
		.async_queue	= NULL,
	},
	[dsp4] = {
		.devname	= "dsp4",
		.devnr		= 5,
		.usecount	= 0,
		.async_queue	= NULL,
	},
};


static struct device_stat zFPGA_device_stat = {
	.fpgabootcount = 0,
	.configured = 0,
	.dspnum = 0,
	.dspbootcount = {0,0,0,0},
	.dsprunning = {0,0,0,0},
};


static struct devNode_data io_groupdata; 

#define __notyet 1
/* no timing settings at the moment */
#ifdef __notyet
static struct gpmc_timings FPGA_GPMC_Timing = {
	/* Minimum clock period for synchronous mode (in picoseconds) */
	sync_clk:0, /* the minimum clock period would be L3_ICLK now, but we are using async. mode */

	/* Chip-select signal timings corresponding to GPMC_CS_CONFIG2 */
	cs_on:0,		/* Assertion time */
	cs_rd_off:50,		/* Read deassertion time */
	cs_wr_off:50,		/* Write deassertion time */

	/* ADV signal timings corresponding to GPMC_CONFIG3 */
	adv_on:0,		/* Assertion time */
	adv_rd_off:10,		/* Read deassertion time */
	adv_wr_off:10,		/* Write deassertion time */

	/* WE signals timings corresponding to GPMC_CONFIG4 */
	we_on:15,		/* WE assertion time */
	we_off:35 ,		/* WE deassertion time */

	/* OE signals timings corresponding to GPMC_CONFIG4 */
	oe_on:15,		/* OE assertion time */
	oe_off:45,		/* OE deassertion time */

	/* Access time and cycle time timings corresponding to GPMC_CONFIG5 */
	page_burst_access:30,	/* Multiple access word delay */
	access:50,		/* Start-cycle to first data valid delay */ /* !!!!! p.l. this is read access time i think !!!!! */
	rd_cycle:60,		/* Total read cycle time */
	wr_cycle:60,		/* Total write cycle time */

	/* The following are only on OMAP3430 */
	wr_access:50,		/* WRACCESSTIME */
	wr_data_mux_bus:20,	/* WRDATAONADMUXBUS */
};
#endif


static ssize_t FPGA_boot_write (struct file *file, const char *buf, size_t count,loff_t *offset);
static long FPGA_boot_ioctl (struct file *file,unsigned int cmd, unsigned long arg);
static int FPGA_boot_open (struct inode *inode, struct file *file);
static int FPGA_boot_release (struct inode *inode, struct file *file);


static struct file_operations fpga_boot_fops = {
	owner:		THIS_MODULE,
	llseek:		no_llseek,
	write:		FPGA_boot_write,
	compat_ioctl:	FPGA_boot_ioctl,
	open:		FPGA_boot_open,
	release:	FPGA_boot_release,
};


static loff_t FPGA_reg_lseek (struct file *file, loff_t offset, int origin);
static ssize_t FPGA_reg_read (struct file *file, char *buf, size_t count,loff_t *offset);
static ssize_t FPGA_reg_write (struct file *file, const char *buf, size_t count,loff_t *offset);
static int FPGA_reg_open (struct inode *inode, struct file *file);
static int FPGA_reg_release (struct inode *inode, struct file *file);
static int FPGA_reg_fasync (int fd, struct file *file, int mode);


static struct file_operations fpga_register_fops = {
	owner:		THIS_MODULE,
	llseek:		FPGA_reg_lseek,
	read:		FPGA_reg_read,
	write:		FPGA_reg_write,
	open:		FPGA_reg_open,
	release:	FPGA_reg_release,
	fasync:		FPGA_reg_fasync,
};


static loff_t adspdev_lseek (struct file *file, loff_t offset, int origin);
static ssize_t adspdev_read (struct file *file, char *buf, size_t count,loff_t *offset);
static ssize_t adspdev_write (struct file *file, const char *buf, size_t count,loff_t *offset);
static long adspdev_ioctl (struct file *file,unsigned int cmd, unsigned long arg);
static int adspdev_open (struct inode *inode, struct file *file);
static int adspdev_release (struct inode *inode, struct file *file);
static int adspdev_fasync (int fd, struct file *file, int mode);


static struct file_operations adspdev_fops = {
	owner:		THIS_MODULE,
	llseek:		adspdev_lseek,
	read:		adspdev_read,
	write:		adspdev_write,
	unlocked_ioctl:	adspdev_ioctl,
	open:		adspdev_open,
	release:	adspdev_release,
	fasync:		adspdev_fasync,
};


static wait_queue_head_t adspdev_wqueue; /* wait queue for long duration delays when booting dsp */


/* here we try to handle irq's generated from resister part of fpga, means fpga itself */ 

static irqreturn_t FPGA_irq_isr (int irq_nr,void *dev_id)
{
	irqreturn_t ret = IRQ_NONE; /* default strange interrupt */
#ifdef DEBUG
	pr_info ("%s : received interrupt (fpga irq) ", FPGADEV_NAME);
#endif
	pr_info ("%s : received interrupt (fpga irq) ", FPGADEV_NAME);
	/* put in the handling here later when implemented by hardware */


#ifdef DEBUG
	if (ret == IRQ_NONE) 
	  	pr_info("not ");
	pr_info("handled\n");
#endif
	return ret;
}


/* here we try to handle irq's generated from adsp's */ 

static irqreturn_t ADSP_irq_isr (int irq_nr,void *dev_id)
{
	unsigned long stat, adr;
	struct fpga_device_data *devdata;
	int i;

	irqreturn_t ret = IRQ_NONE; /* default strange interrupt */
#ifdef DEBUG
	pr_info ("%s : received interrupt (dsp irq) ", FPGADEV_NAME);
#endif
	pr_info ("%s : received interrupt (dsp irq) ", FPGADEV_NAME);
	
	for (i = 0; i < 4 ; i++) {
		devdata = zFPGA_device_data + (dsp1 + i);
		adr = devdata->base_adr + DSPSTAT;
		stat = ioread32(adr);
#ifdef DEBUG
		pr_info("stat irq dsp: %ld from adress: 0x%lx\n", stat, adr);
#endif // DEBUG
		if (stat) {
			if (stat & IS_DSP_IRQ) {
#ifdef DEBUG
				pr_info("irq dsp reset at adress: 0x%lx\n", adr);
#endif // DEBUG
				iowrite32( IS_DSP_IRQ, adr); /* quit irq */
				if (devdata->async_queue)
					kill_fasync(&(devdata->async_queue), SIGIO, POLL_IN);
			}
			if (stat & IS_TIMEOUT_IRQ) {
#ifdef DEBUG
				pr_info("irq timeout reset at adress: 0x%lx\n", adr);
#endif // DEBUG
				iowrite32( IS_TIMEOUT_IRQ, adr); /* quit irq */
			}
			ret = IRQ_HANDLED;
		}
	}
#ifdef DEBUG
	if (ret == IRQ_NONE) 
	  	pr_info("not ");
	pr_info("handled\n");
#endif
	return ret;
}



/* the irq service for the fpga done bit */

static irqreturn_t FPGA_done_isr ( int irq_nr, void *dev_id )
{
	/* interrupt will tell us wether fpga is configured or not */
	zFPGA_device_stat.configured = gpio_get_value(zFPGA_platform_data.gpio_done); 

#ifdef DEBUG
	pr_info( "%s : received interrupt (fpga done) handled\n", FPGADEV_NAME);
#endif
	pr_info( "%s : received interrupt (fpga done) handled\n", FPGADEV_NAME);
	return IRQ_HANDLED;
}



/* fpga boot device file operations */

ssize_t FPGA_boot_write (struct file *file, const char *buf, size_t count,loff_t *offset)
{
	char* tmp;
	unsigned long adr;
	unsigned long len;
	struct fpga_device_data *devdata;

#ifdef DEBUG
	pr_info("%s: fpga boot write entered\n", FPGADEV_NAME);
#endif /* DEBUG */

	if (zFPGA_device_stat.configured)
	{
#ifdef DEBUG
		pr_info("%s: fpga boot write failed (fpga already configured)\n", FPGADEV_NAME);
#endif /* DEBUG */
		return -ENODEV;
	}

	devdata = file->private_data;

	tmp = kmalloc(count,GFP_KERNEL);
	if (tmp == NULL) {
#ifdef DEBUG
		pr_info("%s : fpga boot write , kernel memory allocation failed\n", FPGADEV_NAME);
#endif /* DEBUG */
		return -ENOMEM;
	}

	/* copy user space data to kernel space */
	if ( copy_from_user(tmp,buf,count)) {
#ifdef DEBUG
		pr_info("%s : fpga boot write, copy_from_user failed\n",FPGADEV_NAME);
#endif /* DEBUG */
		kfree(tmp);
		return -EFAULT;
	}

	adr = devdata->base_adr;
	len = count;

	
	while (len--)
		writeb(*(buf++), adr);
	
	zFPGA_device_stat.fpgabootcount += count;

	kfree(tmp);

#ifdef DEBUG
	pr_info("%s : fpga boot write 0x%lx bytes written\n", FPGADEV_NAME, (unsigned long)(count)); 
#endif /* DEBUG */

	return count; /* fpga boot data written sucessfully */
}


int reset_FPGA(void)
{
#ifdef DEBUG
	pr_info("%s : reset_FPGA entered\n", FPGADEV_NAME); 
#endif /* DEBUG */
	gpio_set_value(zFPGA_platform_data.gpio_reset, 1);
	udelay(10);
	gpio_set_value(zFPGA_platform_data.gpio_reset, 0);
	return 0;
}


long FPGA_boot_ioctl (struct file *file,unsigned int cmd, unsigned long arg)
{
#ifdef DEBUG
	pr_info("%s: fpga boot ioctl entered, cmd: 0x%x, arg: %lx\n", FPGADEV_NAME, cmd, arg);
#endif /* DEBUG */

	switch ( cmd ) {
		case FPGA_RESET: return(reset_FPGA());
		default: 
#ifdef DEBUG	
			pr_info("%s : fpga boot ioctl, cmd: 0x%x invalid\n", FPGADEV_NAME, cmd);	
#endif /* DEBUG */
			return(-EINVAL);
	}
	
}


int FPGA_boot_open (struct inode *inode, struct file *file)
{
	struct fpga_device_data *devdata;
	unsigned int minor;

#ifdef DEBUG
	pr_info("%s: fpga boot open entered\n", FPGADEV_NAME);
#endif /* DEBUG */

	minor = MINOR(inode->i_rdev);
	if (minor != boot) {
#ifdef DEBUG
		pr_info("%s : trying to open unsupported device\n", FPGADEV_NAME);
#endif /* DEBUG */
		return -ENODEV;
	}

	devdata = zFPGA_device_data + minor;

	if (devdata->usecount) // only once usable
		return -EBUSY;
	else devdata->usecount++;

	file->private_data = devdata;

#ifdef DEBUG
	pr_info("%s : device opened\n", FPGADEV_NAME);
#endif /* DEBUG */
	return 0;
}


int FPGA_boot_release (struct inode *inode, struct file *file)
{
	struct fpga_device_data *devdata;

#ifdef DEBUG
	pr_info("%s: fpga boot release entered\n", FPGADEV_NAME);
#endif /* DEBUG */

	devdata = file->private_data;
	devdata->usecount--;

	file->private_data = NULL;

#ifdef DEBUG
	pr_info("%s : device closed\n", FPGADEV_NAME);
#endif /* DEBUG */

	return 0;
}


/* fpga register device file operations */


static loff_t FPGA_reg_lseek (struct file *file, loff_t offset, int origin)
{
#ifdef DEBUG
	struct fpga_device_data *devdata;
	devdata = file->private_data;
	pr_info("%s lseek to adress 0x%lx \n",devdata->devname,(long)offset);
#endif
	file->f_pos = offset; /* only absolut positioning */
	return offset;
}


ssize_t FPGA_reg_read (struct file *file, char *buf, size_t count,loff_t *offset)
{
	struct fpga_device_data *devdata;
	unsigned long adr;
	unsigned long *dest;
	unsigned long *start;
	unsigned long data;
	unsigned long len;
	int i;
	
#ifdef DEBUG
	pr_info("%s: fpga reg read entered\n", FPGADEV_NAME);
#endif /* DEBUG */

	len = count >> 2;
	
	if ( (*offset < FPGARegMemBase) || ((*offset + count) > (FPGARegMemBase+FPGARegMemSize)) || (count & 3)) 
	{
#ifdef DEBUG
		pr_info("%s: fpga reg read adress fault\n", FPGADEV_NAME);
#endif /* DEBUG */
		return -EFAULT; /* bad adress */
	}

	devdata = file->private_data;

	dest = kmalloc(count,GFP_KERNEL);
	
	if (dest == NULL) {
#ifdef DEBUG
		pr_info("%s : fpga reg read , kernel memory allocation failed\n", FPGADEV_NAME);
#endif /* DEBUG */
		return -ENOMEM;
	}

	adr = devdata->base_adr + *offset;
	start =dest;
	
#ifdef DEBUG
		pr_info("%s : fpga reg read , start adress: 0x%lx\n", FPGADEV_NAME, adr);
#endif /* DEBUG */	
	
	for (i = 0; i < len; i++, start++, adr+=4)
	{
		data = ioread32(adr);
#ifdef DEBUG
		pr_info("%s : fpga reg read : 0x%lx\n", FPGADEV_NAME, data);
#endif /* DEBUG */		
		*start = data;
	}	

	if (copy_to_user(buf,(void*)dest,count)) {
#ifdef DEBUG
		pr_info("%s : fpga reg read, copy_to_user failed\n", FPGADEV_NAME); 
#endif /* DEBUG */
		kfree(dest);
		return -EFAULT;
	}
	
	kfree(dest);

#ifdef DEBUG
	pr_info("%s : fpga reg read 0x%lx bytes read\n", FPGADEV_NAME, (unsigned long)(count)); 
#endif /* DEBUG */
	
	return count; /* fpga reg data read sucessfully */
}


ssize_t FPGA_reg_write (struct file *file, const char *buf, size_t count,loff_t *offset)
{
	struct fpga_device_data *devdata;
	unsigned long adr;
	unsigned long data;
	unsigned long *dp;
	unsigned long *source;
	unsigned long len;
	int i;

#ifdef DEBUG
	pr_info("%s: fpga reg write entered\n", FPGADEV_NAME);
#endif /* DEBUG */

	len = count >> 2;
	
	if ( (*offset < FPGARegMemBase) || ((*offset + count) > (FPGARegMemBase+FPGARegMemSize)) || (count & 3)) 
	{
#ifdef DEBUG
		pr_info("%s: fpga reg write adress fault\n", FPGADEV_NAME);
#endif /* DEBUG */
		return -EFAULT; /* bad adress */
	}

	devdata = file->private_data;

	dp = kmalloc(count,GFP_KERNEL);
	
	if (dp == NULL) {
#ifdef DEBUG
		pr_info("%s : fpga reg write , kernel memory allocation failed\n", FPGADEV_NAME);
#endif /* DEBUG */
		return -ENOMEM;
	}

	source = dp;
	
	/* copy user space data to kernel space */
	if ( copy_from_user(dp,buf,count)) {
#ifdef DEBUG
		pr_info("%s : fpga reg write, copy_from_user failed\n",FPGADEV_NAME);
#endif /* DEBUG */
		kfree(dp);
		return -EFAULT;
	}

	dp = source;
	adr = devdata->base_adr + *offset;
	
	for (i = 0; i < len; i++, source++, adr+=4)
	{
		data = *source;
#ifdef DEBUG
		pr_info("%s : fpga reg write data 0x%lx to adr 0x%lx\n", FPGADEV_NAME, data, adr); 
#endif /* DEBUG */
		iowrite32(data, adr);
	}
	
	kfree(dp);

#ifdef DEBUG
	pr_info("%s : fpga reg write 0x%lx bytes written\n", FPGADEV_NAME, (unsigned long)(count)); 
#endif /* DEBUG */

	return count; /* fpga reg data written sucessfully */
}




int FPGA_reg_open (struct inode *inode, struct file *file)
{
	struct fpga_device_data *devdata;
	unsigned int minor;

#ifdef DEBUG
	pr_info("%s: fpga reg open entered\n", FPGADEV_NAME);
#endif /* DEBUG */
	pr_info("%s: fpga reg open entered\n", FPGADEV_NAME);
	
	minor = MINOR(inode->i_rdev);
	if (minor != reg) {
#ifdef DEBUG
		pr_info("%s : trying to open unsupported device\n", FPGADEV_NAME);
#endif /* DEBUG */
		pr_info("%s : trying to open unsupported device\n", FPGADEV_NAME);
		return -ENODEV;
	}

	if (!zFPGA_device_stat.configured)
	{
#ifdef DEBUG
		pr_info("%s: fpga reg open failed (fpga not configured)\n", FPGADEV_NAME);
#endif /* DEBUG */
		pr_info("%s: fpga reg open failed (fpga not configured)\n", FPGADEV_NAME);
		return -ENODEV;
	}

	devdata = zFPGA_device_data + minor;

	if (devdata->usecount) // only once usable
		return -EBUSY;
	else devdata->usecount++;

	file->private_data = devdata;

#ifdef DEBUG
	pr_info("%s : device opened\n", FPGADEV_NAME);
#endif /* DEBUG */
	pr_info("%s : device opened\n", FPGADEV_NAME);
	return 0;

}


int FPGA_reg_release (struct inode *inode, struct file *file)
{
	struct fpga_device_data *devdata;
#ifdef DEBUG
	pr_info("%s: fpga reg release entered\n", FPGADEV_NAME);
#endif /* DEBUG */
	pr_info("%s: fpga reg release entered\n", FPGADEV_NAME);
	
	devdata = file->private_data;
	devdata->usecount--;

	file->private_data = NULL;

#ifdef DEBUG
	pr_info("%s : device closed\n", FPGADEV_NAME);
#endif /* DEBUG */
	pr_info("%s : device closed\n", FPGADEV_NAME);
	
	return 0;
}


int FPGA_reg_fasync (int fd, struct file *file, int mode)
{
	struct fpga_device_data *devdata;
#ifdef DEBUG
	pr_info("%s: fpga reg fasync entered\n", FPGADEV_NAME);
#endif
	devdata = file->private_data;
	return ( fasync_helper (fd, file, mode, &devdata->async_queue));
}


/* adsp device(s) ioctl utilities */


static unsigned long GetByteCount(bootheader *h) 
{
	unsigned long nr = h -> Count;
	switch ( h->Tag ) {
		case	FinalInit:	return 0x600; /* fix length */
		case	ZeroLData:	return 0;
		case	ZeroL48:	return 0;
		case	InitL16:	return (nr << 1);
		case 	InitL32:	return (nr << 2);
		case	InitL48:	return (((nr + 1) & 0xFFFFFFFE) * 6);
		case 	InitL64:	return (nr << 3);
		case	ZeroExt8:	return 0;
		case	ZeroExt16:	return 0;
	}
	return nr;
}


static int reset_dsp(struct file *file)
{
	struct fpga_device_data *devdata;
	unsigned long adr;
	unsigned long tmp;

	devdata = file->private_data;
	adr = devdata->base_adr + DSPCTRL;
	tmp = ioread32(adr);
	iowrite32(tmp | RESET_DSP, adr);
	zFPGA_device_stat.dspbootcount[devdata->devnr - dsp1] = 0; /* status bootcount 0 */

#ifdef DEBUG
	pr_info("%s : ioctl reset DSP%d\n", FPGADEV_NAME, devdata->devnr - dsp1 +1);
#endif /* DEBUG */

	return 0;
}


static int boot_dsp(struct file *file, unsigned long arg)
{
	struct fpga_device_data *devdata;
	unsigned long adr, nr, tmp, i, *data;
	char *KMem;
	char *UserData;
	bootheader ActHeader;

	UserData  = (char*) arg; /* here we can get what we need */
	devdata = file->private_data;
	adr = devdata->base_adr;

	/* reset dsp before booting and clear all other settings*/

	tmp = ioread32(adr + DSPCTRL);
	iowrite32(tmp | RESET_DSP, adr + DSPCTRL);
	udelay(400); // dsp requires 4096 CLKIN (25MHz) cycles after deasserting reset
	zFPGA_device_stat.dspbootcount[devdata->devnr - dsp1] = 0;
	
	/* the first data block to load is the dsp´s bootstrap loader */
	ActHeader.Tag = InitL48;
	ActHeader.Count = 0x100; /* with fix length */
	ActHeader.Adress = 0x40000; /* and fix adress */
	

#ifdef DEBUG
	pr_info("%s : ioctl, entry boot loop\n", FPGADEV_NAME);
#endif /* DEBUG */

	for (;;) {
		if (ActHeader.Tag > MaxTag) {
			pr_info("%s : ioctl, boot invalid tag %lu found\n", FPGADEV_NAME, ActHeader.Tag); 
			return -EFAULT;
		}

		nr  = GetByteCount(&ActHeader);

#ifdef DEBUG
	pr_info("%s : ioctl, boot datablock type %lu length 0x%lx start 0x%lx\n",FPGADEV_NAME, ActHeader.Tag, nr, ActHeader.Adress); 
#endif /* DEBUG */

		if (nr > 0) { /* are there any bytes to send ? */
			KMem = kmalloc(nr,GFP_KERNEL);
			if (KMem == NULL) {
#ifdef DEBUG
				pr_info("%s : ioctl boot memory allocation failed\n", FPGADEV_NAME);
#endif /* DEBUG */
				return -ENOMEM;
			}


			if ( copy_from_user(KMem,UserData,nr) ) {
#ifdef DEBUG
				pr_info("%s : ioctl, reading boot data for dsp failed\n", FPGADEV_NAME); 
#endif /* DEBUG */
				kfree(KMem);
				return -EFAULT;
			}
		
			UserData += nr;
			data = (unsigned long*) KMem;
						
			for (i = 0;i < (nr >>2);i++,data++)
				iowrite32(*data, adr + SPI); 

			/* writes all the data of the desired block to the SPI */

			zFPGA_device_stat.dspbootcount[devdata->devnr - dsp1] += nr; /* counting the bytes booted */
			kfree(KMem);
			msleep(10);
			/*sleep_on_timeout (&adspdev_wqueue,1+ HZ/100); */ /* for each boot block wait at least 10 mS */

		}

#ifdef DEBUG
		pr_info("%s : ioctl, boot datablock type %lx done\n", FPGADEV_NAME, ActHeader.Tag); 
#endif /* DEBUG */

		if (ActHeader.Tag == FinalInit) break; /* we have finished */

		if ( copy_from_user(&ActHeader,UserData,12) ) { /* read the next header */
#ifdef DEBUG
			pr_info("%s : ioctl, reading boot header information for dsp failed\n", FPGADEV_NAME); 
#endif /* DEBUG */
			return -EFAULT;
		}

		UserData += 12;

		/* and now we send the next boot header via SPI */		
		iowrite32(ActHeader.Tag, adr + SPI); 
		iowrite32(ActHeader.Count,adr + SPI);
		iowrite32(ActHeader.Adress, adr + SPI);

		zFPGA_device_stat.dspbootcount[devdata->devnr - dsp1] += 12;
		
		/* in case of ZeroX Tag Headers there must be a delay of 2mS + ActHeader.Count * 100nS */

		switch (ActHeader.Tag) {
			case	FinalInit:	break;
			case	InitL16:	break;
			case 	InitL32:	break;
			case	InitL48:	break;
			case 	InitL64:	break;
			/*default: sleep_on_timeout (&adspdev_wqueue,1+(2*HZ)/100); *//* -> min. (1/HZ)*1000 mS   normal: 2ms  (Count * 100nS) */
			default: msleep(20); /* should be 2ms + count*100nS, count is max 1.5*2^16 */
		}
	}	
#ifdef DEBUG
	pr_info("%s : ioctl, booting dsp%d sucessful\n", FPGADEV_NAME, devdata->devnr - dsp1 + 1); 
#endif /* DEBUG */

	return 0;
}


static int int_dsp(struct file *file)
{
	struct fpga_device_data *devdata;
	unsigned long adr;
	unsigned long tmp;

	devdata = file->private_data;
	adr = devdata->base_adr + DSPCTRL;
	tmp = ioread32(adr);
	iowrite32(tmp | IRQ_2_DSP, adr);

#ifdef DEBUG
	pr_info("%s : ioctl IRQ_2_DSP generated for dsp%d\n", FPGADEV_NAME, devdata->devnr - dsp1 + 1);
#endif /* DEBUG */

	return 0;
}


static int int_dsp_enable(struct file *file)
{
	struct fpga_device_data *devdata;
	unsigned long adr;
	unsigned long tmp;

	devdata = file->private_data;
	adr = devdata->base_adr + DSPCFG;
	tmp = ioread32(adr);
	iowrite32(tmp | DSPIRQ_ENABLE, adr);

#ifdef DEBUG
	pr_info("%s : ioctl DSPIRQ enabled for dsp%d\n", FPGADEV_NAME, devdata->devnr - dsp1 + 1);
#endif /* DEBUG */

	return 0;
}


static int int_dsp_disable(struct file *file)
{
	struct fpga_device_data *devdata;
	unsigned long adr;
	unsigned long tmp;

	devdata = file->private_data;
	adr = devdata->base_adr + DSPCFG;
	tmp = ioread32(adr);
	iowrite32(tmp & (~DSPIRQ_ENABLE), adr);
	
#ifdef DEBUG
	pr_info("%s : ioctl DSPIRQ disabled for dsp%d\n", FPGADEV_NAME, devdata->devnr - dsp1 + 1);
#endif /* DEBUG */

	return 0;
}


static int read_io(struct file *file, unsigned long arg)
{	// reads the register[num] with num in arg
	struct fpga_device_data *devdata;
	unsigned long adr;
	unsigned long tmp;

	devdata = file->private_data;
	adr = devdata->base_adr + (arg << 2);
	tmp = ioread32(adr);
#ifdef DEBUG
	pr_info("%s : ioctl READ_IO for dsp%d = 0x%lx\n", FPGADEV_NAME, devdata->devnr - dsp1 + 1,tmp);
#endif /* DEBUG */

	return (int) tmp;
}



/* adsp device(s) file operations */


static loff_t adspdev_lseek (struct file *file, loff_t offset, int origin)
{
#ifdef DEBUG
	struct fpga_device_data *devdata;
	devdata = file->private_data;
	pr_info("%s lseek to adress 0x%lx \n",devdata->devname,(long)offset);
#endif
	file->f_pos = offset; /* only absolut positioning */
	return offset;
}


ssize_t adspdev_read (struct file *file, char *buf, size_t count,loff_t *offset)
{
	struct fpga_device_data *devdata;
	unsigned long adr, len;
#ifdef DEBUG
	unsigned long temp;
#endif
	unsigned long *dest;
	int i;
	char* tmp;

#ifdef DEBUG
	pr_info("%s : dsp read entered startadr: 0x%lx, length: 0x%lx\n",FPGADEV_NAME,(unsigned long)(*offset),(unsigned long) count);
#endif /* DEBUG */
	
	len = count >> 2;
	
	if ( (*offset < DSPDataMemBase) ||  ((*offset + len) > DSPDataMemTop) || (count & 3)) {
#ifdef DEBUG
		pr_info("%s: dsp read adress fault\n", FPGADEV_NAME);
#endif /* DEBUG */
		return -EFAULT; /* bad adress */
	}

	tmp = kmalloc(count,GFP_KERNEL);
	if (tmp == NULL) {
#ifdef DEBUG
		pr_info("%s : dsp read , kernel memory allocation failed\n", FPGADEV_NAME);
#endif /* DEBUG */
		return -ENOMEM;
	}

	devdata = file->private_data;

	adr = devdata->base_adr;
	dest = (unsigned long*) tmp;

	/* serial interface and dma initialization */
	iowrite32(DSPREAD, adr + SPI);
	iowrite32((unsigned long)*offset,adr + SPI);
	iowrite32(len, adr + SPI);

	udelay(100); /* give the dsp 100 uS for initialzing serial and dma */

#ifdef DEBUG
	for (i = 0;i < len;i++,dest++) {
		temp = ioread32(adr + SERIAL);
		*dest = temp;
		pr_info("%s : dsp read data 0x%lx\n", FPGADEV_NAME, temp);
	}
#else
	for (i = 0;i < len;i++,dest++) 
		*dest = ioread32(adr + SERIAL);
#endif /* DEBUG */
	
	
	if (copy_to_user(buf,(void*)tmp,count)) {
#ifdef DEBUG
		pr_info("%s : dsp read, copy_to_user failed\n", FPGADEV_NAME); 
#endif /* DEBUG */
		kfree(tmp);
		return -EFAULT;
	}
	
	kfree(tmp);

#ifdef DEBUG
	pr_info("%s : dsp%d read 0x%lx bytes read\n", FPGADEV_NAME, devdata->devnr -dsp1 +1, (unsigned long)(count)); 
#endif /* DEBUG */

	return count; /* data read from dsp sucessfully */
}


ssize_t adspdev_write (struct file *file, const char *buf, size_t count,loff_t *offset)
{
	struct fpga_device_data *devdata;
	unsigned long adr, len;
#ifdef DEBUG
	unsigned long temp;
#endif
	unsigned long *source;
	int i;
	char* tmp;

#ifdef DEBUG
	pr_info("%s : dsp write entered startadr: 0x%lx, length: 0x%lx\n",FPGADEV_NAME,(unsigned long)(*offset),(unsigned long) count);
#endif /* DEBUG */
	
	len = count >> 2;
	
	if ( (*offset < DSPDataMemBase) ||  ((*offset + len) > DSPDataMemTop) || (count & 3)) {
#ifdef DEBUG
		pr_info("%s: dsp write adress fault\n", FPGADEV_NAME);
#endif /* DEBUG */
		return -EFAULT; /* bad adress */
	}

	tmp = kmalloc(count,GFP_KERNEL);
	if (tmp == NULL) {
#ifdef DEBUG
		pr_info("%s : dsp write , kernel memory allocation failed\n", FPGADEV_NAME);
#endif /* DEBUG */
		return -ENOMEM;
	}

	/* copy user space data to kernel space */
	if ( copy_from_user(tmp,buf,count)) {
#ifdef DEBUG
		pr_info("%s : dsp write, copy_from_user failed\n", FPGADEV_NAME);
#endif /* DEBUG */
		kfree(tmp); 
		return -EFAULT;
	}

	devdata = file->private_data;

	adr = devdata->base_adr;
	source = (unsigned long*) tmp;
	
	/* serial interface and dma initialization */
	iowrite32(DSPWRITE, adr + SPI);
	iowrite32((unsigned long)*offset, adr + SPI);
	iowrite32(len, adr + SPI);

	udelay(100); /* give the dsp 100 uS for initialzing serial and dma */

	/* write the data now */
#ifdef DEBUG
	for (i = 0;i < len;i++,source++) {
		temp = *source;
		pr_info("%s : dsp write data 0x%lx\n", FPGADEV_NAME,temp);
		iowrite32(temp, adr + SERIAL);
	}
#else
	for (i = 0;i < len;i++,source++)
		iowrite32(*source, adr +SERIAL);
#endif // DEBUG

	kfree(tmp);

#ifdef DEBUG
	pr_info("%s : dsp%d write 0x%lx bytes written\n", FPGADEV_NAME, devdata->devnr -dsp1 +1, (unsigned long)(count)); 
#endif /* DEBUG */

	return count; /* data written to dsp sucessfully */
}


long adspdev_ioctl( struct file *file, unsigned int cmd, unsigned long arg)
{
#ifdef DEBUG
	struct fpga_device_data *devdata;
	pr_info("%s : dsp ioctl, cmd: 0x%x, arg: %lx\n", FPGADEV_NAME, cmd, arg);
#endif /* DEBUG */
	switch ( cmd ) {
		case ADSP_RESET: return(reset_dsp(file));
		case ADSP_BOOT: return(boot_dsp(file, arg));
		case ADSP_INT_REQ: return(int_dsp(file));
		case ADSP_INT_ENABLE: return(int_dsp_enable(file));
		case ADSP_INT_DISABLE: return(int_dsp_disable(file));
		case IO_READ: return(read_io(file,arg));
		default: 
#ifdef DEBUG	
			devdata = file->private_data;
			pr_info("%s : dsp%d ioctl, cmd: 0x%x invalid\n", FPGADEV_NAME, devdata->devnr -dsp1 +1, cmd);	
#endif /* DEBUG */
			return(-EINVAL);
	}
}


int adspdev_open (struct inode *inode, struct file *file)
{
	struct fpga_device_data *devdata;
	unsigned int minor;
	unsigned long id, addr;

	minor = MINOR(inode->i_rdev);
	
#ifdef DEBUG
	pr_info("%s: dsp%d open entered\n", FPGADEV_NAME, minor - dsp1 + 1);
#endif /*DEBUG*/

	if ( (minor < dsp1) || (minor > dsp4)) {
#ifdef DEBUG
		pr_info("%s : trying to open unsupported device\n", FPGADEV_NAME);
#endif /* DEBUG */
		return -ENODEV;
	}

	if (!zFPGA_device_stat.configured)
	{
#ifdef DEBUG
		pr_info("%s: dsp open failed (fpga not configured)\n", FPGADEV_NAME);
#endif /* DEBUG */
		return -ENODEV;
	}

	devdata = zFPGA_device_data + minor;
	addr = devdata->base_adr + MAGICID;
	id = ioread32(addr);

	if (id != adsp_21262_1_magic) {
#ifdef DEBUG
		pr_info("%s : dsp%d not available, id read 0x%lx from adress 0x%lx\n", FPGADEV_NAME, devdata->devnr - dsp1 + 1, id, addr);
#endif /* DEBUG */
		return -ENODEV;
	}
		
	if (devdata->usecount) // only once usable
		return -EBUSY;
	else devdata->usecount++;

	file->private_data = devdata;

#ifdef DEBUG
	pr_info("%s : device dsp%d opened\n", FPGADEV_NAME, devdata->devnr - dsp1 + 1);
#endif /* DEBUG */
	return 0;
}


int adspdev_release (struct inode *inode, struct file *file)
{
	struct fpga_device_data *devdata;
#ifdef DEBUG
	pr_info("%s: dsp%d release entered\n", FPGADEV_NAME, MINOR(inode->i_rdev) - dsp1 + 1);
#endif /* DEBUG */

	devdata = file->private_data;
	devdata->usecount--;

	file -> private_data = NULL;

#ifdef DEBUG
	pr_info("%s : device closed\n", FPGADEV_NAME);
#endif /* DEBUG */

	return 0;
}


int adspdev_fasync (int fd, struct file *file, int mode)
{
	struct fpga_device_data *devdata;
	devdata = file->private_data;
#ifdef DEBUG
	pr_info("%s: dsp%d fasync entered\n", FPGADEV_NAME, devdata->devnr -dsp1 +1);
#endif /*DEBUG*/
	return ( fasync_helper (fd, file, mode, &devdata->async_queue));
}


static int __devexit zFPGA_remove(struct platform_device *pdev)
{
	struct resource *res;

	/* we free the requested irq's */
	res = platform_get_resource_byname(pdev, IORESOURCE_IRQ, RESOURCE_NAME_IRQSYS);
	free_irq(res->start, FPGADEV_NAME);

	res = platform_get_resource_byname(pdev, IORESOURCE_IRQ, RESOURCE_NAME_IRQDONE);
	free_irq(res->start, FPGADEV_NAME);

	/* and the requested mem regions */ 
	
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, RESOURCE_NAME_REG);
	release_mem_region(res->start, resource_size(res));

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, RESOURCE_NAME_HIP);
	release_mem_region(res->start, resource_size(res));

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, RESOURCE_NAME_BOOT);
	release_mem_region(res->start, resource_size(res));


	return 0;
}


static int __devinit zFPGA_probe(struct platform_device *pdev)
{
	int i,ret;
	struct resource *res_ioboot, *res_ioreg, *res_iohip, *res_irqsys, *res_irqdone;
	
	/* what the heck is that ? */
	/* for future purpose we do as if the device registration had been done outside */
	
	/* so first we have to fetch all the resources needed */

	res_ioboot = platform_get_resource_byname(pdev, IORESOURCE_MEM, RESOURCE_NAME_BOOT);
	if (!res_ioboot) {
		pr_warning("%s: platform_get_resource %s failed\n", FPGADEV_NAME, RESOURCE_NAME_BOOT);
		ret = -ENODEV;
		goto out;
	}
	res_ioreg = platform_get_resource_byname(pdev, IORESOURCE_MEM, RESOURCE_NAME_REG);
	if (!res_ioreg) {
		pr_warning("%s: platform_get_resource %s failed\n", FPGADEV_NAME, RESOURCE_NAME_REG);
		ret = -ENODEV;
		goto out;
	}

	res_iohip = platform_get_resource_byname(pdev, IORESOURCE_MEM, RESOURCE_NAME_HIP);
	if (!res_iohip) {
		pr_warning("%s: platform_get_resource %s failed\n", FPGADEV_NAME, RESOURCE_NAME_HIP);
		ret = -ENODEV;
		goto out;
	}

	res_irqsys = platform_get_resource_byname(pdev, IORESOURCE_IRQ, RESOURCE_NAME_IRQSYS);
	if (!res_irqsys) {
		pr_warning("%s: platform_get_resource %s failed\n", FPGADEV_NAME, RESOURCE_NAME_IRQSYS);
		ret = -ENODEV;
		goto out;
	}

	res_irqdone = platform_get_resource_byname(pdev, IORESOURCE_IRQ, RESOURCE_NAME_IRQDONE);
	if (!res_irqdone) {
		pr_warning("%s: platform_get_resource %s failed\n", FPGADEV_NAME, RESOURCE_NAME_IRQSYS);
		ret = -ENODEV;
		goto out;
	}

	/* and then allocate them */
	if (!request_mem_region(res_ioboot->start, resource_size(res_ioboot), FPGADEV_NAME)) {
		pr_warning("%s: request_mem_region for %s failed\n", FPGADEV_NAME, RESOURCE_NAME_BOOT);
		ret = -EBUSY;
		goto out;
	}
	
	zFPGA_device_data[boot].base_adr = (unsigned long)ioremap(res_ioboot->start, res_ioboot->end - res_ioboot->start + 1);
	
	if (!request_mem_region(res_ioreg->start, resource_size(res_ioreg), FPGADEV_NAME)) {
		pr_warning("%s: request_mem_region for %s failed\n", FPGADEV_NAME, RESOURCE_NAME_REG);
		ret = -EBUSY;
		goto free_MEMBoot;
	}
	
	zFPGA_device_data[reg].base_adr = (unsigned long)ioremap(res_ioreg->start, res_ioreg->end - res_ioreg->start + 1);

#ifdef DEBUG
	pr_info ( "%s : request_mem_region for %s 0x%lx bytes at adr 0x%lx remapped to 0x%lx successful\n", FPGADEV_NAME, RESOURCE_NAME_REG, (unsigned long)resource_size(res_ioreg), (unsigned long) res_ioreg->start,zFPGA_device_data[reg].base_adr);
#endif	/* DEBUG */

	if (!request_mem_region(res_iohip->start, resource_size(res_iohip), FPGADEV_NAME)) {
		pr_warning("%s: request_mem_region for %s failed\n", FPGADEV_NAME, RESOURCE_NAME_HIP);
		ret = -EBUSY;
		goto free_MEMReg_MEMBoot;
	} 

	for ( i = 0 ; i < 4; i++ )
		zFPGA_device_data[dsp1+i].base_adr = (unsigned long)ioremap(res_iohip->start + sizeof(dspmmap)*i, sizeof(dspmmap));	


#ifdef DEBUG
	pr_info ( "%s : request_mem_region for %s 0x%lx bytes at adr 0x%lx remapped to 0x%lx successful\n", FPGADEV_NAME, RESOURCE_NAME_HIP, (unsigned long) resource_size(res_iohip), (unsigned long) res_iohip->start,zFPGA_device_data[dsp1].base_adr);
#endif	/* DEBUG */

	/* int request_irq(unsigned int irq, irqreturn_t (*handler) (int, void*, struct pt_regs*), unsigned long irqflags, const char* devname, void* dev_id)*/
	if ( request_irq( res_irqdone->start, FPGA_done_isr, FPGA_irqdone_flags, INTERRUPT_NAME_DONE, zFPGA_device_data + boot ) < 0) {
		pr_warning( "%s: request_irq (IRQ%d) for %s failed\n", FPGADEV_NAME, res_irqdone->start, INTERRUPT_NAME_DONE);
		ret = -EIO;
		goto free_MEMHip_MEMReg_MEMBoot; 
	}

#ifdef DEBUG
	pr_info ( "%s: request_irq (IRQ%d) for %s successful\n", FPGADEV_NAME, res_irqdone->start, INTERRUPT_NAME_DONE);
#endif	/* DEBUG */


	if ( request_irq( res_irqsys->start, FPGA_irq_isr, FPGA_irqsys_flags, INTERRUPT_NAME_REG, zFPGA_device_data + reg) < 0) {
		pr_warning( "%s: request_irq (IRQ%d) for %s failed\n", FPGADEV_NAME, res_irqsys->start, INTERRUPT_NAME_REG);
		ret = -EIO;
		goto free_IRQDone_MEMHip_MEMReg_MEMBoot; 
	}

#ifdef DEBUG
	pr_info ( "%s: request_irq (IRQ%d) for %s successful\n", FPGADEV_NAME, res_irqdone->start, INTERRUPT_NAME_REG);
#endif	/* DEBUG */

	if ( request_irq( res_irqsys->start, ADSP_irq_isr, ADSP_irqsys_flags, INTERRUPT_NAME_DSP, zFPGA_device_data + dsp1) < 0) {
		pr_warning( "%s: request_irq (IRQ%d) for %s failed\n", FPGADEV_NAME, res_irqsys->start, INTERRUPT_NAME_DSP);
		ret = -EIO;
		goto free_IRQFPGA_IRQDone_MEMHip_MEMReg_MEMBoot; 
	}

#ifdef DEBUG
	pr_info ( "%s: request_irq (IRQ%d) for %s successful\n", FPGADEV_NAME, res_irqsys->start, INTERRUPT_NAME_DSP);
#endif	/* DEBUG */

	/* int alloc_chrdev_region(dev_t *dev, unsigned baseminor, unsigned count, const char *name) */
	if ( (ret = alloc_chrdev_region(&io_groupdata.devNode, 0, 6, FPGADEV_NAME)) < 0) { /* we try to get 6 devices with minors 0..5 */
		pr_warning( "%s: alloc_chrdev_region failed\n", FPGADEV_NAME);		
		goto free_IRQADSP_IRQFPGA_IRQDone_MEMHip_MEMReg_MEMBoot;
	}

#ifdef DEBUG
	pr_info ( "%s: alloc_chrdev_region successful\n", FPGADEV_NAME);
#endif	/* DEBUG */

	/* we initialize the required cdev's */
	cdev_init(&(io_groupdata.FPGA_cdev[boot]), &fpga_boot_fops);
	cdev_init(&(io_groupdata.FPGA_cdev[reg]), &fpga_register_fops);
	cdev_init(&(io_groupdata.FPGA_cdev[dsp1]), &adspdev_fops);
	for ( i=0; i<2; i++)
		io_groupdata.FPGA_cdev[i].owner = THIS_MODULE;

	/* and add them to the system */

	/* int cdev_add(struct cdev *p, dev_t dev, unsigned count) */
	if ( (ret = cdev_add(&io_groupdata.FPGA_cdev[boot], io_groupdata.devNode, 1)) < 0) {
		pr_warning( "%s: cdev_add failed\n", FPGADEV_NAME);		
		goto free_chrdev_IRQADSP_IRQFPGA_IRQDone_MEMHip_MEMReg_MEMBoot;	
	}

	if ( (ret = cdev_add(&io_groupdata.FPGA_cdev[reg], io_groupdata.devNode + 1, 1)) < 0) {
		pr_warning( "%s: cdev_add failed\n", FPGADEV_NAME);		
		goto free_chrdev_IRQADSP_IRQFPGA_IRQDone_MEMHip_MEMReg_MEMBoot;	
	}
	
	if ( (ret = cdev_add(&io_groupdata.FPGA_cdev[dsp1], io_groupdata.devNode + 2, 4)) < 0) {
		pr_warning( "%s: cdev_add failed\n", FPGADEV_NAME);		
		goto free_chrdev_IRQADSP_IRQFPGA_IRQDone_MEMHip_MEMReg_MEMBoot;	
	}

	#ifdef DEBUG
	pr_info ( "%s: all cdev_add successful\n", FPGADEV_NAME);
#endif	/* DEBUG */

	for (i = 0; i < 6; i++)
		device_create( zeraIOClass, &pdev->dev, io_groupdata.devNode + i, NULL, "%s%s", FPGADEV_NAME, zFPGA_device_data[i].devname); 

	return 0;

free_chrdev_IRQADSP_IRQFPGA_IRQDone_MEMHip_MEMReg_MEMBoot:
	unregister_chrdev_region(io_groupdata.devNode, 6);

free_IRQADSP_IRQFPGA_IRQDone_MEMHip_MEMReg_MEMBoot:
	free_irq(res_irqsys->start, INTERRUPT_NAME_DSP);

free_IRQFPGA_IRQDone_MEMHip_MEMReg_MEMBoot: 
	free_irq(res_irqsys->start, INTERRUPT_NAME_REG);

free_IRQDone_MEMHip_MEMReg_MEMBoot: 
	free_irq(res_irqdone->start, INTERRUPT_NAME_DONE);

free_MEMHip_MEMReg_MEMBoot: 
	release_mem_region(res_iohip->start, resource_size(res_iohip));

free_MEMReg_MEMBoot: 
	release_mem_region(res_ioreg->start, resource_size(res_ioreg));

free_MEMBoot: 
	release_mem_region(res_ioboot->start, resource_size(res_ioboot));

out: 	
	return ret;

}


static int __init fpga_config(void)
{
	int ret;
	unsigned long cs_mem_base;

	ret = 0;

#ifdef DEBUG
	pr_info ("%s: Start configuring\n", FPGADEV_NAME);
#endif // DEBUG


	if ( (ret = gpmc_cs_request(zFPGA_platform_data.cs, SZ_16M, &cs_mem_base)) <  0) {
		pr_warning( "%s: CS%d already in use \n", FPGADEV_NAME, zFPGA_platform_data.cs );
		goto out;
	}

#ifdef DEBUG
	pr_info( "%s: GPMC CS%d allocated\n", FPGADEV_NAME, zFPGA_platform_data.cs ); 
#endif // DEBUG

	/* we init the resources actual needed here*/

	/* the mem resource for the fpga boot device is 1 adress beyond the last */
        /* fpga adress available and used as adress for boot write access */
	zFPGA_resources[bootMem].start = cs_mem_base + FPGATotalMemSize;
	zFPGA_resources[bootMem].end = cs_mem_base + FPGATotalMemSize;

	/* the mem resource for the fpga register file */
	zFPGA_resources[regMem].start = cs_mem_base + FPGARegMemBase;
	zFPGA_resources[regMem].end = cs_mem_base + FPGARegMemBase + FPGARegMemSize - 1;

	/* the mem resource for the dsp register files (up to 4) */
	zFPGA_resources[hipMem].start = cs_mem_base + DSPHipMemBase;
	zFPGA_resources[hipMem].end = cs_mem_base + DSPHipMemBase + sizeof(dspmmap)*4 - 1;

	/* now we have to configure the cs  */
	// gpmc_cs_configure( zFPGA_platform_data.cs, GPMC_CONFIG_DEV_SIZE, GPMC_CONFIG1_DEVICESIZE_16); /* we'll take 16bit for booting as well as normal operation */
	gpmc_cs_configure( zFPGA_platform_data.cs, GPMC_CONFIG_DEV_TYPE, GPMC_DEVICETYPE_NOR); /* async. nor flash device type with muxed adress/data pins */
	
	/* timing settings are not a bad idea */
	/* but normally it shouldn't be a good idea to do it before gpmc_cs_request */
	/* but after gpmc_cs_request the cs is enabled and timing settings will be disgarded (see datasheet) */	
	/* but gpmc_cs_set_timings not exported from kernel,..... we changed the kernel */
	if ( (ret = gpmc_cs_set_timings(zFPGA_platform_data.cs, &FPGA_GPMC_Timing)) <  0) {
		pr_warning( "%s: timing settings for CS%d rejected\n", FPGADEV_NAME, zFPGA_platform_data.cs);
		// goto free_CS;
	}

#ifdef DEBUG
		pr_info( "%s: timing settings for CS%d done, configuring now\n", FPGADEV_NAME, zFPGA_platform_data.cs);
#endif // DEBUG

	gpmc_cs_configure( zFPGA_platform_data.cs, GPMC_CONFIG_RDY_BSY, 1); /* enable wait monitoring for read/write */
	
	/* we will request the gpio's needed now */

	if ( (ret = gpio_request(zFPGA_platform_data.gpio_reset, GPIO_RESET_NAME)) < 0) {
		pr_warning( "%s: failed to request GPIO%d for reset\n", FPGADEV_NAME, zFPGA_platform_data.gpio_reset);
		goto free_CS;
	}
	else gpio_direction_output(zFPGA_platform_data.gpio_reset, 0); /* reset is output, inactive (low) */

#ifdef DEBUG
	pr_info ( "%s: GPIO%d for reset allocated\n", FPGADEV_NAME, zFPGA_platform_data.gpio_reset ); 
#endif // DEBUG

	if ( (ret = gpio_request(zFPGA_platform_data.gpio_done, GPIO_DONE_NAME)) < 0) {
		pr_warning( "%s: failed to request GPIO%d for done\n", FPGADEV_NAME, zFPGA_platform_data.gpio_done);
		goto free_IOReset_CS;
	}
	else gpio_direction_input(zFPGA_platform_data.gpio_done); /* done is input */

#ifdef DEBUG
	pr_info( "%s: GPIO%d for done allocated\n", FPGADEV_NAME, zFPGA_platform_data.gpio_done);
#endif // DEBUG

	if ( (ret = gpio_request(zFPGA_platform_data.gpio_irq, GPIO_IRQ_NAME)) < 0) {
		pr_warning( "%s: failed to request GPIO%d for irq\n", FPGADEV_NAME, zFPGA_platform_data.gpio_irq);
		goto free_IODone_IOReset_CS;
	}
	else gpio_direction_input(zFPGA_platform_data.gpio_irq); /* irq is input */

#ifdef DEBUG
	pr_info( "%s: GPIO%d for irq allocated\n", FPGADEV_NAME, zFPGA_platform_data.gpio_irq );
#endif // DEBUG


	/* we have to link gpio's to linux interrupts and actualize used resources */

	if ( (zFPGA_resources[sysIrq].start = zFPGA_resources[sysIrq].end = gpio_to_irq(zFPGA_platform_data.gpio_irq)) == 0) {
		pr_warning ( "%s: failed to link interrupt for GPIO%d (irq)\n", FPGADEV_NAME, zFPGA_platform_data.gpio_irq);
		ret = -EIO;
		goto free_IOIrq_IODone_IOReset_CS;
	}

#ifdef DEBUG
	pr_info( "%s: GPIO%d (irq) linked to irq%d\n", FPGADEV_NAME, zFPGA_platform_data.gpio_irq, zFPGA_resources[sysIrq].start);
#endif // DEBUG


	if ( (zFPGA_resources[doneIrq].start = zFPGA_resources[doneIrq].end = gpio_to_irq(zFPGA_platform_data.gpio_done)) == 0) {
		pr_warning( "%s: failed to link interrupt for GPIO%d (done)\n", FPGADEV_NAME, zFPGA_platform_data.gpio_done);
		ret = -EIO;
		goto free_IOIrq_IODone_IOReset_CS;
	}

#ifdef DEBUG
	pr_info( "%s: GPIO%d (done) linked to irq%d\n", FPGADEV_NAME, zFPGA_platform_data.gpio_done, zFPGA_resources[doneIrq].start);
#endif // DEBUG

	reset_FPGA();

#ifdef DEBUG
	pr_info( "%s: output reset pulse for FPGA\n", FPGADEV_NAME);
#endif // DEBUG


	init_waitqueue_head(&adspdev_wqueue);

	return 0;

free_IOIrq_IODone_IOReset_CS: gpio_free(zFPGA_platform_data.gpio_irq);

free_IODone_IOReset_CS: gpio_free(zFPGA_platform_data.gpio_done);

free_IOReset_CS: gpio_free(zFPGA_platform_data.gpio_reset);

free_CS: gpmc_cs_free(zFPGA_platform_data.cs);

out:	return ret;
}


static int __exit fpga_free(void)
{
	gpio_free(zFPGA_platform_data.gpio_irq);
	gpio_free(zFPGA_platform_data.gpio_done);	
	gpio_free(zFPGA_platform_data.gpio_reset);
	
	gpmc_cs_free(zFPGA_platform_data.cs);

	return 0;
}


/*int init_module(void)*/
static int __init zfpga_1_init_module(void)
{
	int result;

	pr_info( "Module %s init\n", FPGADEV_NAME );

	/* let's try to get all needed hardware stuff first */	
	if ( (result = fpga_config()) < 0) {
		pr_err( "%s: unable to initialize zFPGA device -> shutdown\n", FPGADEV_NAME );
		goto out;
	}

	/* ok, now we have to create a device class to create our device files later */ 
	zeraIOClass = class_create(THIS_MODULE, FPGADEV_NAME);
	if (IS_ERR(zeraIOClass)) {
		pr_err( "%s: unable to create device class -> shutdown\n", FPGADEV_NAME);
		goto out;
	}

	if ( (result = platform_device_register(&zFPGA_platform_device)) < 0) {
		pr_err( "%s: unable to register zFPGA device -> shutdown\n", FPGADEV_NAME );
		goto out;
	}

	if ( (result = platform_driver_register(&zFPGA_platform_driver)) < 0) {
		pr_err( "%s: unable to register zFPGA driver -> shutdown\n", FPGADEV_NAME );
		goto free_device;
	}
	
	pr_info ( "Module %s install successful\n", FPGADEV_NAME);

	return 0;

free_device: 
	platform_device_unregister(&zFPGA_platform_device);

out:	
	return result;
}	


/* void cleanup_module(void) */
static void __exit zfpga_1_exit_module(void)
{
	pr_info( "Module %s exit\n", FPGADEV_NAME);

	class_destroy( zeraIOClass);

	platform_driver_unregister(&zFPGA_platform_driver);
	platform_device_unregister(&zFPGA_platform_device);
	
	fpga_free();
}


MODULE_DESCRIPTION("ZERA FPGA Type 1 kernel module");
MODULE_AUTHOR("Peter Lohmer (p.lohmer@zera.de)");
MODULE_LICENSE("GPL");
MODULE_SUPPORTED_DEVICE("ZERA zFPGA1");

module_init(zfpga_1_init_module);
module_exit(zfpga_1_exit_module);
