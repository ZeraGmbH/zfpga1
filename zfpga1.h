/***************************************************************************
 *   Copyright (C) 2012 by Peter Lohmer (Zera GmbH)			   *
 *   p.lohmer@zera.de							   *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/


/* FPGA's on Gumstix OMAP will be connected through GPMC.

   1) This means that the GPMC has to be programmed for the used 
   chip select, the start adress and memory size associated with it.
   Because fpga wiring is fix, chip select and size are define when compiled.
 
   2) The timing for the chip select in use has to be set. These parameters
   are also fixed.

   3) FPGA makes use of 3 gpio's, we have to request and configure them.
   First will be used with ioctl for resetting the fpga.
   Second will be used as input and irq source for testing whether
   boot sequence finished successfully. 
   Third will be used as input and irq source for providing fpga and
   dsp interrupts to the system. 
  
   4) We have to provide functions for reading and writing the fpga . 
   After reset or power up the fpga is not yet programmed and all the 
   write accesses to the device, regardless from adress, will be used 
   for programming the fpga. Once being programmed, the fpga adress 
   decoder is working, and subsequent read/write operations will access 
   the fpga's internal registers. DSP read/write access requires setting up
   a dsp internal dma for subsequent access. We see that there are different
   behaviours necessary on read/write access. 
   Regarding this, we will make use of several minors , each supporting an
   other char device with different file operations. 
   First minor(0) is the boot device.
   Second minor(1) is the fpga register device, which performs transparent
   fpga register access.
   The following minors (2..5) are dsp devices.

   5) The driver makes use of the new device driver model and sysfs. 
   Driver status information can be examined by reading the device driver attributes */
    
	
#ifndef FPGA_DRIVER_H
#define FPGA_DRIVER_H

#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/types.h>

#define FPGA_IOC_MAGIC 'f' 
/* using 'f' as the magic number to build the ioctl commands */
#define FPGADEV_NAME "zFPGA-1" 
/* the device name */
#define ZERACLASS_NAME "zeraIO" 
/* the device class name... we need a device class for creating device file names and adding attributes */


#define GPMC_CS 1 /* gpmc chip select = 1 */

#define GPIO_RESET_PIN 186 /* gpio pins in use */
#define GPIO_DONE_PIN 146
#define GPIO_IRQ_PIN 145

#define GPIO_RESET_NAME "zFPGA reset"
#define GPIO_DONE_NAME "zFPGA done"
#define GPIO_IRQ_NAME "zFPGA irq"

enum resourceType {bootMem, regMem, hipMem, sysIrq, doneIrq};

#define RESOURCE_NAME_BOOT "zFPGA-IOBOOT"
#define RESOURCE_NAME_REG "zFPGA-IOReg"
#define RESOURCE_NAME_HIP "zFPGA-IOHIP"
#define RESOURCE_NAME_IRQSYS "zFPGA-IRQSys"
#define RESOURCE_NAME_IRQDONE "zFPGA-IRQDone"

#define INTERRUPT_NAME_DONE "zFPGA-IRQDone"
#define INTERRUPT_NAME_REG "zFPGA-IRQReq"
#define INTERRUPT_NAME_DSP "zFPGA-IRQDSP"

#define FPGA_irqdone_flags IRQF_SHARED | IRQF_SAMPLE_RANDOM | IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING
#define FPGA_irqsys_flags IRQF_SHARED | IRQF_SAMPLE_RANDOM | IRQF_TRIGGER_RISING
#define ADSP_irqsys_flags IRQF_SHARED | IRQF_SAMPLE_RANDOM | IRQF_TRIGGER_RISING

struct device_stat {
	unsigned long fpgabootcount; /* 0 after reset or init */
	int configured; /* 0 oder 1 */
	int dspnum; /* how much dsp in the system */
	unsigned long dspbootcount[4]; /* 0 after reset or init */
	int dsprunning[4]; /* we have up to 4 dsp's in the system */
};


struct zFPGA_platform_config {
	unsigned int	cs;
	unsigned int	gpio_irq;
	unsigned int	gpio_reset;
	unsigned int 	gpio_done;
};


enum deviceType {boot, reg, dsp1, dsp2, dsp3, dsp4};

struct fpga_device_data {
	char devname[10];
	unsigned long base_adr; /* base adress for device */
	int devnr;
	int usecount; /* for permission handling */
	struct fasync_struct *async_queue; /* for used interrupt */
};


struct devNode_data {
	dev_t devNode;
	struct cdev FPGA_cdev[3];
};
	

/* ioctl commands */
#define FPGA_RESET _IOR(FPGA_IOC_MAGIC,1,char*)

#define ADSP_RESET _IOR(FPGA_IOC_MAGIC,1,char*)
#define ADSP_BOOT _IOR(FPGA_IOC_MAGIC,2,char*)
#define ADSP_INT_REQ _IOR(FPGA_IOC_MAGIC,3,char*)
#define ADSP_INT_ENABLE _IOR(FPGA_IOC_MAGIC,4,char*)
#define ADSP_INT_DISABLE _IOR(FPGA_IOC_MAGIC,5,char*)
#define IO_READ _IOR(FPGA_IOC_MAGIC,6,char*)


/* dsp internal data memory space */
#define DSPMemBase 0x80000 
#define DSPMemLength 0x7FFF
#define DSPDataMemBase 0x82800
#define DSPDataMemTop 0x87FFF


/* fpga register memory space */
/* for direct hardware access */
#define FPGARegMemBase 0x0
#define FPGARegMemSize 0x400
#define FPGATotalMemSize 0x2000

/* dsp hip (host interface port) emulation memory space */
/* for access dsp internal data */
#define DSPHipMemBase 0x400


static struct dspmmap { /* dsp interface memory map */
	volatile unsigned long SPI; /* Serial Port Interface */
	volatile unsigned long Serial; /* Serial Interface */
	volatile unsigned long DSPCtrl; /* Device Ctrl Register */
	volatile unsigned long DSPStat; /* Device Status Information */
	volatile unsigned long DSPCFG; /* Device Config Reg. */
	volatile unsigned long VersionNr; /* what ? */
	volatile unsigned long MagicId; /* the device name */
	volatile unsigned long Res[1]; /* reserved */
} dspmmap; /* dspmmap is 0x20 byte long ... for other allocation use increased Res */


/* macros for hip access */
#define SPI 0
#define SERIAL 4
#define DSPCTRL 8
#define DSPSTAT 12
#define DSPCFG 16
#define VERSIONNR 20
#define MAGICID 24


typedef struct bootheader {
	unsigned long Tag;
	unsigned long Count;
	unsigned long Adress;
} bootheader;


/* all possible boot block tags */
#define FinalInit 0
#define ZeroLData 1
#define ZeroL48 2
#define	InitL16 3
#define InitL32 4
#define InitL48 5
#define InitL64 6
#define ZeroExt8 7
#define ZeroExt16 8
#define MaxTag ZeroExt16


/* commands for initialization of dsp internal serial interface
   and dma, which work as host interface port emulation */
#define DSPREAD 0x80000001
#define DSPWRITE 0x00000001


/* irq ctrl bits */
#define RESET_DSP 0x00000080
#define IRQ_2_DSP 0x00000002
#define DSPIRQ_FORCE 0x00000001
/* status bits */
#define DSP_RUNNING 0x00000080
#define IS_DSP_IRQ 0x00000001
#define IS_TIMEOUT_IRQ  0x00000002
/* cfg bits */
#define DSPIRQ_ENABLE 0x00000001


#define adsp_21262_1_magic 0xAA55BB44


#endif /* FPGA_DRIVER_H */

