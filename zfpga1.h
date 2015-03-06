/***************************************************************************
 *   Copyright (C) 2012 by Peter Lohmer (Zera GmbH)                        *
 *   p.lohmer@zera.de                                                      *
 *   Copyright (C) 2015 by Andreas Mueller (Zera GmbH)                     *
 *   a.mueller@zera.de / schnitzeltony@googlemail.com                      *
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

/* the device name */
#define FPGADEV_NAME "zFPGA1" 
/* the device class name for creating device file names */
#define ZERACLASS_NAME "zeraIO" 
/* using 'a' as the magic number to build the ioctl commands */
#define FPGA_IOC_MAGIC 'a'


/* ioctl commands */
#define FPGA_RESET _IOR(FPGA_IOC_MAGIC,1,char*)

#define ADSP_RESET _IOR(FPGA_IOC_MAGIC,1,char*)
#define ADSP_BOOT _IOR(FPGA_IOC_MAGIC,2,char*)
#define ADSP_INT_REQ _IOR(FPGA_IOC_MAGIC,3,char*)
#define ADSP_INT_ENABLE _IOR(FPGA_IOC_MAGIC,4,char*)
#define ADSP_INT_DISABLE _IOR(FPGA_IOC_MAGIC,5,char*)
#define IO_READ _IOR(FPGA_IOC_MAGIC,6,char*)


#endif /* FPGA_DRIVER_H */

