#include <linux/vmalloc.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/completion.h>
#include <asm/uaccess.h>
#include <linux/usb.h>
#include <asm/scatterlist.h>
#include <linux/mm.h>
#include <linux/pci.h>    //for scatterlist macros
#include <linux/pagemap.h>
#include "ooptDriver.h"
#ifdef OOPT_NO_BKL
#include <linux/mutex.h>
#else
#include <linux/smp_lock.h>
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35)
#define usb_alloc_coherent usb_buffer_alloc
#define usb_free_coherent usb_buffer_free
#endif

/*
 * ooptDriver.c
 *
 *                         Copyright (C) 2012-2014 SLAC National Accelerator Laboratory
 *
 *                 Author: Chang-Ming Tsai
 * Last Modification Date: 03/05/2014
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation version 2 of the License
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

/* Use our own dbg macro */
#undef dbg
//#define dbg(format, arg...) do { if (debug) printk(KERN_DEBUG KBUILD_MODNAME ": %d: " format "" , __LINE__, ## arg); } while (0)
#define dbg(format, arg...) do { if (debug) printk(KERN_INFO KBUILD_MODNAME ": %s(): %d: " format "\n", __FUNCTION__, __LINE__, ## arg); } while (0)

#undef info
#define info(format, arg...) printk(KERN_INFO KBUILD_MODNAME ": %s(): %d: " format "\n", __FUNCTION__, __LINE__, ## arg)

#undef warn
#define warn(format, arg...) printk(KERN_WARNING KBUILD_MODNAME ": %s(): %d: " format "\n", __FUNCTION__, __LINE__, ## arg)

#undef err
#define err(format, arg...) printk(KERN_ERR KBUILD_MODNAME ": %s(): %d: " format "\n", __FUNCTION__, __LINE__, ## arg)

#ifdef OOPT_NO_BKL
DEFINE_MUTEX(oopt_openmtx);
#endif

static int debug = 0;

static int liDeviceMinor  [MAX_OOPT_DEVICES];
static int liIoctlCommand [MAX_OOPT_DEVICES] = {11, 22, 33, 44};
static int liSimpleRead   [MAX_OOPT_DEVICES];

/* Module parameters */
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "Debug enabled");

module_param_array(liDeviceMinor, int, NULL, 0444);
MODULE_PARM_DESC(liDeviceMinor, "Device Minors (Array)");

module_param_array(liIoctlCommand, int, NULL, 0644);
MODULE_PARM_DESC(liIoctlCommand, "Ioctl Command (Array)");

module_param_array(liSimpleRead, int, NULL, 0644);
MODULE_PARM_DESC(liSimpleRead, "Simple Read Flag (Array)");

static void oopt_deleteDeviceInfo(struct kref *kref);
static int oopt_runCommandNoRet(DeviceInfo* pDevInfo, IoctlInfo* ioctlInfo);
static int oopt_runCommandRet(DeviceInfo* pDevInfo, IoctlInfo* pIoctlInfo, void __user * arg);
static int oopt_runArm(DeviceInfo* pDevInfo, ArmInfo* pArmInfo, void __user * arg);
static int oopt_runUnarm(DeviceInfo* pDevInfo, ArmInfo* pArmInfo, void __user * arg);
static int oopt_runPollEnable(DeviceInfo* pDevInfo, PollInfo* pPollInfo, void __user * arg);
static int oopt_runQueryDevice(DeviceInfo* pDevInfo, QueryDeviceInfo* pQueryDeviceInfo, void __user * arg);

static int oopt_reset_data(DeviceInfo *pDevInfo);
static void oopt_finish_frame(DeviceInfo* pDevInfo, int iUrbId);
static int oopt_rearm(DeviceInfo* pDevInfo);
static void oopt_arm_callback1(struct urb *urb, struct pt_regs *regs );
static void oopt_arm_callback2(struct urb *urb, struct pt_regs *regs );
static void oopt_waitLastArm(DeviceInfo* pDevInfo);

static ssize_t oopt_read_simple(struct file *file, char *buffer, size_t count, loff_t *ppos);
static ssize_t oopt_read_urb(struct file *file, char *buffer, size_t count, loff_t *ppos);
//static int discardSpectra(DeviceInfo* pDevInfo);


/**
 *  oopt_init
 */
static int __init oopt_init(void)
{
  int iRetVal;

  // Initialize static variables
  memset(liDeviceMinor, -1, sizeof(liDeviceMinor));
  memset(liSimpleRead ,  0, sizeof(liSimpleRead));

  info("%s Version: %s", DRIVER_DESC, DRIVER_VERSION);
#ifdef __x86_64__
  info("64 bit driver");
#else
  info("32 bit driver");
#endif

  info("Param: debug = %d", debug);

  //!!!debug
  info("HZ                   = %d", HZ);
  info("PAGE_SIZE            = %ld", PAGE_SIZE);
  info("OOPT_DATA_BULK_MAXUM = %ld", (long) OOPT_DATA_BULK_MAXNUM);
  info("OOPT_IOCTL_CMD_NORET = 0x%lx", OOPT_IOCTL_CMD_NORET);
  info("OOPT_IOCTL_CMD_RET   = 0x%lx", OOPT_IOCTL_CMD_RET);
  info("size of SpectraPostheader  = %ld", sizeof(SpectraPostheader));
  info("size of DataInfo           = %ld", sizeof(DataInfo));
  info("size of IoctlInfo          = %ld", sizeof(IoctlInfo));
  info("size of ArmInfo            = %ld", sizeof(ArmInfo));
  info("size of timespec           = %ld", sizeof(struct timespec));

  info("EINVAL               = %d", EINVAL);
  /*
   * register this driver with the USB subsystem
   *
   * Note: oopt_probe() will be called right inside this call
   */
  iRetVal = usb_register(&oopt_driver);
  if (iRetVal)
  {
    err("usb_register failed: %d", iRetVal);
    return iRetVal;
  }

  return 0;
}

/**
 *  oopt_exit
 */
static void __exit oopt_exit(void)
{
  /* deregister this driver with the USB subsystem */
  usb_deregister(&oopt_driver);

  info("unloading module...");
}

static int oopt_initDeviceInfo(DeviceInfo* pDevInfo)
{
  const int iNumFirst2kBulk   = 4;
  int       i;
  int       iBulk;
  uint8_t*  pUrbArmBuffer;

  dbg("init start");

  for (i=0; i<MAX_OOPT_DEVICES; ++i)
    if (liDeviceMinor[i] == -1)
      break;

  if (i >= MAX_OOPT_DEVICES)
  {
    err("Max suppported OOPT devices: %d", MAX_OOPT_DEVICES);
    return -EFAULT;
  }
  pDevInfo->iDeviceIndex      = i;
  liDeviceMinor[i] = pDevInfo->minor;

  pDevInfo->pIoctlInfo = kmalloc(sizeof(IoctlInfo), GFP_KERNEL);
  if (pDevInfo->pIoctlInfo == NULL)
  {
    err("Out of memory for Iotctl Buffer");
    return -ENOMEM;
  }

  pDevInfo->pSpectraBuffer = (uint8_t*) __get_free_pages(GFP_KERNEL | __GFP_DMA, OOPT_SPECTRABUF_PAGEORDER);
  if (pDevInfo->pSpectraBuffer == NULL)
  {
    err("Out of memory for Spetra Buffer");
    return -ENOMEM;
  }

  pDevInfo->pUrbArm = usb_alloc_urb(0, GFP_KERNEL);
  if (pDevInfo->pUrbArm == NULL)
  {
    dbg("usb_alloc_urb() failed");
    return -ENOMEM;
  }
  pUrbArmBuffer = usb_alloc_coherent(pDevInfo->udev, 1, GFP_KERNEL, &(pDevInfo->pUrbArm->transfer_dma));
  if (pUrbArmBuffer == NULL)
  {
    dbg("usb_alloc_coherent() failed");
    return -ENOMEM;
  }
  pUrbArmBuffer[0] = 0x09;

  usb_fill_bulk_urb(pDevInfo->pUrbArm, pDevInfo->udev, pDevInfo->luEpAddr[ENDPOINT_COMMAND], pUrbArmBuffer,
    1, (usb_complete_t) oopt_arm_callback1, pDevInfo);
  pDevInfo->pUrbArm->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;

  switch (pDevInfo->udev->descriptor.idProduct)
  {
  case PRODUCT_ID_HR4000:
  case PRODUCT_ID_USB4000:
    pDevInfo->iSpectraDataSize = 8192;
    pDevInfo->iEpFirst2K       = ENDPOINT_RESERVED;
    break;
  case PRODUCT_ID_USB2000P:
    pDevInfo->iSpectraDataSize = 4608;
    pDevInfo->iEpFirst2K       = ENDPOINT_DATA;
    break;
  }
  pDevInfo->iNumDataBulk     = pDevInfo->iSpectraDataSize / OOPT_DATA_BULK_SIZE;
  pDevInfo->iNumSpectraInBuf = (int) (OOPT_SPECTRABUF_SIZE / pDevInfo->iSpectraDataSize);

  for (iBulk = 0; iBulk < pDevInfo->iNumDataBulk; ++iBulk)
  {
    struct urb* urb = usb_alloc_urb(0, GFP_KERNEL);
    uint8_t*    pUrbBuffer;
    int iEndpSpectra;

    if (urb == NULL)
    {
      dbg("usb_alloc_urb() failed for bulk %d", iBulk);
      return -ENOMEM;
    }
    pDevInfo->lUrb[iBulk]       = urb;
    pDevInfo->luUrbId[iBulk]    = iBulk;
    pUrbBuffer = usb_alloc_coherent(pDevInfo->udev, OOPT_DATA_BULK_SIZE, GFP_KERNEL, &urb->transfer_dma);
    if (pUrbBuffer == NULL)
    {
      dbg("usb_alloc_coherent() failed for bulk %d", iBulk);
      return -ENOMEM;
    }

    iEndpSpectra= (iBulk < iNumFirst2kBulk ? pDevInfo->luEpAddr[pDevInfo->iEpFirst2K] : pDevInfo->luEpAddr[ENDPOINT_DATA]);
    usb_fill_bulk_urb(urb, pDevInfo->udev, iEndpSpectra, pUrbBuffer,
      OOPT_DATA_BULK_SIZE, (usb_complete_t) oopt_arm_callback2, &pDevInfo->luUrbId[iBulk]);

    if (iBulk == pDevInfo->iNumDataBulk-1)
      urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
    else
      urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP | URB_NO_INTERRUPT;
  }

  for (;iBulk < OOPT_DATA_BULK_MAXNUM; ++iBulk)
  {
    pDevInfo->lUrb   [iBulk] = NULL;
    pDevInfo->luUrbId[iBulk] = 0;
  }

  init_waitqueue_head(&pDevInfo->wait_single_data);
  init_waitqueue_head(&pDevInfo->wait_final_arm);

  return 0;
}

static int oopt_probe(struct usb_interface *interface, const struct usb_device_id *id)
{
  struct usb_host_interface *iface_desc;
  int i;
  DeviceInfo *pDevInfo = NULL;
  int iRetVal = -ENOMEM;

  pDevInfo = kzalloc(sizeof(DeviceInfo), GFP_KERNEL);
  if (pDevInfo == NULL)
  {
    err("Out of memory for Device Data");
    return -ENOMEM;
  }

  kref_init(&pDevInfo->kref);

  pDevInfo->udev      = usb_get_dev(interface_to_usbdev(interface));

  /* See if the device offered us matches what we can accept */
  if ((pDevInfo->udev->descriptor.idVendor != VENDOR_ID)
      || (pDevInfo->udev->descriptor.idProduct != PRODUCT_ID_HR4000 &&
          pDevInfo->udev->descriptor.idProduct != PRODUCT_ID_USB2000P &&
          pDevInfo->udev->descriptor.idProduct != PRODUCT_ID_USB4000))
  {
    iRetVal = -ENODEV;
    goto error;
  }

  iface_desc = interface->cur_altsetting;
  if (iface_desc->desc.bNumEndpoints != ENDPOINT_NUMBER)
  {
    err("# EndPoints (%d) != Driver Support Number (%d)", iface_desc->desc.bNumEndpoints, ENDPOINT_NUMBER);
    iRetVal = -ENODEV;
    goto error;
  }

  for (i = 0; i < iface_desc->desc.bNumEndpoints; ++i)
  {
    struct usb_endpoint_descriptor *
          endpoint    = &iface_desc->endpoint[i].desc;
    int   iBufferSize = le16_to_cpu(endpoint->wMaxPacketSize);

    if ((endpoint->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) != USB_ENDPOINT_XFER_BULK)
    {
      err("Endpoint %d is not a bulk endpoint", i);
      iRetVal = -ENODEV;
      goto error;
    }

    if (iBufferSize != OOPT_DATA_BULK_SIZE)
    {
      err("Endpoint %d has wrong data size %d (expected %d)", i, iBufferSize, OOPT_DATA_BULK_SIZE);
      iRetVal = -ENODEV;
      goto error;
    }

    if (endpoint->bEndpointAddress & USB_DIR_IN)
      pDevInfo->luEpAddr[i] =
        usb_rcvbulkpipe(pDevInfo->udev, endpoint->bEndpointAddress);
    else
      pDevInfo->luEpAddr[i] =
        usb_sndbulkpipe(pDevInfo->udev, endpoint->bEndpointAddress);

    dbg("Endpoint %d addr 0x%02x", i, endpoint->bEndpointAddress);
  }

  iRetVal = oopt_initDeviceInfo(pDevInfo);
  if (iRetVal != 0)
    goto error;

  pDevInfo->interface         = interface;
  pDevInfo->iProductId        = pDevInfo->udev->descriptor.idProduct;
  pDevInfo->minor             = interface->minor;

  usb_set_intfdata(interface, pDevInfo);
  iRetVal = usb_register_dev(interface, &oopt_class);
  if (iRetVal)
  {
    err("Not able to get a minor for this device.");
    usb_set_intfdata(interface, NULL);

    goto error;
  }
  pDevInfo->present = 1;

  info("OOPT USB2.0 device now attached to oopt-%d", pDevInfo->minor);
  return 0;

error:
  if (pDevInfo)
    kref_put(&pDevInfo->kref, oopt_deleteDeviceInfo);
  return iRetVal;
}

/**
 *  oopt_disconnect
 *
 *  Called by the usb core when the device is removed from the system.
 *
 *  This routine guarantees that the driver will not submit any more urbs
 *  by clearing pDevInfo->udev.  It is also supposed to terminate any currently
 *  active urbs.  Unfortunately, usb_bulk_msg(), used in oopt_read(), does
 *  not provide any way to do this.  But at least we can cancel an active
 *  write.
 */
static void oopt_disconnect(struct usb_interface *interface)
{
  DeviceInfo *pDevInfo;
  int minor = interface->minor;

#ifdef OOPT_NO_BKL
  mutex_lock(&oopt_openmtx);
#else
  lock_kernel();
#endif
  pDevInfo = usb_get_intfdata(interface);
  usb_set_intfdata(interface, NULL);
  /* give back our minor */
  usb_deregister_dev(interface, &oopt_class);
#ifdef OOPT_NO_BKL
  mutex_unlock(&oopt_openmtx);
#else
  unlock_kernel();
#endif
  

  /* prevent device read and ioctl */
  pDevInfo->present = 0;

  kref_put(&pDevInfo->kref, oopt_deleteDeviceInfo);
  info("OOPT USB2.0 device #%d now disconnected", minor);
}

/**
 *  oopt_deleteDeviceInfo
 */
static void oopt_deleteDeviceInfo(struct kref *kref)
{
  DeviceInfo *pDevInfo      = container_of( kref, DeviceInfo, kref );
  int         iBulk;

  dbg("Freeing device buffers");

  if (pDevInfo->udev)
    usb_put_dev(pDevInfo->udev);

  if ( pDevInfo->pSpectraBuffer)
    free_pages( (long) pDevInfo->pSpectraBuffer, OOPT_SPECTRABUF_PAGEORDER);

  if (pDevInfo->pIoctlInfo)
    kfree(pDevInfo->pIoctlInfo);

  if (pDevInfo->pUrbArm)
  {
    usb_free_coherent(pDevInfo->pUrbArm->dev, pDevInfo->pUrbArm->transfer_buffer_length, pDevInfo->pUrbArm->transfer_buffer, pDevInfo->pUrbArm->transfer_dma);
    usb_free_urb(pDevInfo->pUrbArm);
  }

  for (iBulk = 0; iBulk < OOPT_DATA_BULK_MAXNUM; ++iBulk)
  {
    struct urb* urb = pDevInfo->lUrb[iBulk];
    if (urb == NULL)
      continue;

    usb_free_coherent(urb->dev, urb->transfer_buffer_length, urb->transfer_buffer, urb->transfer_dma);
    usb_free_urb(urb);
  }

  kfree(pDevInfo);
}


/**
 *  oopt_open
 */
static int oopt_open(struct inode *inode, struct file *file)
{
  DeviceInfo *pDevInfo = NULL;
  struct usb_interface *interface;
  int iMinor;
  int iRetVal = 0;

#ifdef OOPT_NO_BKL
  mutex_lock(&oopt_openmtx);
#endif

  iMinor = iminor(inode);

  interface = usb_find_interface(&oopt_driver, iMinor);
  if (!interface)
  {
    err("Can't find device for minor %d", iMinor);
    iRetVal = -ENODEV;
    goto done;
  }

  pDevInfo = usb_get_intfdata(interface);
  if (!pDevInfo)
  {
    iRetVal = -ENODEV;
    goto done;
  }

  iRetVal = oopt_reset_data(pDevInfo);
  if (iRetVal != 0)
  {
    err("oopt_reset_data() failed");
    iRetVal = -EFAULT;
    goto done;
  }

  /* increment our usage count for the device */
  kref_get(&pDevInfo->kref);
  /* save our object in the file's private structure */
  file->private_data = pDevInfo;

  dbg("set device data %p", pDevInfo);

  done:
#ifdef OOPT_NO_BKL
    mutex_unlock(&oopt_openmtx);
#endif
    return iRetVal;
}

/**
 *  oopt_release
 */
static int oopt_release(struct inode *inode, struct file *file)
{
  DeviceInfo *pDevInfo;
  int iRetVal = 0;

  pDevInfo = (DeviceInfo *) file->private_data;
  if (pDevInfo == NULL)
  {
    dbg("device data is NULL");
    return -ENODEV;
  }

  iRetVal = oopt_reset_data(pDevInfo);
  if (iRetVal != 0)
  {
    err("oopt_reset_data() failed");
    return -EFAULT;
  }

  info("release device data %p", pDevInfo);

  /* decrement the count on our device */
  kref_put(&pDevInfo->kref, oopt_deleteDeviceInfo);
  return iRetVal;
}

static long oopt_compat_ioctl (struct file *file, unsigned int cmd, unsigned long arg)
{
  return oopt_ioctl(NULL, file, cmd, arg);
}


static int oopt_ioctl(struct inode *inode, struct file *file,
          unsigned int cmd, unsigned long arg)
{
  DeviceInfo* pDevInfo;
  int         iRetVal;
  pDevInfo = (DeviceInfo *) file->private_data;

  /* verify that the device wasn't unplugged */
  if (!pDevInfo->present)
  {
    dbg("No Device Present");
    return -ENODEV;
  }

  if (!access_ok(VERIFY_READ,  (void __user *) arg, _IOC_SIZE(cmd)))
  {
    warn("No read permission for the argument");
    return -EFAULT;
  }

  if( (_IOC_DIR( cmd ) & _IOC_READ ) &&
      !access_ok(VERIFY_WRITE,  (void __user *) arg, _IOC_SIZE(cmd)))
  {
    warn("No write permission for the argument");
    return -EFAULT;
  }

  switch (cmd)
  {
  case OOPT_IOCTL_CMD_NORET:
    if (copy_from_user(pDevInfo->pIoctlInfo, (void __user *) arg, sizeof(IoctlInfo)) != 0)
      goto error;
    iRetVal = oopt_runCommandNoRet(pDevInfo, pDevInfo->pIoctlInfo);
    break;
  case OOPT_IOCTL_CMD_RET:
    if (copy_from_user(pDevInfo->pIoctlInfo, (void __user *) arg, sizeof(IoctlInfo)) != 0)
      goto error;
    iRetVal = oopt_runCommandRet(pDevInfo, pDevInfo->pIoctlInfo, (void __user *) arg);
    break;
  case OOPT_IOCTL_ARM:
    if (copy_from_user(pDevInfo->pIoctlInfo, (void __user *) arg, sizeof(ArmInfo)) != 0)
      goto error;
    iRetVal = oopt_runArm(pDevInfo, (ArmInfo*) pDevInfo->pIoctlInfo, (void __user *) arg);
    break;
  case OOPT_IOCTL_UNARM:
    if (copy_from_user(pDevInfo->pIoctlInfo, (void __user *) arg, sizeof(ArmInfo)) != 0)
      goto error;
    iRetVal = oopt_runUnarm(pDevInfo, (ArmInfo*) pDevInfo->pIoctlInfo, (void __user *) arg);
    break;
  case OOPT_IOCTL_POLLENABLE:
    if (copy_from_user(pDevInfo->pIoctlInfo, (void __user *) arg, sizeof(PollInfo)) != 0)
      goto error;
    iRetVal = oopt_runPollEnable(pDevInfo, (PollInfo*) pDevInfo->pIoctlInfo, (void __user *) arg);
    break;
  case OOPT_IOCTL_DEVICEINFO:
    if (copy_from_user(pDevInfo->pIoctlInfo, (void __user *) arg, sizeof(QueryDeviceInfo)) != 0)
      goto error;
    iRetVal = oopt_runQueryDevice(pDevInfo, (QueryDeviceInfo*) pDevInfo->pIoctlInfo, (void __user *) arg);
    break;
  default:
    /* return that we did not understand this ioctl call */
    warn("Invalid ioctl command 0x%x", cmd);
    return -ENOTTY;
  }

  return iRetVal;

error:
  warn("Failure of copy_from_user of ioctl command 0x%x", cmd);
  return -EFAULT;
}

static int oopt_runCommandNoRet(DeviceInfo* pDevInfo, IoctlInfo* pIoctlInfo)
{
  int iCommand = pIoctlInfo->lu8Command[0];
  int iCmdSendLen;
  int iRetVal;

  liIoctlCommand[pDevInfo->iDeviceIndex] = iCommand;

  if (debug >= 2)
  {
    int i;
    dbg("IoctlInfo Command:");
    dbg("  u8LenCommand     : %d", pIoctlInfo->u8LenCommand);
    dbg("  u16CmdTimeoutInMs: %d", pIoctlInfo->u16CmdTimeoutInMs);
    dbg("  lu8Command       :");
    for (i=0; i<pIoctlInfo->u8LenCommand; ++i)
    {
      uint8_t iByte = pIoctlInfo->lu8Command[i];
      char    c     = ' ';
      if (iByte >= 32 && iByte <= 126) c = iByte;
      dbg("    [%d] 0x%x  %d  \'%c\'", i, iByte, iByte, c);
    }
  }

  iRetVal = usb_bulk_msg(pDevInfo->udev, pDevInfo->luEpAddr[ENDPOINT_COMMAND],
    pIoctlInfo->lu8Command, pIoctlInfo->u8LenCommand, &iCmdSendLen, pIoctlInfo->u16CmdTimeoutInMs * HZ / 1000);

  if (iRetVal != 0)
  {
    if (iRetVal == -ETIMEDOUT)
      dbg("usb_bulk_msg() timeout: %d", iRetVal);
    else
      dbg("usb_bulk_msg() failed: %d", iRetVal);
    return iRetVal;
  }

  if (iCmdSendLen != pIoctlInfo->u8LenCommand )
  {
    dbg("usb_bulk_msg() send different command length %d != orignal %d", iCmdSendLen, pIoctlInfo->u8LenCommand);
    return -EFAULT;
  }

  return 0;
}

static int oopt_runCommandRet(DeviceInfo* pDevInfo, IoctlInfo* pIoctlInfo, void __user * arg)
{
  int iRetVal = oopt_runCommandNoRet(pDevInfo, pIoctlInfo);
  int iRetLen;

  if (iRetVal != 0)
    return iRetVal;

  iRetVal = usb_bulk_msg(pDevInfo->udev, pDevInfo->luEpAddr[ENDPOINT_COMMAND_RESULT],
    pIoctlInfo->lu8Result, sizeof(pIoctlInfo->lu8Result), &iRetLen, pIoctlInfo->u16RetTimeoutInMs * HZ / 1000);

  if (iRetVal != 0)
  {
    if (iRetVal == -ETIMEDOUT)
      dbg("usb_bulk_msg() timeout: %d", iRetVal);
    else
      dbg("usb_bulk_msg() failed: %d", iRetVal);
    return iRetVal;
  }

  pIoctlInfo->u8LenResult = iRetLen;

  /* if the read was successful, copy the data to userspace */
  iRetVal = copy_to_user(arg, pIoctlInfo, sizeof(*pIoctlInfo));
  if (iRetVal != 0)
  {
    err("copy_to_user() failed");
    return -EFAULT;
  }

  if (debug >= 2)
  {
    int i;
    dbg("IoctlInfo Return:");
    dbg("  u8LenResult      : %d", pIoctlInfo->u8LenResult);
    dbg("  u16RetTimeoutInMs: %d", pIoctlInfo->u16RetTimeoutInMs);
    dbg("  lu8Result       :");
    for (i=0; i<pIoctlInfo->u8LenResult; ++i)
    {
      uint8_t iByte = pIoctlInfo->lu8Result[i];
      char    c     = ' ';
      if (iByte >= 32 && iByte <= 126) c = iByte;
      dbg("    [%d] 0x%x  %d  \'%c\'", i, iByte, iByte, c);
    }
  }

  return 0;
}

static void oopt_waitLastArm(DeviceInfo* pDevInfo)
{
  const int   iArmTimeoutInJiffy = HZ * 0.1; // 0.1 sec

  pDevInfo->eArmLimitType             = LIMIT_NUM;
  pDevInfo->u64FrameLimit             = 0;

  // wait for final arm to complete (from the previous run)
  wait_event_interruptible_timeout(pDevInfo->wait_final_arm, pDevInfo->iArmState == 0, iArmTimeoutInJiffy);

  if (pDevInfo->iArmState != 0)
  {
    dbg("wait for final arm: timeout, state = %d", pDevInfo->iArmState);
    pDevInfo->iArmState = 0;
  }
}

static int oopt_reset_data(DeviceInfo *pDevInfo)
{
  oopt_waitLastArm(pDevInfo);

  dbg("waited urbs");

  pDevInfo->iSpectraBufferIndexRead   = 0;
  pDevInfo->iSpectraBufferIndexWrite  = 0;
  pDevInfo->eWaitState                = WS_NO_WAIT;
  pDevInfo->u64FrameCounter           = 0;
  pDevInfo->u64NumDelayedFrames       = 0;
  pDevInfo->u64NumDiscardFrames       = 0;

  return 0;
}

static void oopt_arm_callback1(struct urb *urb, struct pt_regs *regs )
{
  DeviceInfo*         pDevInfo        = (DeviceInfo*) urb->context;
  SpectraPostheader*  pPostheader     = (SpectraPostheader*) ( pDevInfo->pSpectraBuffer +
                                          pDevInfo->iSpectraBufferIndexWrite * pDevInfo->iSpectraDataSize +
                                          (pDevInfo->iNumDataBulk-1) * OOPT_DATA_BULK_SIZE );
  int                 iRetVal;
  int                 iBulk;

#ifdef __x86_64__
  getnstimeofday((struct timespec*) &pPostheader->tsTimeFrameStart);
#else
  {
  struct timespec tsTime;
  getnstimeofday(&tsTime);
  pPostheader->tsTimeFrameStart.tv_sec  = tsTime.tv_sec;
  pPostheader->tsTimeFrameStart.tv_nsec = tsTime.tv_nsec;
  }
#endif


  if (unlikely(
    urb->status != 0 &&
    !(urb->status == -ENOENT || urb->status == -ECONNRESET) ))
  {
    err("incorrect urb status %d", urb->status);
    pDevInfo->iArmState = 0;
    wake_up_interruptible(&pDevInfo->wait_final_arm);
    return;
  }

  if (debug)
  {
    if (urb->actual_length != urb->transfer_buffer_length)
      err("urb read length (%d) != expected length (%d)", urb->actual_length, urb->transfer_buffer_length);
  }


  pDevInfo->iArmState = 2;
  for (iBulk = 0; iBulk < pDevInfo->iNumDataBulk; ++iBulk)
  {
    iRetVal = usb_submit_urb( pDevInfo->lUrb[iBulk], GFP_ATOMIC );

    if (likely(iRetVal == 0))
      continue;
    /*
     * Special logic: If we set poll to be disabled, we are finishing a data run,
     *   so there is no more external trigger coming. In this case, it is possible
     *   to have suspended urbs, which causes the -EINVAL error.
     *   Hence, we suppress the error report in this case. Otherwise we report the error.
     */
    if(iRetVal != -EINVAL || pDevInfo->iPollEnable)
    {
      err( "usb_submit_urb() failed for bulk %d: %d", iBulk, iRetVal );
    }
  }

  return;
}

static void oopt_arm_callback2(struct urb *urb, struct pt_regs *regs )
{
  const uint8_t   u8Terminator    = 0x69;
  uint8_t*        pUrbId          = (uint8_t*) urb->context;
  int             iUrbId          = *pUrbId;
  DeviceInfo*     pDevInfo        = container_of( pUrbId, DeviceInfo, luUrbId[iUrbId] );
  uint8_t*        pbSpectraBulk;

  if (unlikely(
    urb->status != 0 &&
    !(urb->status == -ENOENT || urb->status == -ECONNRESET) ))
  {
    err("incorrect urb status %d", urb->status);
    pDevInfo->iArmState = 0;
    wake_up_interruptible(&pDevInfo->wait_final_arm);
    return;
  }

  if (likely(urb->actual_length == OOPT_DATA_BULK_SIZE))
  {
    pbSpectraBulk = pDevInfo->pSpectraBuffer + pDevInfo->iSpectraBufferIndexWrite * pDevInfo->iSpectraDataSize
      + iUrbId * OOPT_DATA_BULK_SIZE;
    memcpy(pbSpectraBulk, urb->transfer_buffer, urb->actual_length);

    if (pDevInfo->iUrbOrder == 0)
    {
      SpectraPostheader* pPostheader = (SpectraPostheader*) ( pDevInfo->pSpectraBuffer +
        pDevInfo->iSpectraBufferIndexWrite * pDevInfo->iSpectraDataSize + (pDevInfo->iNumDataBulk-1) * OOPT_DATA_BULK_SIZE );

#ifdef __x86_64__
      getnstimeofday((struct timespec*) &pPostheader->tsTimeFrameFirstData);
#else
      {
      struct timespec tsTime;
      getnstimeofday(&tsTime);
      pPostheader->tsTimeFrameFirstData.tv_sec  = tsTime.tv_sec;
      pPostheader->tsTimeFrameFirstData.tv_nsec = tsTime.tv_nsec;
      }
#endif
    }
  }
  else if (urb->actual_length == 1)
  {
    if ( *(uint8_t*)urb->transfer_buffer != u8Terminator )
      err("bulk [%d] [0] = 0x%x -> not a valid terminator (0x%x)", iUrbId, *(uint8_t*)urb->transfer_buffer, u8Terminator);
  }
  else
    err("bulk [%d] has incorrect transfer length %d", iUrbId, urb->actual_length);

  ++pDevInfo->iUrbOrder;

  if (pDevInfo->iUrbOrder >= pDevInfo->iNumDataBulk)
    oopt_finish_frame(pDevInfo, iUrbId);
}

static void oopt_finish_frame(DeviceInfo* pDevInfo, int iUrbId)
{
  SpectraPostheader* pPostheader = (SpectraPostheader*) ( pDevInfo->pSpectraBuffer +
    pDevInfo->iSpectraBufferIndexWrite * pDevInfo->iSpectraDataSize + (pDevInfo->iNumDataBulk-1) * OOPT_DATA_BULK_SIZE );
  bool               bRearm = (pDevInfo->eArmLimitType == LIMIT_NONE || pDevInfo->u64FrameCounter+1 < pDevInfo->u64FrameLimit);

  pDevInfo->iSpectraBufferIndexWrite = ((pDevInfo->iSpectraBufferIndexWrite+1) % pDevInfo->iNumSpectraInBuf);

  if (bRearm)
    oopt_rearm(pDevInfo);

  pDevInfo->iUrbOrder = 0;
  pPostheader->u64FrameCounter = pDevInfo->u64FrameCounter;
  ++pDevInfo->u64FrameCounter;

  if (
    unlikely(
      pDevInfo->iSpectraBufferIndexWrite != ((pDevInfo->iSpectraBufferIndexRead+1) % pDevInfo->iNumSpectraInBuf)
    ))
  {
    ++pDevInfo->u64NumDelayedFrames;

    if (unlikely(pDevInfo->iSpectraBufferIndexWrite == pDevInfo->iSpectraBufferIndexRead))
    {
      ++pDevInfo->u64NumDiscardFrames;
      pDevInfo->iSpectraBufferIndexRead = ((pDevInfo->iSpectraBufferIndexRead+1) % pDevInfo->iNumSpectraInBuf);
    }
  }

  //dbg("frame %Lu  index write %d read %d over %Lu delay %Lu", pDevInfo->u64FrameCounter,
  //  pDevInfo->iSpectraBufferIndexWrite, pDevInfo->iSpectraBufferIndexRead,
  //  pDevInfo->u64NumDiscardFrames, pDevInfo->u64NumDelayedFrames);

  pPostheader->u64NumDelayedFrames  = pDevInfo->u64NumDelayedFrames;
  pPostheader->u64NumDiscardFrames  = pDevInfo->u64NumDiscardFrames;

#ifdef __x86_64__
  getnstimeofday((struct timespec*) &pPostheader->tsTimeFrameEnd);
#else
  {
  struct timespec tsTime;
  getnstimeofday(&tsTime);
  pPostheader->tsTimeFrameEnd.tv_sec  = tsTime.tv_sec;
  pPostheader->tsTimeFrameEnd.tv_nsec = tsTime.tv_nsec;
  }
#endif

  pDevInfo->eWaitState = WS_DATA_READY;
  wake_up_interruptible(&pDevInfo->wait_single_data);

  if (bRearm)
    return;

  dbg("final arm finished");
  pDevInfo->iArmState = 0;
  wake_up_interruptible(&pDevInfo->wait_final_arm);
}

static int oopt_rearm(DeviceInfo* pDevInfo)
{
  SpectraPostheader*  pPostheader     = (SpectraPostheader*) ( pDevInfo->pSpectraBuffer +
                                          pDevInfo->iSpectraBufferIndexWrite * pDevInfo->iSpectraDataSize +
                                          (pDevInfo->iNumDataBulk-1) * OOPT_DATA_BULK_SIZE );
  int iRetVal;

#ifdef __x86_64__
  getnstimeofday((struct timespec*) &pPostheader->tsTimeFrameInit);
#else
  {
  struct timespec tsTime;
  getnstimeofday(&tsTime);
  pPostheader->tsTimeFrameInit.tv_sec  = tsTime.tv_sec;
  pPostheader->tsTimeFrameInit.tv_nsec = tsTime.tv_nsec;
  }
#endif

  iRetVal = usb_submit_urb( pDevInfo->pUrbArm, GFP_ATOMIC );
  if( unlikely(iRetVal != 0) )
    err( "usb_submit_urb() failed: %d", iRetVal );

  pDevInfo->iArmState = 1;
  return iRetVal;
}


static int oopt_runArm(DeviceInfo* pDevInfo, ArmInfo* pArmInfo, void __user * arg)
{
  int iRetVal;
  if (pArmInfo->iArmLimitType < 0 || pArmInfo->iArmLimitType >= (int) LIMIT_MAX)
  {
    err("Unrecognized arm limit type: %d\n", pArmInfo->iArmLimitType);
    return -EFAULT;
  }

  pDevInfo->iUrbOrder     = 0;
  pDevInfo->eArmLimitType = (ArmLimitType) pArmInfo->iArmLimitType;
  pDevInfo->u64FrameLimit = pArmInfo->u64NumTotalFrames;
  pDevInfo->eWaitState    = WS_WAITING_FOR_DATA;

  dbg("arm type %d  frame# %Lu", pArmInfo->iArmLimitType, pDevInfo->u64FrameLimit);

  iRetVal = oopt_rearm( pDevInfo );
  if( iRetVal != 0 )
  {
    err("oopt_rearm() failed");
    pDevInfo->eWaitState = WS_NO_WAIT;
    return -EFAULT;
  }

  return 0;
}

static int oopt_runUnarm(DeviceInfo* pDevInfo, ArmInfo* pArmInfo, void __user * arg)
{
  int iRetVal;

  oopt_waitLastArm(pDevInfo);

  pArmInfo->u64NumTotalFrames   = pDevInfo->u64FrameCounter;
  pArmInfo->u64NumDelayedFrames = pDevInfo->u64NumDelayedFrames;
  pArmInfo->u64NumDiscardFrames = pDevInfo->u64NumDiscardFrames;

  dbg("unarm");

  iRetVal = oopt_reset_data(pDevInfo);
  if (iRetVal != 0)
  {
    err("oopt_reset_data() failed");
    return -EFAULT;
  }

  /* if the read was successful, copy the data to userspace */
  iRetVal = copy_to_user(arg, pArmInfo, sizeof(*pArmInfo));
  if (iRetVal != 0)
  {
    err("copy_to_user() failed");
    return -EFAULT;
  }

  if (debug)
  {
    dbg("ArmInfo Return:");
    dbg("  u64NumTotalFrames    : %Lu", pArmInfo->u64NumTotalFrames);
    dbg("  u64NumDelayedFrames  : %Lu", pArmInfo->u64NumDelayedFrames);
    dbg("  u64NumDiscardFrames  : %Lu", pArmInfo->u64NumDiscardFrames);
  }

  return 0;
}

static int oopt_runPollEnable(DeviceInfo* pDevInfo, PollInfo* pPollInfo, void __user * arg)
{
  pDevInfo->iPollEnable = pPollInfo->iPollEnable;
  return 0;
}


unsigned int oopt_poll( struct file* file, poll_table* wait )
{
  DeviceInfo* pDevInfo = (DeviceInfo *) file->private_data;
  unsigned int poll_mask = 0;  // Not readable or writable.

  poll_wait( file, &pDevInfo->wait_single_data, wait );

  if( pDevInfo->iPollEnable &&
    pDevInfo->iSpectraBufferIndexRead != pDevInfo->iSpectraBufferIndexWrite )
    poll_mask |= ( POLLIN | POLLRDNORM );

  //dbg("poll mask: 0x%x", poll_mask);
  return poll_mask;
}


static int oopt_runQueryDevice(DeviceInfo* pDevInfo, QueryDeviceInfo* pQueryDeviceInfo, void __user * arg)
{
  int iRetVal;

  pQueryDeviceInfo->iProductId = pDevInfo->iProductId;
  iRetVal                      = copy_to_user(arg, pQueryDeviceInfo, sizeof(*pQueryDeviceInfo));
  if (iRetVal != 0)
  {
    err("copy_to_user() failed");
    return -EFAULT;
  }
  return 0;
}

static ssize_t oopt_read(struct file *file, char *buffer, size_t count, loff_t *ppos)
{
  DeviceInfo* pDevInfo = (DeviceInfo *) file->private_data;
  SpectraPostheader*  pPostheader;
  int                 iNumSpctraAvailable;
  int                 iMaxSpectraInData;
  int                 iRetVal;
  int                 iSpectraBufferIndexReadEnd;

  /* verify that the device wasn't unplugged */
  if (!pDevInfo->present)
  {
    dbg("No Device Present");
    return -ENODEV;
  }

  if (liSimpleRead[pDevInfo->iDeviceIndex] == 1)
    return oopt_read_simple(file, buffer, count, ppos);

  if (liSimpleRead[pDevInfo->iDeviceIndex] == 2)
    return oopt_read_urb(file, buffer, count, ppos);

  iNumSpctraAvailable = ( (pDevInfo->iSpectraBufferIndexWrite + pDevInfo->iNumSpectraInBuf -
    pDevInfo->iSpectraBufferIndexRead) % pDevInfo->iNumSpectraInBuf );

  iMaxSpectraInData   = (int) (count / pDevInfo->iSpectraDataSize);
  if (iMaxSpectraInData > iNumSpctraAvailable)
    iMaxSpectraInData = iNumSpctraAvailable;

  if ( iMaxSpectraInData == 0 )
    return 0;

  pPostheader = (SpectraPostheader*) ( pDevInfo->pSpectraBuffer +
    pDevInfo->iSpectraBufferIndexRead * pDevInfo->iSpectraDataSize + (pDevInfo->iNumDataBulk-1) * OOPT_DATA_BULK_SIZE );

  dbg("In frame %Lu, read frame %Lu index %d size %d", pDevInfo->u64FrameCounter, pPostheader->u64FrameCounter,
    pDevInfo->iSpectraBufferIndexRead, iMaxSpectraInData);

  pPostheader->dataInfo.i32Version           = 0x00010001;
  pPostheader->dataInfo.i8NumSpectraInData   = iMaxSpectraInData;
  pPostheader->dataInfo.i8NumSpectraInQueue  = iNumSpctraAvailable - iMaxSpectraInData;
  pPostheader->dataInfo.i8NumSpectraUnused   = pDevInfo->iNumSpectraInBuf - iNumSpctraAvailable;

  iSpectraBufferIndexReadEnd = pDevInfo->iSpectraBufferIndexRead + iMaxSpectraInData;
  if (iSpectraBufferIndexReadEnd <= pDevInfo->iNumSpectraInBuf)
  {
    const uint8_t*  pSpetraBufferReadStart  = pDevInfo->pSpectraBuffer + pDevInfo->iSpectraBufferIndexRead * pDevInfo->iSpectraDataSize;
    const int       iCopyLen                = pDevInfo->iSpectraDataSize * iMaxSpectraInData;
    iRetVal = copy_to_user(buffer, pSpetraBufferReadStart, iCopyLen);
    if (iRetVal != 0)
    {
      dbg("Write data failed: %d", iRetVal);
      return -EFAULT;
    }
  }
  else
  {
    const uint8_t*  pSpetraBufferReadStart  = pDevInfo->pSpectraBuffer + pDevInfo->iSpectraBufferIndexRead * pDevInfo->iSpectraDataSize;
    int             iCopyLen                = pDevInfo->iSpectraDataSize * (pDevInfo->iNumSpectraInBuf - pDevInfo->iSpectraBufferIndexRead);
    int             iCopyLen2;
    iRetVal = copy_to_user(buffer, pSpetraBufferReadStart, iCopyLen);
    if (iRetVal != 0)
    {
      dbg("Write data part1 failed: %d", iRetVal);
      return -EFAULT;
    }

    iCopyLen2 = pDevInfo->iSpectraDataSize * (iSpectraBufferIndexReadEnd - pDevInfo->iNumSpectraInBuf);
    iRetVal   = copy_to_user(buffer + iCopyLen, pDevInfo->pSpectraBuffer, iCopyLen2);
    if (iRetVal != 0)
    {
      dbg("Write data part2 failed: %d", iRetVal);
      return -EFAULT;
    }
  }

  pDevInfo->iSpectraBufferIndexRead = (iSpectraBufferIndexReadEnd % pDevInfo->iNumSpectraInBuf);
  return pDevInfo->iSpectraDataSize * iMaxSpectraInData;
}

static ssize_t oopt_read_urb(struct file *file, char *buffer, size_t count, loff_t *ppos)
{
  const int       iReadTimeoutInJiffy = HZ * 4; // 4 sec
  DeviceInfo*     pDevInfo          = (DeviceInfo *) file->private_data;
  int             iRetVal;
  struct timespec ts0, ts1;

  getnstimeofday(&ts0);
  if (count < pDevInfo->iSpectraDataSize)
  {
    dbg("Buffer size %ld is not big enough to hold the device data size %ld", count, (long) pDevInfo->iSpectraDataSize);
    return -EFAULT;
  }

  iRetVal = oopt_reset_data(pDevInfo);
  if (iRetVal != 0)
  {
    err("oopt_reset_data() failed");
    return -EFAULT;
  }

  pDevInfo->iUrbOrder     = 0;
  pDevInfo->eArmLimitType = LIMIT_NUM;
  pDevInfo->u64FrameLimit = 1;
  pDevInfo->eWaitState    = WS_WAITING_FOR_DATA;

  iRetVal = oopt_rearm( pDevInfo );
  if( iRetVal != 0 )
  {
    pDevInfo->eWaitState = WS_NO_WAIT;
    return -EFAULT;
  }

  wait_event_interruptible_timeout(pDevInfo->wait_single_data, pDevInfo->eWaitState != WS_WAITING_FOR_DATA, iReadTimeoutInJiffy);
  dbg("read wait finished. Urb order = %d. Wait State = %d", pDevInfo->iUrbOrder, pDevInfo->eWaitState);

  if ( pDevInfo->eWaitState == WS_DATA_READY)
  {
    /* if the read was successful, copy the data to userspace */
    uint8_t* pbSpectraBulk = pDevInfo->pSpectraBuffer + pDevInfo->iSpectraBufferIndexRead * pDevInfo->iSpectraDataSize;
    pDevInfo->iSpectraBufferIndexRead = ((pDevInfo->iSpectraBufferIndexRead+1) %  pDevInfo->iNumSpectraInBuf);

    pDevInfo->eWaitState = WS_NO_WAIT;
    iRetVal = copy_to_user(buffer, pbSpectraBulk, pDevInfo->iSpectraDataSize);
    if (iRetVal != 0)
    {
      err("copy_to_user() failed");
      return -EFAULT;
    }
  }
  else
  {
    pDevInfo->eWaitState = WS_NO_WAIT;

    warn("read timeout or interrupted");
    return -EFAULT;
  }

  getnstimeofday(&ts1);
  {
    int iTimeFunc = (ts1.tv_sec - ts0.tv_sec) * 1000000 + (ts1.tv_nsec - ts0.tv_nsec ) / 1000;
    dbg("read index = %d time = %d us", pDevInfo->iSpectraBufferIndexRead, iTimeFunc);
  }

  return pDevInfo->iSpectraDataSize;
}

static ssize_t oopt_read_simple(struct file *file, char *buffer, size_t count, loff_t *ppos)
{
  const int       iCmdTimeoutInJiffy      = HZ * 0.1; // 0.1 sec
  const int       iRetTimeoutInJiffy      = HZ * 0.1; // 0.1 sec
  const int       iRetFirstTimeoutInJiffy = HZ * 0.1; // 0.1 sec

  const int       iNumFirst2kBulk   = 4;
  const int       iMaxTry           = 5;

  DeviceInfo*     pDevInfo          = (DeviceInfo *) file->private_data;
  uint8_t         lu8CommandRead[]  = { 0x09 };
  int             iTry              = 0;
  int             iTotalTry         = 0;
  char*           pbSpectraBulk;

  int             iCmdSendLen;
  int             iRetVal;
  int             iCommand;
  int             iBulk;

  if (count < pDevInfo->iSpectraDataSize)
  {
    dbg("Buffer size %ld is not big enough to hold the device data size %d", count, pDevInfo->iSpectraDataSize);
    return -EFAULT;
  }

  //iRetVal = discardSpectra(pDevInfo);
  //if (iRetVal != 0)
  //{
  //  dbg("discardSpectra() failed\n");
  //  return iRetVal;
  //}

  iCommand = lu8CommandRead[0];
  liIoctlCommand[pDevInfo->iDeviceIndex] = iCommand;

  iRetVal = usb_bulk_msg(pDevInfo->udev, pDevInfo->luEpAddr[ENDPOINT_COMMAND],
    lu8CommandRead, sizeof(lu8CommandRead), &iCmdSendLen, iCmdTimeoutInJiffy);

  if (iRetVal != 0)
  {
    if (iRetVal == -ETIMEDOUT)
      dbg("usb_bulk_msg() timeout: %d", iRetVal);
    else
      dbg("usb_bulk_msg() failed: %d", iRetVal);
    return iRetVal;
  }

  if (iCmdSendLen != sizeof(lu8CommandRead) )
  {
    dbg("usb_bulk_msg() send different command length %d != orignal %ld", iCmdSendLen, sizeof(lu8CommandRead));
    return -EFAULT;
  }

  pbSpectraBulk = pDevInfo->pSpectraBuffer;
  for (iBulk = 0; iBulk < pDevInfo->iNumDataBulk; ++iBulk, pbSpectraBulk += OOPT_DATA_BULK_SIZE)
  {
    int iEndpSpectra    = (iBulk < iNumFirst2kBulk ? pDevInfo->luEpAddr[pDevInfo->iEpFirst2K] : pDevInfo->luEpAddr[ENDPOINT_DATA]);
    int iTimeoutInJiffy = (iBulk == 0? iRetFirstTimeoutInJiffy: iRetTimeoutInJiffy);
    int iRetLen;

    /* do a blocking bulk read to get data from the device */
    iRetVal = usb_bulk_msg(pDevInfo->udev, iEndpSpectra, pbSpectraBulk, OOPT_DATA_BULK_SIZE, &iRetLen, iTimeoutInJiffy);
    if (iRetVal < 0)
    {
      if( iRetVal == -ETIMEDOUT )
      {
        ++iTotalTry;
        if ( ++iTry >= iMaxTry )
        {
          dbg("usb_bulk_read() max timeout reached for bulk %d", iBulk);
          break;
        }

        --iBulk;
        pbSpectraBulk -= OOPT_DATA_BULK_SIZE;
        continue;
      }
      dbg("usb_bulk_read() failed at Bulk %02d : %d", iBulk, iRetVal);
      iRetVal = -EFAULT;
      goto error;
    }
    iTry = 0;

    if (iRetLen != OOPT_DATA_BULK_SIZE )
    {
      if ( iBulk == pDevInfo->iNumDataBulk -1 && iRetLen == 1 )
      {
        if (pbSpectraBulk[0] != 0x69)
          dbg("Invalid Spectra bulk %d end byte : 0x%x", iBulk, pbSpectraBulk[0]);
      }
      else
        dbg("Invalid Spectra bulk %d size: %d", iBulk, iRetLen);
    }
  }

  if (iTry > 0)
    dbg("Total Try: %d", iTotalTry);

  if (iTry >= iMaxTry)
  {
    iRetVal = -EFAULT;
    goto error;
  }

  /* if the read was successful, copy the data to userspace */
  iRetVal = copy_to_user(buffer, pDevInfo->pSpectraBuffer, pDevInfo->iSpectraDataSize);
  if (iRetVal != 0) goto error;

  return pDevInfo->iSpectraDataSize;

error:
  return iRetVal;
}

//static int discardSpectra(DeviceInfo* pDevInfo)
//{
//  const int iNumTotalBulk           = 20;
//  const int iNumFirst2kBulk         = 5;
//  const int iRetFirstTimeoutInJiffy = HZ * 0.005; // 5 ms
//  const int iRetTimeoutInJiffy      = HZ * 0.005; // 5 ms
//  int       iBulk;
//  int       iRetVal;
//  uint8_t*  pbSpectraBulk = pDevInfo->pSpectraBuffer;

//  for (iBulk = 0; iBulk < iNumTotalBulk; ++iBulk, pbSpectraBulk += OOPT_DATA_BULK_SIZE)
//  {
//    int iEndpSpectra    = (iBulk < iNumFirst2kBulk ? pDevInfo->luEpAddr[ENDPOINT_DATA_FIRST2K] : pDevInfo->luEpAddr[ENDPOINT_DATA_REMAIN]);
//    int iTimeoutInJiffy = (iBulk == 0? iRetFirstTimeoutInJiffy: iRetTimeoutInJiffy);
//    int iRetLen;

//    /* do a blocking bulk read to get data from the device */
//    iRetVal = usb_bulk_msg(pDevInfo->udev, iEndpSpectra, pbSpectraBulk, OOPT_DATA_BULK_SIZE, &iRetLen, iTimeoutInJiffy);
//    if (iRetVal < 0)
//    {
//      if( iRetVal == -ETIMEDOUT )
//        continue;

//      dbg("usb_bulk_read() failed at Bulk %02d : %d", iBulk, iRetVal);
//      return -EFAULT;
//    }

//    if (iRetLen != OOPT_DATA_BULK_SIZE )
//    {
//      if ( iBulk == iNumTotalBulk -1 && iRetLen == 1 )
//      {
//        if (pbSpectraBulk[0] != 0x69)
//          dbg("Invalid Spectra bulk %d end byte : 0x%x", iBulk, pbSpectraBulk[0]);
//      }
//      else
//        dbg("Invalid Spectra bulk %d size: %d", iBulk, iRetLen);
//    }
//    dbg("discarding spctra %d", iBulk);
//  }

//  return 0;
//}

module_init(oopt_init);
module_exit(oopt_exit);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL v2");
