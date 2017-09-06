/* ooptDriver.h */

#include <linux/ioctl.h>
#include <linux/version.h>
#include <linux/poll.h>

/* Version Information */
#define DRIVER_VERSION        "V1.1.8"
#define DRIVER_AUTHOR         "SLAC National Acceleratory Laboratory"
#define DRIVER_DESC           "Ocean Optics USB2.0 Device Driver for Linux"
/* Define these values to match your devices */
#define VENDOR_ID             0x2457
#define PRODUCT_ID_HR4000     0x1012
#define PRODUCT_ID_USB4000    0x1022
#define PRODUCT_ID_USB2000P   0x101E
/* Get a minor range for your devices from the usb maintainer */
#ifdef CONFIG_USB_DYNAMIC_MINORS
#define OOPT_MINOR_BASE       0
#else
#define OOPT_MINOR_BASE       128
#endif

/* The big kernel lock has been removed for newer versions of linux */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,39)
#define OOPT_NO_BKL
#endif

/*
 * Maximal number of OOPT devices supported by this driver
 */
#define MAX_OOPT_DEVICES    8

//#define OOPT_SPECTRA_PAGEORDER      1  // Each spectra contains 2^(OOPT_SPECTRA_PAGE_ORDER) pages
//#define OOPT_SPECTRA_PAGESIZE       (1<<OOPT_SPECTRA_PAGEORDER)
//#define OOPT_SEPCTRA_DATASIZE       (PAGE_SIZE * OOPT_SPECTRA_PAGESIZE)
//#define OOPT_SEPCTRA_DATASIZE       4608 // 2048 * 2 + 512
#define OOPT_SEPCTRA_MAXDATASIZE    8192


//#define OOPT_SPECTRABUF_COUNTORDER  6  // The spctra ring buffer contrains 2^(OOPT_SPECTRA_RING_ORDER) spectrum
#define OOPT_SPECTRABUF_PAGEORDER   7
#define OOPT_SPECTRABUF_SIZE        ((1<<OOPT_SPECTRABUF_PAGEORDER) * PAGE_SIZE)
//#define OOPT_SPECTRABUF_COUNT     (1<<OOPT_SPECTRABUF_COUNTORDER)

enum Device_Endpoint
{
  ENDPOINT_COMMAND        = 0,
  ENDPOINT_DATA           = 1,
  ENDPOINT_RESERVED       = 2,
  ENDPOINT_COMMAND_RESULT = 3,
  ENDPOINT_NUMBER         = 4
};

#define OOPT_DATA_BULK_SIZE    512
#define OOPT_CMD_BULK_SIZE     64
#define OOPT_DATA_BULK_MAXNUM  (OOPT_SEPCTRA_MAXDATASIZE / OOPT_DATA_BULK_SIZE)

struct tagIoctlInfo;  // forward declaration

typedef enum
{
  WS_NO_WAIT          = 0,
  WS_WAITING_FOR_DATA = 1,
  WS_DATA_READY       = 2,
} WaitState;

typedef enum
{
  LIMIT_NONE = 0,
  LIMIT_NUM  = 1,
  LIMIT_MAX  = 2,
} ArmLimitType;

typedef struct tagDeviceInfo
{
    struct  kref            kref;
    int                     iProductId;               /*0x1012 for HR4000, 0x101e for USB2000+*/
    unsigned char           minor;
    struct usb_device*      udev;
    struct usb_interface*   interface;
    int                     present;                  /* if the device is connected */
    int                     iDeviceIndex;             /* serial number among all OOPT devices */
    int                     iSpectraDataSize;
    int                     iNumDataBulk;
    int                     iNumSpectraInBuf;
    unsigned int            luEpAddr[ENDPOINT_NUMBER];/* index defined in enum Device_Endpoint */
    int                     iEpFirst2K;
    struct tagIoctlInfo*    pIoctlInfo;               /* buffer for exchanging Iotctl commands */
    uint8_t*                pSpectraBuffer;           /* buffer for capturing spectra frames */
    int                     iSpectraBufferIndexRead;  /* frame index for buffer reading */
    int                     iSpectraBufferIndexWrite;

    int                     iUrbOrder;
    struct urb*             pUrbArm;
    uint8_t                 luUrbId     [OOPT_DATA_BULK_MAXNUM];
    struct urb*             lUrb        [OOPT_DATA_BULK_MAXNUM];

    wait_queue_head_t       wait_single_data;
    wait_queue_head_t       wait_final_arm;
    WaitState               eWaitState;

    uint64_t                u64FrameCounter;
    uint64_t                u64FrameLimit;
    ArmLimitType            eArmLimitType;            /* 0: No limit, 1: Limit by frame number */
    int                     iArmState;                /* set by urb callbacks. 0-> no arm. 1-> arm */
    int                     iPollEnable;              /* 0-> no poll. 1-> enable poll*/

    uint64_t                u64NumDelayedFrames;
    uint64_t                u64NumDiscardFrames;
} DeviceInfo;

/*
 * IOCTL command and data structure for the argument
 */

#define OOPT_IOCTL_MAGIC      'k' // 0x6b , not used by official drivers
#define OOPT_IOCTL_BASE       128
#define OOPT_IOCTL_CMD_NORET  _IOW ( OOPT_IOCTL_MAGIC, OOPT_IOCTL_BASE + 0, IoctlInfo )
#define OOPT_IOCTL_CMD_RET    _IOWR( OOPT_IOCTL_MAGIC, OOPT_IOCTL_BASE + 1, IoctlInfo )
#define OOPT_IOCTL_ARM        _IOWR( OOPT_IOCTL_MAGIC, OOPT_IOCTL_BASE + 2, ArmInfo )
#define OOPT_IOCTL_UNARM      _IOWR( OOPT_IOCTL_MAGIC, OOPT_IOCTL_BASE + 3, ArmInfo )
#define OOPT_IOCTL_POLLENABLE _IOWR( OOPT_IOCTL_MAGIC, OOPT_IOCTL_BASE + 4, PollInfo )
#define OOPT_IOCTL_DEVICEINFO _IOWR( OOPT_IOCTL_MAGIC, OOPT_IOCTL_BASE + 5, QueryDeviceInfo )

#pragma pack(4)

typedef struct tagIoctlInfo
{
  uint8_t   u8LenCommand;
  uint8_t   u8LenResult;
  uint16_t  u16CmdTimeoutInMs;
  uint16_t  u16RetTimeoutInMs;
  uint8_t   lu8Command[OOPT_CMD_BULK_SIZE];
  uint8_t   lu8Result [OOPT_CMD_BULK_SIZE];
} IoctlInfo;

typedef struct
{
  int       iArmLimitType;
  uint64_t  u64NumTotalFrames;
  uint64_t  u64NumDelayedFrames;
  uint64_t  u64NumDiscardFrames;
} ArmInfo;

typedef struct
{
  int       iPollEnable;
} PollInfo;

typedef struct
{
  int       iProductId;
} QueryDeviceInfo;

#pragma pack()

static int  oopt_probe        (struct usb_interface *interface, const struct usb_device_id *id);
static void oopt_disconnect   (struct usb_interface *interface);
static int  oopt_open         (struct inode *inode, struct file *file);
static int  oopt_release      (struct inode *inode, struct file *file);
static int  oopt_ioctl        (struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg);
static long oopt_compat_ioctl (struct file *file, unsigned int cmd, unsigned long arg);
static ssize_t oopt_read      (struct file *file, char *buffer, size_t count, loff_t *ppos);
static unsigned oopt_poll( struct file* file, poll_table* wait );

static struct file_operations oopt_fops = {
  /*
   * The owner field is part of the module-locking
   * mechanism. The idea is that the kernel knows
   * which module to increment the use-counter of
   * BEFORE it calls the device's open() function.
   * This also means that the kernel can decrement
   * the use-counter again before calling release()
   * or should the open() function fail.
   */
  .owner   = THIS_MODULE,
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35))
  .ioctl   = oopt_ioctl,
#else
  .unlocked_ioctl = oopt_compat_ioctl,
#endif
  .open    = oopt_open,
  .release = oopt_release,
  .read    = oopt_read,
  .poll    = oopt_poll,
  .compat_ioctl
           = oopt_compat_ioctl,
};

/*
 * usb class driver info in order to get a minor number from the usb core,
 * and to have the device registered with devfs and the driver core
 */
static struct usb_class_driver oopt_class = {
  .name =   "usb/oopt%d",
  .fops =   &oopt_fops,
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,15)
  .mode =   S_IFCHR | S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH,
#endif
  .minor_base = OOPT_MINOR_BASE,
};

/* table of devices that work with this driver */
static struct usb_device_id oopt_device_table [] = {
  { USB_DEVICE( VENDOR_ID, PRODUCT_ID_HR4000   ) },
  { USB_DEVICE( VENDOR_ID, PRODUCT_ID_USB2000P ) },
  { USB_DEVICE( VENDOR_ID, PRODUCT_ID_USB4000  ) },
  { }         /* Terminating entry */
};
MODULE_DEVICE_TABLE (usb, oopt_device_table);
/* usb specific object needed to register this driver with the usb subsystem */
static struct usb_driver oopt_driver = {
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,16)
  .owner      = THIS_MODULE,
#endif
  .name       = "OOPT",
  .probe      = oopt_probe,
  .disconnect = oopt_disconnect,
  .id_table   = oopt_device_table,
};

#pragma pack(4)

typedef struct
{
  int32_t i32Version;
  int8_t  i8NumSpectraInData;
  int8_t  i8NumSpectraInQueue;
  int8_t  i8NumSpectraUnused;
  int8_t  iReserved1;
} DataInfo;

typedef struct
{
  uint64_t tv_sec;
  uint64_t tv_nsec;
} timespec64;

typedef struct
{
  uint64_t          u64FrameCounter;    // count from 0
  uint64_t          u64NumDelayedFrames;
  uint64_t          u64NumDiscardFrames;
  timespec64        tsTimeFrameInit;
  timespec64        tsTimeFrameStart;
  timespec64        tsTimeFrameFirstData;
  timespec64        tsTimeFrameEnd;
  DataInfo          dataInfo;
} SpectraPostheader;

#pragma pack()
