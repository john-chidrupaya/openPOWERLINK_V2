/**
********************************************************************************
\file   main.c

\brief  main file for Linux kernel PCIe interface module

This file contains the main part of the Linux kernel PCIe interface module
for the openPOWERLINK kernel stack.

\ingroup module_driver_linux_kernel_pcie
*******************************************************************************/
/*------------------------------------------------------------------------------
Copyright (c) 2014, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
Copyright (c) 2015, Kalycito Private Limited
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holders nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
------------------------------------------------------------------------------*/

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/version.h>
#include <linux/mm.h>
#include <asm/uaccess.h>
#include <asm/page.h>
#include <asm/atomic.h>
#include <linux/kthread.h>
#include <linux/delay.h>

#include <common/driver.h>
#include <common/memmap.h>
#include <drvintf.h>
#include <pcieDrv.h>
//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("chidrupaya.sr@kalycito.com");
MODULE_DESCRIPTION("openPOWERLINK pcie driver");

// VM_RESERVED is removed in kernels > 3.7
#ifndef VM_RESERVED
#define VM_RESERVED    (VM_DONTEXPAND | VM_DONTDUMP)
#endif

//------------------------------------------------------------------------------
// module global vars
//------------------------------------------------------------------------------

int                     plkMajor_g = 0;
int                     plkMinor_g = 0;
int                     plkNrDevs_g = 1;
dev_t                   plkDev_g;
struct class*           plkClass_g;
struct cdev             plkCdev_g;
atomic_t                openCount_g;

//------------------------------------------------------------------------------
// global function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P R I V A T E   D E F I N I T I O N S                           //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define QUEUE_WAIT_TIMEOUT          (10 * HZ / 1000)    // 10ms timeout
#define K2U_EVENT_WAIT_TIMEOUT      (500 * HZ / 1000)

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------

typedef struct
{
    wait_queue_head_t       userWaitQueue;
    UINT8                   aK2URxBuffer[sizeof(tEvent) + MAX_EVENT_ARG_SIZE];
    ULONG                   pdoBufOffset;
    BYTE*                   pPdoMem;
    size_t                  pdoMemSize;
    BOOL                    fSyncEnabled;
} tDrvInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
tDrvInstance    instance_l;                                 // Instance of this driver
static BYTE     aAsyncFrameSwapBuf_l[C_DLL_MAX_ASYNC_MTU];  // Array to store ASync frame data
//FIXME Update the mmap call handling in this driver so that the above swap array
// is not required as there would no copying of data anymore.
//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

static int __init   plkIntfInit(void);
static void __exit  plkIntfExit(void);

static int          plkIntfOpen(struct inode* pDeviceFile_p, struct file* pInstance_p);
static int          plkIntfRelease(struct inode* pDeviceFile_p, struct file* pInstance_p);
static ssize_t      plkIntfRead(struct file* pInstance_p, char* pDstBuff_p, size_t BuffSize_p, loff_t* pFileOffs_p);
static ssize_t      plkIntfWrite(struct file* pInstance_p, const char* pSrcBuff_p, size_t BuffSize_p, loff_t* pFileOffs_p);
#ifdef HAVE_UNLOCKED_IOCTL
static long         plkIntfIoctl(struct file* filp, unsigned int cmd, unsigned long arg);
#else
static int          plkIntfIoctl(struct inode* dev, struct file* filp, unsigned int cmd, unsigned long arg);
#endif

static int          plkIntfMmap(struct file* filp, struct vm_area_struct* vma);
static void         plkIntfVmaOpen(struct vm_area_struct* vma);
static void         plkIntfVmaClose(struct vm_area_struct* vma);

static int          executeCmd(unsigned long arg_p);
static int          readInitParam(unsigned long arg_p);
static int          storeInitParam(unsigned long arg_p);
static int          getStatus(unsigned long arg_p);
static int          getHeartbeat(unsigned long arg);
static int          sendAsyncFrame(unsigned long arg);
static int          writeErrorObject(unsigned long arg);
static int          readErrorObject(unsigned long arg);

static int          getEventForUser(unsigned long arg_p);
static int          postEventFromUser(unsigned long arg);

static int          mapMemoryForUserIoctl(unsigned long arg_p);
//static int          mapMemoryForUserMmap(BYTE** ppUserBuf_p, ULONGLONG* pKernelBuf_p);

//------------------------------------------------------------------------------
//  Kernel module specific data structures
//------------------------------------------------------------------------------
module_init(plkIntfInit);
module_exit(plkIntfExit);

static struct file_operations powerlinkFileOps_g =
{
    .owner =     THIS_MODULE,
    .open =      plkIntfOpen,
    .release =   plkIntfRelease,
    .read =      plkIntfRead,
    .write =     plkIntfWrite,
#ifdef HAVE_UNLOCKED_IOCTL
    .unlocked_ioctl = plkIntfIoctl,
#else
    .ioctl =     plkIntfIoctl,
#endif
    .mmap =      plkIntfMmap,
};

static struct vm_operations_struct powerlinkVmOps =
{
    .open = plkIntfVmaOpen,
    .close = plkIntfVmaClose,
};

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  module initialization

The function implements openPOWERLINK kernel pcie interface module
initialization function.

\ingroup module_driver_linux_kernel_pcie
*/
//------------------------------------------------------------------------------
static int __init plkIntfInit(void)
{
    int    err;

    DEBUG_LVL_ALWAYS_TRACE("PLK: plkIntfInit()  Driver build: %s / %s\n", __DATE__, __TIME__);
    DEBUG_LVL_ALWAYS_TRACE("PLK: plkIntfInit()  Stack version: %s\n", PLK_DEFINED_STRING_VERSION);
    plkDev_g = 0;
    atomic_set(&openCount_g, 0);

    if ((err = alloc_chrdev_region(&plkDev_g, plkMinor_g, plkNrDevs_g, PLK_DRV_NAME)) < 0)
    {
        DEBUG_LVL_ERROR_TRACE("PLK: Failing allocating major number\n");
        return err;
    }

    plkMajor_g = MAJOR(plkDev_g);
    TRACE("Allocated major number: %d\n", plkMajor_g);

    if ((plkClass_g = class_create(THIS_MODULE, PLK_DRV_NAME)) == NULL)
    {
        DEBUG_LVL_ERROR_TRACE("class_create() failed!\n");
        unregister_chrdev_region(plkDev_g, plkNrDevs_g);
        return -1;
    }

    if (device_create(plkClass_g, NULL, plkDev_g, NULL, PLK_DRV_NAME) == NULL)
    {
        DEBUG_LVL_ERROR_TRACE("device_create() failed!\n");
        class_destroy(plkClass_g);
        unregister_chrdev_region(plkDev_g, plkNrDevs_g);
        return -1;
    }

    cdev_init(&plkCdev_g, &powerlinkFileOps_g);
    if ((err = cdev_add(&plkCdev_g, plkDev_g, 1)) == -1)
    {
        DEBUG_LVL_ERROR_TRACE("cdev_add() failed!\n");
        device_destroy(plkClass_g, plkDev_g);
        class_destroy(plkClass_g);
        unregister_chrdev_region(plkDev_g, plkNrDevs_g);
        return -1;
    }

    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  module clean up and exit

The function implements openPOWERLINK kernel pcie interface module exit function.

\ingroup module_driver_linux_kernel_pcie
*/
//------------------------------------------------------------------------------
static void __exit plkIntfExit(void)
{
    DEBUG_LVL_ALWAYS_TRACE("PLK: plkIntfExit...\n");

    cdev_del(&plkCdev_g);
    device_destroy(plkClass_g, plkDev_g);
    class_destroy(plkClass_g);
    unregister_chrdev_region(plkDev_g, plkNrDevs_g);

    DEBUG_LVL_ALWAYS_TRACE("PLK: Driver '%s' removed.\n", PLK_DRV_NAME);
}

//------------------------------------------------------------------------------
/**
\brief  openPOWERLINK pcie driver open function

The function implements openPOWERLINK kernel pcie interface module open function.

\ingroup module_driver_linux_kernel_pcie
*/
//------------------------------------------------------------------------------
static int plkIntfOpen(struct inode* pDeviceFile_p, struct file* pInstance_p)
{
    DEBUG_LVL_ALWAYS_TRACE("PLK: + plkIntfOpen...\n");

    if (atomic_inc_return(&openCount_g) > 1)
    {
        atomic_dec(&openCount_g);
        return -ENOTTY;
    }

    instance_l.pdoBufOffset = 0;
    instance_l.pPdoMem = NULL;
    instance_l.fSyncEnabled = FALSE;

    if (pcieDrv_init() != kErrorOk)
    {
        atomic_dec(&openCount_g);
        return -EIO;
    }

    if (drvintf_init() != kErrorOk)
    {
        atomic_dec(&openCount_g);
        return -EIO;
    }

    init_waitqueue_head(&instance_l.userWaitQueue);

    DEBUG_LVL_ALWAYS_TRACE("PLK: + plkIntfOpen - OK\n");

    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  openPOWERLINK pcie driver close function

The function implements openPOWERLINK kernel module close function.

\ingroup module_driver_linux_kernel_pcie
*/
//------------------------------------------------------------------------------
static int  plkIntfRelease(struct inode* pDeviceFile_p, struct file* pInstance_p)
{
    DEBUG_LVL_ALWAYS_TRACE("PLK: + plkIntfRelease...\n");

    instance_l.pdoBufOffset = 0;
    instance_l.pPdoMem = NULL;
    instance_l.fSyncEnabled = FALSE;

    drvintf_exit();
    pcieDrv_shutdown();
    atomic_dec(&openCount_g);
    DEBUG_LVL_ALWAYS_TRACE("PLK: + plkIntfRelease - OK\n");
    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  openPOWERLINK pcie driver read function

The function implements openPOWERLINK kernel pcie interface module read function.

\ingroup module_driver_linux_kernel_pcie
*/
//------------------------------------------------------------------------------
static ssize_t plkIntfRead(struct file* pInstance_p, char* pDstBuff_p,
                             size_t BuffSize_p, loff_t* pFileOffs_p)
{
    int    ret;

    DEBUG_LVL_ALWAYS_TRACE("PLK: + plkIntfRead...\n");
    DEBUG_LVL_ALWAYS_TRACE("PLK:   Sorry, this operation isn't supported.\n");
    ret = -EINVAL;
    DEBUG_LVL_ALWAYS_TRACE("PLK: - plkIntfRead (iRet=%d)\n", ret);
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  openPOWERLINK pcie driver write function

The function implements openPOWERLINK kernel pcie interface module write function.

\ingroup module_driver_linux_kernel_pcie
*/
//------------------------------------------------------------------------------
static ssize_t plkIntfWrite(struct file* pInstance_p, const char* pSrcBuff_p,
                              size_t BuffSize_p, loff_t* pFileOffs_p)
{
    int    ret;

    DEBUG_LVL_ALWAYS_TRACE("PLK: + plkIntfWrite...\n");
    DEBUG_LVL_ALWAYS_TRACE("PLK:   Sorry, this operation isn't supported.\n");
    ret = -EINVAL;
    DEBUG_LVL_ALWAYS_TRACE("PLK: - plkIntfWrite (iRet=%d)\n", ret);
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  openPOWERLINK pcie driver ioctl function

The function implements openPOWERLINK kernel pcie interface module ioctl function.

\ingroup module_driver_linux_kernel_pcie
*/
//------------------------------------------------------------------------------
#ifdef HAVE_UNLOCKED_IOCTL
static long plkIntfIoctl(struct file* filp, unsigned int cmd,
                           unsigned long arg)
#else
static int  plkIntfIoctl(struct inode* dev, struct file* filp,
                           unsigned int cmd, unsigned long arg)
#endif
{
    int             ret = -EINVAL;
    tOplkError      oplRet;

    switch (cmd)
    {
        case PLK_CMD_CTRL_EXECUTE_CMD:
            ret = executeCmd(arg);
            break;

        case PLK_CMD_CTRL_STORE_INITPARAM:
            ret = storeInitParam(arg);
            break;

        case PLK_CMD_CTRL_READ_INITPARAM:
            ret = readInitParam(arg);
            break;

        case PLK_CMD_CTRL_GET_STATUS:
            ret = getStatus(arg);
            break;

        case PLK_CMD_CTRL_GET_HEARTBEAT:
            ret = getHeartbeat(arg);
            break;

        case PLK_CMD_POST_EVENT:
            ret = postEventFromUser(arg);
            break;

        case PLK_CMD_GET_EVENT:
            ret = getEventForUser(arg);
            break;

        case PLK_CMD_DLLCAL_ASYNCSEND:
            ret = sendAsyncFrame(arg);
            break;

        case PLK_CMD_ERRHND_WRITE:
            ret = writeErrorObject(arg);
            break;

        case PLK_CMD_ERRHND_READ:
            ret = readErrorObject(arg);
            break;

        case PLK_CMD_PDO_SYNC:
            if (instance_l.fSyncEnabled == FALSE)
            {
                pcieDrv_regSyncHandler(pdokcal_sendSyncEvent);
                pcieDrv_enableSync(TRUE);
                instance_l.fSyncEnabled = TRUE;
            }

            // $$ Handle other errors
            if ((oplRet = drvintf_waitSyncEvent()) == kErrorRetry)
                ret = -ERESTARTSYS;
            else
                ret = 0;

            break;

        case PLK_CMD_MEMMAP_MAP_MEM:
            ret = mapMemoryForUserIoctl(arg);
            break;

        case PLK_CMD_PDO_MAP_OFFSET:
            if (copy_to_user((void __user*)arg, &instance_l.pdoBufOffset, sizeof(ULONG)))
            {
                printk("PDO Offset fetch Error!!\n");
                ret = -EFAULT;
            }
            else
                ret = 0;

            break;

        default:
            DEBUG_LVL_ERROR_TRACE("PLK: - Invalid cmd (cmd=%d type=%d)\n", _IOC_NR(cmd), _IOC_TYPE(cmd));
            ret = -ENOTTY;
            break;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  openPOWERLINK pcie driver mmap function

The function implements openPOWERLINK kernel pcie interface module mmap function.

\ingroup module_driver_linux_kernel_pcie
*/
//------------------------------------------------------------------------------
static int plkIntfMmap(struct file* filp, struct vm_area_struct* vma)
{
    BYTE*           pPdoMem = NULL;
    size_t          memSize = 0;
    tOplkError      ret = kErrorOk;
    ULONG           pfn = 0;

    DEBUG_LVL_ALWAYS_TRACE("%s() vma: vm_start:%lX vm_end:%lX vm_pgoff:%lX\n",
                           __func__, vma->vm_start, vma->vm_end, vma->vm_pgoff);

    vma->vm_flags |= VM_RESERVED | VM_IO;
    vma->vm_ops = &powerlinkVmOps;

    //FIXME Rework the function to map the offset passed from user that the user can get
    // via an ioctl call made before the mmap() call. This way both pdo and memmap modules
    // can use mmap and multiple mmap() calls to this driver would be possible.

    ret = drvintf_getPdoMem(&pPdoMem, &memSize);

    if ((pPdoMem == NULL) || (ret != kErrorOk))
    {
        DEBUG_LVL_ERROR_TRACE("%s() no pdo memory allocated!\n", __func__);
        return -ENOMEM;
    }

    // Get the bus address of the PDO memory
    pfn = pcieDrv_getBarPhyAddr(0) + ((ULONG)pPdoMem - pcieDrv_getBarAddr(0));

    // Save the offset of the PDO memory address from the start of page boundary
    instance_l.pdoBufOffset = (ULONG)(pfn - ((pfn >> PAGE_SHIFT) << PAGE_SHIFT));
    instance_l.pPdoMem = pPdoMem;
    instance_l.pdoMemSize = memSize;

    if (io_remap_pfn_range(vma, vma->vm_start, (pfn >> PAGE_SHIFT),
                           vma->vm_end - vma->vm_start + instance_l.pdoBufOffset - PAGE_SIZE, vma->vm_page_prot))
    {
        DEBUG_LVL_ERROR_TRACE("%s() remap_pfn_range failed\n", __func__);
        return -EAGAIN;
    }

    plkIntfVmaOpen(vma);
    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  openPOWERLINK pcie driver VMA open function

The function implements openPOWERLINK kernel module VMA open function.

\ingroup module_driver_linux_kernel_pcie
*/
//------------------------------------------------------------------------------
static void plkIntfVmaOpen(struct vm_area_struct* vma)
{
    DEBUG_LVL_ALWAYS_TRACE("%s() vma: vm_start:%lX vm_end:%lX vm_pgoff:%lX\n",
                           __func__, vma->vm_start, vma->vm_end, vma->vm_pgoff);
}

//------------------------------------------------------------------------------
/**TRACE
\brief  openPOWERLINK pcie driver VMA close function

The function implements openPOWERLINK kernel module VMA close function.

\ingroup module_driver_linux_kernel_pcie
*/
//------------------------------------------------------------------------------
static void plkIntfVmaClose(struct vm_area_struct* vma)
{
    DEBUG_LVL_ALWAYS_TRACE("%s() vma: vm_start:%lX vm_end:%lX vm_pgoff:%lX\n",
                           __func__, vma->vm_start, vma->vm_end, vma->vm_pgoff);
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief    Get an event for the user layer

This function waits for events to the user.

\param  arg                Ioctl argument. Contains the received event.

\return The function returns Linux error code.
*/
//------------------------------------------------------------------------------
int getEventForUser(ULONG arg_p)
{
    int             ret;
    size_t          readSize;
    int             loopCount = (K2U_EVENT_WAIT_TIMEOUT / QUEUE_WAIT_TIMEOUT);
    int             i = 0;

    for (i = 0; i < loopCount; i++)
    {
        ret = wait_event_interruptible_timeout(instance_l.userWaitQueue, 0, QUEUE_WAIT_TIMEOUT);

        // ignore timeout (ret = 0) condition as we are using it to sleep

        if (ret == -ERESTARTSYS)
        {
            printk("%s() interrupted\n", __func__);
            break;
        }

        if (drvintf_getEvent(instance_l.aK2URxBuffer, &readSize) != kErrorOk)
        {
            ret = -EFAULT;
            break;
        }

        if (readSize > 0)
        {
            DEBUG_INTF("%s() copy kernel event to user: %d Bytes\n", __func__, readSize);
            if (copy_to_user((void __user*)arg_p, instance_l.aK2URxBuffer, readSize))
            {
                printk("Event fetch Error!!\n");
                ret = -EFAULT;
                break;
            }

            ret = 0;
            break;
        }

        // else ignore the rest of the cases as we are not using any condition or signal to wakeup
    }

    if (i == loopCount)
    {
        // No event received from the kernel
        ret = -ERESTARTSYS;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief    Post event from user

This function posts an event from the user layer to the kernel queue.

\param  arg                Ioctl argument. Contains the event to post.

\return The function returns Linux error code.
*/
//------------------------------------------------------------------------------
INT postEventFromUser(ULONG arg)
{
    tOplkError      ret = kErrorOk;
    tEvent          event;
    UINT8*          pArg = NULL;
    INT             order = 0;

    if (copy_from_user(&event, (const void __user*)arg, sizeof(tEvent)))
        return -EFAULT;

    if (event.eventArgSize != 0)
    {
        order = get_order(event.eventArgSize);
        pArg = (UINT8*)__get_free_pages(GFP_KERNEL, order);

        if (!pArg)
            return -EIO;

        if (copy_from_user(pArg, (const void __user*)event.eventArg.pEventArg, event.eventArgSize))
        {
            free_pages((ULONG)pArg, order);
            return -EFAULT;
        }

        event.eventArg.pEventArg = (void*)pArg;
    }

    switch (event.eventSink)
    {
        case kEventSinkNmtk:
        case kEventSinkSync:
        case kEventSinkDllk:
        case kEventSinkDllkCal:
        case kEventSinkPdok:
        case kEventSinkPdokCal:
        case kEventSinkErrk:
            /* TRACE("U2K  type:(%d) sink:(%d) size:%d!\n",
                     event.eventType,
                     event.eventSink,
                     event.eventArgSize); */
            if (drvintf_postEvent(&event) != kErrorOk)
                ret = -EIO;
            break;

        case kEventSinkNmtMnu:
        case kEventSinkNmtu:
        case kEventSinkSdoAsySeq:
        case kEventSinkApi:
        case kEventSinkDlluCal:
        case kEventSinkErru:
        default:
            ret = -EIO;
            break;
    }

    if (event.eventArgSize != 0)
        free_pages((ULONG)pArg, order);

    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  Execute control command ioctl

The function implements the calling of the executeCmd function in the control
module using the ioctl interface and forwards it to the driver using via the
shared memory interface.

\param arg_p    Control command argument passed by the ioctl interface.
*/
//------------------------------------------------------------------------------
static int executeCmd(unsigned long arg_p)
{
    tCtrlCmd    ctrlCmd;

    if (copy_from_user(&ctrlCmd, (const void __user*)arg_p, sizeof(tCtrlCmd)))
        return -EFAULT;

    if (drvintf_executeCmd(&ctrlCmd) != kErrorOk)
        return -EFAULT;

    if (copy_to_user((void __user*)arg_p, &ctrlCmd, sizeof(tCtrlCmd)))
        return -EFAULT;

    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  Store init param ioctl

The function implements the calling of the storeInitParam function in the
control module using the ioctl interface and forwards it to the driver using
via the shared memory interface.

\param arg_p    Control module initialization parameters argument passed by
                the ioctl interface.
*/
//------------------------------------------------------------------------------
static int storeInitParam(unsigned long arg_p)
{
    tCtrlInitParam    initParam;

    if (copy_from_user(&initParam, (const void __user*)arg_p, sizeof(tCtrlInitParam)))
        return -EFAULT;

    if (drvintf_storeInitParam(&initParam) != kErrorOk)
        return -EFAULT;

    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  Read init param ioctl

The function implements the calling of the readInitParam function in the control
module using the ioctl interface and forwards it to the driver using via the
shared memory interface.

\param arg_p    Pointer to the control module initialization parameters
                argument passed by the ioctl interface.
*/
//------------------------------------------------------------------------------
static int readInitParam(unsigned long arg_p)
{
    tCtrlInitParam    initParam;

    if (drvintf_readInitParam(&initParam) != kErrorOk)
        return -EFAULT;

    if (copy_to_user((void __user*)arg_p, &initParam, sizeof(tCtrlInitParam)))
        return -EFAULT;

    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  Get status ioctl

The function implements the calling of the getStatus function in the control
module using the ioctl interface and forwards it to the driver using via the
shared memory interface.

\param arg_p    Pointer to the control status argument passed by the ioctl interface.
*/
//------------------------------------------------------------------------------
static int getStatus(unsigned long arg_p)
{
    UINT16    status;

    if (drvintf_getStatus(&status) != kErrorOk)
        return -EFAULT;

    put_user(status, (unsigned short __user*)arg_p);
    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  Get heartbeat ioctl

The function implements the calling of the getHeartbeat function in the control
module using the ioctl interface and forwards it to the driver using via the
shared memory interface.

\param arg_p    Pointer to the PCP heartbeat argument passed by the ioctl interface.
*/
//------------------------------------------------------------------------------
static int getHeartbeat(unsigned long arg)
{
    UINT16    heartbeat;

    if (drvintf_getHeartbeat(&heartbeat) != kErrorOk)
        return -EFAULT;

    put_user(heartbeat, (unsigned short __user*)arg);
    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  Sending async frame ioctl

The function implements the ioctl used for sending asynchronous frames.

\param arg_p    Pointer to the async send argument passed by the ioctl interface.
*/
//------------------------------------------------------------------------------
static int sendAsyncFrame(unsigned long arg)
{
    BYTE*                   pBuf;
    tIoctlDllCalAsync       asyncFrameInfo;
    int                     order;
    int                     ret = 0;

    order = get_order(C_DLL_MAX_ASYNC_MTU);
    pBuf = (BYTE*)__get_free_pages(GFP_KERNEL, order);

    if (copy_from_user(&asyncFrameInfo, (const void __user*)arg, sizeof(tIoctlDllCalAsync)))
    {
        free_pages((ULONG)pBuf, order);
        return -EFAULT;
    }

    if (copy_from_user(pBuf, (const void __user*)asyncFrameInfo.pData, asyncFrameInfo.size))
    {
        free_pages((ULONG)pBuf, order);
        return -EFAULT;
    }

    asyncFrameInfo.pData = pBuf;
    if (drvintf_sendAsyncFrame((unsigned char*)&asyncFrameInfo) != kErrorOk)
        ret = -EFAULT;

    free_pages((ULONG)pBuf, order);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Write error object ioctl

The function implements the ioctl for writing an error object.

\param arg_p    Pointer to the error object argument passed by the ioctl interface.
*/
//------------------------------------------------------------------------------
static int writeErrorObject(unsigned long arg)
{
    tErrHndIoctl        writeObject;

    if (copy_from_user(&writeObject, (const void __user*)arg, sizeof(tErrHndIoctl)))
        return -EFAULT;

    if (drvintf_writeErrorObject(&writeObject) != kErrorOk)
        return -EFAULT;

    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  Read error object ioctl

The function implements the ioctl for reading error objects.

\param arg_p    Pointer to the error object argument passed by the ioctl interface.
*/
//------------------------------------------------------------------------------
static int readErrorObject(unsigned long arg)
{
    tErrHndIoctl        readObject;

    if (copy_from_user(&readObject, (const void __user*)arg, sizeof(tErrHndIoctl)))
        return -EFAULT;

    if (drvintf_readErrorObject(&readObject) != kErrorOk)
        return -EFAULT;

    if (copy_to_user((void __user*)arg, &readObject, sizeof(tErrHndIoctl)))
        return -EFAULT;

    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  Map PCP memory into user memory

The function implements the ioctl for mapping the PCP memory into user memory.

\param arg_p    Pointer to the memmap instance argument passed by the ioctl interface.
*/
//------------------------------------------------------------------------------
static int mapMemoryForUserIoctl(unsigned long arg_p)
{
    tMemmap         memMapParams;
    int             ret = 0;
    tOplkError      retVal = kErrorOk;
    void*           pMappedUserBuf = NULL;

    //FIXME Rework this function to return the ioremapped PCP memory offset alone,
    // so that mmap() can be called from user layer using the offset. This will avoid
    // the copying of the entire async frame memory.
    copy_from_user(&memMapParams, (const void __user*)arg_p, sizeof(tMemmap));
    retVal = drvintf_mapKernelMem((UINT8*)memMapParams.pKernelBuf,
                                  (UINT8**)&pMappedUserBuf,
                                  (size_t)memMapParams.memSize);

    if (retVal != kErrorOk)
    {
        DEBUG_LVL_ERROR_TRACE("%s() --> Error: Unable to locate shared memory region\n");
        // Let user free its own memory; just return error.
        ret = -EFAULT;
    }
    else
    {
        OPLK_MEMCPY(aAsyncFrameSwapBuf_l, pMappedUserBuf, sizeof(aAsyncFrameSwapBuf_l));
        ret = 0;
    }

    copy_to_user((void __user*)memMapParams.pUserBuf, aAsyncFrameSwapBuf_l, sizeof(aAsyncFrameSwapBuf_l));

    return ret;
}
///\}
