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
#include <asm/page.h>
#include <asm/uaccess.h>
#include <asm/page.h>
#include <asm/atomic.h>

//TODO Remove these headers
#include <oplk/oplk.h>
#include <common/oplkinc.h>
#include <common/driver.h>

#include <common/ctrl.h>
#include <common/ctrlcal-mem.h>
#include <kernel/ctrlk.h>
#include <kernel/ctrlkcal.h>
#include <kernel/dllkcal.h>
#include <kernel/pdokcal.h>

#include <kernel/eventk.h>
#include <kernel/eventkcal.h>
#include <errhndkcal.h>
#include <drvintf.h>
#include <pcieDrv.h>

#include <linux/kthread.h>

#include <linux/delay.h>

#include <common/timer.h>
#include <common/memmap.h>
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

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------

typedef struct
{
    wait_queue_head_t       userWaitQueue;
    BYTE                    aK2URxBuffer[sizeof(tEvent) + MAX_EVENT_ARG_SIZE];
    BYTE*                   pdoBufOffset;
    BYTE*                   pPdoMem;
    BOOL                    fSyncEnabled;
    BYTE*                   pShmMemLocal;
    BYTE*                   pShmMemRemote;
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

static int __init   powerlinkInit(void);
static void __exit  powerlinkExit(void);

static int          powerlinkOpen(struct inode* pDeviceFile_p, struct file* pInstance_p);
static int          powerlinkRelease(struct inode* pDeviceFile_p, struct file* pInstance_p);
static ssize_t      powerlinkRead(struct file* pInstance_p, char* pDstBuff_p, size_t BuffSize_p, loff_t* pFileOffs_p);
static ssize_t      powerlinkWrite(struct file* pInstance_p, const char* pSrcBuff_p, size_t BuffSize_p, loff_t* pFileOffs_p);
#ifdef HAVE_UNLOCKED_IOCTL
static long         powerlinkIoctl(struct file* filp, unsigned int cmd, unsigned long arg);
#else
static int          powerlinkIoctl(struct inode* dev, struct file* filp, unsigned int cmd, unsigned long arg);
#endif

static int          powerlinkMmap(struct file* filp, struct vm_area_struct* vma);
static void         powerlinkVmaOpen(struct vm_area_struct* vma);
static void         powerlinkVmaClose(struct vm_area_struct* vma);

static int          executeCmd(unsigned long arg);
static int          readInitParam(unsigned long arg);
static int          storeInitParam(unsigned long arg);
static int          getStatus(unsigned long arg);
static int          getHeartbeat(unsigned long arg);
static int          sendAsyncFrame(unsigned long arg);
static int          writeErrorObject(unsigned long arg);
static int          readErrorObject(unsigned long arg);

static int          getEventForUser(unsigned long arg);
static int          postEventFromUser(unsigned long arg);

static int          mapMemoryForUserIoctl(unsigned long arg_p);
//static int          mapMemoryForUserMmap(BYTE** ppUserBuf_p, ULONGLONG* pKernelBuf_p);
//TODO Remove this. This is the signature for older(< 2.6.19) kernels' <.nopage> implementation
//struct page*        powerlinkNoPage(struct vm_area_struct *vma, unsigned long address, int *type);
static int          powerlinkNoPage(struct vm_area_struct*vma, struct vm_fault*vmf);

//------------------------------------------------------------------------------
//  Kernel module specific data structures
//------------------------------------------------------------------------------
//FIXME Change this module's function names as this is not the main powerlink module
module_init(powerlinkInit);
module_exit(powerlinkExit);

static struct file_operations           powerlinkFileOps_g =
{
    .owner =     THIS_MODULE,
    .open =      powerlinkOpen,
    .release =   powerlinkRelease,
    .read =      powerlinkRead,
    .write =     powerlinkWrite,
#ifdef HAVE_UNLOCKED_IOCTL
    .unlocked_ioctl = powerlinkIoctl,
#else
    .ioctl =     powerlinkIoctl,
#endif
    .mmap =      powerlinkMmap,
};

static struct vm_operations_struct      powerlinkVmOps =
{
    .open = powerlinkVmaOpen,
    .close = powerlinkVmaClose,
    .fault = powerlinkNoPage,
};

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//---------------------------------------------------------------------------
//  Initailize driver
//---------------------------------------------------------------------------

//------------------------------------------------------------------------------
/**
\brief  module initialization

The function implements openPOWERLINK kernel pcie interface module
initialization function.

\ingroup module_driver_linux_kernel
*/
//------------------------------------------------------------------------------
static int __init powerlinkInit(void)
{
    int    err;

    DEBUG_LVL_ALWAYS_TRACE("PLK: powerlinkInit()  Driver build: %s / %s\n", __DATE__, __TIME__);
    DEBUG_LVL_ALWAYS_TRACE("PLK: powerlinkInit()  Stack version: %s\n", PLK_DEFINED_STRING_VERSION);
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

\ingroup module_driver_linux_kernel
*/
//------------------------------------------------------------------------------
static void __exit powerlinkExit(void)
{
    DEBUG_LVL_ALWAYS_TRACE("PLK: powerlinkExit...\n");

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

\ingroup module_driver_linux_kernel
*/
//------------------------------------------------------------------------------
static int powerlinkOpen(struct inode* pDeviceFile_p, struct file* pInstance_p)
{
    DEBUG_LVL_ALWAYS_TRACE("PLK: + powerlinkOpen...\n");

    if (atomic_inc_return(&openCount_g) > 1)
    {
        atomic_dec(&openCount_g);
        return -ENOTTY;
    }

    instance_l.pdoBufOffset = NULL;
    instance_l.pPdoMem = NULL;
    instance_l.fSyncEnabled = FALSE;
    instance_l.pShmMemLocal = NULL;
    instance_l.pShmMemRemote = NULL;

    if (pcieDrv_init() != kErrorOk)
    {
        atomic_dec(&openCount_g);
        return -EIO;
    }

    if (drv_initDualProcDrv() != kErrorOk)
    {
        atomic_dec(&openCount_g);
        return -EIO;
    }

    init_waitqueue_head(&instance_l.userWaitQueue);

    DEBUG_LVL_ALWAYS_TRACE("PLK: + powerlinkOpen - OK\n");

    return 0;
}

//------------------------------------------------------------------------------
/**
\brief


\ingroup module_driver_linux_kernel
*/
//------------------------------------------------------------------------------
static int powerlinkNoPage(struct vm_area_struct*vma, struct vm_fault*vmf)
{
    struct page*        page;
    unsigned long       pfn;

    printk("%s() --> vma_start: 0x%X, vma_end: 0x%X, vma_pgoff: 0x%X, vmf_pgoff: 0x%X, vmf_page: 0x%X\n",
           __func__, (UINT)vma->vm_start, (UINT)vma->vm_end, (UINT)vma->vm_pgoff, (UINT)vmf->pgoff, (UINT)vmf->page);
    pfn = virt_to_phys(instance_l.pPdoMem);
    page = virt_to_page(instance_l.pPdoMem);
    vmf->page = page + (vmf->pgoff << PAGE_SHIFT);
    get_page(vmf->page);

    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  openPOWERLINK pcie driver close function

The function implements openPOWERLINK kernel module close function.

\ingroup module_driver_linux_kernel
*/
//------------------------------------------------------------------------------
static int  powerlinkRelease(struct inode* pDeviceFile_p, struct file* pInstance_p)
{
    DEBUG_LVL_ALWAYS_TRACE("PLK: + powerlinkRelease...\n");

    instance_l.pdoBufOffset = NULL;
    instance_l.pPdoMem = NULL;
    instance_l.fSyncEnabled = FALSE;
    instance_l.pShmMemLocal = NULL;
    instance_l.pShmMemRemote = NULL;
    drv_exitDualProcDrv();
    pcieDrv_shutdown();
    atomic_dec(&openCount_g);
    DEBUG_LVL_ALWAYS_TRACE("PLK: + powerlinkRelease - OK\n");
    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  openPOWERLINK pcie driver read function

The function implements openPOWERLINK kernel pcie interface module read function.

\ingroup module_driver_linux_kernel
*/
//------------------------------------------------------------------------------
static ssize_t powerlinkRead(struct file* pInstance_p, char* pDstBuff_p,
                             size_t BuffSize_p, loff_t* pFileOffs_p)
{
    int    ret;

    DEBUG_LVL_ALWAYS_TRACE("PLK: + powerlinkRead...\n");
    DEBUG_LVL_ALWAYS_TRACE("PLK:   Sorry, this operation isn't supported.\n");
    ret = -EINVAL;
    DEBUG_LVL_ALWAYS_TRACE("PLK: - powerlinkRead (iRet=%d)\n", ret);
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  openPOWERLINK pcie driver write function

The function implements openPOWERLINK kernel pcie interface module write function.

\ingroup module_driver_linux_kernel
*/
//------------------------------------------------------------------------------
static ssize_t powerlinkWrite(struct file* pInstance_p, const char* pSrcBuff_p,
                              size_t BuffSize_p, loff_t* pFileOffs_p)
{
    int    ret;

    DEBUG_LVL_ALWAYS_TRACE("PLK: + powerlinkWrite...\n");
    DEBUG_LVL_ALWAYS_TRACE("PLK:   Sorry, this operation isn't supported.\n");
    ret = -EINVAL;
    DEBUG_LVL_ALWAYS_TRACE("PLK: - powerlinkWrite (iRet=%d)\n", ret);
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  openPOWERLINK pcie driver ioctl function

The function implements openPOWERLINK kernel pcie interface module ioctl function.

\ingroup module_driver_linux_kernel
*/
//------------------------------------------------------------------------------
#ifdef HAVE_UNLOCKED_IOCTL
static long powerlinkIoctl(struct file* filp, unsigned int cmd,
                           unsigned long arg)
#else
static int  powerlinkIoctl(struct inode* dev, struct file* filp,
                           unsigned int cmd, unsigned long arg)
#endif
{
    int             ret;
    tOplkError      oplRet;

    //DEBUG_LVL_ALWAYS_TRACE("PLK: + powerlinkIoctl (cmd=%d type=%d)...\n", _IOC_NR(cmd), _IOC_TYPE(cmd));
    ret = -EINVAL;

    // Add some checks for valid commands here

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
                //TODO Solve the callback type mismatch warning and replace the pdo module if necessary.
                pcieDrv_regSyncHandler(pdokcal_sendSyncEvent);
                pcieDrv_enableSync(TRUE);
            }

            if ((oplRet = drv_waitSyncEvent()) == kErrorRetry)
            {
                ret = -ERESTARTSYS;
            }
            else
            {
                ret = 0;
            }

            break;

        case PLK_CMD_MEMMAP_MAP_MEM:
            ret = mapMemoryForUserIoctl(arg);
            break;

        case PLK_CMD_PDO_MAP_OFFSET: //TODO Check if PDO is initialized!!
            if (copy_to_user((void __user*)arg, &instance_l.pdoBufOffset, sizeof(BYTE*)))
            {
                printk("PDO Offset fetch Error!!\n");
                ret = -EFAULT;
            }

            ret = 0;
            break;

        default:
            DEBUG_LVL_ERROR_TRACE("PLK: - Invalid cmd (cmd=%d type=%d)\n", _IOC_NR(cmd), _IOC_TYPE(cmd));
            ret = -ENOTTY;
            break;
    }

    //TRACE("PLK: - powerlinkIoctl (cmd=%d type=%d)..(ret=%d)\n", _IOC_NR(cmd), _IOC_TYPE(cmd), ret);
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  openPOWERLINK pcie driver mmap function

The function implements openPOWERLINK kernel pcie interface module mmap function.

\ingroup module_driver_linux_kernel
*/
//------------------------------------------------------------------------------
static int powerlinkMmap(struct file* filp, struct vm_area_struct* vma)
{
    BYTE*           pPdoMem = NULL;
    UINT16          memSize = 0;
    tOplkError      ret = kErrorOk;
    tPdoMemRegion*  pPdoMem_l = NULL;
    ULONG           pfn = 0;

    DEBUG_LVL_ALWAYS_TRACE("%s() vma: vm_start:%lX vm_end:%lX vm_pgoff:%lX\n",
                           __func__, vma->vm_start, vma->vm_end, vma->vm_pgoff);

    vma->vm_flags |= VM_RESERVED | VM_IO;
    vma->vm_ops = &powerlinkVmOps;

    //FIXME Rework the function to map the offset passed from user that the user can get
    // via an ioctl call made before the mmap() call. This way both pdo and memmap modules
    // can use mmap and multiple mmap() calls to this driver would be possible.

    // if (vma->vm_pgoff == 0)
    //{
    ret = drv_getPdoMem(&pPdoMem, memSize);
    //}
    //else
    //{
    //    ret = mapMemoryForUserMmap(&pPdoMem, (ULONGLONG*) &vma->vm_pgoff);
    //}

    if ((pPdoMem == NULL) || (ret != kErrorOk))
    {
        DEBUG_LVL_ERROR_TRACE("%s() no pdo memory allocated!\n", __func__);
        return -ENOMEM;
    }

    pPdoMem_l = (tPdoMemRegion*)pPdoMem;

    //pfn = virt_to_phys(pPdoMem);
    pfn = pcieDrv_getBarInst(0) + ((ULONG)pPdoMem - pcieDrv_getBarAddr(0));
    instance_l.pdoBufOffset = (BYTE*)(pfn - ((pfn >> PAGE_SHIFT) << PAGE_SHIFT));
    instance_l.pPdoMem = pPdoMem;
    printk("virt MAP addr: 0x%lX --> 0x%lX --> 0x%lX\n", (ULONG)pPdoMem, pfn, (pfn >> PAGE_SHIFT));
    /*
    if (remap_pfn_range(vma, vma->vm_start, (pfn >> PAGE_SHIFT),
                        vma->vm_end - vma->vm_start + instance_l.pdoBufOffset - 4095, vma->vm_page_prot))
    {
            DEBUG_LVL_ERROR_TRACE("%s() remap_pfn_range failed\n", __func__);
            return -EAGAIN;
    }
    /*///FIXME Align the memory to be mapped to PAGE boundary than a fixed random offset of 4095
    if (io_remap_pfn_range(vma, vma->vm_start, (pfn >> PAGE_SHIFT),
                           vma->vm_end - vma->vm_start + (ULONG)instance_l.pdoBufOffset - 4095, vma->vm_page_prot))
    {
        DEBUG_LVL_ERROR_TRACE("%s() remap_pfn_range failed\n", __func__);
        return -EAGAIN;
    } //*/

    powerlinkVmaOpen(vma);
    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  openPOWERLINK pcie driver VMA open functionnet

The function implements openPOWERLINK kernel module VMA open function.

\ingroup module_driver_linux_kernel
*/
//------------------------------------------------------------------------------
static void powerlinkVmaOpen(struct vm_area_struct* vma)
{
    DEBUG_LVL_ALWAYS_TRACE("%s() vma: vm_start:%lX vm_end:%lX vm_pgoff:%lX\n",
                           __func__, vma->vm_start, vma->vm_end, vma->vm_pgoff);
}

//------------------------------------------------------------------------------
/**TRACE
\brief  openPOWERLINK pcie driver VMA close function

The function implements openPOWERLINK kernel module VMA close function.

\ingroup module_driver_linux_kernel
*/
//------------------------------------------------------------------------------
static void powerlinkVmaClose(struct vm_area_struct* vma)
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

\ingroup module_eventkcal
*/
//------------------------------------------------------------------------------
int getEventForUser(ULONG arg)
{
    int             ret;
    size_t          readSize;
    signed long     timeout = 10 * HZ / 1000;   // 10ms timeout
    signed long     loopTimeout = 500 * HZ / 1000;
    int             loopCount = (loopTimeout / timeout);
    int             i = 0;

    for (i = 0; i < loopCount; i++)
    {
        ret = wait_event_interruptible_timeout(instance_l.userWaitQueue, 0, timeout);

        // ignore timeout (ret = 0) condition as we are using it to sleep

        if (ret == -ERESTARTSYS)
        {
            printk("%s() interrupted\n", __func__);
            break;
        }

        drv_getEvent(instance_l.aK2URxBuffer, &readSize);

        if (readSize > 0)
        {
            DEBUG_INTF("%s() copy kernel event to user: %d Bytes\n", __func__, readSize);
            if (copy_to_user((void __user*)arg, instance_l.aK2URxBuffer, readSize))
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

This function posts a event from the user layer to a queue.

\param  arg                Ioctl argument. Contains the event to post.

\return The function returns Linux error code.

\ingroup module_eventkcal
*/
//------------------------------------------------------------------------------
int postEventFromUser(ULONG arg)
{
    tOplkError      ret = kErrorOk;
    tEvent          event;
    BYTE*           pArg = NULL;

    if (copy_from_user(&event, (const void __user*)arg, sizeof(tEvent)))
        return -EFAULT;

    if (event.eventArgSize != 0)
    {
        //order = get_order(event.eventArgSize);
        //pArg = (BYTE*)__get_free_pages(GFP_KERNEL, order);
        //TODO Find a way to not allocate memory every time a U2K event is posted
        pArg = (BYTE*)OPLK_MALLOC(event.eventArgSize);
        if (pArg == NULL)
            return -EIO;

        //TRACE("%s() allocated %d Bytes at %p\n", __func__, event.eventArgSize, pArg);
        if (copy_from_user(pArg, (const void __user*)event.eventArg.pEventArg, event.eventArgSize))
        {
            //free_pages((ULONG)pArg, order);
            OPLK_FREE(pArg);
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
            /*TRACE("U2K  type:%s(%d) sink:%s(%d) size:%d!\n",
                   debugstr_getEventTypeStr(event.eventType), event.eventType,
                   debugstr_getEventSinkStr(event.eventSink), event.eventSink,
                   event.eventArgSize);*/
            /*
              TRACE("U2K  type:(%d) sink:(%d) size:%d!\n",
                   event.eventType,
                   event.eventSink,
                   event.eventArgSize);//*/
            drv_postEvent(&event);
            break;

        case kEventSinkNmtMnu:
        case kEventSinkNmtu:
        case kEventSinkSdoAsySeq:
        case kEventSinkApi:
        case kEventSinkDlluCal:
        case kEventSinkErru:
        /*TRACE("UINT type:%s(%d) sink:%s(%d) size:%d!\n",
               debugstr_getEventTypeStr(event.eventType), event.eventType,
               debugstr_getEventSinkStr(event.eventSink), event.eventSink,
               event.eventArgSize);*/
        default:
            ret = -EIO;
            break;
    }

    if (event.eventArgSize != 0)
        //free_pages((ULONG)pArg, order);
        OPLK_FREE(pArg);

    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  Execute control command ioctl

The function implements the calling of the executeCmd function in the control
module using the ioctl interface.

\ingroup module_driver_linux_kernel
*/
//------------------------------------------------------------------------------
static int executeCmd(unsigned long arg)
{
    tCtrlCmd    ctrlCmd;

    if (copy_from_user(&ctrlCmd, (const void __user*)arg, sizeof(tCtrlCmd)))
        return -EFAULT;

    drv_executeCmd(&ctrlCmd);

    if (copy_to_user((void __user*)arg, &ctrlCmd, sizeof(tCtrlCmd)))
        return -EFAULT;

    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  Store init param ioctl

The function implements the calling of the storeInitParam function in the
control module using the ioctl interface.

\ingroup module_driver_linux_kernel
*/
//------------------------------------------------------------------------------
static int storeInitParam(unsigned long arg)
{
    tCtrlInitParam    initParam;

    if (copy_from_user(&initParam, (const void __user*)arg, sizeof(tCtrlInitParam)))
        return -EFAULT;

    drv_storeInitParam(&initParam);

    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  Read init param ioctl

The function implements the calling of the readInitParam function in the control
module using the ioctl interface.

\ingroup module_driver_linux_kernel
*/
//------------------------------------------------------------------------------
static int readInitParam(unsigned long arg)
{
    tCtrlInitParam    initParam;

    drv_readInitParam(&initParam);
    if (copy_to_user((void __user*)arg, &initParam, sizeof(tCtrlInitParam)))
        return -EFAULT;

    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  Get status ioctl

The function implements the calling of the getStatus function in the control
module using the ioctl interface.

\ingroup module_driver_linux_kernel
*/
//------------------------------------------------------------------------------
static int getStatus(unsigned long arg)
{
    UINT16    status;

    drv_getStatus(&status);
    put_user(status, (unsigned short __user*)arg);
    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  Get heartbeat ioctl

The function implements the calling of the getHeartbeat function in the control
module using the ioctl interface.

\ingroup module_driver_linux_kernel
*/
//------------------------------------------------------------------------------
static int getHeartbeat(unsigned long arg)
{
    UINT16    heartbeat;

    drv_getHeartbeat(&heartbeat);
    put_user(heartbeat, (unsigned short __user*)arg);
    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  Sending async frame ioctl

The function implements the ioctl used for sending asynchronous frames.

\ingroup module_driver_linux_kernel
*/
//------------------------------------------------------------------------------
static int sendAsyncFrame(unsigned long arg)
{
    BYTE*                   pBuf;
    tIoctlDllCalAsync       asyncFrameInfo;
    int                     order;

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

    //TRACE("%s() Received frame size:%d\n", __func__, asyncFrame.size);

    asyncFrameInfo.pData = pBuf;
    drv_sendAsyncFrame((unsigned char*)&asyncFrameInfo);

    free_pages((ULONG)pBuf, order);

    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  Write error object ioctl

The function implements the ioctl for writing an error object.

\ingroup module_driver_linux_kernel
*/
//------------------------------------------------------------------------------
static int writeErrorObject(unsigned long arg)
{
    tErrHndIoctl        writeObject;

    if (copy_from_user(&writeObject, (const void __user*)arg, sizeof(tErrHndIoctl)))
        return -EFAULT;

    drv_writeErrorObject(&writeObject);

    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  Read error object ioctl

The function implements the ioctl for reading error objects.

\ingroup module_driver_linux_kernel
*/
//------------------------------------------------------------------------------
static int readErrorObject(unsigned long arg)
{
    tErrHndIoctl        readObject;

    if (copy_from_user(&readObject, (const void __user*)arg, sizeof(tErrHndIoctl)))
        return -EFAULT;

    drv_readErrorObject(&readObject);

    if (copy_to_user((void __user*)arg, &readObject, sizeof(tErrHndIoctl)))
        return -EFAULT;

    return 0;
}

//------------------------------------------------------------------------------
/**
\brief  Map PCP memory into user memory

The function implements the ioctl for mapping the PCP memory into user memory.

\ingroup module_driver_linux_kernel
*/
//------------------------------------------------------------------------------
static int mapMemoryForUserIoctl(unsigned long arg_p)
{
    tMemmap         memMapParams;
    int             ret = 0;
    tOplkError      retVal = kErrorOk;
    void*           mappedUserBuf = NULL;

    //FIXME Rework this function to return the ioremapped PCP memory offset alone,
    // so that mmap() can be called from user layer using the offset. This will avoid
    // the copying of the entire async frame memory.
    copy_from_user(&memMapParams, (const void __user*)arg_p, sizeof(tMemmap));
    if ((instance_l.pShmMemLocal == NULL) || (instance_l.pShmMemRemote == NULL))
        retVal = drv_mapKernelMem((UINT8**)&instance_l.pShmMemRemote, (UINT8**)&instance_l.pShmMemLocal);

    if (retVal != kErrorOk)
    {
        DEBUG_LVL_ERROR_TRACE("%s() --> Error: Unable to locate shared memory region\n");
        //memMapParams.pUserBuf = NULL; // Not needed, let user free its own memory; just return error.
        ret = -EFAULT;
    }
    else if (memMapParams.pKernelBuf <= instance_l.pShmMemRemote)
    {
        DEBUG_LVL_ERROR_TRACE("%s() --> Error: PCP buffer pointer lies outside the shared memory region\n");
        //memMapParams.pUserBuf = NULL;
        ret = -EFAULT;
    }
    else
    {
        mappedUserBuf = (void*)(((UINT32)memMapParams.pKernelBuf - (ULONG)instance_l.pShmMemRemote) +
                                (ULONG)instance_l.pShmMemLocal);
        DEBUG_INTF("LA: 0x%lX, RA: 0x%lX, KA: 0x%lX, UA: 0x%lX\n",
                   (ULONG)instance_l.pShmMemLocal, (ULONG)instance_l.pShmMemRemote,
                   (ULONG)memMapParams.pKernelBuf, (ULONG)mappedUserBuf);
        OPLK_MEMCPY(aAsyncFrameSwapBuf_l, mappedUserBuf, sizeof(aAsyncFrameSwapBuf_l));
        ret = 0;
    }

    copy_to_user((void __user*)memMapParams.pUserBuf, aAsyncFrameSwapBuf_l, sizeof(aAsyncFrameSwapBuf_l));

    return ret;
}

////------------------------------------------------------------------------------
///**
//\brief  Map PCP memory into user memory
//
//The function maps the PCP memory into user memory
////FIXME Remove this function after mmap finalization.
//
//\ingroup module_driver_linux_kernel
//*/
////------------------------------------------------------------------------------
//static int mapMemoryForUserMmap(BYTE** ppUserBuf_p, ULONGLONG* pKernelBuf_p)
//{
//    void*           pShmMemLocal = NULL;
//    ULONGLONG       pShmMemRemote = (ULONGLONG)NULL;
//    int             ret = 0;
//
//    drv_mapKernelMem((UINT8**)&pShmMemRemote, (UINT8**)&pShmMemLocal);
//
//    if (*pKernelBuf_p <= (ULONG)pShmMemRemote)
//    {
//        *ppUserBuf_p = NULL;
//        ret = -EFAULT;
//    }
//    else
//    {
//        *pKernelBuf_p = *pKernelBuf_p - pShmMemRemote; // change the actual offset of vm struct
//        *ppUserBuf_p = (void*)(*pKernelBuf_p + (ULONGLONG)pShmMemLocal);
//        ret = 0;
//    }
//
//    return ret;
//}

///\}
