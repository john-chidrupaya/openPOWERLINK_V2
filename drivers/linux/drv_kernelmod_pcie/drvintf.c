/**
********************************************************************************
\file   drvintf.c

\brief  Interface module for openPOWERLINK PCIe interface driver to PCP

This module handles all the application request forwarded to the openPOWERLINK
PCIe driver in Linux kernel. It uses dualprocshm and circbuf libraries to manage PDO
memory, error objects shared memory, event and DLL queues.

The module also implements mapping of kernel memory into user space to provide
direct access to user application for specific shared memory regions.

\ingroup module_driver_linux_kernel_pcie
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2015, Kalycito Infotech Private Limited
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
//TODO Remove unnecessary headers
#include <oplk/oplk.h>

#include <kernel/eventk.h>
#include <kernel/eventkcal.h>
#include <errhndkcal.h>
#include <dualprocshm.h>
#include <common/circbuffer.h>

#include <drvintf.h>
#include <kernel/pdokcal.h>

#include <common/timer.h>
//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define PROC_INSTANCE_ID                0xBA

//------------------------------------------------------------------------------
// module global vars
//------------------------------------------------------------------------------
//TODO get these values from stack headers
#define DUALPROCSHM_BUFF_ID_ERRHDLR     12
#define DUALPROCSHM_BUFF_ID_PDO         13
#define BENCHMARK_OFFSET                0x00001000 //TODO: Get this value from PCIe header files

//------------------------------------------------------------------------------
// global function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P R I V A T E   D E F I N I T I O N S                           //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define CMD_TIMEOUT_CNT                 500 // Loop count for command timeout in units of 10ms
#define DPSHM_ENABLE_TIMEOUT_SEC        10      // wait for dpshm interface enable time out
//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------
/**
\brief Mapped memory information

This structure stores the information for a memory shared between user layer
and PCP.
*/
typedef struct
{
    size_t      memSize;                                        ///< Size of memory
    void*       pKernelVa;                                      ///< Pointer to memory in kernel space.
    void*       pUserVa;                                        ///< Pointer to memory mapped in user space.
} tMemInfo;

/**
\brief Interface module instance - User Layer

This instance stores the local parameters used by the
interface module during runtime.
*/
typedef struct
{
    tDualprocDrvInstance    dualProcDrvInst;                    ///< Dual processor driver instance.
    BOOL                    fIrqMasterEnable;                   ///< Master interrupts status.
    tCircBufInstance*       apEventQueueInst[kEventQueueNum];       ///< Event queue instances.
    tCircBufInstance*       apDllQueueInst[kDllCalQueueTxVeth + 1]; ///< DLL queue instances.
    tErrHndObjects*         pErrorObjects;                      ///< Pointer to error objects.
    tMemInfo                pdoMem;                             ///< PDO memory information mapped to user space.
    tMemInfo                benchmarkMem;                       ///< Benchmark memory information mapped to user space.
    tMemInfo                kernel2UserMem;                     ///< Kernel to user mapped memory.
    BOOL                    fDriverActive;                      ///< Flag to identify status of driver interface.
}tDrvIntfInstance;
//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tDrvIntfInstance    drvIntfInstance_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static tOplkError initDualProcShm(void);
static tOplkError   initEvent(void);
static tOplkError   initDllQueues(void);
static tOplkError   initErrHndl(void);
static void       exitDualProcShm(void);
static void         exitEvent(void);
static void         exitDllQueues(void);
static void         exitErrHndl(void);
static tOplkError   insertDataBlock(tCircBufInstance* pDllCircBuffInst_p,   //XXX There should be a remove for this function too.
                                    BYTE* pData_p,
                                    UINT* pDataSize_p);
static tOplkError   mapMemory(tMemInfo* pMemInfo_p);
static void         unmapMemory(tMemInfo* pMemInfo_p);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Initialize driver interface

This function initializes necessary resources required for driver interface.

\return Returns tOplkError error code.

\ingroup module_driver_ndispcie
*/
//------------------------------------------------------------------------------
tOplkError drv_init(void)
{
    tOplkError ret = kErrorOk;

    TRACE("Initialize driver interface...");

    OPLK_MEMSET(&drvIntfInstance_l, 0, sizeof(tDrvIntfInstance));

    // Initialize the dualprocshm library
    ret = initDualProcShm();
    if (ret != kErrorOk)
    {
        DEBUG_LVL_ERROR_TRACE("Dual processor shared memory interface Initialization failed (0x%X)\n",
                              ret);
        return ret;
    }

    drvIntfInstance_l.fDriverActive = TRUE;

    TRACE(" OK\n");

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Close the driver interface

This function frees all the resources used by the driver interface and shuts down
the interface.

\ingroup module_driver_ndispcie
*/
//------------------------------------------------------------------------------
void drv_exit(void)
{
    TRACE("Exit driver interface...\n");

    if (drvIntfInstance_l.fDriverActive)
    {
        drvIntfInstance_l.fDriverActive = FALSE;

        // Close dualprocshm library interface
        exitDualProcShm();
    }
}

//------------------------------------------------------------------------------
/**
\brief  Execute a control command from user application

This function parses the control command from user and passes it to PCP
control module for processing. The return value is again passed to user by
copying it into the common control structure.

\param  pCtrlCmd_p       Pointer to control command structure.

\return Returns tOplkError error code.

\ingroup module_driver_linux_kernel_pcie
*/
//------------------------------------------------------------------------------
tOplkError drv_executeCmd(tCtrlCmd* pCtrlCmd_p)
{
    tOplkError    ret = kErrorOk;
    UINT16          cmd = pCtrlCmd_p->cmd;
    INT             timeout;
    // Clean up stack
    if (cmd == kCtrlCleanupStack || cmd == kCtrlShutdown)
    {
        exitDllQueues();
        exitErrHndl();
        exitEvent();
    }

    if (dualprocshm_writeDataCommon(drvIntfInstance_l.dualProcDrvInst, FIELD_OFFSET(tCtrlBuf, ctrlCmd),
                                    sizeof(tCtrlCmd), (UINT8*)pCtrlCmd_p) != kDualprocSuccessful)
        return kErrorNoResource;

    // wait for response
    for (timeout = 0; timeout < CMD_TIMEOUT_CNT; timeout++)
    {
        msleep(10);

        if (dualprocshm_readDataCommon(drvIntfInstance_l.dualProcDrvInst, FIELD_OFFSET(tCtrlBuf, ctrlCmd),
                                       sizeof(tCtrlCmd), (UINT8*)pCtrlCmd_p) != kDualprocSuccessful)
        {
            return kErrorNoResource;
        }

        if (pCtrlCmd_p->cmd == 0)
            break;
    }

    if (timeout == CMD_TIMEOUT_CNT)
        return kErrorGeneralError;

    if ((cmd == kCtrlInitStack) && (pCtrlCmd_p->retVal == kErrorOk))
    {
        ret = initEvent();
        if (ret != kErrorOk)
        {
            DEBUG_LVL_ERROR_TRACE("Event Initialization Failed (0x%X)\n", ret);
            return ret;
        }

        ret = initErrHndl();
        if (ret != kErrorOk)
        {
            DEBUG_LVL_ERROR_TRACE("Error Module Initialization Failed (0x%X)\n", ret);
            return ret;
        }

        ret = initDllQueues();
        if (ret != kErrorOk)
        {
            DEBUG_LVL_ERROR_TRACE("Dll Queues Initialization Failed (0x%X)\n", ret);
            return ret;
        }

        ret = pdokcal_initSync();
        if (ret != kErrorOk)
        {
            DEBUG_LVL_ERROR_TRACE("PDO Sync Initialization Failed %x\n", ret);
            pCtrlCmd_p->retVal = ret;
            return ret;
        }
    }

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Wait for a sync event

The function waits for a sync event.

\return The function returns a tOplkError error code.

\ingroup module_driver_linux_kernel_pcie
*/
//------------------------------------------------------------------------------
tOplkError drv_waitSyncEvent(void)
{
    return pdokcal_waitSyncEvent();
}

//------------------------------------------------------------------------------
/**
\brief  Read initialization parameters

Read the initialization parameters from the kernel stack.

\param  pInitParam_p       Pointer to initialization parameters structure.

\return Returns tOplkError error code.

\ingroup module_driver_linux_kernel_pcie
*/
//------------------------------------------------------------------------------
tOplkError drv_readInitParam(tCtrlInitParam* pInitParam_p)
{
    tDualprocReturn    dualRet;

    if (!drvIntfInstance_l.fDriverActive)
        return kErrorNoResource;

    dualRet = dualprocshm_readDataCommon(drvIntfInstance_l.dualProcDrvInst,
                                         FIELD_OFFSET(tCtrlBuf, initParam),
                                         sizeof(tCtrlInitParam),
                                         (UINT8*)pInitParam_p);

    if (dualRet != kDualprocSuccessful)
    {
        DEBUG_LVL_ERROR_TRACE("Cannot read initparam (0x%X)\n", dualRet);
        return kErrorNoResource;
    }

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Write initialization parameters

Write the initialization parameters from the user layer into kernel layer
memory of stack.

\param  pInitParam_p       Pointer to initialization parameters structure.

\return Returns tOplkError error code.

\ingroup module_driver_linux_kernel_pcie
*/
//------------------------------------------------------------------------------
tOplkError drv_storeInitParam(tCtrlInitParam* pInitParam_p)
{
    tDualprocReturn    dualRet;
    if (!drvIntfInstance_l.fDriverActive)
        return kErrorNoResource;

    dualprocshm_writeDataCommon(drvIntfInstance_l.dualProcDrvInst,
                                FIELD_OFFSET(tCtrlBuf, initParam),
                                sizeof(tCtrlInitParam),
                                (UINT8*)pInitParam_p);
    if (dualRet != kDualprocSuccessful)
    {
        DEBUG_LVL_ERROR_TRACE("Cannot store initparam (0x%X)\n", dualRet);
        return kErrorNoResource;
    }

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Get PCP status

Return the current status of kernel stack.

\param  pStatus_p       Pointer to status variable to return.

\return Returns tOplkError error code.

\ingroup module_driver_linux_kernel_pcie
*/
//------------------------------------------------------------------------------
tOplkError drv_getStatus(UINT16* pStatus_p)
{
    if (!drvIntfInstance_l.fDriverActive)
        return kErrorNoResource;

    if (dualprocshm_readDataCommon(drvIntfInstance_l.dualProcDrvInst,
                                   FIELD_OFFSET(tCtrlBuf, status),
                                   sizeof(UINT16),
                                   (UINT8*)pStatus_p) != kDualprocSuccessful)
    {
        DEBUG_LVL_ERROR_TRACE("Error Reading Status\n");
        return kErrorNoResource;
    }

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Get heartbeat

Return the current heartbeat value in kernel.

\param  pHeartbeat       Pointer to heartbeat variable to return.

\return Returns tOplkError error code.

\ingroup module_driver_linux_kernel_pcie
*/
//------------------------------------------------------------------------------
tOplkError drv_getHeartbeat(UINT16* pHeartbeat_p)
{
    if (!drvIntfInstance_l.fDriverActive)
        return kErrorNoResource;

    if (dualprocshm_readDataCommon(drvIntfInstance_l.dualProcDrvInst,
                                   FIELD_OFFSET(tCtrlBuf, heartbeat),
                                   sizeof(UINT16),
                                   (UINT8*)pHeartbeat_p) != kDualprocSuccessful)
    {
        DEBUG_LVL_ERROR_TRACE("Error Reading HeartBeat\n");
        return kErrorNoResource;
    }

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Write asynchronous frame

This routines extracts the asynchronous frame from the IOCTL buffer and writes
it into the specified DLL queue for processing by PCP.

\param  pArg_p       Pointer to IOCTL buffer.

\ingroup module_driver_linux_kernel_pcie
*/
//------------------------------------------------------------------------------
tOplkError drv_sendAsyncFrame(unsigned char* pArg_p)
{
    tIoctlDllCalAsync*      asyncFrameInfo;
    tFrameInfo              frameInfo;
    tOplkError              ret = kErrorOk;

    if (!drvIntfInstance_l.fDriverActive)
        return kErrorNoResource;

    asyncFrameInfo = (tIoctlDllCalAsync*)pArg_p;
    frameInfo.frameSize = asyncFrameInfo->size;
    frameInfo.frame.pBuffer = (tPlkFrame*)asyncFrameInfo->pData;

    ret = insertDataBlock(drvIntfInstance_l.apDllQueueInst[asyncFrameInfo->queue],
                          (UINT8*)frameInfo.frame.pBuffer,
                          &(frameInfo.frameSize));

    if (ret != kErrorOk)
    {
        DEBUG_LVL_ERROR_TRACE("Error sending async frame queue %d\n", asyncFrameInfo->queue);
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Write error object

This routines updates the error objects in shared memory with the value passed
from user layer.

\param  pWriteObject_p       Pointer to writeobject to update.

\ingroup module_driver_linux_kernel_pcie
*/
//------------------------------------------------------------------------------
tOplkError drv_writeErrorObject(tErrHndIoctl* pWriteObject_p)
{
    tErrHndObjects*   errorObjects = drvIntfInstance_l.pErrorObjects;

    if (!drvIntfInstance_l.fDriverActive)
        return kErrorNoResource;

    *((UINT32*)((UINT8*)errorObjects + pWriteObject_p->offset)) = pWriteObject_p->errVal;

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Read error object

This routines fetches the error objects in shared memory to be passed to user
layer.

\param  pWriteObject_p       Pointer to pReadObject_p to fetch.

\ingroup module_driver_linux_kernel_pcie
*/
//------------------------------------------------------------------------------
tOplkError drv_readErrorObject(tErrHndIoctl* pReadObject_p)
{
    tErrHndObjects*   errorObjects = drvIntfInstance_l.pErrorObjects;

    if (!drvIntfInstance_l.fDriverActive)
        return kErrorNoResource;

    pReadObject_p->errVal = *((UINT32*)((char*)errorObjects + pReadObject_p->offset));
    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Initialize dual processor shared memory driver instance

This routine initializes the driver instance of dualprocshm for user layer(host).

\return Returns tOplkError error code.

\ingroup module_driver_linux_kernel_pcie
*/
//------------------------------------------------------------------------------
tOplkError drv_initDualProcDrv(void)
{
    tDualprocReturn     dualRet;
    tDualprocConfig     dualProcConfig;
    INT                 loopCount = 0;

    TRACE(" Initialize Driver interface...\n");
    //TODO Change this to the init function for the interface and add check for
    // previously initialized instances.
    OPLK_MEMSET(&drvIntfInstance_l, 0, sizeof(tDrvIntfInstance));

    OPLK_MEMSET(&dualProcConfig, 0, sizeof(tDualprocConfig));

    dualProcConfig.procInstance = kDualProcSecond;
    dualProcConfig.procInstance = PROC_INSTANCE_ID;

    dualRet = dualprocshm_create(&dualProcConfig, &drvIntfInstance_l.dualProcDrvInst);
    if (dualRet != kDualprocSuccessful)
    {
        DEBUG_LVL_ERROR_TRACE(" %s(): Could not create dual processor driver instance (0x%X)\n",
                              __func__, dualRet);
        dualprocshm_delete(drvIntfInstance_l.dualProcDrvInst);
        return kErrorNoResource;
    }

    for (loopCount = 0; loopCount <= DPSHM_ENABLE_TIMEOUT_SEC; loopCount++)
    {
        msleep(1000U);
        dualRet = dualprocshm_checkShmIntfState(drvIntfInstance_l.dualProcDrvInst);
        if (dualRet != kDualprocshmIntfDisabled)
            break;
    }

    if (dualRet != kDualprocshmIntfEnabled)
    {
        DEBUG_LVL_ERROR_TRACE("%s dualprocshm  interface is not enabled (0x%X)\n",
                              __func__, dualRet);
        return kErrorNoResource;
    }

    // Disable the Interrupts from PCP
    drvIntfInstance_l.fIrqMasterEnable = FALSE;

    dualRet = dualprocshm_initInterrupts(drvIntfInstance_l.dualProcDrvInst);
    if (dualRet != kDualprocSuccessful)
    {
        DEBUG_LVL_ERROR_TRACE("%s(): Error Initializing interrupts %x\n ", __func__, dualRet);
        return kErrorNoResource;
    }

    drvIntfInstance_l.fDriverActive = TRUE;
    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  De-initialize dual processor shared memory driver instance

This routine deletes the driver instance of dualprocshm created during
initialization.

\ingroup module_driver_linux_kernel_pcie
*/
//------------------------------------------------------------------------------
void drv_exitDualProcDrv(void)
{
    tDualprocReturn    dualRet;

    //TODO Check for an existing driver instance
    drvIntfInstance_l.fIrqMasterEnable = FALSE;
    drvIntfInstance_l.fDriverActive = FALSE;

    // disable system irq
    dualprocshm_freeInterrupts(drvIntfInstance_l.dualProcDrvInst);

    dualRet = dualprocshm_delete(drvIntfInstance_l.dualProcDrvInst);
    if (dualRet != kDualprocSuccessful)
    {
        DEBUG_LVL_ERROR_TRACE("Could not delete dual proc driver inst (0x%X)\n", dualRet);
    }
}

//------------------------------------------------------------------------------
/**
\brief  Post an user event

Copies the event from user layer into user to kernel(U2K) event queue.

\param  pEvent_p    Pointer to user event memory.

\return Returns tOplkError error code.

\ingroup module_driver_linux_kernel_pcie
*/
//------------------------------------------------------------------------------
tOplkError drv_postEvent(void* pEvent_p)
{
    tOplkError          ret = kErrorOk;
    tCircBufError       circError;

    tCircBufInstance*   pCircBufInstance = drvIntfInstance_l.apEventQueueInst[kEventQueueU2K];

    if ((pEvent_p == NULL) || !drvIntfInstance_l.fDriverActive)
        return kErrorNoResource;

    if (((tEvent*)pEvent_p)->eventArgSize == 0)
    {
        circError = circbuf_writeData(pCircBufInstance, pEvent_p, sizeof(tEvent));
    }
    else
    {
        circError = circbuf_writeMultipleData(pCircBufInstance, pEvent_p, sizeof(tEvent),
                                              ((tEvent*)pEvent_p)->eventArg.pEventArg, ((tEvent*)pEvent_p)->eventArgSize);
    }

    if (circError != kCircBufOk)
    {
        DEBUG_LVL_ERROR_TRACE("Error in Post event %x\n", circError);
        ret = kErrorEventPostError;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Get an user event

Retrieves an event from kernel to user event(K2U) queue for the user layer.

\param  pEvent_p    Pointer to user event memory.
\param  pSize_p     Pointer to store the size of the event buffer.

\return Returns tOplkError error code and the size of the read data.

\ingroup module_driver_linux_kernel_pcie
*/
//------------------------------------------------------------------------------
tOplkError drv_getEvent(void* pEvent_p, size_t* pSize_p)
{
    tCircBufError       errCode = kCircBufOk;
    tCircBufInstance*   pCircBufInstance = drvIntfInstance_l.apEventQueueInst[kEventQueueK2U];
    if ((pEvent_p == NULL) || (pSize_p == NULL) || !drvIntfInstance_l.fDriverActive)
        return kErrorNoResource;

    if (circbuf_getDataCount(pCircBufInstance) > 0)
    {
        errCode = circbuf_readData(pCircBufInstance, pEvent_p,
                                   sizeof(tEvent) + MAX_EVENT_ARG_SIZE, pSize_p);
    }
    else
    {
        *pSize_p = 0;
    }

    if (errCode != kCircBufOk)
    {
        *pSize_p = 0;
        DEBUG_LVL_ERROR_TRACE("Error in reading circular buffer event data!!\n");
    }
	
    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Get PDO memory

Retrieves the PDO memory address from the dualprocshm library and maps it into
user space before sharing it to user layer(host).

\param  ppPdoMem_p    Pointer to PDO memory.
\param  memSize_p     Size of the PDO memory.

\return Returns tOplkError error code.

\ingroup module_driver_linux_kernel_pcie
*/
//------------------------------------------------------------------------------
tOplkError drv_getPdoMem(UINT8** ppPdoMem_p, size_t memSize_p)
{
    tDualprocReturn     dualRet;
    UINT8*              pMem = NULL;
    tMemInfo*           pPdoMemInfo = &drvIntfInstance_l.pdoMem;

    if (!drvIntfInstance_l.fDriverActive)
        return kErrorNoResource;

    dualRet = dualprocshm_getMemory(drvIntfInstance_l.dualProcDrvInst,
                                    DUALPROCSHM_BUFF_ID_PDO, &pMem, &memSize_p, FALSE);
    if (dualRet != kDualprocSuccessful)
    {
        DEBUG_LVL_ERROR_TRACE("%s() couldn't allocate Pdo buffer (%d)\n",
                              __func__, dualRet);
        return kErrorNoResource;
    }

    pPdoMemInfo->pKernelVa = pMem;
    pPdoMemInfo->memSize = memSize_p;

    if (mapMemory(pPdoMemInfo) != kErrorOk)
    {
        DEBUG_LVL_ERROR_TRACE("%s() error mapping memory\n", __func__);
        return kErrorNoResource;
    }

    *ppPdoMem_p = pPdoMemInfo->pUserVa;
    return kErrorOk;
}

//TODO @J Remove this function
//------------------------------------------------------------------------------
/**
\brief  Free PDO memory

Frees the PDO memory previously allocated.

\param  ppPdoMem_p    Pointer to PDO memory.
\param  memSize_p     Size of the PDO memory.

\ingroup module_driver_linux_kernel_pcie
*/
//------------------------------------------------------------------------------
void drv_freePdoMem(UINT8* pPdoMem_p, size_t memSize_p)
{
    tMemInfo*           pPdoMemInfo = &drvIntfInstance_l.pdoMem;
    tDualprocReturn     dualRet;

    unmapMemory(pPdoMemInfo);

    dualRet = dualprocshm_freeMemory(drvIntfInstance_l.dualProcDrvInst, DUALPROCSHM_BUFF_ID_PDO, FALSE);
    if (dualRet != kDualprocSuccessful)
    {
        DEBUG_LVL_ERROR_TRACE("%s() couldn't free PDO buffer (%d)\n",
                              __func__, dualRet);
        return;
    }

    pPdoMem_p = NULL;
}

//------------------------------------------------------------------------------
/**
\brief  Get Benchmark Base

Retrieves the benchmark memory from NDIS driver and maps it into user virtual
address space for accessing for user layer.

\param  ppBenchmarkMem_p    Pointer to benchmark memory.

\return Returns tOplkError error code.

\ingroup module_driver_linux_kernel_pcie
*/
//------------------------------------------------------------------------------
tOplkError drv_getBenchmarkMem(UINT8** ppBenchmarkMem_p)
{
    //    UINT8*      pMem;
    //    tMemInfo*   pBenchmarkMemInfo = &drvIntfInstance_l.benchmarkMem;
    //
    //    if (!drvIntfInstance_l.fDriverActive)
    //        return kErrorNoResource;
    //
    //    // Check if memory is already allocated and mapped
    //    if (pBenchmarkMemInfo->pUserVa != NULL)
    //        goto Exit;
    //
    //    pMem = (UINT8*) ndis_getBarAddr(OPLK_PCIEBAR_COMM_MEM);
    //
    //    if (pMem == NULL)
    //        return kErrorNoResource;
    //
    //    pBenchmarkMemInfo->pKernelVa = pMem + BENCHMARK_OFFSET;
    //    pBenchmarkMemInfo->memSize = 4;
    //
    //    if (mapMemory(pBenchmarkMemInfo) != kErrorOk)
    //    {
    //        DEBUG_LVL_ERROR_TRACE("%s() error mapping memory\n", __func__);
    //        return kErrorNoResource;
    //    }
    //
    //Exit:
    //    *ppBenchmarkMem_p = pBenchmarkMemInfo->pUserVa;
    //
    //    TRACE("%s() Benchmark memory address in user space %p\n", __func__, pBenchmarkMemInfo->pUserVa);
    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Free Benchmark memory

Frees the benchmark memory previously allocated.

\param  pBenchmarkMem_p    Pointer to benchmark memory.

\ingroup module_driver_linux_kernel_pcie
*/
//------------------------------------------------------------------------------
void drv_freeBenchmarkMem(UINT8* pBenchmarkMem_p)
{
    tMemInfo*   pBenchmarkMemInfo = &drvIntfInstance_l.benchmarkMem;

    unmapMemory(pBenchmarkMemInfo);

    pBenchmarkMem_p = NULL;
}

//------------------------------------------------------------------------------
/**
\brief  Map memory in openPOWERLINK kernel into user layer

Maps the kernel layer memory specified by the caller into user layer.

\param  ppKernelMem_p    Double pointer to kernel memory.
\param  ppUserMem_p      Double pointer to mapped kernel memory in user layer.
\param  pSize_p          Pointer to the size of the memory to be mapped.

\return Returns tOplkError error code.

\ingroup module_driver_linux_kernel_pcie
*/
//------------------------------------------------------------------------------
tOplkError drv_mapKernelMem(UINT8** ppKernelMem_p, UINT8** ppUserMem_p, UINT32* pSize_p)
{
    tDualprocReturn             dualRet;
    tMemInfo*                   pKernel2UserMemInfo = &drvIntfInstance_l.kernel2UserMem;
    tDualprocSharedMemInst      localProcSharedMemInst;
    tDualprocSharedMemInst      remoteProcSharedMemInst;
    tDualProcInstance           localProcInst;
    tDualProcInstance           remoteProcInst;

    if (ppKernelMem_p == NULL || ppUserMem_p == NULL)
        return kErrorNoResource;


    UNUSED_PARAMETER(pSize_p);
    localProcInst = dualprocshm_getLocalProcInst();
    remoteProcInst = dualprocshm_getRemoteProcInst();


    //TODO Check for a valid DPSHM handle initialisation
    // read the local processor's shared memory base address
    dualRet = dualprocshm_getSharedMemInfo(drvIntfInstance_l.dualProcDrvInst,
                                           localProcInst, &localProcSharedMemInst);

    if (dualRet != kDualprocSuccessful || localProcSharedMemInst.baseAddr == (UINT64)0)
    {
        DEBUG_LVL_ERROR_TRACE("%s() Unable to map kernel memory error %x\n",
                              __func__, dualRet);
        return kErrorNoResource;
    }

    //TODO Avoid multiple calls to the same function for base addresses
    dualRet = dualprocshm_getSharedMemInfo(drvIntfInstance_l.dualProcDrvInst,
                                           remoteProcInst, &remoteProcSharedMemInst);

    if (dualRet != kDualprocSuccessful || remoteProcSharedMemInst.baseAddr == (UINT64)0)
    {
        DEBUG_LVL_ERROR_TRACE("%s() Unable to map kernel memory error %x\n",
                              __func__, dualRet);
        return kErrorNoResource;
    }

    pKernel2UserMemInfo->pKernelVa = (void*)localProcSharedMemInst.baseAddr;
    pKernel2UserMemInfo->memSize = remoteProcSharedMemInst.span;

    if (mapMemory(pKernel2UserMemInfo) != kErrorOk)
    {
        DEBUG_LVL_ERROR_TRACE("%s() error mapping memory\n", __func__);
        return kErrorNoResource;
    }

    *ppUserMem_p = pKernel2UserMemInfo->pUserVa;
    *ppKernelMem_p = (UINT8*)remoteProcSharedMemInst.baseAddr;

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Unmap mapped memory

Unmap and free the kernel to user memory mapped before.

\param  pUserMem_p    Pointer to mapped user memory.

\ingroup module_driver_linux_kernel_pcie
*/
//------------------------------------------------------------------------------
void drv_unmapKernelMem(UINT8* pUserMem_p)
{
    tMemInfo*   pKernel2UserMemInfo = &drvIntfInstance_l.kernel2UserMem;

    unmapMemory(pKernel2UserMemInfo);

    pUserMem_p = NULL;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Initialize dual processor shared memory driver instance

This routine initializes the driver instance of dualprocshm for host processor.

\return Returns tOplkError error code.

*/
//------------------------------------------------------------------------------
static tOplkError initDualProcShm(void)
{
    tDualprocReturn     dualRet;
    tDualprocConfig     dualProcConfig;
    INT                 loopCount = 0;

    OPLK_MEMSET(&dualProcConfig, 0, sizeof(tDualprocConfig));

    dualProcConfig.procInstance = kDualProcSecond;

    dualRet = dualprocshm_create(&dualProcConfig, &drvIntfInstance_l.dualProcDrvInst);
    if (dualRet != kDualprocSuccessful)
    {
        DEBUG_LVL_ERROR_TRACE(" %s(): Could not create dual processor driver instance (0x%X)\n",
                              __func__, dualRet);
        dualprocshm_delete(drvIntfInstance_l.dualProcDrvInst);
        return kErrorNoResource;
    }

    for (loopCount = 0; loopCount <= DPSHM_ENABLE_TIMEOUT_SEC; loopCount++)
    {
        msleep(1000U);
        dualRet = dualprocshm_checkShmIntfState(drvIntfInstance_l.dualProcDrvInst);
        if (dualRet != kDualprocshmIntfDisabled)
            break;
    }

    if (dualRet != kDualprocshmIntfEnabled)
    {
        DEBUG_LVL_ERROR_TRACE("%s(): Dual processor interface is not enabled (0x%X)\n",
                              __func__, dualRet);
        return kErrorNoResource;
    }

    dualRet = dualprocshm_initInterrupts(drvIntfInstance_l.dualProcDrvInst);
    if (dualRet != kDualprocSuccessful)
    {
        DEBUG_LVL_ERROR_TRACE("{%s} Error Initializing interrupts %x\n ", __func__, dualRet);
        return kErrorNoResource;
    }

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Delete dual processor shared memory driver instance

This routine deletes the driver instance of dualprocshm created during
initialization.

*/
//------------------------------------------------------------------------------
static void exitDualProcShm(void)
{
    tDualprocReturn    dualRet;

    // disable system irq
    dualRet = dualprocshm_freeInterrupts(drvIntfInstance_l.dualProcDrvInst);
    if (dualRet != kDualprocSuccessful)
    {
        DEBUG_LVL_ERROR_TRACE("Could not free dual processor interrupts (0x%X)\n", dualRet);
    }

    dualRet = dualprocshm_delete(drvIntfInstance_l.dualProcDrvInst);
    if (dualRet != kDualprocSuccessful)
    {
        DEBUG_LVL_ERROR_TRACE("Could not delete dual processor driver inst (0x%X)\n", dualRet);
    }
}

//------------------------------------------------------------------------------
/**
\brief  Initialize event queues

Initializes shared event queues between user and kernel layer of stack. The memory for the
queues allocated in PCIe memory by PCP, are retrieved using dualprocshm library.
The circular buffer library is used to manage the queues.

\return Returns tOplkError error code.

*/
//------------------------------------------------------------------------------
static tOplkError initEvent(void)
{
    tCircBufError    circError = kCircBufOk;

    if (!drvIntfInstance_l.fDriverActive)
        return kErrorNoResource;

    circError = circbuf_connect(CIRCBUF_USER_TO_KERNEL_QUEUE, &drvIntfInstance_l.apEventQueueInst[kEventQueueU2K]);
    if (circError != kCircBufOk)
    {
        TRACE("PLK : Could not allocate CIRCBUF_USER_TO_KERNEL_QUEUE circbuffer\n");
        return kErrorNoResource;
    }

    circError = circbuf_connect(CIRCBUF_KERNEL_TO_USER_QUEUE, &drvIntfInstance_l.apEventQueueInst[kEventQueueK2U]);
    if (circError != kCircBufOk)
    {
        TRACE("PLK : Could not allocate CIRCBUF_KERNEL_TO_USER_QUEUE circbuffer\n");
        return kErrorNoResource;
    }

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Close user layer event queues

Close event queues initialized earlier.

\return Returns tOplkError error code.

*/
//------------------------------------------------------------------------------
static void exitEvent(void)
{
    if (drvIntfInstance_l.apEventQueueInst[kEventQueueK2U] != NULL)
    circbuf_disconnect(drvIntfInstance_l.apEventQueueInst[kEventQueueK2U]);

    if (drvIntfInstance_l.apEventQueueInst[kEventQueueU2K] != NULL)
    circbuf_disconnect(drvIntfInstance_l.apEventQueueInst[kEventQueueU2K]);
}

//------------------------------------------------------------------------------
/**
\brief  Initialize user layer error handler memory

Retrieves the shared memory for the error handler module. This memory is only
accessible to user space through ioctl calls.

\return Returns tOplkError error code.

*/
//------------------------------------------------------------------------------
static tOplkError initErrHndl(void)
{
    tDualprocReturn     dualRet;
    UINT8*              pBase;
    size_t              span;

    if (!drvIntfInstance_l.fDriverActive)
        return kErrorNoResource;

    if (drvIntfInstance_l.pErrorObjects != NULL)
        return kErrorInvalidOperation;

    dualRet = dualprocshm_getMemory(drvIntfInstance_l.dualProcDrvInst, DUALPROCSHM_BUFF_ID_ERRHDLR,
                                    &pBase, &span, FALSE);
    if (dualRet != kDualprocSuccessful)
    {
        DEBUG_LVL_ERROR_TRACE("%s() couldn't get Error counter buffer(%d)\n",
                              __func__, dualRet);
        return kErrorNoResource;
    }

    if (span < sizeof(tErrHndObjects))
    {
        DEBUG_LVL_ERROR_TRACE("%s: Error Handler Object Buffer too small\n",
                              __func__);
        return kErrorNoResource;
    }

    drvIntfInstance_l.pErrorObjects = (tErrHndObjects*)pBase;

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Free error handler memory

*/
//------------------------------------------------------------------------------
static void exitErrHndl(void)
{
    if (drvIntfInstance_l.pErrorObjects != NULL)
    {
        dualprocshm_freeMemory(drvIntfInstance_l.dualProcDrvInst, DUALPROCSHM_BUFF_ID_ERRHDLR, FALSE);
        drvIntfInstance_l.pErrorObjects = NULL;
    }
}

//------------------------------------------------------------------------------
/**
\brief  Initialize DLL queues for user layer

This routine retrieves the DLL queues shared between user and kernel stack, for
user layer. The queues are located in PCIe memory and are accessed using
circular buffer and dualprocshm library.

\return Returns tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError initDllQueues(void)
{
    tCircBufError    circError = kCircBufOk;

    if (!drvIntfInstance_l.fDriverActive)
        return kErrorNoResource;

    circError = circbuf_connect(CIRCBUF_DLLCAL_TXGEN, &drvIntfInstance_l.apDllQueueInst[kDllCalQueueTxGen]);
    if (circError != kCircBufOk)
    {
        TRACE("PLK : Could not allocate CIRCBUF_DLLCAL_TXGEN circbuffer\n");
        return kErrorNoResource;
    }

    circError = circbuf_connect(CIRCBUF_DLLCAL_TXNMT, &drvIntfInstance_l.apDllQueueInst[kDllCalQueueTxNmt]);
    if (circError != kCircBufOk)
    {
        TRACE("PLK : Could not allocate CIRCBUF_DLLCAL_TXNMT circbuffer\n");
        return kErrorNoResource;
    }

    circError = circbuf_connect(CIRCBUF_DLLCAL_TXSYNC, &drvIntfInstance_l.apDllQueueInst[kDllCalQueueTxSync]);
    if (circError != kCircBufOk)
    {
        TRACE("PLK : Could not allocate CIRCBUF_DLLCAL_TXSYNC circbuffer\n");
        return kErrorNoResource;
    }

    //TODO: VETH to be integrated later
    /*    circError = circbuf_connect(CIRCBUF_DLLCAL_TXVETH, &drvIntfInstance_l.dllQueueInst[kDllCalQueueTxVeth]);

        if (circError != kCircBufOk)
        {
            TRACE("PLK : Could not allocate CIRCBUF_DLLCAL_TXVETH circbuffer\n");
            return kErrorNoResource;
        }
    */

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Free DLL queues for user layer

*/
//------------------------------------------------------------------------------
static void exitDllQueues(void)
{
    if (drvIntfInstance_l.apDllQueueInst[kDllCalQueueTxGen] != NULL)
    circbuf_disconnect(drvIntfInstance_l.apDllQueueInst[kDllCalQueueTxGen]);

    if (drvIntfInstance_l.apDllQueueInst[kDllCalQueueTxNmt] != NULL)
    circbuf_disconnect(drvIntfInstance_l.apDllQueueInst[kDllCalQueueTxNmt]);

    if (drvIntfInstance_l.apDllQueueInst[kDllCalQueueTxSync] != NULL)
    circbuf_disconnect(drvIntfInstance_l.apDllQueueInst[kDllCalQueueTxSync]);
    /*
        //TODO: VETH to be integrated later
    if (drvInstance_l.dllQueueInst[kDllCalQueueTxVeth] != NULL)
        circbuf_disconnect(drvIntfInstance_l.dllQueueInst[kDllCalQueueTxVeth]);
    */
}

//------------------------------------------------------------------------------
/**
\brief  Write data into DLL queue

Writes the data into specified DLL queue shared between user and kernel.

\param  pDllCircBuffInst_p  Pointer to the DLL queue instance.
\param  pData_p             Pointer to the data to be inserted.
\param  pDataSize_p         Pointer to size of data.

\return Returns tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError insertDataBlock(tCircBufInstance* pDllCircBuffInst_p,
                                  UINT8* pData_p, UINT* pDataSize_p)
{
    tOplkError          ret = kErrorOk;
    tCircBufError       error;

    if (!drvIntfInstance_l.fDriverActive)
        return kErrorNoResource;

    if (pDllCircBuffInst_p == NULL)
    {
        ret = kErrorInvalidInstanceParam;
        goto Exit;
    }

    error = circbuf_writeData(pDllCircBuffInst_p, pData_p, *pDataSize_p);
    switch (error)
    {
        case kCircBufOk:
            break;

        case kCircBufExceedDataSizeLimit:
        case kCircBufBufferFull:
            ret = kErrorDllAsyncTxBufferFull;   //XXX Only ASync??
            break;

        case kCircBufInvalidArg:
        default:
            ret = kErrorNoResource;
            break;
    }

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Map memory to user space

Maps the specified memory into user space.

\param  pMemInfo_p          Pointer to the memory map information structure for the
                            memory region to map.

\return Returns tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError mapMemory(tMemInfo* pMemInfo_p)
{
    //FIXME Rework this function to make some sense or remove this and rework and
    // redocument all calling functions.
    pMemInfo_p->pUserVa = pMemInfo_p->pKernelVa;

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Unmap memory from user space

Unmap the specified memory, mapped into user space

\param  pMemInfo_p          Pointer to the memory map information structure for the
                            memory region to unmap.

\return Returns tOplkError error code.
*/
//------------------------------------------------------------------------------
static void unmapMemory(tMemInfo* pMemInfo_p)
{
    //FIXME Rework this function to make some sense or remove this and rework and
    // redocument all calling functions.
    pMemInfo_p->pKernelVa = NULL;
    pMemInfo_p->pUserVa = NULL;
    pMemInfo_p->memSize = 0;
}

///\}
