/**
********************************************************************************
\file   dualprocshm-pci.c

\brief  Dual Processor Library Support File - PCIe

This file provides specific function definitions to support PCIe interface using
dual processor library.

\ingroup module_dualprocshm
*******************************************************************************/
/*------------------------------------------------------------------------------
Copyright (c) 2014 Kalycito Infotech Private Limited
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

#include <dualprocshm.h>

#include <common/target.h>

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define DEFAULT_LOCK_ID             0x00    ///< Default lock Id
#define DYN_MEM_TABLE_ENTRY_SIZE    4       ///< Size of Dynamic table entry
//------------------------------------------------------------------------------
// module global vars
//------------------------------------------------------------------------------
static tDualprocHeader* pHeader_l;

//------------------------------------------------------------------------------
// global function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P R I V A T E   D E F I N I T I O N S                           //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Initialize target resources

This routine initializes the memory address for the processor instance assigned
to the calling processor.

\param  procInstance_p      Processor instance of the calling processor.

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
void dualprocshm_targetInit(UINT32 procInstance_p)
{
    pHeader_l = (UINT8*) pcieDrv_getBarAddr(OPLK_PCIEBAR_COMM_MEM);

    if (pHeader_l == NULL)
        return;

    if (procInstance_p == kDualProcFirst)
    {
        UINT8*   sharedMemBase = (UINT8*) pcieDrv_getBarAddr(OPLK_PCIEBAR_SHM);
        DPSHM_WRITE32(&pHeader_l->sharedMemBase, (UINT32) sharedMemBase);
        DUALPROCSHM_FLUSH_DCACHE_RANGE(&pHeader_l->sharedMemBase, sizeof(UINT32));
    }
}

//------------------------------------------------------------------------------
/**
\brief  Get common memory address for platform

Target specific routine to retrieve the base address of common memory between
two processors.

\param  pSize_p      Minimum size of the common memory on input, returns the
                     actual size of common memory as output.

\return Pointer to base address of common memory.

\ingroup module_dualprocshm
 */
//------------------------------------------------------------------------------
UINT8* dualprocshm_getCommonMemAddr(UINT16* pSize_p)
{
    UINT8*   pAddr;

    if (*pSize_p > MAX_COMMON_MEM_SIZE || pHeader_l == NULL)
    {
        TRACE("%s Common memory not available\n", __func__);
        return NULL;
    }

    pAddr = (UINT8*) pHeader_l + sizeof(tDualprocHeader);

    *pSize_p = MAX_COMMON_MEM_SIZE - 1;
    return pAddr;
}

//------------------------------------------------------------------------------
/**
\brief  Free common memory address

Target specific routine to release the base address of
common memory.

\param  pSize_p      Size of the common memory

\ingroup module_dualprocshm
 */
//------------------------------------------------------------------------------
void dualprocshm_releaseCommonMemAddr(UINT16 pSize_p)
{
    UNUSED_PARAMETER(pSize_p);
}

//------------------------------------------------------------------------------
/**
\brief  Get shared memory information for platform

Target specific routine to retrieve the shared memory base and size.

/param pSize_p     Pointer to size of the shared memory.

\return Pointer to base address of common memory.

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
UINT8* dualprocshm_targetGetSharedMemInfo(UINT32* pSize_p)
{
    UINT8*   pAddr;

    pAddr = (UINT8*) pcieDrv_getBarAddr(OPLK_PCIEBAR_SHM);

    if (pAddr == NULL || pSize_p == NULL)
        return NULL;

    *pSize_p = pcieDrv_getBarLength(OPLK_PCIEBAR_SHM);

    return pAddr;
}

//------------------------------------------------------------------------------
/**
\brief  Get remote shared memory base address

Target specific routine to retrieve the base address of shared memory on other
processor.

\return Base address of shared memory on other processor.

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
UINT64 dualprocshm_targetGetRemoteMemBase(void)
{
    if (pHeader_l == NULL)
        return -1;

    return (UINT64) pHeader_l->sharedMemBase;
}

//------------------------------------------------------------------------------
/**
\brief  Get dynamic mapping table base address

Target specific routine to retrieve the base address for storing the
dynamic mapping table.

\return Pointer to base address of dynamic mapping table.

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
UINT8* dualprocshm_getDynMapTableAddr(void)
{
    UINT8*     pAddr = (UINT8*) pHeader_l + sizeof(tDualprocHeader);

    if (pHeader_l == NULL)
        return NULL;

    pAddr = (UINT8*)(pAddr + MEM_ADDR_TABLE_OFFSET);
    return pAddr;
}

//------------------------------------------------------------------------------
/**
\brief  Free dynamic mapping table base address

Target specific routine to free the base address used for storing the
dynamic mapping table.

\ingroup module_dualprocshm
 */
//------------------------------------------------------------------------------
void dualprocshm_releaseDynMapTableAddr(void)
{
}

//------------------------------------------------------------------------------
/**
\brief  Get interrupt synchronization base address

Target specific routine to retrieve the base address for storing
interrupt synchronization registers.

\return Pointer to base address of interrupt synchronization registers.

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
UINT8* dualprocshm_getIntrMemAddr(void)
{
    UINT8*     pAddr = (UINT8*) pHeader_l + sizeof(tDualprocHeader);

    if (pHeader_l == NULL)
        return NULL;

    pAddr = (UINT8*)(pAddr + MEM_INTR_OFFSET);

    return pAddr;
}

//------------------------------------------------------------------------------
/**
\brief  Free interrupt synchronization base address

Target specific routine to free the base address used for storing
interrupt synchronization registers.

\ingroup module_dualprocshm
 */
//------------------------------------------------------------------------------
void dualprocshm_releaseIntrMemAddr()
{
}

//------------------------------------------------------------------------------
/**
\brief  Read data from memory

Target specific memory read routine.

\param  pBase_p    Address to read data from.
\param  size_p     Number of bytes to be read.
\param  pData_p    Pointer to store the read data.

\ingroup module_dualprocshm
 */
//------------------------------------------------------------------------------
void dualprocshm_targetReadData(UINT8* pBase_p, UINT16 size_p, UINT8* pData_p)
{
    if (pBase_p == NULL || pData_p == NULL)
    {
        TRACE("%s Invalid parameters\n", __FUNCTION__);
        return;
    }

    DUALPROCSHM_INVALIDATE_DCACHE_RANGE(pBase_p, size_p);

    DUALPROCSHM_MEMCPY(pData_p, pBase_p, size_p);
}

//------------------------------------------------------------------------------
/**
\brief  Write data to memory

Target specific routine used to write data to the specified memory address.

\param  pBase_p      Address to write data to.
\param  size_p       Number of bytes to be written.
\param  pData_p      Pointer to memory containing data to be written.

\ingroup module_dualprocshm
 */
//------------------------------------------------------------------------------
void dualprocshm_targetWriteData(UINT8* pBase_p, UINT16 size_p, UINT8* pData_p)
{
    if (pBase_p == NULL || pData_p == NULL)
    {
        TRACE("%s Invalid parameters\n", __FUNCTION__);
        return;
    }

    DUALPROCSHM_MEMCPY(pBase_p, pData_p, size_p);

    DUALPROCSHM_FLUSH_DCACHE_RANGE(pBase_p, size_p);
}

//------------------------------------------------------------------------------
/**
\brief  Target specific memory lock routine(acquire)

This routine provides support for a token based lock using the common memory.
The caller needs to pass the base address and the token for locking a resource
such as memory buffers

\param  pBase_p         Base address of the lock memory
\param  lockToken_p     Token to be used for locking

\ingroup module_dualprocshm
 */
//------------------------------------------------------------------------------
void dualprocshm_targetAcquireLock(UINT8* pBase_p, UINT8 lockToken_p)
{
    UINT8    lock = 0;

    if (pBase_p == NULL)
    {
        TRACE("%s Invalid parameters\n", __FUNCTION__);
        return;
    }

    // spin till the passed token is written into memory
    do
    {
        lock = DPSHM_READ8(pBase_p);

        if (lock == DEFAULT_LOCK_ID)
        {
            DPSHM_WRITE8(pBase_p, lockToken_p);
            continue;
        }
    } while (lock != lockToken_p);
}

//------------------------------------------------------------------------------
/**
\brief  Target specific memory lock routine(release)

This routine is used to release a lock acquired before at a address specified

\param  pBase_p         Base address of the lock memory

\ingroup module_dualprocshm
 */
//------------------------------------------------------------------------------
void dualprocshm_targetReleaseLock(UINT8* pBase_p)
{
    UINT8    defaultlock = DEFAULT_LOCK_ID;

    if (pBase_p == NULL)
    {
        TRACE("%s Invalid parameters\n", __FUNCTION__);
        return;
    }

    DPSHM_WRITE8(pBase_p, defaultlock);

    DUALPROCSHM_FLUSH_DCACHE_RANGE(pBase_p, sizeof(UINT8));
}

//------------------------------------------------------------------------------
/**
\brief Register synchronization interrupt handler

The function registers the ISR for the target specific synchronization interrupt
used by the application for synchronization.

\param  callback_p              Interrupt handler.
\param  pArg_p                  Argument to be passed when calling the handler.

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
void dualprocshm_regSyncIrqHdl(targetSyncHdl callback_p, void* pArg_p)
{
    DPSHM_REG_SYNC_INTR(callback_p, pArg_p);
}

//------------------------------------------------------------------------------
/**
\brief Sync interrupt control routine

The function is used to enable or disable the sync interrupt.

\param  fEnable_p              Enable if TRUE, disable if FALSE.

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
void dualprocshm_enableSyncIrq(BOOL fEnable_p)
{
    if (fEnable_p)
        DPSHM_ENABLE_SYNC_INTR();
    else
        DPSHM_DISABLE_SYNC_INTR();
}

//------------------------------------------------------------------------------
/**
\brief  Write the buffer address in dynamic memory mapping table

\param  pInstance_p  Driver instance.
\param  index_p      Buffer index.
\param  addr_p       Address of the buffer.

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
void dualprocshm_targetSetDynBuffAddr(UINT8* pMemTableBase, UINT16 index_p, UINT32 addr_p)
{
    UINT32    tableEntryOffs = index_p * DYN_MEM_TABLE_ENTRY_SIZE;
    UINT32    offset = 0;

    if (addr_p != 0)
    {
        offset = (UINT32) (addr_p - (UINT32) pcieDrv_getBarAddr(OPLK_PCIEBAR_SHM));
    }

    DPSHM_WRITE32(pMemTableBase + tableEntryOffs, offset);
    DUALPROCSHM_FLUSH_DCACHE_RANGE((UINT32) (pMemTableBase + tableEntryOffs), sizeof(UINT32));
}

//------------------------------------------------------------------------------
/**
\brief  Read the buffer address from dynamic memory mapping table

\param  pInstance_p  Driver instance.
\param  index_p      Buffer index.

\return Address of the buffer requested.

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------

UINT8* dualprocshm_targetGetDynBuffAddr(UINT8* pMemTableBase, UINT16 index_p)
{
    UINT32    tableEntryOffs = index_p * DYN_MEM_TABLE_ENTRY_SIZE;
    UINT32    buffoffset = 0x00;
    UINT8*    bar0Addr = (UINT8*) pcieDrv_getBarAddr(OPLK_PCIEBAR_SHM);
    UINT8*    bufAddr;
    UINT8*    memEntryAddr = (pMemTableBase + tableEntryOffs);
    UINT      count = 0;

    if (bar0Addr == NULL)
        return NULL;

    DUALPROCSHM_INVALIDATE_DCACHE_RANGE((pMemTableBase + tableEntryOffs), sizeof(UINT32));
    buffoffset = DPSHM_READ32(memEntryAddr);

    while (buffoffset == 0)
    {
        if (count == 20)
        {
            TRACE("%s() Failed to get address for index %d\n", __FUNCTION__, index_p);
            return NULL;
        }
        DUALPROCSHM_MSLEEP(10);
        buffoffset = DPSHM_READ32(memEntryAddr);
        count++;
    }
    bufAddr = (bar0Addr + buffoffset);
    return bufAddr;
}

//------------------------------------------------------------------------------
/**
\brief  Map target memory address to local memory

This routines translates the address in target processor memory into local
memory.

\param  baseAddr_p   Base address in target processor memory.

\return Mapped address in local memory.

\ingroup module_dualprocshm
*/
//------------------------------------------------------------------------------
UINT8* dualprocshm_targetMapMem(UINT32 baseAddr_p)
{
    UINT8*    pLocalAddr;
    UINT8*    pShmBase = (UINT8*) pcieDrv_getBarAddr(OPLK_PCIEBAR_SHM);
    UINT32    offset;

    if (pShmBase == NULL || pHeader_l == NULL)
        return NULL;

    offset = baseAddr_p - pHeader_l->sharedMemBase;

    pLocalAddr = (UINT32) pShmBase + offset;

    return pLocalAddr;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

/// \}
