/**
********************************************************************************
\file   memmap/memmap-linuxpcie.c

\brief  Memory mapping implementation for openPOWERLINK PCIe driver on Linux

This file contains the architecture specific memory mapping implementation
for Linux systems using openPOWERLINK PCIe driver.

\ingroup module_lib_memmap
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2015, Kalycito Infotech Private Limited.
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
#include <common/oplkinc.h>
#include <common/memmap.h>
#include <common/driver.h>
#include <user/ctrlucal.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <errno.h>

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// module global vars
//------------------------------------------------------------------------------

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

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static INT fd_l;
static UINT8 aAsyncFrameSwapBuf_l[C_DLL_MAX_ASYNC_MTU];
tMemmap     memmap;
ULONG offset;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Initialize memory mapping

The function initializes the memory mapping service.

\return The function returns a tMemMapReturn error code.

\ingroup module_lib_memmap
*/
//------------------------------------------------------------------------------
tMemMapReturn memmap_init(void)
{
    fd_l = ctrlucal_getFd();
    return kMemMapOk;
}

//------------------------------------------------------------------------------
/**
\brief  Shut down memory mapping

The function shuts down the memory mapping service.

\return The function returns a tMemMapReturn error code.

\ingroup module_lib_memmap
*/
//------------------------------------------------------------------------------
tMemMapReturn memmap_shutdown(void)
{
    fd_l = (INT)NULL;
    return kMemMapOk;
}

//------------------------------------------------------------------------------
/**
\brief  Map kernel buffer

The function maps a kernel buffer address.
\note   This implementation uses the ioctl interface to the PCIe driver to copy
        the data from kernel space to the user space.

\param  pKernelBuffer_p     The pointer to the kernel buffer.
\param  bufferSize_p        The size of the kernel buffer.

\return The functions returns the pointer to the mapped kernel buffer.

\ingroup module_lib_memmap
*/
//------------------------------------------------------------------------------
void* memmap_mapKernelBuffer(void* pKernelBuffer_p, UINT bufferSize_p)
{
    INT         ret = 0;

    /*
    tMemmap     memmap;

    memmap.pKernelBuf = pKernelBuffer_p;
    memmap.pUserBuf = aAsyncFrameSwapBuf_l;
    memmap.memSize = bufferSize_p;

    if ((ret = ioctl(fd_l, PLK_CMD_MEMMAP_MAP_MEM, &memmap)) != 0)
    {
        DEBUG_LVL_ERROR_TRACE("%s() error %d\n", __func__, ret);
        memmap.pUserBuf = NULL;
    }
     /*/

    memmap.pKernelBuf = (UINT8*)((ULONG)pKernelBuffer_p & ~(sysconf(_SC_PAGE_SIZE) - 1));
    memmap.pUserBuf = aAsyncFrameSwapBuf_l;
    memmap.memSize = bufferSize_p;

    offset = (ULONG)pKernelBuffer_p & (sysconf(_SC_PAGE_SIZE) - 1);
    printf("memmap: of: 0x%X, kenPg: 0x%X\n", offset, (ULONG)memmap.pKernelBuf);
    memmap.pUserBuf = mmap(NULL, memmap.memSize + 2* getpagesize(), PROT_READ, MAP_SHARED,
                       fd_l, (ULONG)memmap.pKernelBuf);
    if (memmap.pUserBuf == MAP_FAILED)
    {
        DEBUG_LVL_ERROR_TRACE("%s() mmap failed!\n", __func__);
        memmap.pUserBuf = NULL;
    }
    else
        memmap.pUserBuf = (UINT8*)((ULONG)memmap.pUserBuf + offset);

//    if ((ret = ioctl(fd_l, PLK_CMD_PDO_MAP_OFFSET, &offset)) != 0)
//    {
//        DEBUG_LVL_ERROR_TRACE("%s() error %d\n", __func__, ret);
//        memmap.pUserBuf = NULL;
//    }
//    else
//    {
//        memmap.pUserBuf = (UINT8*)((size_t)(memmap.pUserBuf) + (size_t)offset);
//    }

    //*/
    return memmap.pUserBuf;
}

//------------------------------------------------------------------------------
/**
\brief  Disconnect from a memory mapping

The function disconnects from a memory mapping.

\param  pBuffer_p       The pointer to the previously mapped buffer.

\ingroup module_lib_memmap
*/
//------------------------------------------------------------------------------
void memmap_unmapKernelBuffer(void* pBuffer_p)
{
    /*
    UNUSED_PARAMETER(pBuffer_p);
    /*/
    if (memmap.pUserBuf != pBuffer_p)
    {
        DEBUG_LVL_ERROR_TRACE("munmap called with unknown memory address\n");
    }

    pBuffer_p = (UINT8*)((size_t)(pBuffer_p) - (size_t)offset);
    if (munmap(pBuffer_p, memmap.memSize + getpagesize()) != 0)
    {
        DEBUG_LVL_ERROR_TRACE("%s() munmap failed (%s)\n", __func__, strerror(errno));
    }

    memmap.pUserBuf = NULL;
    memmap.pKernelBuf = NULL;
    //*/
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

/// \}
