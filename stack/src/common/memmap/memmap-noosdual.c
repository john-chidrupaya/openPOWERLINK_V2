/**
********************************************************************************
\file   memmap/memmap-noosdual.c

\brief  Memory mapping implementation for non-os systems with shared memory

This file contains the architecture specific memory mapping implementation
for systems without operating system (e.g. microcontrollers, Nios II, Microblaze)
and shared memory interface.

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
#include <common/target.h>

#include <dualprocshm.h>

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
typedef struct
{
    tDualProcInstance       localProcInst;
    tDualProcInstance       remoteProcInst;
    UINT32                  localProcSharedMemBaseAddr;
    UINT32                  remoteProcSharedMemBaseAddr;
} tMemMapInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tMemMapInstance  memMapInstance_l;

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
    tDualprocReturn         ret = kDualprocSuccessful;
    tDualprocDrvInstance*    pDrvInst = NULL;

    OPLK_MEMSET(&memMapInstance_l, 0, sizeof(tMemMapInstance));
    memMapInstance_l.localProcInst = dualprocshm_getLocalProcInst();
    memMapInstance_l.remoteProcInst = dualprocshm_getRemoteProcInst();

    pDrvInst = dualprocshm_getLocalProcDrvInst();
    if (pDrvInst == NULL)
    {
    	DEBUG_LVL_ERROR_TRACE("%s DPSHM interface instance not found!\n", __func__);
    	return kMemMapNoResource;
    }

    // read the local processor's shared memory base address
    ret = dualprocshm_readSharedMemAddr(pDrvInst, memMapInstance_l.localProcInst,
    		                            (UINT8*) (&memMapInstance_l.localProcSharedMemBaseAddr));
    if (ret != kDualprocSuccessful)
    {
    	DEBUG_LVL_ERROR_TRACE("%s DPSHM local shared memory read failed!\n", __func__);
    	return kMemMapNoResource;
    }

    // Bridge is initialized in ctrl module, so no wait to read the shared memory
    ret = dualprocshm_readSharedMemAddr(pDrvInst, memMapInstance_l.remoteProcInst,
    		                            (UINT8*) (&memMapInstance_l.remoteProcSharedMemBaseAddr));
    if (ret != kDualprocSuccessful)
    {
    	DEBUG_LVL_ERROR_TRACE("%s DPSHM remote shared memory read failed!\n", __func__);
    	return kMemMapNoResource;
    }

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
    return kMemMapOk;
}

//------------------------------------------------------------------------------
/**
\brief  Map kernel buffer

The function maps a kernel buffer address.

\param  pKernelBuffer_p     The pointer to the kernel buffer.

\return The functions returns the pointer to the mapped kernel buffer.

\ingroup module_lib_memmap
*/
//------------------------------------------------------------------------------
void* memmap_mapKernelBuffer(void* pKernelBuffer_p)
{
    UINT8*              pBuffer = NULL;

    pBuffer = ((UINT32) pKernelBuffer_p
               - memMapInstance_l.remoteProcSharedMemBaseAddr
               + memMapInstance_l.localProcSharedMemBaseAddr);

    return (void*)pBuffer;
}

//------------------------------------------------------------------------------
/**
\brief  Disconnect from a memory mapping

The function disconnects from a memory mapping.

\param  pBuffer_p           The pointer to the previously mapped buffer.

\ingroup module_lib_memmap
*/
//------------------------------------------------------------------------------
void memmap_unmapKernelBuffer(void* pBuffer_p)
{
    UNUSED_PARAMETER(pBuffer_p);
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

/// \}
