/**
********************************************************************************
\file   dualprocshm-target.h

\brief  Dual Processor Library - Target header file

This header file defines platform specific macros (e.g. data types) and selects
the platform specific header files (e.g. dualprocshm-zynq.h) which contains the
information of processors part of the platform.

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

#ifndef _INC_dualprocshm_target_H_
#define _INC_dualprocshm_target_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------

#if defined(__ZYNQ__)

#include "dualprocshm-zynq.h"

#elif defined(_KERNEL_MODE)

#include "dualprocshm-pcie.h"

#elif defined (__BR_ANTARESIF__)

#include "dualprocshm-pcie.h"

#else

#error "Platform is not supported! Please point the target platform file in dualprocshm-target.h "

#endif

#ifndef SECTION_DUALPROCSHM_ACQUIRE_LOCK
#define SECTION_DUALPROCSHM_ACQUIRE_LOCK
#endif
#ifndef SECTION_DUALPROCSHM_RELEASE_LOCK
#define SECTION_DUALPROCSHM_RELEASE_LOCK
#endif
#ifndef SECTION_DUALPROCSHM_AQ_BUFF_LOCK
#define SECTION_DUALPROCSHM_AQ_BUFF_LOCK
#endif
#ifndef SECTION_DUALPROCSHM_RE_BUFF_LOCK
#define SECTION_DUALPROCSHM_RE_BUFF_LOCK
#endif
#ifndef SECTION_DUALPROCSHM_IRQ_ENABLE
#define SECTION_DUALPROCSHM_IRQ_ENABLE
#endif
#ifndef SECTION_DUALPROCSHM_IRQ_SET
#define SECTION_DUALPROCSHM_IRQ_SET
#endif
#ifndef SECTION_DUALPROCSHM_IRQ_HDL
#define SECTION_DUALPROCSHM_IRQ_HDL
#endif

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
/**
\name Data types

If the following data types are not defined in the environment, then they are
set to those provided by stdint.h.
*/
/**@{*/
#ifndef _NTDEF_        // defined in ntdef.H, included by dualprocshm-winkernel.h
#ifndef INT
#define INT       int32_t
#endif

#ifndef UINT8
#define UINT8     uint8_t
#endif

#ifndef UINT16
#define UINT16    uint16_t
#endif

#ifndef UINT32
#define UINT32    uint32_t
#endif

#ifndef UINT64
#define UINT64    uint64_t
#endif

/**@}*/

#ifndef FALSE
#define FALSE     0x00
#endif

#ifndef TRUE
#define TRUE      0xFF
#endif

#endif // _NTDEF_

#ifndef BOOL
#if defined(_WIN32) || defined(_WIN64)
#define BOOL      unsigned char
#else
#define BOOL      uint8_t
#endif // _WIN32
#endif // BOOL

#ifndef UNUSED_PARAMETER
#define UNUSED_PARAMETER(par)    (void)par
#endif

#ifndef TRACE
#define TRACE(...)
#endif

#ifndef MAX_COMMON_MEM_SIZE
#define MAX_COMMON_MEM_SIZE        2048                         ///< Max common memory size
#endif

#ifndef MAX_DYNAMIC_BUFF_COUNT
#define MAX_DYNAMIC_BUFF_COUNT     20                           ///< Number of maximum dynamic buffers
#endif

#ifndef MAX_DYNAMIC_BUFF_COUNT
#define MAX_DYNAMIC_BUFF_SIZE      MAX_DYNAMIC_BUFF_COUNT * 4   ///< Max dynamic buffer size
#endif

#ifndef SHARED_MEM_BASE
#define SHARED_MEM_BASE            NULL
#endif

#ifndef COMMON_MEM_BASE
#define COMMON_MEM_BASE            NULL
#endif

#ifndef MEM_ADDR_TABLE_BASE
#define MEM_ADDR_TABLE_BASE        NULL
#endif

#ifndef MEM_INTR_BASE
#define MEM_INTR_BASE              NULL
#endif

//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------
/**
\brief Function type definition for target synchronization handle

This function callback is called by the synchronization event.
*/
typedef void (*targetSyncHdl)(void*);

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------

#ifdef __cplusplus
extern "C" {
#endif
UINT8* dualprocshm_getCommonMemAddr(UINT16* pSize_p);
UINT8* dualprocshm_getDynMapTableAddr(void);
UINT8* dualprocshm_getIntrMemAddr(void);
void    dualprocshm_releaseIntrMemAddr(void);
void   dualprocshm_targetReadData(UINT8* pBase_p, UINT16 size_p, UINT8* pData_p);
void   dualprocshm_targetWriteData(UINT8* pBase_p, UINT16 size_p, UINT8* pData_p);
void   dualprocshm_releaseCommonMemAddr(UINT16 pSize_p);
void   dualprocshm_releaseDynMapTableAddr(void);
void   dualprocshm_targetAcquireLock(UINT8* pBase_p, UINT8 lockToken_p) SECTION_DUALPROCSHM_ACQUIRE_LOCK;
void   dualprocshm_targetReleaseLock(UINT8* pBase_p) SECTION_DUALPROCSHM_RELEASE_LOCK;
void   dualprocshm_regSyncIrqHdl(targetSyncHdl callback_p, void* pArg_p);
void   dualprocshm_enableSyncIrq(BOOL fEnable_p);
void   dualprocshm_targetSetDynBuffAddr(UINT8* pMemTableBase, UINT16 index_p, UINT32 addr_p);
UINT8* dualprocshm_targetGetDynBuffAddr(UINT8* pMemTableBase, UINT16 index_p);
void   dualprocshm_targetInit(UINT32 procInstance_p);
UINT8* dualprocshm_targetMapMem(UINT32 baseAddr_p);
UINT64 dualprocshm_targetGetRemoteMemBase(void);
UINT8* dualprocshm_targetGetSharedMemInfo(UINT32* pSize_p);

#ifdef __cplusplus
}
#endif

#endif /* _INC_dualprocshm_target_H_ */
