/**
********************************************************************************
\file   hostiflib-altera_arm.h

\brief  Host Interface Library - For Altera Cyclone-V ARM target

This header file provides specific macros for Altera Cyclone-V ARM CPU.

*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2014, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
Copyright (c) 2015, Kalycito Infotech Private Ltd.
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

#ifndef _INC_hostiflib-altera_arm_H_
#define _INC_hostiflib-altera_arm_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <stdint.h>
#include <unistd.h>

#include <system.h>

#include <socal/socal.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
// Get hostinterface base address from system.h
#if defined(HOSTINTERFACE_0_BASE)
#define HOSTIF_BASE                         HOSTINTERFACE_0_BASE
#else
#define HOSTIF_BASE                         0x10000000  //FIXME: Use multiplex ipcore base here
#endif

#define HOSTIF_IRQ_IC_ID                    0           ///< Irq Controller Id
#define HOSTIF_IRQ                          0           ///< Irq Id

#define HOSTIF_SYNC_IRQ_TARGET_CPU          0x1 // dual Core

/// cache
#warning "Cache bypassing not available"
#define HOSTIF_MAKE_NONCACHEABLE(ptr)       (void*)(((unsigned long)ptr))

#define HOSTIF_UNCACHED_MALLOC(size)        malloc(size)
#define HOSTIF_UNCACHED_FREE(ptr)           free(ptr)

/// sleep
#define HOSTIF_USLEEP(x)                    hostif_usSleep(x)
#define HOSTIF_BRIDGE_INIT_TIMEOUT_MS       10

/// hw access
#define HOSTIF_RD32(base, offset)           alt_read_word((unsigned int)base + (unsigned int)offset)
#define HOSTIF_RD16(base, offset)           alt_read_hword((unsigned int)base + (unsigned int)offset)
#define HOSTIF_RD8(base, offset)            alt_read_byte((unsigned int)base + (unsigned int)offset)

#define HOSTIF_WR32(base, offset, dword)    alt_write_word((unsigned int)base + (unsigned int)offset, dword)
#define HOSTIF_WR16(base, offset, word)     alt_write_hword((unsigned int)base + (unsigned int)offset, word)
#define HOSTIF_WR8(base, offset, byte)      alt_write_byte((unsigned int)base + (unsigned int)offset, byte)

#define HOSTIF_INLINE                       inline
//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------
void hostif_usSleep(uint32_t usDelay_p);

#endif /* _INC_hostiflib-altera_arm_H_ */
