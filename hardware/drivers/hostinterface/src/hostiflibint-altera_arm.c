/**
********************************************************************************
\file   hostiflibint_arm.c

\brief  Host Interface Library - Driver Implementation for Altera Cyclone-V ARM

The file contains the high level driver for the host interface library for
Altera Cyclone-V ARM targets.

\ingroup module_hostiflib
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

/**
********************************************************************************
\defgroup   module_hostiflib    Host Interface Library
\ingroup    libraries

The host interface library provides a software interface for using the host
interface IP-Core. It provides several features like queues and linear memory
modules.
*******************************************************************************/

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <alt_interrupt.h>
#include <alt_timers.h>
#include <alt_globaltmr.h>
#include <alt_clock_manager.h>

#include "hostiflib.h"
#include "hostiflib_target.h"
#include "hostiflibint.h"

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
static tHostifIrqCb    pfnIrqCb_l = NULL;
//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static void cs3ISR(uint32_t icciar_p, void* pArg_p);

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Register interrupt service routine

This function registers the interrupt service routine in the host processor
interrupt services.

\param  pfnIrqCb_p              The interrupt service routine callback
\param  pArg_p                  Argument pointer provided to the callback

\return The function returns a tHostifReturn error code.
\retval kHostifSuccessful       The interrupt service routine was registered
                                successfully.

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
tHostifReturn hostif_sysIrqRegHandler(tHostifIrqCb pfnIrqCb_p, void* pArg_p)
{
    ALT_INT_INTERRUPT_t    irqId = ALT_INT_INTERRUPT_F2S_FPGA_IRQ0 + HOSTIF_IRQ;

    if (pfnIrqCb_p == NULL)
    {
        alt_int_isr_unregister(irqId);
    }
    else
    {
        if ( alt_int_isr_register(irqId, cs3ISR, pArg_p) != ALT_E_SUCCESS)
        {
            return kHostifUnspecError;
        }
        else
        {
            pfnIrqCb_l = pfnIrqCb_p;
            return kHostifSuccessful;
        }
    }
}

//------------------------------------------------------------------------------
/**
\brief  Enable interrupt for host interface driver

This function enables the interrupt for the host interface driver instance.

\param  fEnable_p   Determines if the interrupt must be enabled (TRUE) or
                    disabled (FALSE)

\return The function returns a tHostifReturn error code.
\retval kHostifSuccessful       Interrupt enable/disable failed

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
tHostifReturn hostif_sysIrqEnable(BOOL fEnable_p)
{
    ALT_INT_INTERRUPT_t     irqId = ALT_INT_INTERRUPT_F2S_FPGA_IRQ0 + HOSTIF_IRQ;

    if (fEnable_p)
    {
        if (alt_int_dist_trigger_set(irqId, ALT_INT_TRIGGER_LEVEL) != ALT_E_SUCCESS)
        {
            return kHostifNoResource;
        }
        else if (alt_int_dist_target_set(irqId, HOSTIF_SYNC_IRQ_TARGET_CPU) != ALT_E_SUCCESS)
        {
            return kHostifNoResource;
        }
        else if (alt_int_dist_enable(irqId) != ALT_E_SUCCESS)
        {
            // Set interrupt distributor target
            return kHostifNoResource;
        }
        else
            return kHostifSuccessful;
    }
    else
    {
        if (alt_int_dist_disable(irqId) == ALT_E_SUCCESS)
            return kHostifSuccessful;
        else
            return kHostifNoResource;
    }
}

//------------------------------------------------------------------------------
/**
\brief  Delay execution

\param usDelay_p        Delay in microseconds

\ingroup module_hostiflib
*/
//------------------------------------------------------------------------------
void hostif_usSleep(UINT32 usDelay_p)
{
    uint64_t        startTime = alt_globaltmr_get64();
    uint32_t        timerPrescaler = alt_globaltmr_prescaler_get() + 1;
    uint64_t        endTime;
    alt_freq_t      timerClkSrc;

    alt_clk_freq_get(ALT_CLK_MPU_PERIPH, &timerClkSrc);
    endTime = startTime + usDelay_p * ((timerClkSrc / timerPrescaler) / 1000000);

    while (alt_globaltmr_get64() < endTime)
    {
    }
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Primary ISR for hostinterface sync interrupt

The function receives the sync IRQ from HPS distributor and forwards to the
hostinterface module callback function

 \param icciar_p     The Interrupt Controller CPU Interrupt
                     Acknowledgement Register value (ICCIAR) value
                     corresponding to the current interrupt.

 \param pArg_p       argument to the function

*/
//------------------------------------------------------------------------------
static void cs3ISR(uint32_t icciar_p, void* pArg_p)
{
    // clear the interrupt in the distributor
    alt_int_dist_pending_clear(ALT_INT_INTERRUPT_F2S_FPGA_IRQ0 + HOSTIF_IRQ);
    if (pfnIrqCb_l != NULL)
        pfnIrqCb_l(pArg_p);
    return;
}

///\}
