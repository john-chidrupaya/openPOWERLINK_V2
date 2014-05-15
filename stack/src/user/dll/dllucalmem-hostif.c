/**
********************************************************************************
\file   dllucalmem-hostif.c

\brief  Source file for user DLL CAL memory module

This file contains an implementation of the user DLL CAL memory module for
host interface IP-Core. It uses the MMU hardware logic to access memory in the
kernel layer.

\ingroup module_dllucal
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2014, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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
#include <user/dllucal.h>
#include <hostiflib.h>

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

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Get Rx buffer from kernel layer

The function gets the frame pointer to the Rx buffer in kernel layer. The returned
pointer can be used by the user layer to access frame data.

\param  pFrame_p    Reference to Rx buffer in kernel layer

\return The function returns a pointer to the Rx buffer.
\retval NULL    The Rx buffer cannot be accessed.
*/
//------------------------------------------------------------------------------
tPlkFrame* dllucal_getRxBufferInKernel(tPlkFrame* pFrame_p)
{
    tHostifReturn   ret;
    UINT8*          pBuffer;

    ret = hostif_dynBufAcquire(hostif_getInstance(0), (UINT32)pFrame_p, &pBuffer);
    if(ret != kHostifSuccessful)
    {
        DEBUG_LVL_ERROR_TRACE("%s Dynamic buffer acquire failed (%d)\n", __func__, ret);
        pBuffer = NULL;
    }

    return (tPlkFrame*)pBuffer;
}

//------------------------------------------------------------------------------
/**
\brief  Free Rx buffer from kernel layer

The function frees Rx buffer acquired with dllucal_getRxBufferInKernel function.

\param  pFrame_p    Acquired Rx buffer to be freed

*/
//------------------------------------------------------------------------------
void dllucal_freeRxBufferInKernel(tPlkFrame* pFrame_p)
{
    tHostifReturn ret;

    ret = hostif_dynBufFree(hostif_getInstance(0), (UINT8*)pFrame_p);
    if(ret != kHostifSuccessful)
    {
        DEBUG_LVL_ERROR_TRACE("%s Dynamic buffer free failed (%d)\n", __func__, ret);
    }
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//

/// \name Private Functions
/// \{

///\}
