/**
********************************************************************************
\file   vethapi-noos.h

\brief  Implementation of virtual Ethernet driver for noos nodes

*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2012, SYSTEC electronic GmbH
Copyright (c) 2012, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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

#ifndef _INC_vethapi_noos_H_
#define _INC_vethapi_noos_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------

#include <oplk/oplkinc.h>

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------

typedef void (* tVethCbDefGate) ( UINT32 defGateway_p );
typedef void (* tVethCbNetAddr) ( UINT32 ipAddr_p, UINT32 subNetMask_p, UINT16 mtu_p );
typedef tOplkError (* tVethCbFrameRcv) ( UINT8* pFrame_p, UINT32 frameSize_p );

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------

#ifdef __cplusplus
extern "C" {
#endif

tOplkError veth_apiTransmit(UINT8* pFrame_p, UINT16 frameSize);
tOplkError veth_apiReleaseRxFrame(UINT8* pFrame_p, UINT16 length_p);
void veth_apiRegReceiveHandlerCb(tVethCbFrameRcv pfnFrameReceivedCb_p);
void veth_apiRegDefaultGatewayCb(tVethCbDefGate pfnDefGatewayCb_p);
void veth_apiRegNetAddressCb(tVethCbNetAddr pfnNetAddrCb_p);
UINT8* veth_apiGetEthMac(void);

#ifdef __cplusplus
}
#endif

#endif /* _INC_vethapi_noos_H_ */


