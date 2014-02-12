/**
********************************************************************************
\file   veth-noos.c

\brief  Implementation of virtual Ethernet driver for noos nodes

This file contains the implementation of the virtual Ethernet driver for
CNs/MNs without any operating system.

\ingroup module_veth
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2013, SYSTEC electronic GmbH
Copyright (c) 2013, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
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

#include <user/vethapi-noos.h>

#include <kernel/veth.h>        // Move this file to user space?!
#include <user/dllucal.h>
#include <user/nmtu.h>



//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

#define VETH_STATISTICS_ENABLE      FALSE

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

#if VETH_STATISTICS_ENABLE != FALSE
/**
 * \brief Virtual Ethernet module statistics
 */
typedef struct
{
    UINT32                  msgfree;        ///< Receive messages freed
    UINT32                  msgSent;        ///< Count of sent messages
    UINT32                  msgReceived;    ///< Count of received messages
    UINT32                  txBufferFull;   ///< Count of full transmit buffers
} tVEthStatistics;
#endif // VETH_STATISTICS_ENABLE != FALSE

/**
 * \brief Virtual Ethernet driver instance type
 */
typedef struct
{
    UINT8             aEthMac[6];             ///< The MAC address of the node
    UINT32            ipAddress;              ///< The IP address of the node
    UINT32            subnetMask;             ///< The subnet mask of the network
    UINT16            mtu;                    ///< The maximum transmission unit of the network

    UINT32            defaultGateway;         ///< The default gateway of the network

#if VETH_STATISTICS_ENABLE != FALSE
    tVEthStatistics   statistics;             ///< Virtual Ethernet driver statistics
#endif

    tVethCbFrameRcv   pfnFrameReceivedCb;     ///< Frame received callback function
    tVethCbDefGate    pfnDefaultGatewayCb;    ///< Default gateway changed callback function
    tVethCbNetAddr    pfnNetAddrCb;           ///< Subnet mask changed callback function
} tVEthInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static tVEthInstance vethInstance_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static tOplkError veth_recvFrame(tFrameInfo* pFrameInfo_p);
static tOplkError checkNmtState(void);


//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Add an virtual Ethernet driver instance

\param  aSrcMac_p             The MAC Address to set

\return The function returns a tOplkError error code.

\ingroup module_veth
*/
//------------------------------------------------------------------------------
tOplkError veth_addInstance(const UINT8 aSrcMac_p[6])
{
    tOplkError  ret = kErrorOk;

    OPLK_MEMSET(&vethInstance_l, 0 , sizeof(vethInstance_l));

    OPLK_MEMCPY(vethInstance_l.aEthMac, aSrcMac_p, sizeof(vethInstance_l.aEthMac) );

    // register receive callback function for incoming virtual Ethernet frames
    ret = dllucal_regNonPlkHandler(veth_recvFrame);

    return ret;
}


//------------------------------------------------------------------------------
/**
\brief  Delete an virtual Ethernet driver instance

\return The function returns a tOplkError error code.

\ingroup module_veth
*/
//------------------------------------------------------------------------------
tOplkError veth_delInstance(void)
{
    tOplkError  ret = kErrorOk;

    OPLK_MEMSET(&vethInstance_l, 0 , sizeof(vethInstance_l));

    // deregister receive callback function for incoming virtual Ethernet frames
    ret = dllucal_deregNonPlkHandler();

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Set the ip address, subnet mask and mtu

\param  ipAddress_p     The IP address of the node
\param  subnetMask_p    The subnet mask of the network
\param  mtu_p           The MTU of the network

\return kErrorOk

\ingroup module_veth
*/
//------------------------------------------------------------------------------
tOplkError veth_setIpAdrs(UINT32 ipAddress_p, UINT32 subnetMask_p, UINT16 mtu_p)
{
    tOplkError  ret = kErrorOk;

    vethInstance_l.ipAddress = ipAddress_p;
    vethInstance_l.subnetMask = subnetMask_p;
    vethInstance_l.mtu = mtu_p;

    // call address changed callback
    if(vethInstance_l.pfnNetAddrCb != NULL)
    {
        vethInstance_l.pfnNetAddrCb(ipAddress_p, subnetMask_p, mtu_p);
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Set the default gateway

\param  defaultGateway_p        The default gateway to set

\return kErrorOk

\ingroup module_veth
*/
//------------------------------------------------------------------------------
tOplkError veth_setDefaultGateway(UINT32 defaultGateway_p)
{
    tOplkError  ret = kErrorOk;

    vethInstance_l.defaultGateway = defaultGateway_p;

    // call gateway changed callback
    if(vethInstance_l.pfnDefaultGatewayCb != NULL)
    {
        vethInstance_l.pfnDefaultGatewayCb(defaultGateway_p);
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Send a frame over the virtual Ethernet driver

\param pFrame_p     Pointer to the frame to send
\param frameSize    Size of the frame to send

\return tOplkError
\retval kErrorOk                   Message sent successfully
\retval kErrorApiInvalidParam      Maximum MTU is reached
\retval kErrorDllAsyncTxBufferFull Internal buffer is full
\retval kErrorInvalidOperation     Send not allowed in this state

\ingroup module_veth
*/
//------------------------------------------------------------------------------
tOplkError veth_apiTransmit(UINT8* pFrame_p, UINT16 frameSize)
{
    tOplkError      ret = kErrorOk;
    tFrameInfo      frameInfo;

    // Check if CN is already in the right NMT state
    ret = checkNmtState();
    if(ret != kErrorOk)
    {
        goto Exit;
    }

    //check MTU
    if(frameSize > vethInstance_l.mtu)
    {
        ret = kErrorApiInvalidParam;
        DEBUG_LVL_VETH_TRACE("veth_apiTransmit: Error while transmitting! MTU is set to "
                "%d and frame has size %d!\n", vethInstance_l.mtu, frameSize);
        goto Exit;
    }

    //save frame and size
    frameInfo.pFrame = (tPlkFrame *)pFrame_p;
    frameInfo.frameSize = frameSize;

    //call send function on DLL
    ret = dllucal_sendAsyncFrame(&frameInfo, kDllAsyncReqPrioGeneric, kDllAsyncReqBufferEth);
    switch(ret)
    {
#if VETH_STATISTICS_ENABLE != FALSE
        case kErrorOk:
            //set stats for the device
            vethInstance_l.statistics.msgSent++;
            break;
        case kErrorDllAsyncTxBufferFull:
            //buffer is full
            vethInstance_l.statistics.txBufferFull++;
            break;
#endif
        default:
            DEBUG_LVL_VETH_TRACE("veth_apiTransmit: dllucal_sendAsyncFrame returned 0x%02X\n", ret);
            break;
    }

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Free the received packet after processing

\param pFrame_p     Pointer to the frame to free

\ingroup module_veth
*/
//------------------------------------------------------------------------------
tOplkError veth_apiReleaseRxFrame(UINT8* pFrame_p, UINT16 length_p)
{
    tOplkError ret = kErrorOk;

    ret = dllucal_freeNonPlkFrame((tPlkFrame *)pFrame_p, length_p);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Register the default gateway callback

\param pfnFrameReceivedCb_p     Frame received callback function

\ingroup module_veth
*/
//------------------------------------------------------------------------------
void veth_apiRegReceiveHandlerCb(tVethCbFrameRcv pfnFrameReceivedCb_p)
{
    vethInstance_l.pfnFrameReceivedCb = pfnFrameReceivedCb_p;
}


//------------------------------------------------------------------------------
/**
\brief  Register the default gateway callback

\param pfnDefGatewayCb_p     Function pointer to the callback

\ingroup module_veth
*/
//------------------------------------------------------------------------------
void veth_apiRegDefaultGatewayCb(tVethCbDefGate pfnDefGatewayCb_p)
{
    vethInstance_l.pfnDefaultGatewayCb = pfnDefGatewayCb_p;
}

//------------------------------------------------------------------------------
/**
\brief  Register the network address callback

\param pfnNetAddrCb_p     Function pointer to the callback

\ingroup module_veth
*/
//------------------------------------------------------------------------------
void veth_apiRegNetAddressCb(tVethCbNetAddr pfnNetAddrCb_p)
{
    vethInstance_l.pfnNetAddrCb = pfnNetAddrCb_p;
}

//------------------------------------------------------------------------------
/**
\brief  Get the Ethernet MAC address

\return UINT8*
\retval Address     The IP address of the node

\ingroup module_veth
*/
//------------------------------------------------------------------------------
UINT8* veth_apiGetEthMac(void)
{
    return (UINT8 *)&vethInstance_l.aEthMac;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//

/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Receive a frame from the POWERLINK dll module

\param pFrameInfo_p         Frame info of the received payload

\return tOplkError
\retval kErrorOk         Successfully received a frame
\retval kErrorNoResource Invalid input parameters

\ingroup module_veth
*/
//------------------------------------------------------------------------------
static tOplkError veth_recvFrame(tFrameInfo* pFrameInfo_p)
{
tOplkError  ret = kErrorOk;

    DEBUG_LVL_VETH_TRACE("veth_recvFrame: FrameSize=%u\n", pFrameInfo_p->frameSize);
    DEBUG_LVL_VETH_TRACE("veth_recvFrame: SrcMAC=0x%llx\n", ami_getUint48Be(pFrameInfo_p->pFrame->aSrcMac));

    if(vethInstance_l.pfnFrameReceivedCb != NULL)
    {
        // Call receive handler for upper layer
        ret = vethInstance_l.pfnFrameReceivedCb((UINT8*)pFrameInfo_p->pFrame, pFrameInfo_p->frameSize);
#if VETH_STATISTICS_ENABLE != FALSE
        if(ret == kErrorOk)
        {
            // set receive statistics
            vethInstance_l.statistics.msgReceived++;
        }
#endif
    }

    return ret;
}


//------------------------------------------------------------------------------
/**
\brief  Check the NMT stack of the node

\return tOplkError
\retval kErrorOk               State of the node is valid
\retval kErrorInvalidOperation Invalid state of the node

\ingroup module_veth
*/
//------------------------------------------------------------------------------
static tOplkError checkNmtState(void)
{
    tOplkError ret = kErrorOk;
    tNmtState nmtState;

    //check POWERLINK state
    nmtState = nmtu_getNmtState();
    if (nmtState <= kNmtCsNotActive || nmtState == kNmtMsNotActive)
    {
        ret = kErrorInvalidOperation;
    }

    return ret;
}

///\}
