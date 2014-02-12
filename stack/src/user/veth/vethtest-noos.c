/**
********************************************************************************
\file   vethtest-noos.c

\brief  Implementation of virtual Ethernet driver for noos - test framework

This file contains the test framework of the virtual Ethernet driver for
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

#include <user/vethtestapi-noos.h>

#include <user/vethapi-noos.h>

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

#define VETH_TEST_MAC_ADDR_LEN    6       ///< Length of the MAC address

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------

/**
 * \brief Test framework module instance
 */
typedef struct
{
    tVethTestCbRcv      pfnRcvCb;  ///< Frame received callback function
} tVEthTestInstance;


//---------------------------------------------------------------------------
// module global vars
//---------------------------------------------------------------------------

/**
 * \brief Test framework example transmit data
 *
 *  Frame 22 (42 bytes on wire, 42 bytes captured)
 *  Ethernet II, Src: CamilleB_56:78:9a (00:12:34:56:78:9a), Dst: Broadcast (ff:ff:ff:ff:ff:ff)
 *  Destination: Broadcast (ff:ff:ff:ff:ff:ff)
 *  Source: CamilleB_56:78:9a (00:12:34:56:78:9a)
 *  Type: ARP (0x0806)
 *  Address Resolution Protocol (request)
 *  Hardware type: Ethernet (0x0001)
 *  Protocol type: IP (0x0800)
 *  Hardware size: 6
 *  Protocol size: 4
 *  Opcode: request (0x0001)
 *  Sender MAC address: CamilleB_56:78:9a (00:12:34:56:78:9a)
 *  Sender IP address: 192.168.100.1 (192.168.100.1) (correct last byte: Node ID)
 *  Target MAC address: 00:00:00_00:00:00 (00:00:00:00:00:00)
 *  Target IP address: 192.168.100.240 (192.168.100.240)
 */
static UINT8 aNonPlkData[] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
        0x0, 0x12, 0x34, 0x56, 0x78, 0x9a,
        0x8, 0x6,
        0x0, 0x1,
        0x8, 0x0,
        0x6,
        0x4,
        0x0, 0x1,
        0x0, 0x12, 0x34, 0x56, 0x78, 0x9a,
        0xc0, 0xa8, 0x64, 0x01,
        0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
        0xc0, 0xa8, 0x64, 0xF0 };


static tVEthTestInstance vethTestInstance_l;

//---------------------------------------------------------------------------
// local function prototypes
//---------------------------------------------------------------------------

static void vethtest_addressChanged(UINT32 ipAddr_p, UINT32 subnetMask_p, UINT16 mtu_p);
static tOplkError vethtest_frameReceived(UINT8* pFrame_p, UINT32 frameSize_p);

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------


//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Initialize virtual Ethernet test framework

\param  pfnVethTestCbRcv_p          Frame received user callback function

\return The function returns a tOplkError error code.

\ingroup module_veth
*/
//------------------------------------------------------------------------------
tOplkError vethtest_apiInitialize(tVethTestCbRcv pfnVethTestCbRcv_p)
{
    tOplkError  ret = kErrorOk;
    UINT8       *pbEthMac;

    OPLK_MEMSET(&vethTestInstance_l, 0, sizeof(tVEthTestInstance));

    // make receive callback global
    vethTestInstance_l.pfnRcvCb = pfnVethTestCbRcv_p;

    // register address changed callback function
    veth_apiRegNetAddressCb(vethtest_addressChanged);

    // get MAC address
    pbEthMac = veth_apiGetEthMac();

    //set correct MAC address
    OPLK_MEMCPY(aNonPlkData+6, pbEthMac, sizeof(UINT8) * VETH_TEST_MAC_ADDR_LEN );
    OPLK_MEMCPY(aNonPlkData+22, pbEthMac, sizeof(UINT8) * VETH_TEST_MAC_ADDR_LEN );

    // Register receive callback
    veth_apiRegReceiveHandlerCb(vethtest_frameReceived);

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Send a ARP test frame to the master which responds

\return The function returns a tOplkError error code.

\ingroup module_veth
*/
//------------------------------------------------------------------------------
tOplkError vethtest_apiTransmit(void)
{
    tOplkError  ret = kErrorOk;

    //transmit test frame
    ret = veth_apiTransmit(aNonPlkData, sizeof(aNonPlkData));

    return ret;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//

/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  IP address changed callback function

\param ipAddr_p         The new IP address
\param subnetMask_p     The new subnet mask of the network
\param mtu_p            The new maximum transmission unit of the network

\ingroup module_veth
*/
//------------------------------------------------------------------------------
static void vethtest_addressChanged(UINT32 ipAddr_p, UINT32 subnetMask_p, UINT16 mtu_p)
{
    UNUSED_PARAMETER(subnetMask_p);
    UNUSED_PARAMETER(mtu_p);

    // set correct IP address to test frame
    aNonPlkData[31] = (UINT8)(0x000000FF & ipAddr_p);
}

//------------------------------------------------------------------------------
/**
\brief  Frame received callback function

\param pFrame_p        Pointer to the received payload
\param frameSize_p     Size of the received frame

\return kErrorOk

\ingroup module_veth
*/
//------------------------------------------------------------------------------
static tOplkError vethtest_frameReceived(UINT8* pFrame_p, UINT32 frameSize_p)
{
    tOplkError ret = kErrorOk;

    //PRINTF("VETHTEST: Frame received with size %d\n", frameSize_p);

    if(vethTestInstance_l.pfnRcvCb != NULL)
    {
        // Inform user application
        vethTestInstance_l.pfnRcvCb(&pFrame_p, frameSize_p);
    }

    // Frame received and processed -> free the memory!
    veth_apiReleaseRxFrame(pFrame_p, frameSize_p);

    return ret;
}

///\}
