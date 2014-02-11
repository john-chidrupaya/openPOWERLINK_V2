/**
********************************************************************************
\file   sdoudp-noos.c

\brief  SDO/UDP protocol abstraction layer module for NoOs nodes

This module implements the communication abstraction layer of the SDO stack to
the IP stack.

\ingroup module_sdoudp
*******************************************************************************/

/*------------------------------------------------------------------------------
* License Agreement
*
* Copyright 2013 BERNECKER + RAINER, AUSTRIA, 5142 EGGELSBERG, B&R STRASSE 1
* All rights reserved.
*
* Redistribution and use in source and binary forms,
* with or without modification,
* are permitted provided that the following conditions are met:
*
*   * Redistributions of source code must retain the above copyright notice,
*     this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above copyright notice,
*     this list of conditions and the following disclaimer
*     in the documentation and/or other materials provided with the
*     distribution.
*   * Neither the name of the B&R nor the names of its contributors
*     may be used to endorse or promote products derived from this software
*     without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
* THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
* A PARTICULAR PURPOSE ARE DISCLAIMED.
* IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
* THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
------------------------------------------------------------------------------*/

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <oplk/ami.h>
#include <user/sdoudp.h>
#include <user/nmtu.h>
#include <common/target.h>

#if defined(CONFIG_INCLUDE_SDO_UDP)

#if !defined(CONFIG_INCLUDE_VETH)
  #error "SDO/UDP for NoOs in enabled but the VETH module is disabled! \
Please enable the VETH module when using the SDO/UDP module."
#endif


#include <ip.h>             // IP stack main header file
#include <edrv2veth.h>      // Header for edrv to virtual Ethernet wrapper


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

#ifndef SDO_MAX_CONNECTION_UDP
#define SDO_MAX_CONNECTION_UDP  5
#endif

#define INADDR_ANY    0xc0a86401        ///< Default IP address

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------

/**
 * \brief SDO/UDP connection handle
 */
typedef struct
{
    unsigned long   ipAddr;     ///< IP address (in network byte order)
    unsigned int    port;       ///< Port (in network byte order)
} tSdoUdpCon;

/**
 *  \brief Instance of the SDO/UDP abstraction layer module
 */
typedef struct
{
    tSdoUdpCon              aSdoAbsUdpConnection[SDO_MAX_CONNECTION_UDP];
    tSequLayerReceiveCb     pfnSdoAsySeqCb;     ///< SDO sequence layer receive callback

    IP_STACK_H              ipStackH;           ///< IP stack handler
    ipState_enum            ipStatus;           ///< Status of the IP stack

    struct in_addr          ipAddr;             ///< Node IP address (platform endian)
    eth_addr                macAddr;            ///< Node MAC address
    unsigned int            localPort;          ///< Local port
} tSdoUdpInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------

static tSdoUdpInstance  sdoudpInstance_l;

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

static void sdoudpu_receiveFrom( void *arg, ip_udp_info *pInfo );

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Initialize SDO over UDP module

The function initializes the SDO over UDP module.

\param  pfnReceiveCb_p          Pointer to SDO sequence layer receive callback function.

\return The function returns a tOplkError error code.

\ingroup module_sdo_udp
*/
//------------------------------------------------------------------------------
tOplkError sdoudp_init(tSequLayerReceiveCb pfnReceiveCb_p)
{
    return sdoudp_addInstance(pfnReceiveCb_p);
}

//------------------------------------------------------------------------------
/**
\brief  Add an instance of a SDO over UDP module

The function adds an instance of a SDO over UDP module.

\param  pfnReceiveCb_p          Pointer to SDO sequence layer receive callback function.

\return The function returns a tOplkError error code.

\ingroup module_sdo_udp
*/
//------------------------------------------------------------------------------
tOplkError sdoudp_addInstance(tSequLayerReceiveCb pfnReceiveCb_p)
{
    tOplkError      ret = kErrorOk;

    // set instance variables to 0
    OPLK_MEMSET(&sdoudpInstance_l, 0x00, sizeof(sdoudpInstance_l));

    // assume to be in INIT_ARP at the beginning
    sdoudpInstance_l.ipStatus = IP_STATE_INIT_ARP;

    // save pointer to callback-function
    if (pfnReceiveCb_p != NULL)
    {
        sdoudpInstance_l.pfnSdoAsySeqCb = pfnReceiveCb_p;
    }
    else
    {
        ret = kErrorSdoUdpMissCb;
        goto Exit;
    }

    //set dummy ip-address (will be filled in EplSdoUdpuConfig)
    sdoudpInstance_l.ipAddr.S_un.S_addr = INADDR_ANY;   // 192.168.100.01

    // init edrv2veth converter module
    edrv2veth_init(&sdoudpInstance_l.macAddr);

    // initialize IP stack
    sdoudpInstance_l.ipStackH  =  ipInit(&sdoudpInstance_l.macAddr,
                                         &sdoudpInstance_l.ipAddr,
                                         &edrv2veth_transmit,
                                         NULL);
    if(sdoudpInstance_l.ipStackH == NULL)
    {
        ret = kErrorSdoUdpNoSocket;
        goto Exit;
    }

    ret = edrv2veth_setIpHandler(sdoudpInstance_l.ipStackH);
    if(ret != kErrorOk)
    {
        goto Exit;
    }

    ret = sdoudp_config(sdoudpInstance_l.ipAddr.S_un.S_addr, 0);

Exit:
    return ret;
}


//------------------------------------------------------------------------------
/**
\brief  Delete an instance of a SDO over UDP module

The function deletes an instance of a SDO over UDP module. It deletes the created
sockets and deletes the listener thread.

\return The function returns a tOplkError error code.

\ingroup module_sdo_udp
*/
//------------------------------------------------------------------------------
tOplkError sdoudp_delInstance(void)
{
tOplkError      Ret = kErrorOk;

    //call shutdown of IP-stack
    ipDestroy(sdoudpInstance_l.ipStackH);

    OPLK_MEMSET(&sdoudpInstance_l, 0 , sizeof(sdoudpInstance_l));

    return Ret;
}

//------------------------------------------------------------------------------
/**
\brief  Reconfigure socket

The function reconfigures socket with a new IP address. It is needed for
NMT_ResetConfiguration.

\param  ipAddr_p            IP address to configure.
\param  port_p              Port to configure

\return The function returns a tOplkError error code.

\ingroup module_sdo_udp
*/
//------------------------------------------------------------------------------
tOplkError sdoudp_config(ULONG ipAddr_p, UINT port_p)
{
    tOplkError    ret = kErrorOk;
    INT           error;
    UINT32        ipAddr;


    if (port_p == 0)
    {   // set UDP port to default port number
        port_p = C_SDO_EPL_PORT;
    }
    else if (port_p > 65535)
    {
        ret = kErrorSdoUdpSocketError;
        goto Exit;
    }

    // change IP address
    ipAddr = htonl(ipAddr_p);
    ipChangeAddress(sdoudpInstance_l.ipStackH, (struct in_addr *)&ipAddr);

    // open IP stack listen port
    error = ipUdpListen(sdoudpInstance_l.ipStackH, port_p, sdoudpu_receiveFrom, NULL);
    if(error < 0)
    {
        DEBUG_LVL_SDO_TRACE("EplSdoUdpuConfig: ipUdpListen() returned %d!\n", error);
        ret = kErrorSdoUdpInvalidHdl;
        goto Exit;
    }

    // remember local port
    sdoudpInstance_l.localPort = port_p;

Exit:
    return ret;

}


//------------------------------------------------------------------------------
/**
\brief  Initialize new connection

The function initializes a new connection.

\param  pSdoConHandle_p           Pointer for the new connection handle.
\param  targetNodeId_p            Node ID of the target.

\return The function returns a tOplkError error code.

\ingroup module_sdo_udp
*/
//------------------------------------------------------------------------------
tOplkError sdoudp_initCon(tSdoConHdl* pSdoConHandle_p, UINT targetNodeId_p)
{
    tOplkError     ret = kErrorOk;
    UINT           conCount;
    UINT           freeCon;
    tSdoUdpCon*    pSdoUdpCon;
    eth_addr       macAddr;
    eth_addr       macAddrZero;


    OPLK_MEMSET(&macAddrZero, 0, sizeof(eth_addr));
    OPLK_MEMSET(&macAddr, 0, sizeof(eth_addr));

    // get free entry in control structure
    conCount = 0;
    freeCon = SDO_MAX_CONNECTION_UDP;
    pSdoUdpCon = &sdoudpInstance_l.aSdoAbsUdpConnection[0];
    while (conCount < SDO_MAX_CONNECTION_UDP)
    {
        if ((htonl(pSdoUdpCon->ipAddr) & 0xFF) == targetNodeId_p)
        {   // existing connection to target node found
            // set handle
            *pSdoConHandle_p = (conCount | SDO_UDP_HANDLE);

            goto Exit;
        }
        else if ((pSdoUdpCon->ipAddr == 0)
                && (pSdoUdpCon->port == 0))
        {
            freeCon = conCount;
        }
        conCount++;
        pSdoUdpCon++;
    }

    if (freeCon == SDO_MAX_CONNECTION_UDP)
    {
        // error no free handle
        ret = kErrorSdoUdpNoFreeHandle;
    }
    else
    {
        pSdoUdpCon = &sdoudpInstance_l.aSdoAbsUdpConnection[freeCon];
        // save infos for connection
        pSdoUdpCon->port = C_SDO_EPL_PORT;
        pSdoUdpCon->ipAddr = sdoudpInstance_l.ipAddr.S_un.S_addr & 0xFFFFFF00;
        pSdoUdpCon->ipAddr |= (BYTE)targetNodeId_p;   // 192.168.100.uiTargetNodeId_p
        pSdoUdpCon->ipAddr = htonl(pSdoUdpCon->ipAddr);

        // if target node MAC is unknown return with error
        ipArpQuery(pSdoUdpCon->ipAddr, &macAddr);
        if(!OPLK_MEMCMP(&macAddr , &macAddrZero , sizeof(eth_addr)))
        {
            // send ARP request when MAC is unknown!
            ipArpRequest(pSdoUdpCon->ipAddr, &macAddr);

            // Reset connection handle
            pSdoUdpCon->port = 0;
            pSdoUdpCon->ipAddr = 0;

            ret = kErrorSdoUdpArpInProgress;
            goto Exit;
        }

        // set handle
        *pSdoConHandle_p = (freeCon | SDO_UDP_HANDLE);
    }

Exit:
    return ret;


}

//------------------------------------------------------------------------------
/**
\brief  Send data using existing connection

The function sends data on an existing connection.

\param  sdoConHandle_p          Connection handle to use for data transfer.
\param  pSrcData_p              Pointer to data which should be sent.
\param  dataSize_p              Size of data to send

\return The function returns a tOplkError error code.

\ingroup module_sdo_udp
*/
//------------------------------------------------------------------------------
tOplkError sdoudp_sendData(tSdoConHdl sdoConHandle_p, tPlkFrame* pSrcData_p, UINT32 dataSize_p)
{
    tOplkError   ret;
    INT          error;
    UINT         connIdx;
    ip_udp_info  udpInfo;
    eth_addr     macAddr;
    eth_addr     macAddrZero;

    ret = kErrorOk;

    OPLK_MEMSET(&macAddrZero, 0, sizeof(eth_addr));
    OPLK_MEMSET(&macAddr, 0, sizeof(eth_addr));

    connIdx = (sdoConHandle_p & ~SDO_ASY_HANDLE_MASK);
    if(connIdx >= SDO_MAX_CONNECTION_UDP)
    {
        ret = kErrorSdoUdpInvalidHdl;
        goto Exit;
    }

    // if target node MAC is unknown return with error
    ipArpQuery(sdoudpInstance_l.aSdoAbsUdpConnection[connIdx].ipAddr,
            &macAddr);
    if(!OPLK_MEMCMP(&macAddr , &macAddrZero , sizeof(eth_addr)))
    {
        ret = kErrorSdoUdpSendError;
        goto Exit;
    }

    //set message type
    ami_setUint8Le(&pSrcData_p->messageType, 0x06); // SDO
    // target node id (for Udp = 0)
    ami_setUint8Le(&pSrcData_p->dstNodeId, 0x00);
    // set source-nodeid (for Udp = 0)
    ami_setUint8Le(&pSrcData_p->srcNodeId, 0x00);

    // calc size
    dataSize_p += ASND_HEADER_SIZE;

    //setup send message
    udpInfo.pData = &pSrcData_p->messageType;
    udpInfo.len = dataSize_p;
    udpInfo.localPort = sdoudpInstance_l.localPort;
    udpInfo.localHost = sdoudpInstance_l.ipAddr;
    udpInfo.remotePort = (unsigned short) sdoudpInstance_l.aSdoAbsUdpConnection[connIdx].port;
    udpInfo.remoteHost.S_un.S_addr = sdoudpInstance_l.aSdoAbsUdpConnection[connIdx].ipAddr;

    error = ipUdpSend(sdoudpInstance_l.ipStackH, &udpInfo);
    if(error <= 0)
    {
        DEBUG_LVL_SDO_TRACE("EplSdoUdpuSendData: ipUdpSend() finished with %i\n", error);
        ret = kErrorSdoUdpSendError;
    }
    else if(error != (INT)dataSize_p)
    {
        DEBUG_LVL_SDO_TRACE("EplSdoUdpuSendData: ipUdpSend() sent %d bytes but %ld should "
                "be transfered!\n", error, dataSize_p);
        ret = kErrorSdoUdpSendError;
    }

Exit:
    return ret;

}


//------------------------------------------------------------------------------
/**
\brief  Delete connection

The function deletes an existing connection.

\param  sdoConHandle_p          Connection handle to use for data transfer.

\return The function returns a tOplkError error code.

\ingroup module_sdo_udp
*/
//------------------------------------------------------------------------------
tOplkError sdoudp_delConnection(tSdoConHdl sdoConHandle_p)
{
    tOplkError   ret = kErrorOk;
    unsigned int connIdx;


    connIdx = (sdoConHandle_p & ~SDO_ASY_HANDLE_MASK);

    if(connIdx >= SDO_MAX_CONNECTION_UDP)
    {
        ret = kErrorSdoUdpInvalidHdl;
    }
    else
    {
        // delete connection
        sdoudpInstance_l.aSdoAbsUdpConnection[connIdx].ipAddr = 0;
        sdoudpInstance_l.aSdoAbsUdpConnection[connIdx].port = 0;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Process IP stack

This functions processes the background task of the IP stack.

\return The function returns a tOplkError error code.

\ingroup module_sdo_udp
*/
//------------------------------------------------------------------------------
tOplkError sdoudp_process(void)
{
    tOplkError ret = kErrorOk;

    tNmtState nmtState = nmtu_getNmtState();

    if(nmtState > kNmtCsNotActive)
    {
        // disable Init-ARP if changed to other state than BASIC_ETHERNET
        if(sdoudpInstance_l.ipStatus == IP_STATE_INIT_ARP &&
                nmtState < kNmtCsBasicEthernet )
        {
            ipDisableInitArp(sdoudpInstance_l.ipStackH);
        }

        // process ip stack
        sdoudpInstance_l.ipStatus = ipPeriodic(sdoudpInstance_l.ipStackH,
                target_getTickCount());

    }

    return ret;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Receive payload from IP stack

This functions processes the background task of the IP stack.

\param arg      Unused user argument
\param pInfo    Info structure of the received frame

\ingroup module_sdo_udp
*/
//------------------------------------------------------------------------------
static void sdoudpu_receiveFrom( void *arg, ip_udp_info *pInfo )
{
    tOplkError  ret;
    tSdoConHdl  sdoConHdl;
    UINT        freeEntry;
    UINT        conCount;

    UNUSED_PARAMETER(arg);

    // get handle for higher layer
    conCount = 0;
    freeEntry = 0xFFFF;

    while (conCount < SDO_MAX_CONNECTION_UDP)
    {
        // check if this connection is already known
        if((sdoudpInstance_l.aSdoAbsUdpConnection[conCount].ipAddr == pInfo->remoteHost.S_un.S_addr)
            && (sdoudpInstance_l.aSdoAbsUdpConnection[conCount].port == pInfo->remotePort))
        {
            break;
        }

        if((sdoudpInstance_l.aSdoAbsUdpConnection[conCount].ipAddr == 0)
            && (sdoudpInstance_l.aSdoAbsUdpConnection[conCount].port == 0)
            && (freeEntry == 0xFFFF))

        {
            freeEntry  = conCount;
        }

        conCount++;
    }

    if (conCount == SDO_MAX_CONNECTION_UDP)
    {
        // connection unknown
        // see if there is a free handle
        if (freeEntry != 0xFFFF)
        {
            // save address infos
            sdoudpInstance_l.aSdoAbsUdpConnection[freeEntry].ipAddr =
                    pInfo->remoteHost.S_un.S_addr;
            sdoudpInstance_l.aSdoAbsUdpConnection[freeEntry].port =
                    pInfo->remotePort;

            // call callback
            sdoConHdl = freeEntry;
            sdoConHdl |= SDO_UDP_HANDLE;
            // offset 4 -> start of SDO Sequence header
            // call SDO receive callback
            ret = sdoudpInstance_l.pfnSdoAsySeqCb(sdoConHdl, (tAsySdoSeq*)((UINT32)pInfo->pData + 4), pInfo->len);
            if (ret != kErrorOk)
            {
                DEBUG_LVL_SDO_TRACE("%s new con: ip=%lX, port=%u, Ret=0x%X\n",
                        __func__,
                        (ULONG) htonl(sdoudpInstance_l.aSdoAbsUdpConnection[freeEntry].ipAddr),
                        (USHORT) htons(sdoudpInstance_l.aSdoAbsUdpConnection[freeEntry].port),
                        ret);
            }
        }
        else
        {
            DEBUG_LVL_SDO_TRACE("Error in EplSdoUdpuReceiveFrom() no free handle\n");
        }

    }
    else
    {
        // known connection
        // call callback with correct handle
        sdoConHdl = conCount;
        sdoConHdl |= SDO_UDP_HANDLE;

        // offset 4 -> start of SDO Sequence header
        ret = sdoudpInstance_l.pfnSdoAsySeqCb(sdoConHdl, (tAsySdoSeq*)((UINT32)pInfo->pData + 4), pInfo->len);
        if (ret != kErrorOk)
        {
            PRINTF("%s known con: ip=%lX, port=%u, Ret=0x%X\n",
                    __func__,
                    (ULONG) htonl(sdoudpInstance_l.aSdoAbsUdpConnection[conCount].ipAddr),
                    (USHORT) htons(sdoudpInstance_l.aSdoAbsUdpConnection[conCount].port),
                    ret);
        }
    }
}

/// \}

#endif // defined(CONFIG_INCLUDE_SDO_UDP)
