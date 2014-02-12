/****************************************************************************

  (c) Kalycito Infotech Private Limited

  Project:      openPOWERLINK - on Windows

  Description:  source file for EdrvPcap.

  License:

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    1. Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.

    3. Neither the name of Kalycito Infotech Private Limited nor the names of
       its contributors may be used to endorse or promote products derived
       from this software without prior written permission. For written
       permission, please contact info@kalycito.com.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.

    Severability Clause:

        If a provision of this License is or becomes illegal, invalid or
        unenforceable in any jurisdiction, that shall not affect:
        1. the validity or enforceability in that jurisdiction of any other
           provision of this License; or
        2. the validity or enforceability in other jurisdictions of that or
           any other provision of this License.

****************************************************************************/

#define _WINSOCKAPI_ // prevent windows.h from including winsock.h
#include <kernel/edrv.h>
#include "pcap.h"
//#include <windows.h>
#include <iphlpapi.h>
#include <string.h>

#include <kernel/hrestimer.h>



/***************************************************************************/
/*                                                                         */
/*                                                                         */
/*          G L O B A L   D E F I N I T I O N S                            */
/*                                                                         */
/*                                                                         */
/***************************************************************************/

//---------------------------------------------------------------------------
// const defines
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// module global types
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// local vars
//---------------------------------------------------------------------------

//---------------------------------------------------------------------------
// local function prototypes
//---------------------------------------------------------------------------

static void EplTimerHighReskCbTimer (unsigned int uiIndex_p);



/***************************************************************************/
/*                                                                         */
/*                                                                         */
/*          C L A S S  <edrv>                                              */
/*                                                                         */
/*                                                                         */
/***************************************************************************/
//
// Description:
//
//
/***************************************************************************/


//=========================================================================//
//                                                                         //
//          P R I V A T E   D E F I N I T I O N S                          //
//                                                                         //
//=========================================================================//

//---------------------------------------------------------------------------
// const defines
//---------------------------------------------------------------------------


#define EDRV_MAX_FRAME_SIZE     0x600

#define EDRV_HANDLE_EVENT       0
#define EDRV_HANDLE_PCAP        1
#define EDRV_HANDLE_TIMER0      2
#define EDRV_HANDLE_TIMER1      3
#define EDRV_HANDLE_COUNT       4


//---------------------------------------------------------------------------
// local types
//---------------------------------------------------------------------------

// Private structure
typedef struct
{

    tEdrvInitParam      m_InitParam;
    tEdrvTxBuffer*      m_pTransmittedTxBufferLastEntry;
    tEdrvTxBuffer*      m_pTransmittedTxBufferFirstEntry;
    CRITICAL_SECTION    m_CriticalSection;

    pcap_t*             m_pPcap;

    HANDLE              m_ahHandle[EDRV_HANDLE_COUNT];
    HANDLE              m_hThread;

} tEdrvInstance;


//---------------------------------------------------------------------------
// local vars
//---------------------------------------------------------------------------

static tEdrvInstance EdrvInstance_l;


//---------------------------------------------------------------------------
// local function prototypes
//---------------------------------------------------------------------------

static void EdrvPacketHandler(u_char *param, const struct pcap_pkthdr *header, const u_char *pkt_data);

static DWORD WINAPI  EdrvWorkerThread(void *);


//---------------------------------------------------------------------------
//
// Function:    edrv_init
//
// Description: function for init of the Ethernet controller
//
// Parameters:  pEdrvInitParam_p    = pointer to struct including the init-parameters
//
// Returns:     Errorcode           = kErrorOk
//                                  = kErrorNoResource
//
// State:
//
//---------------------------------------------------------------------------

tOplkError edrv_init(tEdrvInitParam *pEdrvInitParam_p)
{
tOplkError Ret;
DWORD dwThreadId;
char sErr_Msg[ PCAP_ERRBUF_SIZE ];
// variables for IPHLPAPI
ULONG ulOutBufLen;
PIP_ADAPTER_INFO pAdapterInfo;
PIP_ADAPTER_INFO pAdapter = NULL;
DWORD dwRetVal = 0;

    Ret = kErrorOk;


    // clear instance structure
    OPLK_MEMSET(&EdrvInstance_l, 0, sizeof (EdrvInstance_l));

    if (pEdrvInitParam_p->hwParam.pDevName == NULL)
    {
        Ret = kErrorEdrvInit;
        goto Exit;
    }

    // search for the corresponding MAC address via IPHLPAPI
    ulOutBufLen = sizeof (IP_ADAPTER_INFO);
    pAdapterInfo = (IP_ADAPTER_INFO *) OPLK_MALLOC(sizeof (IP_ADAPTER_INFO));
    if (pAdapterInfo == NULL)
    {
        printf("Error allocating memory needed to call GetAdaptersinfo\n");
        Ret = kErrorNoResource;
        goto Exit;
    }

    // Make an initial call to GetAdaptersInfo to get
    // the necessary size into the ulOutBufLen variable
    dwRetVal = GetAdaptersInfo(pAdapterInfo, &ulOutBufLen);
    if (dwRetVal == ERROR_BUFFER_OVERFLOW)
    {
        OPLK_FREE(pAdapterInfo);
        pAdapterInfo = (IP_ADAPTER_INFO *) OPLK_MALLOC(ulOutBufLen);
        if (pAdapterInfo == NULL)
        {
            printf("Error allocating memory needed to call GetAdaptersinfo\n");
            Ret = kErrorNoResource;
            goto Exit;
        }
    }

    dwRetVal = GetAdaptersInfo(pAdapterInfo, &ulOutBufLen);
    if (dwRetVal == NO_ERROR)
    {
        pAdapter = pAdapterInfo;
        while (pAdapter)
        {
            if (pAdapter->Type == MIB_IF_TYPE_ETHERNET)
            {
                if (strstr(pEdrvInitParam_p->hwParam.pDevName, pAdapter->AdapterName) != NULL)
                {   // corresponding adapter found
                    OPLK_MEMCPY(pEdrvInitParam_p->aMacAddr, pAdapter->Address,
                        min(pAdapter->AddressLength, sizeof (pEdrvInitParam_p->aMacAddr)));
                    break;
                }
            }
            pAdapter = pAdapter->Next;
        }
    }
    else
    {
        printf("GetAdaptersInfo failed with error: %d\n", dwRetVal);

    }
    if (pAdapterInfo)
    {
        OPLK_FREE(pAdapterInfo);
    }

    // save the init data (with updated MAC address)
    EdrvInstance_l.m_InitParam = *pEdrvInitParam_p;


    EdrvInstance_l.m_pPcap = pcap_open_live (
                        pEdrvInitParam_p->hwParam.pDevName,
                        65535,  // snaplen
                        1,      // promiscuous mode
                        1,      // milli seconds read timeout
                        sErr_Msg
                    );

    if ( EdrvInstance_l.m_pPcap == NULL )
    {
        printf("Error!! Can't open pcap: %s\n", sErr_Msg);
        Ret = kErrorEdrvInit;
        goto Exit;
    }

    // configure pcap for maximum responsiveness
    if (pcap_setmintocopy(EdrvInstance_l.m_pPcap, 0) != 0)
    {
        printf("pcap_setmintocopy failed\n");
    }

    // put pcap into nonblocking mode
    if (pcap_setnonblock(EdrvInstance_l.m_pPcap, 1, sErr_Msg) != 0)
    {
        printf("Can't put pcap into nonblocking mode: %s\n", sErr_Msg);
    }

    // get event handle for pcap instance
    EdrvInstance_l.m_ahHandle[EDRV_HANDLE_PCAP] =
        pcap_getevent(EdrvInstance_l.m_pPcap);

    // Create two unnamed waitable timers for EplTimerHighResk sub-module.
    EdrvInstance_l.m_ahHandle[EDRV_HANDLE_TIMER0] =
        CreateWaitableTimer(NULL, FALSE, NULL);
    if (EdrvInstance_l.m_ahHandle[EDRV_HANDLE_TIMER0] == NULL)
    {
        printf("CreateWaitableTimer failed (%d)\n", GetLastError());
        Ret = kErrorEdrvInit;
        goto Exit;
    }

    EdrvInstance_l.m_ahHandle[EDRV_HANDLE_TIMER1] =
        CreateWaitableTimer(NULL, FALSE, NULL);
    if (EdrvInstance_l.m_ahHandle[EDRV_HANDLE_TIMER1] == NULL)
    {
        printf("CreateWaitableTimer failed (%d)\n", GetLastError());
        Ret = kErrorEdrvInit;
        goto Exit;
    }

    // create event for signalling shutdown
    EdrvInstance_l.m_ahHandle[EDRV_HANDLE_EVENT] =
        CreateEvent(NULL, FALSE, FALSE, NULL);


    // Create the thread to begin execution on its own.
    EdrvInstance_l.m_hThread = CreateThread(
            NULL,                   // default security attributes
            0,                      // use default stack size
            EdrvWorkerThread,         // thread function name
            &EdrvInstance_l,        // argument to thread function
            0,                      // use default creation flags
            &dwThreadId);   // returns the thread identifier


    if (EdrvInstance_l.m_hThread == NULL)
    {
         Ret = kErrorEdrvInit;
         goto Exit;
    }

    InitializeCriticalSection(&EdrvInstance_l.m_CriticalSection);

Exit:
    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    edrv_shutdown
//
// Description: Shutdown the Ethernet controller
//
// Parameters:  void
//
// Returns:     Errorcode   = kErrorOk
//
// State:
//
//---------------------------------------------------------------------------

tOplkError edrv_shutdown( void )
{
    // signal shutdown to the thread
    SetEvent(EdrvInstance_l.m_ahHandle[EDRV_HANDLE_EVENT]);

    WaitForSingleObject( EdrvInstance_l.m_hThread, INFINITE );

    CloseHandle ( EdrvInstance_l.m_hThread );

    pcap_close(EdrvInstance_l.m_pPcap);

    CloseHandle ( EdrvInstance_l.m_ahHandle[EDRV_HANDLE_EVENT] );
    CloseHandle ( EdrvInstance_l.m_ahHandle[EDRV_HANDLE_TIMER0] );
    CloseHandle ( EdrvInstance_l.m_ahHandle[EDRV_HANDLE_TIMER1] );

    DeleteCriticalSection(&EdrvInstance_l.m_CriticalSection);

    // clear instance structure
    OPLK_MEMSET(&EdrvInstance_l, 0, sizeof (EdrvInstance_l));

    return kErrorOk; //assuming no problems with closing the handle
}


//---------------------------------------------------------------------------
//
// Function:    edrv_sendTxBuffer
//
// Description: immediately starts the transmission of the buffer
//
// Parameters:  pBuffer_p   = buffer descriptor to transmit
//
// Returns:     Errorcode   = kErrorOk
//
// State:
//
//---------------------------------------------------------------------------

tOplkError edrv_sendTxBuffer(tEdrvTxBuffer *pBuffer_p)
{
tOplkError  Ret = kErrorOk;
int         iRet;

//    TRACE("%s: TxB=%p (%02X), last TxB=%p\n", __func__, pBuffer_p, (UINT)pBuffer_p->pBuffer[5], EdrvInstance_l.m_pTransmittedTxBufferLastEntry);

    if (pBuffer_p->txBufferNumber.pArg != NULL)
    {
        Ret = kErrorInvalidOperation;
        goto Exit;
    }

    EnterCriticalSection(&EdrvInstance_l.m_CriticalSection);
    if (EdrvInstance_l.m_pTransmittedTxBufferLastEntry == NULL)
    {
        EdrvInstance_l.m_pTransmittedTxBufferLastEntry =
            EdrvInstance_l.m_pTransmittedTxBufferFirstEntry = pBuffer_p;
    }
    else
    {
        EdrvInstance_l.m_pTransmittedTxBufferLastEntry->txBufferNumber.pArg = pBuffer_p;
        EdrvInstance_l.m_pTransmittedTxBufferLastEntry = pBuffer_p;
    }
    LeaveCriticalSection(&EdrvInstance_l.m_CriticalSection);

    iRet = pcap_sendpacket(EdrvInstance_l.m_pPcap, pBuffer_p->pBuffer, (int) pBuffer_p->txFrameSize);
    if  (iRet != 0)
    {
        PRINTF("%s pcap_sendpacket returned %d (%s)\n", __func__, iRet, pcap_geterr(EdrvInstance_l.m_pPcap));
        Ret = kErrorInvalidOperation;
    }

Exit:
    return Ret;
}



//---------------------------------------------------------------------------
//
// Function:    edrv_allocTxBuffer
//
// Description: Register a Tx-Buffer
//
// Parameters:  pBuffer_p   = pointer to Buffer structure
//
// Returns:     Errorcode   = kErrorOk
//                          = kErrorEdrvNoFreeBufEntry
//
// State:
//
//---------------------------------------------------------------------------

tOplkError edrv_allocTxBuffer(tEdrvTxBuffer * pBuffer_p)
{
tOplkError Ret = kErrorOk;

    if (pBuffer_p->maxBufferSize > EDRV_MAX_FRAME_SIZE)
    {
        Ret = kErrorEdrvNoFreeBufEntry;
        goto Exit;
    }

    // allocate buffer with malloc
    pBuffer_p->pBuffer = OPLK_MALLOC(pBuffer_p->maxBufferSize);
    if (pBuffer_p->pBuffer == NULL)
    {
        Ret = kErrorEdrvNoFreeBufEntry;
        goto Exit;
    }

    pBuffer_p->txBufferNumber.pArg = NULL;

Exit:
    return Ret;

}



//---------------------------------------------------------------------------
//
// Function:    edrv_freeTxBuffer
//
// Description: Register a Tx-Buffer
//
// Parameters:  pBuffer_p   = pointer to Buffer structure
//
// Returns:     Errorcode   = kErrorOk
//
// State:
//
//---------------------------------------------------------------------------

tOplkError edrv_freeTxBuffer(tEdrvTxBuffer * pBuffer_p)
{
BYTE*   pbBuffer = pBuffer_p->pBuffer;

    // mark buffer as free, before actually freeing it
    pBuffer_p->pBuffer = NULL;

    OPLK_FREE(pbBuffer);

    return kErrorOk;
}


tOplkError edrv_changeRxFilter(tEdrvFilter*    pFilter_p,
                            unsigned int    uiCount_p,
                            unsigned int    uiEntryChanged_p,
                            unsigned int    uiChangeFlags_p)
{
tOplkError      Ret = kErrorOk;

    UNUSED_PARAMETER(pFilter_p);
    UNUSED_PARAMETER(uiCount_p);
    UNUSED_PARAMETER(uiEntryChanged_p);
    UNUSED_PARAMETER(uiChangeFlags_p);
    return Ret;
}


//---------------------------------------------------------------------------
//
// Function:    edrv_clearRxMulticastMacAddr
//
// Description: Reset a multicast entry in the Ethernet controller
//
// Parameters:  pbMacAddr_p     = pointer to multicast entry to reset
//
// Returns:     Errorcode       = kErrorOk
//
// State:
//
//---------------------------------------------------------------------------

tOplkError edrv_clearRxMulticastMacAddr (BYTE * pbMacAddr_p)
{
    UNUSED_PARAMETER(pbMacAddr_p);

    return kErrorOk;
}


//---------------------------------------------------------------------------
//
// Function:    edrv_setRxMulticastMacAddr
//
// Description: Set a multicast entry into the Ethernet controller
//
// Parameters:  pbMacAddr_p     = pointer to multicast entry to set
//
// Returns:     Errorcode       = kErrorOk
//
// State:
//
//---------------------------------------------------------------------------

tOplkError edrv_setRxMulticastMacAddr   (BYTE * pbMacAddr_p)
{
    UNUSED_PARAMETER(pbMacAddr_p);

    return kErrorOk;
}


//---------------------------------------------------------------------------
//
// Function:    EdrvPacketHandler
//
// Description: Pcap packet handler, that forwards the frames to the DLL
//
// Parameters:  pUser_p         = user specific pointer,
//                                which points to the instance structure
//              header          = packet header (size, ...)
//              pkt_data        = packet buffer
//
// Returns:     void
//
// State:
//
//---------------------------------------------------------------------------

static void EdrvPacketHandler(u_char *pUser_p,
                              const struct pcap_pkthdr *header,
                              const u_char *pkt_data)
{
tEdrvInstance*  pInstance = (tEdrvInstance*) pUser_p;
tEdrvRxBuffer   RxBuffer;

    if (memcmp( pkt_data+6, pInstance->m_InitParam.aMacAddr, 6 ) != 0)
    {   // filter out self generated traffic
        RxBuffer.bufferInFrame    = kEdrvBufferLastInFrame;
        RxBuffer.rxFrameSize       = header->caplen;
        RxBuffer.pBuffer         = (BYTE*) pkt_data;

        pInstance->m_InitParam.pfnRxHandler(&RxBuffer);
    }
    else
    {   // self generated traffic
        if (pInstance->m_pTransmittedTxBufferFirstEntry != NULL)
        {
        tEdrvTxBuffer* pTxBuffer = pInstance->m_pTransmittedTxBufferFirstEntry;

//            TRACE("%s: (%02X) first TxB=%p (%02X), last TxB=%p\n", __func__, (UINT)pkt_data[5], pTxBuffer, (UINT)pTxBuffer->pBuffer[5], EdrvInstance_l.m_pTransmittedTxBufferLastEntry);

            if (pTxBuffer->pBuffer != NULL)
            {
                if (memcmp(pkt_data, pTxBuffer->pBuffer, 6) == 0)
                {
                    EnterCriticalSection(&EdrvInstance_l.m_CriticalSection);
                    pInstance->m_pTransmittedTxBufferFirstEntry = pInstance->m_pTransmittedTxBufferFirstEntry->txBufferNumber.pArg;
                    if (pInstance->m_pTransmittedTxBufferFirstEntry == NULL)
                    {
                        pInstance->m_pTransmittedTxBufferLastEntry = NULL;
                    }
                    LeaveCriticalSection(&EdrvInstance_l.m_CriticalSection);

                    pTxBuffer->txBufferNumber.pArg = NULL;

                    if (pTxBuffer->pfnTxHandler != NULL)
                    {
                        pTxBuffer->pfnTxHandler(pTxBuffer);
                    }
                }
                else
                {
                    TRACE("%s: no matching TxB: DstMAC=%02X%02X%02X%02X%02X%02X\n",
                        __func__,
                        (UINT)pkt_data[0],
                        (UINT)pkt_data[1],
                        (UINT)pkt_data[2],
                        (UINT)pkt_data[3],
                        (UINT)pkt_data[4],
                        (UINT)pkt_data[5]);
                    TRACE("   current TxB %p: DstMAC=%02X%02X%02X%02X%02X%02X\n",
                        pTxBuffer,
                        (UINT)pTxBuffer->pBuffer[0],
                        (UINT)pTxBuffer->pBuffer[1],
                        (UINT)pTxBuffer->pBuffer[2],
                        (UINT)pTxBuffer->pBuffer[3],
                        (UINT)pTxBuffer->pBuffer[4],
                        (UINT)pTxBuffer->pBuffer[5]);
                }
            }
        }
        else
        {
            TRACE("%s: no TxB: DstMAC=%02X%02X%02X%02X%02X%02X\n", __func__, pkt_data[0], pkt_data[1], pkt_data[2], pkt_data[3], pkt_data[4], pkt_data[5]);
        }
    }
}



//---------------------------------------------------------------------------
//
// Function:    EdrvWorkerThread
//
// Description: Worker thread, that processes several events like
//              Pcap events and timer events in one single thread as emulation
//              of non-reentrant interrupt processing. The receive, transmit
//              and timer callback functions of the DLL are mutual exclusive.
//
// Parameters:  pArgument_p     = user specific argument, i.e. pointer to
//                                instance structure
//
// Returns:     DWORD           = thread return code
//
// State:
//
//---------------------------------------------------------------------------

static DWORD WINAPI EdrvWorkerThread( void *pArgument_p )
{
tEdrvInstance*  pInstance = pArgument_p;
int             iRet;
DWORD           dwRet;

    // increase priority
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_TIME_CRITICAL);

    for (;;)
    {

        // Wait for events
        dwRet = WaitForMultipleObjects(
                    EDRV_HANDLE_COUNT,
                    pInstance->m_ahHandle,
                    FALSE,
                    INFINITE);

        switch (dwRet)
        {
            case WAIT_OBJECT_0 + EDRV_HANDLE_EVENT:
            {   // shutdown was signalled
                goto Exit;
            }

            case WAIT_OBJECT_0 + EDRV_HANDLE_PCAP:
            {   // frames were received

                // process all frames, that are available
                iRet = pcap_dispatch(pInstance->m_pPcap,
                                     -1, EdrvPacketHandler, (u_char*)pInstance);

                break;
            }

            case WAIT_OBJECT_0 + EDRV_HANDLE_TIMER0:
            {   // timer 0 triggered

                EplTimerHighReskCbTimer(0);

                break;
            }

            case WAIT_OBJECT_0 + EDRV_HANDLE_TIMER1:
            {   // timer 1 triggered

                EplTimerHighReskCbTimer(1);

                break;
            }

            default:
            case WAIT_FAILED:
            {
                printf("WaitForMultipleObjects failed (%d)\n", GetLastError());
                break;
            }
        }
    }

Exit:
    return 0;
}


static HANDLE EdrvGetTimerHandle(unsigned int uiIndex_p)
{

    uiIndex_p += EDRV_HANDLE_TIMER0;
    if (uiIndex_p > EDRV_HANDLE_COUNT)
    {
        return NULL;
    }

/*
    uiIndex_p = (EDRV_HANDLE_COUNT - 1) - uiIndex_p;

    if (uiIndex_p < EDRV_HANDLE_TIMER0)
    {
        return NULL;
    }
*/
    else
    {
        return EdrvInstance_l.m_ahHandle[uiIndex_p];
    }
}



/***************************************************************************/
/*                                                                         */
/*                                                                         */
/*          C L A S S  <EplTimerHighResk>                                  */
/*                                                                         */
/*                                                                         */
/***************************************************************************/
//
// Description:
//
//
/***************************************************************************/


//=========================================================================//
//                                                                         //
//          P R I V A T E   D E F I N I T I O N S                          //
//                                                                         //
//=========================================================================//

//---------------------------------------------------------------------------
// const defines
//---------------------------------------------------------------------------

#define TIMER_COUNT                 2
#define TIMERHDL_MASK               0x0FFFFFFF
#define TIMERHDL_SHIFT              28


//---------------------------------------------------------------------------
// local types
//---------------------------------------------------------------------------

typedef struct
{
    tTimerEventArg      m_EventArg;
    tTimerkCallback     m_pfnCallback;
    LARGE_INTEGER       m_liDueTime;    // for continuous timers, otherwise 0

} tEplTimerHighReskTimerInfo;

typedef struct
{
    tEplTimerHighReskTimerInfo  m_aTimerInfo[TIMER_COUNT];
    HINSTANCE                   m_hinstLibNtDll;

} tEplTimerHighReskInstance;


// function types from NTDLL.DLL
typedef LONG (NTAPI *NTQUERYTIMERRESOLUTION)
    (OUT PULONG MinimumResolution,
     OUT PULONG MaximumResolution,
     OUT PULONG CurrentResolution);
typedef LONG (NTAPI *NTSETTIMERRESOLUTION)
    (IN ULONG DesiredResolution,
     IN BOOLEAN SetResolution,
     OUT PULONG CurrentResolution);

//---------------------------------------------------------------------------
// local vars
//---------------------------------------------------------------------------

static tEplTimerHighReskInstance    EplTimerHighReskInstance_l;


// function pointers to NTDLL.DLL
NTQUERYTIMERRESOLUTION NtQueryTimerResolution;
NTSETTIMERRESOLUTION NtSetTimerResolution;


//---------------------------------------------------------------------------
// local function prototypes
//---------------------------------------------------------------------------


//=========================================================================//
//                                                                         //
//          P U B L I C   F U N C T I O N S                                //
//                                                                         //
//=========================================================================//


//---------------------------------------------------------------------------
//
// Function:    hrestimer_init()
//
// Description: initializes the high resolution timer module.
//
// Parameters:  void
//
// Return:      tOplkError      = error code
//
// State:       not tested
//
//---------------------------------------------------------------------------

tOplkError PUBLIC hrestimer_init(void)
{
tOplkError  Ret;

    Ret = hrestimer_addInstance();

    return Ret;

}


//---------------------------------------------------------------------------
//
// Function:    hrestimer_addInstance()
//
// Description: initializes the high resolution timer module.
//
// Parameters:  void
//
// Return:      tOplkError      = error code
//
// State:       not tested
//
//---------------------------------------------------------------------------

tOplkError PUBLIC hrestimer_addInstance(void)
{
tOplkError      Ret = kErrorOk;
LONG            lRet = 0;
ULONG           ulMin = ~0UL;
ULONG           ulMax = ~0UL;
ULONG           ulCur = ~0UL;


    OPLK_MEMSET(&EplTimerHighReskInstance_l, 0, sizeof (EplTimerHighReskInstance_l));


    // load NTDLL.DLL
    EplTimerHighReskInstance_l.m_hinstLibNtDll = LoadLibrary("ntdll.dll");
    if (EplTimerHighReskInstance_l.m_hinstLibNtDll == NULL)
    {
        printf("LoadLibrary(ntdll.dll) failed (%d)\n", GetLastError());
        Ret = kErrorNoResource;
        goto Exit;
    }

    // load proc address of NtQueryTimerResolution
    NtQueryTimerResolution = (NTQUERYTIMERRESOLUTION)
        GetProcAddress(EplTimerHighReskInstance_l.m_hinstLibNtDll,
                        "NtQueryTimerResolution");
    if (NtQueryTimerResolution == NULL)
    {
        printf("GetProcAddress(NtQueryTimerResolution) failed (%d)\n", GetLastError());
        Ret = kErrorNoResource;
        goto Exit;
    }

    // load proc address of NtSetTimerResolution
    NtSetTimerResolution = (NTSETTIMERRESOLUTION)
        GetProcAddress(EplTimerHighReskInstance_l.m_hinstLibNtDll,
                        "NtSetTimerResolution");
    if (NtSetTimerResolution == NULL)
    {
        printf("GetProcAddress(NtSetTimerResolution) failed (%d)\n", GetLastError());
        Ret = kErrorNoResource;
        goto Exit;
    }


    // query actual timer resolution
    NtQueryTimerResolution(&ulMin, &ulMax, &ulCur);
    printf("TimerResolution Min = %lu, Max = %lu, Cur = %lu\n", ulMin, ulMax, ulCur);

    // set timer resolution to maximum
    lRet = NtSetTimerResolution(ulMax, TRUE, &ulCur);
    printf("NtSetTimerResolution returnd %ld, current resolution = %lu\n", lRet, ulCur);


Exit:
    return Ret;

}


//---------------------------------------------------------------------------
//
// Function:    hrestimer_delInstance()
//
// Description: shuts down the high resolution timer module.
//
// Parameters:  void
//
// Return:      tOplkError      = error code
//
// State:       not tested
//
//---------------------------------------------------------------------------

tOplkError PUBLIC hrestimer_delInstance(void)
{
tOplkError  Ret = kErrorOk;
LONG            lRet = 0;
ULONG           ulCur = ~0UL;

    // set timer resolution to old value
    lRet = NtSetTimerResolution(0, FALSE, &ulCur);
    printf("NtSetTimerResolution returnd %ld, current resolution = %lu\n", lRet, ulCur);

    // free library NTDLL.DLL
    FreeLibrary(EplTimerHighReskInstance_l.m_hinstLibNtDll);
    EplTimerHighReskInstance_l.m_hinstLibNtDll = NULL;

    return Ret;

}



//---------------------------------------------------------------------------
//
// Function:    hrestimer_modifyTimer()
//
// Description: modifies the timeout of the timer with the specified handle.
//              If the handle the pointer points to is zero, the timer must
//              be created first.
//              If it is not possible to stop the old timer,
//              this function always assures that the old timer does not
//              trigger the callback function with the same handle as the new
//              timer. That means the callback function must check the passed
//              handle with the one returned by this function. If these are
//              unequal, the call can be discarded.
//
// Parameters:  pTimerHdl_p     = pointer to timer handle
//              ullTimeNs_p     = relative timeout in [ns]
//              pfnCallback_p   = callback function, which is called mutual
//                                exclusive with the Edrv callback functions
//                                (Rx and Tx).
//              ulArgument_p    = user-specific argument
//              fContinuously_p = if TRUE, callback function will be called
//                                continuously;
//                                otherwise, it is a oneshot timer.
//
// Return:      tOplkError      = error code
//
// State:       not tested
//
//---------------------------------------------------------------------------

tOplkError PUBLIC hrestimer_modifyTimer(tTimerHdl*     pTimerHdl_p,
                                    unsigned long long  ullTimeNs_p,
                                    tTimerkCallback     pfnCallback_p,
                                    unsigned long       ulArgument_p,
                                    BOOL                fContinuously_p)
{
tOplkError                  Ret = kErrorOk;
BOOL                        fRet;
unsigned int                uiIndex;
tEplTimerHighReskTimerInfo* pTimerInfo;
HANDLE                      hTimer;
LARGE_INTEGER               liDueTime;
//LONG                        lPeriodMs;

    // check pointer to handle
    if(pTimerHdl_p == NULL)
    {
        Ret = kErrorTimerInvalidHandle;
        goto Exit;
    }

    if (*pTimerHdl_p == 0)
    {   // no timer created yet
        // search free timer info structure
        pTimerInfo = &EplTimerHighReskInstance_l.m_aTimerInfo[0];
        for (uiIndex = 0; uiIndex < TIMER_COUNT; uiIndex++, pTimerInfo++)
        {
            if (pTimerInfo->m_pfnCallback == NULL)
            {   // free structure found
                break;
            }
        }
        if (uiIndex >= TIMER_COUNT)
        {   // no free structure found
            Ret = kErrorTimerNoTimerCreated;
            goto Exit;
        }
    }
    else
    {
        uiIndex = (unsigned int)(*pTimerHdl_p >> TIMERHDL_SHIFT) - 1;
        if (uiIndex >= TIMER_COUNT)
        {   // invalid handle
            Ret = kErrorTimerInvalidHandle;
            goto Exit;
        }
        pTimerInfo = &EplTimerHighReskInstance_l.m_aTimerInfo[uiIndex];
        // d.k.: assume that this info structure is the correct one
/*        if ((pTimerInfo->m_EventArg.timerHdl != *pTimerHdl_p)
            && (pTimerInfo->m_pfnCallback == NULL))
        {   // invalid handle
            Ret = kErrorTimerInvalidHandle;
            goto Exit;
        }*/
    }

    // increment timer handle (if timer expires right after this statement,
    // the user would detect an unknown timer handle and discard it)
    pTimerInfo->m_EventArg.timerHdl = ((pTimerInfo->m_EventArg.timerHdl + 1) & TIMERHDL_MASK)
                                        | ((uiIndex + 1) << TIMERHDL_SHIFT);



    // calculate duetime [100 ns] (negative value = relative time)
    liDueTime.QuadPart = (long long) ullTimeNs_p / -100LL;
    if (liDueTime.QuadPart > -10000LL)
    {   // duetime is less than 1 ms
        liDueTime.QuadPart = -10000LL;
    }

    if (fContinuously_p != FALSE)
    {   // continuous timer
        pTimerInfo->m_liDueTime = liDueTime;
//        lPeriodMs = (LONG) (liDueTime.QuadPart / -10000LL);
    }
    else
    {   // one-shot timer
        pTimerInfo->m_liDueTime.QuadPart = 0LL;
//        lPeriodMs = 0;
    }

    pTimerInfo->m_EventArg.argument.value = ulArgument_p;
    pTimerInfo->m_pfnCallback = pfnCallback_p;

    *pTimerHdl_p = pTimerInfo->m_EventArg.timerHdl;

    // configure timer
    hTimer = EdrvGetTimerHandle(uiIndex);

    fRet = SetWaitableTimer(hTimer, &liDueTime, 0L /*lPeriodMs*/, NULL, NULL, 0);
    if (!fRet)
    {
        printf("SetWaitableTimer failed (%d)\n", GetLastError());
        Ret = kErrorTimerNoTimerCreated;
        goto Exit;
    }

Exit:
    return Ret;

}


//---------------------------------------------------------------------------
//
// Function:    hrestimer_deleteTimer()
//
// Description: deletes the timer with the specified handle. Afterward the
//              handle is set to zero.
//
// Parameters:  pTimerHdl_p     = pointer to timer handle
//
// Return:      tOplkError      = error code
//
// State:       not tested
//
//---------------------------------------------------------------------------

tOplkError PUBLIC hrestimer_deleteTimer(tTimerHdl*     pTimerHdl_p)
{
tOplkError                  Ret = kErrorOk;
unsigned int                uiIndex;
tEplTimerHighReskTimerInfo* pTimerInfo;
HANDLE                      hTimer;

    // check pointer to handle
    if(pTimerHdl_p == NULL)
    {
        Ret = kErrorTimerInvalidHandle;
        goto Exit;
    }

    if (*pTimerHdl_p == 0)
    {   // no timer created yet
        goto Exit;
    }
    else
    {
        uiIndex = (unsigned int)(*pTimerHdl_p >> TIMERHDL_SHIFT) - 1;
        if (uiIndex >= TIMER_COUNT)
        {   // invalid handle
            Ret = kErrorTimerInvalidHandle;
            goto Exit;
        }
        pTimerInfo = &EplTimerHighReskInstance_l.m_aTimerInfo[uiIndex];
        if (pTimerInfo->m_EventArg.timerHdl != *pTimerHdl_p)
        {   // invalid handle
            goto Exit;
        }
    }

    pTimerInfo->m_pfnCallback = NULL;

    *pTimerHdl_p = 0;

    // cancel timer
    hTimer = EdrvGetTimerHandle(uiIndex);
    CancelWaitableTimer(hTimer);

Exit:
    return Ret;

}


//---------------------------------------------------------------------------
//
// Function:    EplTimerHighReskCbTimer()
//
// Description: target specific callback function for timer.
//
// Parameters:  uiIndex_p       = timer index (0 or 1)
//
// Return:      void
//
// State:       not tested
//
//---------------------------------------------------------------------------

static void EplTimerHighReskCbTimer (unsigned int uiIndex_p)
{
tEplTimerHighReskTimerInfo* pTimerInfo;

    if (uiIndex_p > TIMER_COUNT)
    {   // invalid handle
        goto Exit;
    }

    pTimerInfo = &EplTimerHighReskInstance_l.m_aTimerInfo[uiIndex_p];

    if (pTimerInfo->m_liDueTime.QuadPart != 0)
    {   // periodic timer
    HANDLE  hTimer;
    BOOL    fRet;

        // configure timer
        hTimer = EdrvGetTimerHandle(uiIndex_p);

        fRet = SetWaitableTimer(hTimer, &pTimerInfo->m_liDueTime, 0L /*lPeriodMs*/, NULL, NULL, 0);
        if (!fRet)
        {
            printf("SetWaitableTimer failed (%d)\n", GetLastError());
            goto Exit;
        }
    }

    if (pTimerInfo->m_pfnCallback != NULL)
    {
        pTimerInfo->m_pfnCallback(&pTimerInfo->m_EventArg);
    }

Exit:
    return;
}



