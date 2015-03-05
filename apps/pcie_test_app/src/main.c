/**
********************************************************************************
\file   main.c

\brief  Main file of PCIe MN test application

This file contains the main file of the openPOWERLINK PCIe MN test
application.

\ingroup module_pcie_test_app
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2014, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
Copyright (c) 2013, SYSTEC electronic GmbH
Copyright (c) 2015, Kalycito Infotech Private Ltd.All rights reserved.
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
#include <stdio.h>
#include <limits.h>
#include <string.h>

#include <oplk/oplk.h>
#include <oplk/debugstr.h>

#include <getopt/getopt.h>

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#define CYCLE_LEN           UINT_MAX
#define NODEID              0xF0                //=> MN
#define IP_ADDR             0xc0a86401          // 192.168.100.1
#define SUBNET_MASK         0xFFFFFF00          // 255.255.255.0
#define DEFAULT_GATEWAY     0xC0A864FE          // 192.168.100.C_ADR_RT1_DEF_NODE_ID

//------------------------------------------------------------------------------
// module global vars
//------------------------------------------------------------------------------
const BYTE      aMacAddr_g[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static BOOL     fGsOff_l;

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
    char    cdcFile[256];
    char*   pLogFile;
} tOptions;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static int          getOptions(int argc_p, char** argv_p, tOptions* pOpts_p);
static tOplkError   initPowerlink(UINT32 cycleLen_p, char* pszCdcFileName_p,
                                  const BYTE* macAddr_p);
static void         shutdownPowerlink(void);
static tOplkError   stubProcessEvents(tOplkApiEventType EventType_p,
                                      tOplkApiEventArg* pEventArg_p,
                                      void* pUserArg_p);    ///< Handle any desired stack events in this function.
static int          testAppFn(void* arg_p);                 ///< Perform necessary tests in this function.

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  main function

This is the main function of the openPOWERLINK MN PCIe test application.

\param  argc                    Number of arguments
\param  argv                    Pointer to argument strings

\return Returns an exit code

\ingroup module_pcie_test_app
*/
//------------------------------------------------------------------------------
int main(int argc, char** argv)
{
    tOplkError      ret = kErrorOk;
    tOptions        opts;

    // Get the user arguments passed to the application
    getOptions(argc, argv, &opts);

    printf("----------------------------------------------------\n");
    printf("openPOWERLINK MN PCIe Lite test application\n");
    printf("using openPOWERLINK Stack: %s\n", PLK_DEFINED_STRING_VERSION);
    printf("----------------------------------------------------\n");

    // Set the initialization parameters for the stack and start it
    if ((ret = initPowerlink(CYCLE_LEN, opts.cdcFile, aMacAddr_g)) != kErrorOk)
        goto Exit;

    printf("Kernel stack initialized successfully!!\n");

    // Perform necessary tests here. For this example, no arguments are being passed
    if (testAppFn(NULL) != 0)
    {
        printf("Testing PCP PCIe Failed...\n");
        goto Exit;
    }

    printf("Testing PCP PCIe completed...\n");

Exit:
    // Shut down the stack
    shutdownPowerlink();

    return 0;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//------------------------------------------------------------------------------
/**
\brief  Initialize the openPOWERLINK stack

The function initializes the openPOWERLINK stack.

\param  cycleLen_p              Length of POWERLINK cycle.
\param  macAddr_p               MAC address to use for POWERLINK interface.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
static tOplkError initPowerlink(UINT32 cycleLen_p, char* pszCdcFileName_p,
                                const BYTE* macAddr_p)
{
    tOplkError                  ret = kErrorOk;
    static tOplkApiInitParam    initParam;
    static char                 devName[128];

    UNUSED_PARAMETER(pszCdcFileName_p);
    printf("Initializing openPOWERLINK stack...\n");

    memset(&initParam, 0, sizeof(initParam));
    initParam.sizeOfInitParam = sizeof(initParam);

    // pass selected device name to Edrv
    initParam.hwParam.pDevName = devName;
    initParam.nodeId = NODEID;
    initParam.ipAddress = (0xFFFFFF00 & IP_ADDR) | initParam.nodeId;

    /* write 00:00:00:00:00:00 to MAC address, so that the driver uses the real hardware address */
    memcpy(initParam.aMacAddress, macAddr_p, sizeof(initParam.aMacAddress));

    initParam.fAsyncOnly              = FALSE;
    initParam.featureFlags            = UINT_MAX;
    initParam.cycleLen                = cycleLen_p;         // required for error detection
    initParam.isochrTxMaxPayload      = 256;                // const
    initParam.isochrRxMaxPayload      = 256;                // const
    initParam.presMaxLatency          = 50000;              // const; only required for IdentRes
    initParam.preqActPayloadLimit     = 36;                 // required for initialisation (+28 bytes)
    initParam.presActPayloadLimit     = 36;                 // required for initialisation of Pres frame (+28 bytes)
    initParam.asndMaxLatency          = 150000;             // const; only required for IdentRes
    initParam.multiplCylceCnt         = 0;                  // required for error detection
    initParam.asyncMtu                = 1500;               // required to set up max frame size
    initParam.prescaler               = 2;                  // required for sync
    initParam.lossOfFrameTolerance    = 500000;
    initParam.asyncSlotTimeout        = 3000000;
    initParam.waitSocPreq             = 1000;
    initParam.deviceType              = UINT_MAX;           // NMT_DeviceType_U32
    initParam.vendorId                = UINT_MAX;           // NMT_IdentityObject_REC.VendorId_U32
    initParam.productCode             = UINT_MAX;           // NMT_IdentityObject_REC.ProductCode_U32
    initParam.revisionNumber          = UINT_MAX;           // NMT_IdentityObject_REC.RevisionNo_U32
    initParam.serialNumber            = UINT_MAX;           // NMT_IdentityObject_REC.SerialNo_U32

    initParam.subnetMask              = SUBNET_MASK;
    initParam.defaultGateway          = DEFAULT_GATEWAY;
    sprintf((char*)initParam.sHostname, "%02x-%08x", initParam.nodeId, initParam.vendorId);
    initParam.syncNodeId              = C_ADR_SYNC_ON_SOA;
    initParam.fSyncOnPrcNode          = FALSE;

    // set callback functions
    initParam.pfnCbEvent = stubProcessEvents;
    initParam.pfnCbSync  = NULL;

    // initialize POWERLINK stack
    ret = oplk_init(&initParam);
    if (ret != kErrorOk)
    {
        fprintf(stderr, "oplk_init() failed with \"%s\" (0x%04x)\n", debugstr_getRetValStr(ret), ret);
        return ret;
    }

    return kErrorOk;
}

//------------------------------------------------------------------------------
/**
\brief  Shutdown the test application

The function shuts down the test application.
*/
//------------------------------------------------------------------------------
static void shutdownPowerlink(void)
{
    UINT    i;

    // NMT_GS_OFF state has not yet been reached
    fGsOff_l = FALSE;

    // halt the NMT state machine so the processing of POWERLINK frames stops
    oplk_execNmtCommand(kNmtEventSwitchOff);

    // small loop to implement timeout waiting for thread to terminate
    for (i = 0; i < 1000; i++)
    {
        if (fGsOff_l)
            break;
    }

    printf("Stack is in state off ... Shutdown\n");
    oplk_shutdown();
}

//------------------------------------------------------------------------------
/**
\brief  Stub for receiving openPOWERLINK events

The function implements the a stub for application stack event handler.

\param  eventType_p         Type of event
\param  pEventArg_p         Pointer to union which describes the event in detail
\param  pUserArg_p          User specific argument

\return The function returns an integer error code.

\ingroup module_demo_mn_console
*/
//------------------------------------------------------------------------------
static tOplkError stubProcessEvents(tOplkApiEventType eventType_p,
                                    tOplkApiEventArg* pEventArg_p,
                                    void* pUserArg_p)
{
    tOplkError    ret = kErrorOk;

    UNUSED_PARAMETER(pUserArg_p);

    // check if NMT_GS_OFF is reached. No other event is handled.
    switch (eventType_p)
    {
        case kOplkApiEventNmtStateChange:
        {
            tEventNmtStateChange*   pNmtStateChange = &pEventArg_p->nmtStateChange;

            if (pNmtStateChange->newNmtState == kNmtGsOff)
            {
                ret = kErrorShutdown;
                // signal that stack is off
                fGsOff_l = TRUE;
            }
            // else case for other states is not handled in this demo.

            break;
        }
        default:
            break;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Run the tests for the application

The function runs the stack related tests in this application

\param  pArg_p       Argument pointer for the test function.

\return The function returns the test completion status.
\retval 0           Tests successfully completed
\retval -1          Tests failed
*/
//------------------------------------------------------------------------------
static int testAppFn(void* pArg_p)
{
    int     ret = 0;
    BYTE    aMacAddr[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    UNUSED_PARAMETER(pArg_p);
    printf("\tKernel stack version:\t0x%08X\n", oplk_getVersion());
    printf("\tKernel stack feature:\t0x%08X\n", oplk_getStackConfiguration());
    ret = oplk_getEthMacAddr((UINT8*) aMacAddr);
    if (ret == kErrorOk)
    {
        printf("\tMAC Address:\t\t%02X:%02X:%02X:%02X:%02X:%02X\n",
               aMacAddr[0], aMacAddr[1], aMacAddr[2],
               aMacAddr[3], aMacAddr[4], aMacAddr[5]);
    }
    else
    {
        printf("ERROR: Failed to read MAC address\n");
        ret = -1;
    }

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Get command line parameters

The function parses the supplied command line parameters and stores the
options at pOpts_p.

\param  argc_p                  Argument count.
\param  argc_p                  Pointer to arguments.
\param  pOpts_p                 Pointer to store options

\return The function returns the parsing status.
\retval 0           Successfully parsed
\retval -1          Parsing error
*/
//------------------------------------------------------------------------------
static int getOptions(int argc_p, char** argv_p, tOptions* pOpts_p)
{
    int    opt;

    /* setup default parameters */
    strncpy(pOpts_p->cdcFile, "mnobd.cdc", 256);
    pOpts_p->pLogFile = NULL;

    /* get command line parameters */
    while ((opt = getopt(argc_p, argv_p, "c:l:")) != -1)
    {
        switch (opt)
        {
            case 'c':
                strncpy(pOpts_p->cdcFile, optarg, 256);
                break;

            case 'l':
                pOpts_p->pLogFile = optarg;
                break;

            default: /* '?' */
                printf("Usage: %s [-c CDC-FILE] [-l LOGFILE]\n", argv_p[0]);
                return -1;
        }
    }
    return 0;
}

/// \}
