/**
********************************************************************************
\file   linux/target-linux.c

\brief  Target specific functions for Linux

The file implements target specific functions used in the openPOWERLINK stack.

\ingroup module_target
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
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>
#include <time.h>
#include <sys/time.h>

#include <common/oplkinc.h>
#include <common/ftracedebug.h>

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
static const UINT16 aCrc16Table_l[256] =
{
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
    0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
    0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
    0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
    0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
    0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
    0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
    0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
    0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
    0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
    0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
    0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
    0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
    0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
    0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
    0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
    0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
    0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
    0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
    0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
    0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
    0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
    0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0
};

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Initialize target specific stuff

The function initialize target specific stuff which is needed to run the
openPOWERLINK stack.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
tOplkError target_init(void)
{
    tOplkError  Ret  = kErrorOk;
    sigset_t    mask;

    /*
     * We have to block the real time signals used by the timer modules so
     * that they are able to wait on them using sigwaitinfo!
     */
    sigemptyset(&mask);
    sigaddset(&mask, SIGRTMIN);
    sigaddset(&mask, SIGRTMIN + 1);
    pthread_sigmask(SIG_BLOCK, &mask, NULL);

    /* Enabling ftrace for debugging */
    FTRACE_OPEN();
    FTRACE_ENABLE(TRUE);

    return Ret;
}

//------------------------------------------------------------------------------
/**
\brief  Clean up target specific stuff

The function cleans up target specific stuff.

\return The function returns a tOplkError error code.
*/
//------------------------------------------------------------------------------
tOplkError target_cleanup(void)
{
    tOplkError  Ret  = kErrorOk;

    /* Disable ftrace debugging */
    FTRACE_ENABLE(FALSE);

    return Ret;
}

//------------------------------------------------------------------------------
/**
\brief Sleep for the specified number of milliseconds

The function makes the calling thread sleep until the number of specified
milliseconds has elapsed.

\param  milliSeconds_p      Number of milliseconds to sleep

\ingroup module_target
*/
//------------------------------------------------------------------------------
void target_msleep(UINT32 milliSeconds_p)
{
    struct  timeval timeout;
    fd_set          readFds;
    int             maxFd;
    int             selectRetVal;
    unsigned int    seconds;
    unsigned int    microSeconds;

    // initialize file descriptor set
    maxFd = 0 + 1;
    FD_ZERO(&readFds);

    // Calculate timeout values
    seconds = milliSeconds_p / 1000;
    microSeconds = (milliSeconds_p - (seconds * 1000)) * 1000;

    // initialize timeout value
    timeout.tv_sec = seconds;
    timeout.tv_usec = microSeconds;

    selectRetVal = select(maxFd, &readFds, NULL, NULL, &timeout);
    switch (selectRetVal)
    {
        case 0:     // select timeout occurred, no packet received
            break;

        case -1:    // select error occurred
            break;

        default:    // packet available for receive
            break;
    }
}

//------------------------------------------------------------------------------
/**
\brief  Set IP address of specified Ethernet interface

The function sets the IP address, subnetMask and MTU of an Ethernet
interface.

\param  ifName_p                Name of ethernet interface.
\param  ipAddress_p             IP address to set for interface.
\param  subnetMask_p            Subnet mask to set for interface.
\param  mtu_p                   MTU to set for interface.

\return The function returns a tOplkError error code.

\ingroup module_target
*/
//------------------------------------------------------------------------------
tOplkError target_setIpAdrs(char* ifName_p, UINT32 ipAddress_p, UINT32 subnetMask_p, UINT16 mtu_p)
{
    tOplkError  ret = kErrorOk;
    INT         iRet;
    char        sBufferIp[16];
    char        sBufferMask[16];
    char        sBufferMtu[6];
    char        command[256];

    // configure IP address of virtual network interface
    // for TCP/IP communication over the POWERLINK network
    snprintf(sBufferIp, sizeof(sBufferIp), "%u.%u.%u.%u",
             (UINT)(ipAddress_p >> 24), (UINT)((ipAddress_p >> 16) & 0xFF),
             (UINT)((ipAddress_p >> 8) & 0xFF), (UINT)(ipAddress_p & 0xFF));

    snprintf(sBufferMask, sizeof(sBufferMask), "%u.%u.%u.%u",
             (UINT)(subnetMask_p >> 24), (UINT)((subnetMask_p >> 16) & 0xFF),
             (UINT)((subnetMask_p >> 8) & 0xFF), (UINT)(subnetMask_p & 0xFF));

    snprintf(sBufferMtu, sizeof(sBufferMtu), "%u", (UINT)mtu_p);

    /* call ifconfig to configure the virtual network interface */
    sprintf (command, "/sbin/ifconfig %s %s netmask %s mtu %s",
             ifName_p, sBufferIp, sBufferMask, sBufferMtu);
    if ((iRet = system(command)) < 0)
    {
        TRACE("ifconfig %s %s returned %d\n", ifName_p, sBufferIp, iRet);
        return kErrorNoResource;
    }

    return ret;
}

//------------------------------------------------------------------------------
/**
\brief  Set default gateway for Ethernet interface

The function sets the default gateway of an Ethernet interface.

\param  defaultGateway_p            Default gateway to set.

\return The function returns a tOplkError error code.

\ingroup module_target
*/
//------------------------------------------------------------------------------
tOplkError target_setDefaultGateway(UINT32 defaultGateway_p)
{
    tOplkError  ret = kErrorOk;
    INT         iRet;
    char        sBuffer[16];
    char        command[128];

    if (defaultGateway_p != 0)
    {
        // configure default gateway of virtual network interface
        // for TCP/IP communication over the POWERLINK network
        snprintf(sBuffer, sizeof(sBuffer), "%u.%u.%u.%u",
                 (UINT)(defaultGateway_p >> 24), (UINT)((defaultGateway_p >> 16) & 0xFF),
                 (UINT)((defaultGateway_p >> 8) & 0xFF), (UINT)(defaultGateway_p & 0xFF));

        sprintf(command, "route del default");
        iRet = system(command);
        TRACE("route del default returned %d\n", iRet);

        /* call route to configure the default gateway */
        sprintf(command, "route add default gw %s", sBuffer);
        if ((iRet = system(command)) < 0)
        {
            TRACE("route add default gw %s returned %d\n", sBuffer, iRet);
            return kErrorNoResource;
        }
    }
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief    Get current system tick

This function returns the current system tick determined by the system timer.

\return Returns the system tick in milliseconds

\ingroup module_target
*/
//------------------------------------------------------------------------------
UINT32 target_getTickCount(void)
{
    UINT32                  ticks;
    struct timespec         curTime;

    clock_gettime(CLOCK_MONOTONIC, &curTime);
    ticks = (curTime.tv_sec * 1000) + (curTime.tv_nsec / 1000000);

    return ticks;
}

//------------------------------------------------------------------------------
/**
\brief  Calculate CRC16 for buffer data

The function calculates CRC16 for the data in the passed buffer.
This CRC16 version is used in CANopen for SDO CRC calculation.

 Generated on Wed Jan  9 14:34:53 2013,
 by pycrc v0.8, http://www.tty1.net/pycrc/
 using the configuration:
    Model        = ccitt
    Width        = 16
    Poly         = 0x1021
    XorIn        = 0xffff
    ReflectIn    = False
    XorOut       = 0x0000
    ReflectOut   = False
    Algorithm    = table-driven

\param  crc_p           Initialized value of CRC.
\param  pData_p         Pointer to the data buffer.
\param  size_p          Size of data in the buffer, in bytes.

\return The function returns an unsigned 16 bit CRC value.

\ingroup module_target
*/
//------------------------------------------------------------------------------
UINT16 target_calculateCrc16(UINT16 crc_p, UINT8* pData_p, UINT32 size_p)
{
    UINT idx;

    while (size_p--)
    {
        idx = ((crc_p >> 8) ^ *pData_p) & 0xFF;
        crc_p = (aCrc16Table_l[idx] ^ (crc_p << 8));
        pData_p++;
    }

    return crc_p;
}
