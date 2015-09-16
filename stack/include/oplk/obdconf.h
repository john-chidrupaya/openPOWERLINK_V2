/****************************************************************************

  (c) SYSTEC electronic GmbH, D-08468 Heinsdorfergrund, Am Windrad 2
      www.systec-electronic.com

  Project:      openPOWERLINK

  Description:  target specific functions for OD store/restore

  License:

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    1. Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.

    3. Neither the name of SYSTEC electronic GmbH nor the names of its
       contributors may be used to endorse or promote products derived
       from this software without prior written permission. For written
       permission, please contact info@systec-electronic.com.

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

  -------------------------------------------------------------------------

                $RCSfile$

                $Author$

                $Revision$  $Date$

                $State$

                Build Environment:
                    GCC V3.4

  -------------------------------------------------------------------------

  Revision History:

  2013/01/08 d.k.:   start of the implementation

****************************************************************************/

#ifndef _EPLTGTOBDARC_H_
#define _EPLTGTOBDARC_H_


//---------------------------------------------------------------------------
// const defines
//---------------------------------------------------------------------------

// storage read access parameters for objects 0x1010 and 0x1011 (see EPSG 301)
#define OBD_STORE_UNSUPPORTED   0x00000000L
#define OBD_STORE_ON_COMMAND    0x00000001L
#define OBD_STORE_AUTONOMOUSLY  0x00000002L


//---------------------------------------------------------------------------
// typedef
//---------------------------------------------------------------------------


//---------------------------------------------------------------------------
// function prototypes
//---------------------------------------------------------------------------
tOplkError obdconf_init(void);
tOplkError obdconf_exit(void);
tOplkError obdconf_createPart(tObdPart odPart_p);
tOplkError obdconf_deletePart (tObdPart odPart_p);
tOplkError obdconf_openPart(tObdPart odPart_p);
tOplkError obdconf_closePart(tObdPart odPart_p);
tOplkError obdconf_storePart(tObdPart odPart_p, UINT8 *pData, UINT32 size_p);
tOplkError obdconf_restorePart(tObdPart odPart_p, UINT8 *pData, UINT32 size_p);
tOplkError obdconf_getTargetCapabilities(UINT index_p, UINT subIndex_p,
                                         tObdPart* pOdPart_p, UINT32* pDevCap_p);
BOOL obdconf_isPartArchiveValid(tObdPart odPart_p);
tOplkError obdconf_setBackupArchivePath(const char* pBackupPath_p);
tOplkError obdconf_setPartSignature(tObdPart odPart_p, UINT32 signature_p);

#endif  // #ifndef _EPLTGTOBDARC_H_

