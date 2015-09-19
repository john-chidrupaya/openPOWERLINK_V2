/**
********************************************************************************
\file   oplk/obdconf.h

\brief  Target specific functions for OD store/restore

This file contains the target specific functions and definitions for OD store/restore.

*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2015, SYSTEC electronic GmbH, D-08468 Heinsdorfergrund, Am Windrad 2
Copyright (c) 2015, Kalycito Infotech Private Limited
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

#ifndef _INC_oplk_obdconf_H_
#define _INC_oplk_obdconf_H_RC_H_


//---------------------------------------------------------------------------
// const defines
//---------------------------------------------------------------------------

// Storage read access parameters for objects 0x1010 and 0x1011 (see EPSG 301)
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

#endif  // #ifndef _INC_oplk_obdconf_H_

