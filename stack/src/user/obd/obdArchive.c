/**
********************************************************************************
\file   obdconf.c

\brief

\ingroup module_obd
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2015, Kalycito Infotech Private Limited
Copyright (c) 2013, SYSTEC electronic GmbH
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
#ifdef _WIN32
    #include <windows.h>
    #pragma warning(disable:4996)
#endif

#include <common/oplkinc.h>
#include <oplk/obd.h>
//#include "EplCrc.h"
#include "obdconf.h"

#include <sys/stat.h>
#include <assert.h>
#include <stdlib.h>
#include <fcntl.h>
#include <errno.h>

#if (TARGET_SYSTEM == _WIN32_)

    #include <io.h>
    #include <sys/types.h>
    #include <sys/utime.h>
    #include <sys/timeb.h>
    #include <time.h>
    #include <direct.h>
    #include <string.h>

#elif (TARGET_SYSTEM == _LINUX_)

    #include <sys/io.h>
    #include <unistd.h>
    #include <sys/vfs.h>
    #include <sys/types.h>
    #include <sys/timeb.h>
    #include <utime.h>
    #include <limits.h>

#elif (DEV_SYSTEM == _DEV_PAR_BECK1X3_)

    #include <io.h>
    #include <string.h>

#endif

//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------
#if (TARGET_SYSTEM == _WIN32_)

    #define flush  _commit

#elif (TARGET_SYSTEM == _LINUX_)

    #define O_BINARY 0
    #define _MAX_PATH PATH_MAX
    #define flush  fsync

#elif (DEV_SYSTEM == _DEV_PAR_BECK1X3_)

    #define flush(h)                    // #define flush() to nothing

#endif
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
#ifndef EPLTGTOBDARC_FILENAME_PREFIX
    #define OBD_ARCHIVE_FILENAME_PREFIX        "oplkOd"
#endif

#ifndef EPLTGTOBDARC_FILENAME_EXTENSION
    #define OBD_ARCHIVE_FILENAME_EXTENSION     ".bin"
#endif

//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------
typedef struct
{
    INT         hBkupArchiveFile;
    char*       pBackupPath;
    BOOL        fOpenForWrite;
    UINT32      odSignPartDev;
    UINT32      odSignPartGen;
    UINT32      odSignPartMan;
    UINT16      odDataCrc;
}tObdConfInstance;

//------------------------------------------------------------------------------
// local vars
//------------------------------------------------------------------------------
static UINT32 obdConfSignature_l;
static tObdConfInstance aObdConfInstance_l[1];

//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------
static tOplkError getFilePath(tObdPart odPart_p, char* pBkupPath_p,
                              char* pFilePathName_p);

/***************************************************************************/
/*          C L A S S  <Store/Load>                                        */
/***************************************************************************/
/**
  Description:

  File XXX%d_Com.bin:
          +----------------------+
  0x0000  | target signature     | (4 byte)
          +----------------------+
  0x0004  | OD signature of part | (4 byte)
          |  0x1000 - 0x1FFF     |
          +----------------------+
  0x0008  | all OD data of part  | (n byte)
          |  0x1000 - 0x1FFF     |
          +----------------------+
  0xNNNN  | OD data CRC          | (2 byte)
          +----------------------+

  File XXX%d_Man.bin:
          +----------------------+
  0x0000  | target signature     | (4 byte)
          +----------------------+
  0x0004  | OD signature of part | (4 byte)
          |  0x2000 - 0x5FFF     |
          +----------------------+
  0x0008  | all OD data of part  | (n byte)
          |  0x2000 - 0x5FFF     |
          +----------------------+
  0xNNNN  | OD data CRC          | (2 byte)
          +----------------------+

  File XXX%d_Dev.bin:
          +----------------------+
  0x0000  | target signature     | (4 byte)
          +----------------------+
  0x0004  | OD signature of part | (4 byte)
          |  0x6000 - 0x9FFF     |
          +----------------------+
  0x0008  | all OD data of part  | (n byte)
          |  0x6000 - 0x9FFF     |
          +----------------------+
  0xNNNN  | OD data CRC          | (2 byte)
          +----------------------+
*/

//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief  Initialize OD module

The function initializes the OD module.

\param  pInitParam_p            Pointer to OD initialization parameters.

\return The function returns a tOplkError error code.

\ingroup module_obd
*/
//------------------------------------------------------------------------------

//---------------------------------------------------------------------------
//
// Function:    EplTgtObdArcInit()
//
// Description: initializes functionality for STORE, RESTORE and LOAD
//              for OD values in other memory mediums, initializes
//              interface to non-volatile memory if it not done by
//              operating systems
//
// Parameters:  void  = (instance handle)
//
// Returns:     tOplkError             = error code
//
//---------------------------------------------------------------------------

tOplkError obdconf_init(void)
{
    tOplkError ret = kErrorOk;
    tObdConfInstance* pInstEntry;

    obdConfSignature_l = 0x444F4C50;  // signature PLOD

    // get current instance entry and initialize all members
    pInstEntry = &aObdConfInstance_l[0];
    OPLK_MEMSET(pInstEntry, 0, sizeof(tObdConfInstance));
    pInstEntry->hBkupArchiveFile = -1;

    return ret;

}

//---------------------------------------------------------------------------
//
// Function:    EplTgtObdArcShutdown()
//
// Description: disable interface to non-volatile memory
//
// Parameters:  void  = (instance handle)
//
// Returns:     tOplkError             = error code
//
//---------------------------------------------------------------------------

tOplkError obdconf_exit(void)
{
    tOplkError ret;

    ret = kErrorOk;

    return ret;

}

//---------------------------------------------------------------------------
//
// Function:    EplTgtObdArcCreate()
//
// Description: Function creates an archiv for the selected OD part. In
//              existence archiv is set unvalid.
//
// Parameters:  void  = (instance handle)
//              CurrentOdPart_p       = OD-part
//
// Returns:     tOplkError             = error code
//
//---------------------------------------------------------------------------

tOplkError obdconf_createPart(tObdPart odPart_p)
{
    tOplkError          ret = kErrorObdStoreHwError;
    INT                 writeCount;
    char                aFilePath[_MAX_PATH];
    UINT32              odSignature;
    tObdConfInstance*   pInstEntry;

    // get current instance entry
    pInstEntry = &aObdConfInstance_l[0];

    // get the file path for current OD part and instance
    ret = getFilePath(odPart_p, pInstEntry->pBackupPath, &aFilePath[0]);
    if (ret != kErrorOk)
    {
        goto Exit;
    }

    // is the file already opened?
    if (pInstEntry->hBkupArchiveFile >= 0)
    {
        ret = kErrorObdStoreInvalidState;
        goto Exit;
    }

    // check OD part for correct OD signature
    switch (odPart_p)
    {
        case kObdPartGen:
            odSignature = pInstEntry->odSignPartGen;
            break;

        case kObdPartMan:
            odSignature = pInstEntry->odSignPartMan;
            break;

        case kObdPartDev:
            odSignature = pInstEntry->odSignPartDev;
            break;

        default:
            ret = kErrorInvalidInstanceParam;
            goto Exit;
    }

    // open file for writing
    pInstEntry->fOpenForWrite = TRUE;
    pInstEntry->hBkupArchiveFile = open(aFilePath, O_CREAT | O_TRUNC | O_WRONLY | O_BINARY, 0666); //TODO@J Move to target source
    if (pInstEntry->hBkupArchiveFile < 0)
    {
        pInstEntry->hBkupArchiveFile = -1;
        goto Exit;
    }

    // write target signature and calculate CRC for it
    pInstEntry->odDataCrc = CALCULATE_CRC16(0,
                                            (UINT8*)&obdConfSignature_l,
                                            sizeof(obdConfSignature_l));
    writeCount = write(pInstEntry->hBkupArchiveFile,
                       (UINT8*)&obdConfSignature_l,
                       sizeof(obdConfSignature_l));
    if (writeCount != (INT)sizeof(obdConfSignature_l))
    {
        goto Exit;
    }

    // write OD signature and calculate CRC for it
    pInstEntry->odDataCrc = CALCULATE_CRC16(pInstEntry->odDataCrc,
                                            (UINT8*)&odSignature,
                                            sizeof(odSignature));
    writeCount = write(pInstEntry->hBkupArchiveFile,
                       (UINT8*)&odSignature,
                       sizeof(odSignature));
    if (writeCount != (INT)sizeof(odSignature))
    {
        goto Exit;
    }

    ret = kErrorOk;

Exit:
    return ret;

}


//---------------------------------------------------------------------------
//
// Function:    EplTgtObdArcDelete()
//
// Description: Function sets an archiv unvalid.
//
// Parameters:  void  = (instance handle)
//              CurrentOdPart_p       = OD-part
//
// Returns:     tOplkError             = error code
//
//---------------------------------------------------------------------------
tOplkError obdconf_deletePart (tObdPart odPart_p)
{
    tOplkError          ret = kErrorObdStoreHwError;
    char                aFilePath[_MAX_PATH];
    tObdConfInstance*   pInstEntry;

    // get current instance entry
    pInstEntry = &aObdConfInstance_l[0];

    // get the file path for current OD part and instance
    ret = getFilePath(odPart_p, pInstEntry->pBackupPath, &aFilePath[0]);
    if (ret != kErrorOk)
    {
        goto Exit;
    }

    // delete the backup archive file
    if (unlink(aFilePath) == -1) //TODO@J Move to target specific source--> close()??
    {
        ret = kErrorObdStoreHwError;
        goto Exit;
    }

    ret = kErrorOk;

Exit:
    return ret;

}


//---------------------------------------------------------------------------
//
// Function:    EplTgtObdArcOpen()
//
// Description: Function opens an in existence archive.
//
// Parameters:  void  = (instance handle)
//              CurrentOdPart_p       = OD-part
//
// Returns:     tOplkError             = error code
//
//---------------------------------------------------------------------------

tOplkError obdconf_openPart(tObdPart odPart_p)
{
    UINT8               ret = kErrorObdStoreHwError;
    char                aFilePath[_MAX_PATH];
    tObdConfInstance*   pInstEntry;

    // get current instance entry
    pInstEntry = &aObdConfInstance_l[0];

    // get the file path for current OD part and instance
    ret = getFilePath(odPart_p, pInstEntry->pBackupPath, &aFilePath[0]);
    if (ret != kErrorOk)
    {
        goto Exit;
    }

    ret = kErrorObdStoreHwError;

    // is the file already opened?
    if (pInstEntry->hBkupArchiveFile >= 0)
    {
        ret = kErrorObdStoreInvalidState;
        goto Exit;
    }

    // open backup archive file for read
    pInstEntry->fOpenForWrite = FALSE;
    pInstEntry->hBkupArchiveFile = open(aFilePath, O_RDONLY | O_BINARY, 0666); //TODO @J: Move to target source file

    if (pInstEntry->hBkupArchiveFile < 0)
    {
        // backup archive file could not be opend
        goto Exit;
    }

    // set file position to the begin of the file
    lseek(pInstEntry->hBkupArchiveFile, 0, SEEK_SET);//TODO @J:Move to target source file

    ret = kErrorOk;

Exit:
    return (ret);

}


//---------------------------------------------------------------------------
//
// Function:    EplTgtObdArcClose()
//
// Description: Function closes an archiv for the selected OD part by
//              setting a valid signature.
//
// Parameters:  void  = (instance handle)
//              CurrentOdPart_p       = OD-part
//
// Returns:     tOplkError             = error code
//
//---------------------------------------------------------------------------

tOplkError obdconf_closePart(tObdPart odPart_p)
{
    tOplkError          ret = kErrorOk;
    INT                 errCode;
    UINT8               data;
    tObdConfInstance*   pInstEntry;

    UNUSED_PARAMETER(odPart_p);

    // get current instance entry
    pInstEntry = &aObdConfInstance_l[0];

    // is the file not opened?
    if (pInstEntry->hBkupArchiveFile < 0)
    {
        ret = kErrorObdStoreInvalidState;
        goto Exit;
    }

    // if file was opened for write we have to add the OD data CRC at the end of the file
    if (pInstEntry->fOpenForWrite != FALSE)
    {
        // write CRC16 to end of the file (in big endian format)
        data  = (UINT8)((pInstEntry->odDataCrc >> 8) & 0xFF);
        errCode   = write(pInstEntry->hBkupArchiveFile, (UINT8*)&data, sizeof(data));//TODO @J:Move to target source file
        data  = (UINT8)((pInstEntry->odDataCrc >> 0) & 0xFF);
        errCode  += write(pInstEntry->hBkupArchiveFile, (UINT8*)&data, sizeof(data));
        if (errCode != (INT)(sizeof(data) * 2))
        {   // save error code and close the file
            ret = kErrorObdStoreHwError;
        }

        // sync file to disc
        flush(pInstEntry->hBkupArchiveFile);//TODO @J:Move to target source file
    }

    // close archive file and set file handle invalid
    close(pInstEntry->hBkupArchiveFile);//TODO @J:Move to target source file
    pInstEntry->hBkupArchiveFile = -1;

Exit:
    return ret;

}


//---------------------------------------------------------------------------
//
// Function:    EplTgtObdArcStore()
//
// Description: Function saves the parameter of the select part into
//              memory.
//
// Parameters:  void  = (instance handle)
//              CurrentOdPart_p       = OD-part
//              pointer to source data
//              number of bytes
//
// Returns:     tOplkError             = error code
//
//---------------------------------------------------------------------------

tOplkError obdconf_storePart(tObdPart odPart_p, UINT8 *pData, UINT32 size_p)
{
    tOplkError          ret = kErrorObdStoreHwError;
    INT                 writeCount;
    tObdConfInstance*   pInstEntry;

    UNUSED_PARAMETER(odPart_p);

    // get current instance entry
    pInstEntry = &aObdConfInstance_l[0];

    // is the file not opened?
    if (pInstEntry->hBkupArchiveFile < 0)
    {
        ret = kErrorObdStoreInvalidState;
        goto Exit;
    }

    // write current OD data to the file and calculate the CRC for it
    pInstEntry->odDataCrc = CALCULATE_CRC16(pInstEntry->odDataCrc, pData, size_p);
    writeCount = write(pInstEntry->hBkupArchiveFile, (UINT8*)pData, size_p);//TODO @J:Move to target source file
    if (writeCount != (INT)size_p)
    {
        goto Exit;
    }

    ret = kErrorOk;
Exit:
    return ret;

}


//---------------------------------------------------------------------------
//
// Function:    EplTgtObdArcRestore()
//
// Description: Function reads the parameter from memory and stores the
//              value into the selected OD part.
//
// Parameters:  void  = (instance handle)
//              CurrentOdPart_p       = OD-part
//              pointer to destination data
//              number of bytes
//
// Returns:     tOplkError             = error code
//
//---------------------------------------------------------------------------

tOplkError obdconf_restorePart(tObdPart odPart_p, UINT8 *pData, UINT32 size_p)
{
    tOplkError          ret = kErrorObdStoreHwError;
    INT                 readCount;
    tObdConfInstance*   pInstEntry;

    UNUSED_PARAMETER(odPart_p);

    // get current instance entry
    pInstEntry = &aObdConfInstance_l[0];

    // is the file not opened?
    if (pInstEntry->hBkupArchiveFile < 0)
    {
        ret = kErrorObdStoreInvalidState;
        goto Exit;
    }

    // read OD data from current file position
    readCount = read(pInstEntry->hBkupArchiveFile, (UINT8*)pData, size_p);//TODO @J:Move to target source file
    if (readCount != (INT)size_p)
    {
        goto Exit;
    }

    ret = kErrorOk;

Exit:
    return ret;

}


//---------------------------------------------------------------------------
//
// Function:    EplTgtObdArcGetCapabilities()
//
// Description: The function returns the storage capabilities corresponding
//              to the specified object index and sub-index.
//              Additionally it returns the OD part corresponding to the
//              specified sub-index.
//
// Parameters:  uiIndex_p           = [IN] identifies command (0x1010 = save, 0x1011 = restore)
//              uiSubIndex_p        = [IN] identifies OD part
//              pOdPart_p           = [OUT] pointer to OD part
//              pdwCap_p            = [OUT] pointer to capabilities bit-field
//                  EPL_OBD_STORE_UNSUPPORTED  = 0x00000000 - Device does not save parameters.
//                  EPL_OBD_STORE_ON_COMMAND   = 0x00000001 - Device saves parameters on command.
//                  EPL_OBD_STORE_AUTONOMOUSLY = 0x00000002 - Device saves parameters autonomously.
//
// Returns:     tOplkError              = error code
//
//---------------------------------------------------------------------------
tOplkError obdconf_getTargetCapabilities(UINT index_p, UINT subIndex_p,
                                         tObdPart* pOdPart_p, UINT32* pDevCap_p)
{
    tOplkError ret = kErrorOk;

    UNUSED_PARAMETER(index_p); //TODO Check index to verify it is indeed the correct one

    if ((pOdPart_p == NULL) || (pDevCap_p == NULL))
    {
        ret = kErrorApiInvalidParam;
        goto Exit;
    }

    switch (subIndex_p)
    {
        // Device supports save/restore of index ranges
        // 0x1000-0x1FFF, 0x2000-0x5FFF, 0x6000-0x9FFF
        case 1:
            *pDevCap_p = OBD_STORE_ON_COMMAND;
            *pOdPart_p = kObdPartAll;
            break;

        // Device supports save/restore of index range
        // 0x1000-0x1FFF
        case 2:
            *pDevCap_p = OBD_STORE_ON_COMMAND;
            *pOdPart_p = kObdPartGen;
            break;

        // Device supports not save/restore of index
        // 0x6000-0x9FFF
        case 3:
            *pDevCap_p = OBD_STORE_ON_COMMAND;
            *pOdPart_p = kObdPartDev;
            break;

        // Device supports save/restore of index
        // 0x2000-0x5FFF
        case 4:
            *pDevCap_p = OBD_STORE_ON_COMMAND;
            *pOdPart_p = kObdPartMan;
            break;

        default:
            *pDevCap_p = 0;
            break;
    }

Exit:
    return ret;
}

//---------------------------------------------------------------------------
//
// Function:    EplTgtObdArcCheckValid()
//
// Description: Function checks up if signature of selected OD
//              part is valid.
//
// Parameters:  void  = (instance handle)
//              CurrentOdPart_p       = OD-part
//
// Returns:     FALSE  -> signature is unvalid
//              TRUE   -> signature is valid
//
//---------------------------------------------------------------------------
BOOL obdconf_verifyPartSignature(tObdPart odPart_p)
{
    BOOL                ret = FALSE;
    INT                 readCount;
    UINT32              readTargetSign;
    UINT32              readOdSign;
    UINT32              curReadSign;
    UINT8               aTempBuffer[8];
    INT                 count;
    UINT16              dataCrc;
    tObdConfInstance*   pInstEntry;

    // get current instance entry
    pInstEntry = &aObdConfInstance_l[0];

    // is the file not opened?
    if (pInstEntry->hBkupArchiveFile < 0)
    {
        goto Exit;
    }

    // check OD part for correct OD signature
    switch (odPart_p)
    {
        case kObdPartGen:
            curReadSign = pInstEntry->odSignPartGen;
            break;

        case kObdPartMan:
            curReadSign = pInstEntry->odSignPartMan;
            break;

        case kObdPartDev:
            curReadSign = pInstEntry->odSignPartDev;
            break;

        default:
            goto Exit;
    }

    // read target signature and calculate the CRC for it
    readCount = read(pInstEntry->hBkupArchiveFile, &readTargetSign, sizeof(readTargetSign));//TODO @J: move to target src
    dataCrc = CALCULATE_CRC16(0, (UINT8*)&readTargetSign, sizeof(readTargetSign));//TODO @J: move to target src
    if (readCount != (INT)sizeof(readTargetSign))
    {
        goto Exit;
    }

    // read OD signature and calculate the CRC for it
    readCount = read(pInstEntry->hBkupArchiveFile, &readOdSign, sizeof(readOdSign));//TODO @J: move to target src
    dataCrc = CALCULATE_CRC16(dataCrc, (UINT8*)&readOdSign, sizeof(readOdSign));//TODO @J: move to target src
    if (readCount != (INT)sizeof(readOdSign))
    {
        goto Exit;
    }

    // check if both target signature and OD signature are correct
    if ((readTargetSign != obdConfSignature_l) || (readOdSign  != curReadSign))
    {
        goto Exit;
    }

    // calculate OD data CRC over all data bytes
    for (;;)
    {
        count = read(pInstEntry->hBkupArchiveFile, &aTempBuffer[0], sizeof(aTempBuffer));//TODO @J: move to target src
        if (count > 0)
        {
            dataCrc = CALCULATE_CRC16(dataCrc, (UINT8*)&aTempBuffer[0], count);
        }
        else
        {
            break;
        }
    }

    // check OD data CRC (always zero because CRC has to be set at the end of the file in big endian format)
    if (dataCrc != 0)
    {
        goto Exit;
    }

    //XXX@J Does this belong here??
    // set file position back to OD data
    lseek(pInstEntry->hBkupArchiveFile, sizeof(readTargetSign) + sizeof(readOdSign), SEEK_SET);//TODO @J: move to target src

    ret = TRUE;

Exit:
    return ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplTgtObdArcSetBackupPath()
//
// Description: Function set the file path for backup archive.
//
// Parameters:  void  = (instance handle)
//              pszBackupPath_p        = selected OD-part
//
// Returns:     tOplkError -> error code
//
//---------------------------------------------------------------------------

tOplkError obdconf_setBackupArchivePath(const char* pBackupPath_p)
{
    tOplkError          ret = kErrorObdStoreHwError;
    tObdConfInstance*   pInstEntry;

    // check pointer to backup path string
    if (pBackupPath_p == NULL)
    {
        ret = kErrorApiInvalidParam;
        goto Exit;
    }

    // get current instance entry
    pInstEntry = &aObdConfInstance_l[0];

    // save pointer to backup path
    pInstEntry->pBackupPath = (char*)pBackupPath_p;

    ret = kErrorOk;

Exit:
    return ret;
}


//---------------------------------------------------------------------------
//
// Function:    EplTgtObdArcSetSignature()
//
// Description: Function set the signature of object dictionary.
//
// Parameters:  void  = (instance handle)
//              CurrentOdPart_p       = OD-part
//
// Returns:     tOplkError -> error code
//
//-------------------------------------------------------------PUBLIC --------------

tOplkError obdconf_setPartSignature(tObdPart odPart_p, UINT32 signature_p)
{
    tOplkError          ret = kErrorOk;
    tObdConfInstance*   pInstEntry;

    // get current instance entry
    pInstEntry = &aObdConfInstance_l[0];

    // check OD part
    switch (odPart_p)
    {
        case kObdPartGen:
            pInstEntry->odSignPartGen = signature_p;
            break;

        case kObdPartMan:
            pInstEntry->odSignPartMan = signature_p;
            break;

        case kObdPartDev:
            pInstEntry->odSignPartDev = signature_p;
            break;

        default:
            ret = kErrorApiInvalidParam;
            goto Exit;
    }

Exit:
    return ret;
}

//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name Private Functions
/// \{

//---------------------------------------------------------------------------
//
// Function:    EplTgtObdArcGetFilePath()
//
// Description: Function returns the file path for the file archive.
//
// Parameters:  void  = (instance handle)
//              CurrentOdPart_p       = [in]  OD-part
//              pszBackupPath_p        = [in]  directory path
//              pszBackupFilePath_p    = [out] pointer to a buffer for receiving the file path
//
// Returns:     tOplkError -> error code
//
//---------------------------------------------------------------------------

static tOplkError getFilePath(tObdPart odPart_p,
                              char* pszBackupPath_p,
                              char* pszBackupFilePath_p)
{
    tOplkError ret = kErrorObdStoreHwError;
    size_t     len;

    // build complete file path string
    if (pszBackupPath_p != NULL)
    {
        strcpy(pszBackupFilePath_p, pszBackupPath_p);
    }
    else
    {
        pszBackupFilePath_p[0] = '\0';
    }
    len = strlen(pszBackupFilePath_p);
    if ((len > 0)
        && (pszBackupFilePath_p[len-1] != '\\')
        && (pszBackupFilePath_p[len-1] != '/'))
    {
        strcat(pszBackupFilePath_p, "/");
    }
    strcat(pszBackupFilePath_p, OBD_ARCHIVE_FILENAME_PREFIX);

    // check OD part
    switch (odPart_p)
    {
        case kObdPartGen:
            strcat(pszBackupFilePath_p, "_Com");
            break;

        case kObdPartMan:
            strcat(pszBackupFilePath_p, "_Man");
            break;

        case kObdPartDev:
            strcat(pszBackupFilePath_p, "_Dev");
            break;

        default:
            ret = kErrorApiInvalidParam;
            goto Exit;
    }

    strcat(pszBackupFilePath_p, OBD_ARCHIVE_FILENAME_EXTENSION);

    ret = kErrorOk;
Exit:
    return ret;

}

/// \}
