/* 
 * FreeModbus Libary: A portable Modbus implementation for Modbus ASCII/RTU.
 * Copyright (c) 2006-2018 Christian Walter <cwalter@embedded-solutions.at>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/* ----------------------- System includes ----------------------------------*/
#include "stdlib.h"
#include "string.h"
#include <stdarg.h>

/* ----------------------- Platform includes --------------------------------*/
#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbconfig.h"
#include "mbproto.h"
#include "mbfunc.h"
#include "mb_typedef.h"

#include "mbport.h"
#if MB_RTU_ENABLED == 1
#include "mbrtu.h"
#endif
#if MB_ASCII_ENABLED == 1
#include "mbascii.h"
#endif
#if MB_TCP_ENABLED == 1
#include "mbtcp.h"
#endif

#ifndef MB_PORT_HAS_CLOSE
#define MB_PORT_HAS_CLOSE 0
#endif

/* ----------------------- Static variables ---------------------------------*/

static UCHAR    ucMBAddress;

/* Functions pointer which are initialized in eMBInit( ). Depending on the
 * mode (RTU or ASCII) the are set to the correct implementations.
 */

/* Callback functions required by the porting layer. They are called when
 * an external event has happend which includes a timeout or the reception
 * or transmission of a character.
 */

/* Modbus structures (example). Structures should not be declared in this file*/

modbus_t mb_RTU = {
    .peMBFrameSendCur = eMBRTUSend,
    .pvMBFrameStartCur = eMBRTUStart,
    .pvMBFrameStopCur = eMBRTUStop,
    .peMBFrameReceiveCur = eMBRTUReceive,
    .pvMBFrameCloseCur = MB_PORT_HAS_CLOSE ? vMBPortClose : NULL,
    .eMBCurrentMode = MB_RTU,
    .ucSlaveAddress = 1,
    .ucPort = 1,
    .ulBaudRate = 9600,
    .eParity = MB_PAR_NONE,
    .mbStatus = STATE_NOT_INITIALIZED
};

modbus_t mb_TCP = {
    .peMBFrameSendCur = eMBTCPSend,
    .pvMBFrameStartCur = eMBTCPStart,
    .pvMBFrameStopCur = eMBTCPStop,
    .peMBFrameReceiveCur = eMBTCPReceive,
    .pvMBFrameCloseCur = MB_PORT_HAS_CLOSE ? vMBTCPPortClose : NULL,
    .eMBCurrentMode = MB_TCP,
    .ucTCPPort = 502,
    .mbStatus = STATE_NOT_INITIALIZED
};

/* For RTU or ASCII */
BOOL(*pxMBFrameCBByteReceived) (void);
BOOL(*pxMBFrameCBTransmitterEmpty) (void);
BOOL(*pxMBPortCBTimerExpired) (void);
/* An array of Modbus functions handlers which associates Modbus function
 * codes with implementing functions.
 */
static xMBFunctionHandler xFuncHandlers[MB_FUNC_HANDLERS_MAX] = {
#if MB_FUNC_OTHER_REP_SLAVEID_ENABLED > 0
    {MB_FUNC_OTHER_REPORT_SLAVEID, eMBFuncReportSlaveID},
#endif
#if MB_FUNC_READ_INPUT_ENABLED > 0
    {MB_FUNC_READ_INPUT_REGISTER, eMBFuncReadInputRegister},
#endif
#if MB_FUNC_READ_HOLDING_ENABLED > 0
    {MB_FUNC_READ_HOLDING_REGISTER, eMBFuncReadHoldingRegister},
#endif
#if MB_FUNC_WRITE_MULTIPLE_HOLDING_ENABLED > 0
    {MB_FUNC_WRITE_MULTIPLE_REGISTERS, eMBFuncWriteMultipleHoldingRegister},
#endif
#if MB_FUNC_WRITE_HOLDING_ENABLED > 0
    {MB_FUNC_WRITE_REGISTER, eMBFuncWriteHoldingRegister},
#endif
#if MB_FUNC_READWRITE_HOLDING_ENABLED > 0
    {MB_FUNC_READWRITE_MULTIPLE_REGISTERS, eMBFuncReadWriteMultipleHoldingRegister},
#endif
#if MB_FUNC_READ_COILS_ENABLED > 0
    {MB_FUNC_READ_COILS, eMBFuncReadCoils},
#endif
#if MB_FUNC_WRITE_COIL_ENABLED > 0
    {MB_FUNC_WRITE_SINGLE_COIL, eMBFuncWriteCoil},
#endif
#if MB_FUNC_WRITE_MULTIPLE_COILS_ENABLED > 0
    {MB_FUNC_WRITE_MULTIPLE_COILS, eMBFuncWriteMultipleCoils},
#endif
#if MB_FUNC_READ_DISCRETE_INPUTS_ENABLED > 0
    {MB_FUNC_READ_DISCRETE_INPUTS, eMBFuncReadDiscreteInputs},
#endif
};

/* ----------------------- Start implementation -----------------------------*/
eMBErrorCode
eMBInit(modbus_t * dev) {
    eMBErrorCode    eStatus = MB_ENOERR;

    if (dev->eMBCurrentMode != MB_TCP) {
        if ((dev->ucSlaveAddress == MB_ADDRESS_BROADCAST) || (dev->ucSlaveAddress < MB_ADDRESS_MIN) ||
            (dev->ucSlaveAddress > MB_ADDRESS_MAX)) {
            eStatus = MB_EINVAL;
        } else {
            ucMBAddress = dev->ucSlaveAddress;

            switch (dev->eMBCurrentMode) {
#if MB_RTU_ENABLED > 0
            case MB_RTU:
                pxMBFrameCBByteReceived     = xMBRTUReceiveFSM;
                pxMBFrameCBTransmitterEmpty = xMBRTUTransmitFSM;
                pxMBPortCBTimerExpired      = xMBRTUTimerT35Expired;
                eStatus = eMBRTUInit(ucMBAddress, dev->ucPort, dev->ulBaudRate, dev->eParity);
                break;
#endif
#if MB_ASCII_ENABLED > 0
            case MB_ASCII:
                pxMBFrameCBByteReceived     = xMBASCIIReceiveFSM;
                pxMBFrameCBTransmitterEmpty = xMBASCIITransmitFSM;
                pxMBPortCBTimerExpired      = xMBASCIITimerT1SExpired;
                eStatus = eMBASCIIInit(ucMBAddress, dev->ucPort, dev->ulBaudRate, dev->eParity);
                break;
#endif
            default:
                eStatus = MB_EINVAL;
            }

            if (eStatus == MB_ENOERR) {
                if (!xMBPortEventInit()) {
                    /* port dependent event module initalization failed. */
                    eStatus = MB_EPORTERR;
                } else {
                    dev->mbStatus = STATE_DISABLED;
                }
            }
        }
    }
    else if (dev->eMBCurrentMode == MB_TCP) {
#if MB_TCP_ENABLED > 0
        if ((eStatus = eMBTCPDoInit(dev->ucTCPPort)) != MB_ENOERR) {
            dev->mbStatus = STATE_DISABLED;
        } else if (!xMBPortEventInit()) {
            /* Port dependent event module initalization failed. */
            eStatus = MB_EPORTERR;
        } else {
            dev->mbStatus = STATE_DISABLED;
            ucMBAddress  = MB_TCP_PSEUDO_ADDRESS;
        }
#endif
    } else
        eStatus = MB_EINVAL;
    return eStatus;
}

eMBErrorCode
eMBRegisterCB( UCHAR ucFunctionCode, pxMBFunctionHandler pxHandler )
{
    int             i;
    eMBErrorCode    eStatus;

    if( ( 0 < ucFunctionCode ) && ( ucFunctionCode <= 127 ) )
    {
        ENTER_CRITICAL_SECTION(  );
        if( pxHandler != NULL )
        {
            for( i = 0; i < MB_FUNC_HANDLERS_MAX; i++ )
            {
                if( ( xFuncHandlers[i].pxHandler == NULL ) ||
                    ( xFuncHandlers[i].pxHandler == pxHandler ) )
                {
                    xFuncHandlers[i].ucFunctionCode = ucFunctionCode;
                    xFuncHandlers[i].pxHandler = pxHandler;
                    break;
                }
            }
            eStatus = ( i != MB_FUNC_HANDLERS_MAX ) ? MB_ENOERR : MB_ENORES;
        }
        else
        {
            for( i = 0; i < MB_FUNC_HANDLERS_MAX; i++ )
            {
                if( xFuncHandlers[i].ucFunctionCode == ucFunctionCode )
                {
                    xFuncHandlers[i].ucFunctionCode = 0;
                    xFuncHandlers[i].pxHandler = NULL;
                    break;
                }
            }
            /* Remove can't fail. */
            eStatus = MB_ENOERR;
        }
        EXIT_CRITICAL_SECTION(  );
    }
    else
    {
        eStatus = MB_EINVAL;
    }
    return eStatus;
}

eMBErrorCode
eMBClose( modbus_t * dev  )
{
    eMBErrorCode    eStatus = MB_ENOERR;

    if (dev->mbStatus == STATE_DISABLED)
    {
        if( dev->pvMBFrameCloseCur != NULL )
        {
            dev->pvMBFrameCloseCur();
        }
    }
    else
    {
        eStatus = MB_EILLSTATE;
    }
    return eStatus;
}

eMBErrorCode
eMBEnable( modbus_t * dev )
{
    eMBErrorCode    eStatus = MB_ENOERR;

    if( dev->mbStatus == STATE_DISABLED )
    {
        /* Activate the protocol stack. */
        dev->pvMBFrameStartCur();
        dev->mbStatus = STATE_ENABLED;
    }
    else
    {
        eStatus = MB_EILLSTATE;
    }
    return eStatus;
}

eMBErrorCode
eMBDisable( modbus_t * dev )
{
    eMBErrorCode    eStatus;

    if( dev->mbStatus == STATE_ENABLED )
    {
        dev->pvMBFrameStopCur();
        dev->mbStatus = STATE_DISABLED;
        eStatus = MB_ENOERR;
    }
    else if( dev->mbStatus == STATE_DISABLED )
    {
        eStatus = MB_ENOERR;
    }
    else
    {
        eStatus = MB_EILLSTATE;
    }
    return eStatus;
}

eMBErrorCode
eMBPoll(modbus_t rtu, modbus_t tcp, ...) {
    static UCHAR   *ucMBFrame;
    static UCHAR    ucRcvAddress;
    static UCHAR    ucFunctionCode;
    static USHORT   usLength;
    static eMBException eException;

#if MB_ASCII_ENABLED > 0
    va_list asii_param;
    va_start(asii_param, tcp);
#endif

    int             i;
    eMBErrorCode    eStatus = MB_ENOERR;
    eMBEventType    eEvent;
    eMBMode         eMBType; //remember the modbus type used

    /* Check if there is a event available. If not return control to caller.
     * Otherwise we will handle the event. */
    if (xMBPortEventGet(&eEvent, &eMBType) == TRUE) {
        switch (eEvent) {
        case EV_READY:
            break;

        case EV_FRAME_RECEIVED:
            switch (eMBType) {
#if MB_RTU_ENABLED > 0
            case MB_RTU:
                /* Check if the protocol stack is ready. */
                if (rtu.mbStatus != STATE_ENABLED) {
                    return MB_EILLSTATE;
                }
                eStatus = rtu.peMBFrameReceiveCur(&ucRcvAddress, &ucMBFrame, &usLength);
                if (eStatus == MB_ENOERR) {
                    /* Check if the frame is for us. If not ignore the frame. */
                    if ((ucRcvAddress == ucMBAddress) || (ucRcvAddress == MB_ADDRESS_BROADCAST)) {
                        (void)xMBPortEventPost(EV_EXECUTE, MB_RTU);
                    }
                }
                break;
#endif
#if MB_TCP_ENABLED > 0
            case MB_TCP:
                /* Check if the protocol stack is ready. */
                if (tcp.mbStatus != STATE_ENABLED) {
                    return MB_EILLSTATE;
                }
                eStatus = tcp.peMBFrameReceiveCur(&ucRcvAddress, &ucMBFrame, &usLength);
                if (eStatus == MB_ENOERR) {
                    /* Check if the frame is for us. If not ignore the frame. */
                    if ((ucRcvAddress == ucMBAddress) || (ucRcvAddress == MB_ADDRESS_BROADCAST)) {
                        (void)xMBPortEventPost(EV_EXECUTE, MB_TCP);
                    }
                }
                break;
#endif
#if MB_ASCII_ENABLED > 0
            case MB_ASCII:
                modbus_t ascii;
                ascii = va_arg(asii_param, modbus_t);
                /* Check if the protocol stack is ready. */
                if (ascii.mbStatus != STATE_ENABLED) {
                    return MB_EILLSTATE;
                }
                eStatus = ascii.peMBFrameReceiveCur(&ucRcvAddress, &ucMBFrame, &usLength);
                if (eStatus == MB_ENOERR) {
                    /* Check if the frame is for us. If not ignore the frame. */
                    if ((ucRcvAddress == ucMBAddress) || (ucRcvAddress == MB_ADDRESS_BROADCAST)) {
                        (void)xMBPortEventPost(EV_EXECUTE, MB_ASCII);
                    }
                }
                va_end(asii_param);
                break;
#endif
            }
            break;
        case EV_EXECUTE:
            ucFunctionCode = ucMBFrame[MB_PDU_FUNC_OFF];
            eException = MB_EX_ILLEGAL_FUNCTION;
            for( i = 0; i < MB_FUNC_HANDLERS_MAX; i++ )
            {
                /* No more function handlers registered. Abort. */
                if (xFuncHandlers[i].ucFunctionCode == 0) {
                    break;
                } else if (xFuncHandlers[i].ucFunctionCode == ucFunctionCode) {
                    eException = xFuncHandlers[i].pxHandler( ucMBFrame, &usLength );
                    break;
                }
            }

            /* If the request was not sent to the broadcast address we
             * return a reply. */

            switch (eMBType) {
#if MB_RTU_ENABLED > 0
            case MB_RTU:
                if (ucRcvAddress != MB_ADDRESS_BROADCAST) {
                    if (eException != MB_EX_NONE) {
                        /* An exception occured. Build an error frame. */
                        usLength = 0;
                        ucMBFrame[usLength++] = (UCHAR)(ucFunctionCode | MB_FUNC_ERROR);
                        ucMBFrame[usLength++] = eException;
                    }
                    eStatus = rtu.peMBFrameSendCur(ucMBAddress, ucMBFrame, usLength);
                }
                break;
#endif
#if MB_TCP_ENABLED > 0
            case MB_TCP:
                if (ucRcvAddress != MB_ADDRESS_BROADCAST) {
                    if (eException != MB_EX_NONE) {
                        /* An exception occured. Build an error frame. */
                        usLength = 0;
                        ucMBFrame[usLength++] = (UCHAR)(ucFunctionCode | MB_FUNC_ERROR);
                        ucMBFrame[usLength++] = eException;
                    }
                    eStatus = tcp.peMBFrameSendCur(ucMBAddress, ucMBFrame, usLength);
                }
                break;
#endif
#if MB_ASCII_ENABLED > 0
            case MB_ASCII:
                if (ucRcvAddress != MB_ADDRESS_BROADCAST) {
                    if (eException != MB_EX_NONE) {
                        /* An exception occured. Build an error frame. */
                        usLength = 0;
                        ucMBFrame[usLength++] = (UCHAR)(ucFunctionCode | MB_FUNC_ERROR);
                        ucMBFrame[usLength++] = eException;
                    }
                    if ((ascii.eMBCurrentMode == MB_ASCII) && MB_ASCII_TIMEOUT_WAIT_BEFORE_SEND_MS) {
                        vMBPortTimersDelay(MB_ASCII_TIMEOUT_WAIT_BEFORE_SEND_MS);
                    }
                    eStatus = ascii.peMBFrameSendCur(ucMBAddress, ucMBFrame, usLength);
                }
                break;
#endif
            }
        case EV_FRAME_SENT:
            break;
        }
    }
    return MB_ENOERR;
}
