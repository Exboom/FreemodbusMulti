#pragma once

typedef enum { 
    STATE_ENABLED, 
    STATE_DISABLED, 
    STATE_NOT_INITIALIZED 
} eMBState;

/*! \ingroup modbus
 * \brief Modbus serial transmission modes (RTU/ASCII).
 *
 * Modbus serial supports two transmission modes. Either ASCII or RTU. RTU
 * is faster but has more hardware requirements and requires a network with
 * a low jitter. ASCII is slower and more reliable on slower links (E.g. modems)
 */
typedef enum {
    MB_RTU,   /*!< RTU transmission mode. */
    MB_ASCII, /*!< ASCII transmission mode. */
    MB_TCP    /*!< TCP mode. */
} eMBMode;

/*! \ingroup modbus
 * \brief Errorcodes used by all function in the protocol stack.
 */
typedef enum {
    MB_ENOERR,    /*!< no error. */
    MB_ENOREG,    /*!< illegal register address. */
    MB_EINVAL,    /*!< illegal argument. */
    MB_EPORTERR,  /*!< porting layer error. */
    MB_ENORES,    /*!< insufficient resources. */
    MB_EIO,       /*!< I/O error. */
    MB_EILLSTATE, /*!< protocol stack in illegal state. */
    MB_ETIMEDOUT  /*!< timeout error occurred. */
} eMBErrorCode;