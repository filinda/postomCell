/*
 * processProtocolSW.h
 *
 *  Created on: Mar 8, 2021
 *      Author: Dmitriy
 */

#ifndef PROCESSPROTOCOLSW_H_
#define PROCESSPROTOCOLSW_H_


#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f1xx_hal.h"

_Bool isAddrUniversal(uint8_t *pAddr1);
_Bool compareAddr(uint8_t *pAddr1, uint8_t *pAddr2);

#ifdef __cplusplus
}
#endif


#endif /* PROCESSPROTOCOLSW_H_ */
