/*
 * processProtocolSW.c
 *
 *  Created on: Mar 8, 2021
 *      Author: Dmitriy
 */

#include "processProtocolSW.h"

_Bool isAddrUniversal(uint8_t *pAddr1){
	return	(pAddr1[0]==0x00) &&
			(pAddr1[1]==0x00) &&
			(pAddr1[2]==0x00) &&
			(pAddr1[3]==0x00) &&
			(pAddr1[4]==0x00) &&
			(pAddr1[5]==0x00) &&
			(pAddr1[6]==0x00) &&
			(pAddr1[7]==0x00);
}
_Bool compareAddr(uint8_t *pAddr1, uint8_t *pAddr2){
	return	(pAddr1[0]==pAddr2[0]) &&
			(pAddr1[1]==pAddr2[1]) &&
			(pAddr1[2]==pAddr2[2]) &&
			(pAddr1[3]==pAddr2[3]) &&
			(pAddr1[4]==pAddr2[4]) &&
			(pAddr1[5]==pAddr2[5]) &&
			(pAddr1[6]==pAddr2[6]) &&
			(pAddr1[7]==pAddr2[7]);
}

