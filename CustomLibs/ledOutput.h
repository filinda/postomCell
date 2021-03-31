/*
 * ledOutput.h
 *
 *  Created on: Mar 8, 2021
 *      Author: Dmitriy
 */

#ifndef INC_LEDOUTPUT_H_
#define INC_LEDOUTPUT_H_

#ifdef __cplusplus
 extern "C" {
#endif
#include "stm32f1xx_hal.h"

void shortBlinkRed(void);
void shortBlinkGreen(void);
void shortBlinkBlue(void);
void shortBlinkYellow(void);
void shortBlinkPurple(void);
void shortBlinkCayan(void);

void longBlinkRed(void);
void longBlinkGreen(void);
void longBlinkBlue(void);
void longBlinkYellow(void);
void longBlinkPurple(void);
void longBlinkCayan(void);

void customBlinkRGB(uint8_t r, uint8_t g, uint8_t b, int tim);

#ifdef __cplusplus
}
#endif


#endif /* INC_LEDOUTPUT_H_ */
