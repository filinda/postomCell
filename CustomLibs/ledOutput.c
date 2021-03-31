/*
 * ledOutput.c
 *
 *  Created on: Mar 8, 2021
 *      Author: Dmitriy Filin
 */
#include "ledOutput.h"

void delay(int k){
	for(int i=0;i<k;i++)for(int j=0;j<k;j++);
}

void shortBlinkRed(void){
	TIM1->CCR2 = 0;
	delay(100);
	TIM1->CCR2 = 2000;
	delay(100);
	TIM1->CCR2 = 0;
}

void shortBlinkGreen(void){
	TIM1->CCR1 = 0;
	delay(100);
	TIM1->CCR1 = 2000;
	delay(100);
	TIM1->CCR1 = 0;
}
void shortBlinkBlue(void){
	TIM1->CCR3 = 0;
	delay(100);
	TIM1->CCR3 = 2000;
	delay(100);
	TIM1->CCR3 = 0;
}
void shortBlinkYellow(void){
	TIM1->CCR1 = 0;
	TIM1->CCR2 = 0;
	delay(100);
	TIM1->CCR1 = 2000;
	TIM1->CCR2 = 3000;
	delay(100);
	TIM1->CCR1 = 0;
	TIM1->CCR2 = 0;
}
void shortBlinkPurple(void){
	TIM1->CCR3 = 0;
	TIM1->CCR2 = 0;
	delay(100);
	TIM1->CCR3 = 2000;
	TIM1->CCR2 = 3000;
	delay(100);
	TIM1->CCR3 = 0;
	TIM1->CCR2 = 0;
}
void shortBlinkCayan(void){
	TIM1->CCR3 = 0;
	TIM1->CCR1 = 0;
	delay(100);
	TIM1->CCR3 = 2000;
	TIM1->CCR1 = 2000;
	delay(100);
	TIM1->CCR3 = 0;
	TIM1->CCR1 = 0;
}

void longBlinkRed(void){
	TIM1->CCR2 = 0;
	delay(400);
	TIM1->CCR2 = 2000;
	delay(400);
	TIM1->CCR2 = 0;
}

void longBlinkGreen(void){
	TIM1->CCR1 = 0;
	delay(400);
	TIM1->CCR1 = 2000;
	delay(400);
	TIM1->CCR1 = 0;
}
void longBlinkBlue(void){
	TIM1->CCR3 = 0;
	delay(400);
	TIM1->CCR3 = 2000;
	delay(400);
	TIM1->CCR3 = 0;
}
void longBlinkYellow(void){
	TIM1->CCR1 = 0;
	TIM1->CCR2 = 0;
	delay(400);
	TIM1->CCR1 = 2000;
	TIM1->CCR2 = 3000;
	delay(400);
	TIM1->CCR1 = 0;
	TIM1->CCR2 = 0;
}
void longBlinkPurple(void){
	TIM1->CCR3 = 0;
	TIM1->CCR2 = 0;
	delay(400);
	TIM1->CCR3 = 2000;
	TIM1->CCR2 = 3000;
	delay(400);
	TIM1->CCR3 = 0;
	TIM1->CCR2 = 0;
}
void longBlinkCayan(void){
	TIM1->CCR3 = 0;
	TIM1->CCR1 = 0;
	delay(400);
	TIM1->CCR3 = 2000;
	TIM1->CCR1 = 2000;
	delay(400);
	TIM1->CCR3 = 0;
	TIM1->CCR1 = 0;
}

void customBlinkRGB(uint8_t r, uint8_t g, uint8_t b, int tim){
	TIM1->CCR3 = 0;
	TIM1->CCR2 = 0;
	TIM1->CCR1 = 0;
	delay(tim);
	TIM1->CCR3 = 2000*b/255;
	TIM1->CCR2 = 3000*r/255;
	TIM1->CCR1 = 2000*g/255;
	delay(tim);
	TIM1->CCR3 = 0;
	TIM1->CCR2 = 0;
	TIM1->CCR1 = 0;
}

