/*
 * button.h
 *
 *  Created on: Mar 20, 2018
 *      Author: Newhb
 */

#ifndef BTN_H_
#define BTN_H_

// BUTTON
#define BTN_PINS GPIO_PIN_0 | GPIO_PIN_4

#define OFF 0
#define RED GPIO_PIN_1
#define BLU GPIO_PIN_2
#define GRN GPIO_PIN_3

#define LED_PINS RED | BLU | GRN

// Period = (80 MHz) / (Desired Frequency Hz) - 1
#define DEBOUNCE_PERIOD (800000 - 1)

#define BUTTON_1 16
#define BUTTON_2 1

void InitButton(void (*onButtonState)(_Bool, _Bool));
void Button_Handler(void);
void LEDWrite(uint32_t ledState);
void Debounce_Handler(void);


#endif /* BTN_H_ */
