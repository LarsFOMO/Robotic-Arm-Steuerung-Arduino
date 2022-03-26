#ifndef PWMSERVO_H_INCLUDED
#define PWMSERVO_H_INCLUDED

#include <avr/io.h>
#include <stdlib.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <string.h>
#include <math.h>
#include <util/delay.h>

void PWM1_INIT(void);
void Duty(uint16_t percentagex100, uint16_t ICR1_value, char pinSelect);
void FrequencyPWM(uint16_t frequency, uint16_t percentagex100, char pinSelect);

uint16_t divider = 256;

#endif // PWMSERVO_H_INCLUDED
