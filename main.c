///////////////////////////////////////////////
//            ROBOTIC ARM CONTROLL           //
//                 06.01.2021                //
//                 LARS KAGER                //
///////////////////////////////////////////////

#include <avr/io.h>
#include <stdlib.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>
#include "../UART.h"
#include "../UART.c"

int main(void)
{
    DDRD = 0x60;
    DDRC = 0x00;

    // JOYSTICK-VAR
    uint8_t xA1JS;                              // 127
    uint8_t yA1JS;                              // 124

    // CALCULATIONS-POS
    float posX = 16;                            // min: 9 - max: 22
    float posY = 16;                            // min: 4 - max: 23
    float hypo = 0;
    float alpha;
    float beta;                                 // 2. Winkel
    float alphaTemp;
    float alphaTotal;                           // 1. Winkel
    float betaGrad;
    float alphaGrad;
    uint8_t alphaPWM;
    uint8_t betaPWM;
    uint8_t speed = 0;
//    uint16_t i = 0;
//    uint8_t enable;
//    char str[10] = "";

    // PWM
    TCCR0A = 0xA3;
    TCCR0B = 0x05;
    // TIMER FOR DELAY
//    TCCR2B |= 0x05;

    init_usart();

    while(1)
    {
    // PD0 READ
        ADMUX = 0x60;                           // AVcc als Referenzspannung; Wert linksbündig; Kanal ADC0
        ADCSRA |= 0x07;                         // Teiler 128: 16MHz/128=125kHz
        ADCSRA |= (1<<ADEN);                    // ADC-Modul aktivieren
                                                // alternativ ADCSRA = 0b10000111
        ADCSRA |= (1<<ADSC);                    // Wandlung starten
        while (ADCSRA & (1<<ADSC));             // warten auf Ende der Wandlung
        xA1JS = ADCH;

//        itoa(betaPWM,str,10);
//        _puts(str);
//        _delay_ms(100);

    // PD1 READ
        ADMUX = 0x61;                           // Kanal ADC1
        ADCSRA |= 0x07;
        ADCSRA |= (1<<ADEN);

        ADCSRA |= (1<<ADSC);
        while (ADCSRA & (1<<ADSC));
        yA1JS = ADCH;

    // EVALUATION AND TRANSFORMATION IN MOTION SPEED
        if(xA1JS > 130)
        {
           xA1JS -= 127;
           speed = ((((xA1JS/10)*(-1))+12)*5)+15;

           if(posX >= 22)
                posX = 22;
           else
                posX += 0.3;
        }
        if(xA1JS < 120)
        {
           xA1JS = 127-xA1JS;
           speed = ((((xA1JS/10)*(-1))+12)*5)+15;

           if(posX <= 9)
                posX = 9;
            else
                posX -= 0.3;
        }
        if(yA1JS < 120)
        {
           yA1JS -= 127;
           speed = ((((yA1JS/10)*(-1))+12)*5)+15;

           if(posY >= 23)
                posY = 23;
           else
                posY += 0.3;
        }
        if(yA1JS > 130)
        {
           yA1JS = 127-yA1JS;
           speed = ((((yA1JS/10)*(-1))+12)*5)+15;

           if(posY <= 4)
                posY = 4;
            else
                posY -= 0.3;
        }
        _delay_ms(speed);

    // TIMER TCCR2A u TCCR2B & EVALUATION AND TRANSFORMATION IN MOTION SPEED
//        if(TCNT2 >= 160)  //Zählerstand überprüfen
//        {
//            TCNT2 = 0;
//            i++;
//        }
//        if(i >= speed)     //Es soll 100x156 Mal raufgezählt werden => 16.000.000/1024 = 16500
//        {
//            i = 0;
//        }

    // CALCULATIONS - INVERTED KINEMATICS
        hypo = pow(posX,2) + pow(posY,2);
        hypo = sqrt(hypo);
        alpha = acos(hypo/32);                  // in RAD
        beta = (atan(1)*4) - (2*alpha);         // atan(1)*4 == PI
        alphaTemp = atan(posY/posX);
        alphaTotal = alphaTemp + alpha;

    // RAD TO DRG
        betaGrad = (beta * 180)/(atan(1)*4);
        alphaGrad = (alphaTotal *180)/(atan(1)*4);

    // DRG TO PULSE WIDTH FOR SERVO MOTORS
        alphaGrad = (27*alphaGrad)/90;
        alphaPWM = alphaGrad;
        betaGrad = (((180-betaGrad)*(1.f/6))+8);
        betaPWM = betaGrad;

    // AUSGABE
        OCR0A = alphaPWM;                       // unterstes Glied: 113: 34 - 90: 27 - 40: 12
        OCR0B = betaPWM;                        // mittleres Glied: 180: 7 - 90: 22 - 30: 32
    }

    return 0;
}
