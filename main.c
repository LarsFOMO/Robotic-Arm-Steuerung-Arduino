///////////////////////////////////////////////
//            ROBOTIC ARM CONTROLL           //
//                 06.02.2021                //
//                 LARS KAGER                //
///////////////////////////////////////////////

#include <avr/io.h>
#include <stdlib.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>
#include <string.h>
#include "../UART.h"
#include "../UART.c"
#include "../pwmServo.h"
#include "../pwmServo.c"


int main(void)
{
    DDRD = 0x60;                                // 0110 0000
    DDRC = 0x00;
    DDRB = 0x06;

    // JOYSTICK-VAR
    uint8_t xA1JS;                              // 124
    uint8_t yA1JS;                              // 127

    // CALCULATIONS-POS
    float posX = 70;                            // min: 90 - max: 220
    float posY = 100;                           // min: 40 - max: 230
    float hypo = 0;
    float alpha;
    float beta;                                 // 2. Winkel
    float alphaTemp;
    float alphaTotal;                           // 1. Winkel
    float betaGrad;
    float alphaGrad;
    uint16_t alphaPWM;
    uint16_t betaPWM;
    uint32_t alphaCalc;
    uint32_t betaCalc;
    int8_t xdelay = 0;
    uint8_t ydelay = 0;
    uint8_t i = 0;
    uint8_t j = 0;
    uint8_t xvorzeichen = 0;
    uint8_t yvorzeichen = 0;
    uint8_t enable = 0;
    uint8_t enablex = 0;
    uint8_t enabley = 0;

    char str[] = "";

    // PWM
//    TCCR0A = 0xA3;
//    TCCR0B = 0x05;
    TCCR1A = 0xA2;
    TCCR1B = 0x1B;
	TCNT1H = 0x00;
	TCNT1L = 0x00;

    // TIMER FOR DELAY
    TCCR2B = 0x07;

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
        xA1JS = ADCH;                           // Pos0 = 124

    // PD1 READ
        ADMUX = 0x61;                           // Kanal ADC1
        ADCSRA |= 0x07;
        ADCSRA |= (1<<ADEN);

        ADCSRA |= (1<<ADSC);
        while (ADCSRA & (1<<ADSC));
        yA1JS = ADCH;

    // X-ACHSE
        if(xA1JS > 130)
        {
            xdelay = 255-xA1JS;
            xvorzeichen = 0;
            enable = 1;
            enablex = 1;
        }
        else if(xA1JS < 120)
        {
            xdelay = xA1JS;
            xvorzeichen = 1;                     // Neg.
            enable = 1;
            enablex = 1;
        }
        else if((xA1JS < 130) && (xA1JS > 120))
            enablex = 0;

    // Y-ACHSE
        if(yA1JS > 130)
        {
            ydelay = 255-yA1JS;
            yvorzeichen = 1;                    // Neg.
            enable = 1;
            enabley = 1;
        }
        else if(yA1JS < 120)
        {
            ydelay = yA1JS;
            yvorzeichen = 0;
            enable = 1;
            enabley = 1;
        }
        else if((yA1JS < 130) && (yA1JS > 120))
            enabley = 0;

        else
            enable = 0;

    // CONTROLL SPEED
        xdelay = ((xdelay*2)/30)+1;
        ydelay = ((ydelay*2)/30)+1;

        if(enable == 1)
        {
            if(TCNT2 >= 60)                    //Zählerstand überprüfen
            {
                TCNT2 = 0;
                i++;
                j++;
            }
            if((i >= xdelay) && (enablex == 1))
            {
                if((xvorzeichen == 1) && (posX >= 10) && (hypo >= 70))
                    posX -= 1;
                else if((xvorzeichen == 0) && (hypo <= 319))
                    posX += 1;
                i = 0;
            }
            if((j >= ydelay) && (enabley == 1))
            {
                if((yvorzeichen == 1) && (posY >= 1) && (hypo >= 70))
                    posY -= 1;
                else if((yvorzeichen == 0) && (hypo <= 319))
                    posY += 1;
                j = 0;
            }
        }
//        itoa(ydelay,str,10);
//        _puts(str);
//      _delay_ms(10);

    // CALCULATIONS - INVERTED KINEMATICS
        hypo = (posX*posX) + (posY*posY);
        hypo = sqrt(hypo);

        alpha = (acos(hypo/320));                // in RAD
        beta = (atan(1)*4) - (2*alpha);          // atan(1)*4 == PI
        alphaTemp = atan(posY/posX);
        alphaTotal = alphaTemp + alpha;

    // RAD TO DRG
        betaGrad = (beta * 180)/(atan(1)*4) * 10;
        betaCalc = betaGrad;
        alphaGrad = (alphaTotal *180)/(atan(1)*4) * 10;
        alphaCalc = alphaGrad;

    // DRG TO PULSE WIDTH FOR SERVO MOTORS
        alphaCalc = ((1850*alphaCalc)/900) + 1000;
        alphaPWM = alphaCalc;
        betaCalc = 5000-((betaCalc*2000)/900);
        betaPWM = betaCalc;

    // AUSGABE

        FrequencyPWM(50, betaPWM, 'A');          // mittleres Glied beta: 180: 1000 - 90: 3000 - -90: 4500
        FrequencyPWM(50, alphaPWM, 'B');         // unteres Glied alpha: 90+: 4000 - 90: 2850 - 0: 1000
    }

    return 0;
}




    // PD1 READ
//        ADMUX = 0x61;                           // Kanal ADC1
//        ADCSRA |= 0x07;
//        ADCSRA |= (1<<ADEN);
//
//        ADCSRA |= (1<<ADSC);
//        while (ADCSRA & (1<<ADSC));
//        yA1JS = ADCH;
//
//     EVALUATION AND TRANSFORMATION IN MOTION SPEED
//        if(xA1JS > 130)
//        {
//           xA1JS -= 127;
//           speed = ((((xA1JS/10)*(-1))+12)*5)+15;
//
//           if(posX >= 220)
//                posX = 220;
//           else
//                posX += 1;
//        }
//        if(xA1JS < 120)
//        {
//           xA1JS = 127-xA1JS;
//           speed = ((((xA1JS/10)*(-1))+12)*5)+15;
//
//           if(posX <= 90)
//                posX = 90;
//            else
//                posX -= 1;
//        }
//        if(yA1JS < 120)
//        {
//           yA1JS -= 127;
//           speed = ((((yA1JS/10)*(-1))+12)*5)+15;
//
//           if(posY >= 230)
//                posY = 230;
//           else
//                posY += 1;
//        }
//        if(yA1JS > 130)
//        {
//           yA1JS = 127-yA1JS;
//           speed = ((((yA1JS/10)*(-1))+12)*5)+15;
//
//           if(posY <= 40)
//                posY = 40;
//            else
//                posY -= 1;
//        }
        //_delay_ms(speed);

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
