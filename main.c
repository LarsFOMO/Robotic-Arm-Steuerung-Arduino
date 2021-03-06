///////////////////////////////////////////////
//            ROBOTIC ARM CONTROLL           //
//                 06.02.2021                //
//                 LARS KAGER                //
///////////////////////////////////////////////

#include <avr/io.h>
#include <stdlib.h>
#include <stdio.h>
#include <avr/interrupt.h>
//#include <util/delay.h>
#include <math.h>
#include <string.h>
#include "../UART.h"
#include "../UART.c"
#include "../pwmServo.h"
#include "../pwmServo.c"

uint16_t counter = 0;//= 255;
uint8_t enable = 0;
uint8_t enablex = 0;
uint8_t enabley = 0;
uint8_t enableRotate = 0;
uint8_t enableg = 0;
uint8_t enablegOC = 0;
uint8_t i = 0;
uint8_t j = 0;
uint16_t r = 0;
uint16_t v = 0;
uint16_t m = 0;
uint8_t xvorzeichen = 0;
uint8_t yvorzeichen = 0;
uint8_t gvorzeichen = 0;
uint8_t gOCvorzeichen = 0;
uint8_t xdelay = 0;
uint8_t ydelay = 0;
uint16_t rotationdelay = 0;
uint16_t gdelay = 0;
uint16_t gOCdelay = 0;
int16_t winkelg = 90;
uint8_t open = 18;
float posX = 70;                            // min: 90 - max: 220
float posY = 100;
float hypo = 0;                           // min: 40 - max: 230


ISR(TIMER2_COMPB_vect)
{
    uint8_t sreg = SREG;
    cli();

    // CALCULATIONS FOR MOTION SPEED
    if(enable == 1)
    {
        i++;
        j++;
        r++;
        v++;
        m++;
    }
    if((i >= xdelay) && (enablex == 1))
    {
        if((xvorzeichen == 1) && (posX >= 1) && (hypo >= 80))//70))
            posX -= 1;
        else if((xvorzeichen == 0) && (hypo <= 319))
            posX += 1;
        i = 0;
    }
    if((j >= ydelay) && (enabley == 1))
    {
        if((yvorzeichen == 1) && (posY >= 1) && (hypo >= 80))
            posY -= 1;
        else if((yvorzeichen == 0) && (hypo <= 319))
            posY += 1;
        j = 0;
    }
    if((r >= rotationdelay) && (enableRotate == 1))
    {
//        if(counter >= 410)                          //
//            counter = 410;
//        else if(counter <= 10)
//            counter = 10;
//        else if((counter < 410) && (counter > 10))  //
        PORTD ^= (1<<PD4);
        r = 0;
    }
    if((v >= gdelay) && (enableg == 1))
    {
        if((gvorzeichen == 1) && (winkelg > 20))             // 0!!
            winkelg -= 1;
        else if((gvorzeichen == 0) && (winkelg < 150))
            winkelg += 1;
        v = 0;
    }
    if((m >= gOCdelay) && (enablegOC == 1))
    {
        if((gOCvorzeichen == 1) && (open > 10))             // 0!!
            open -= 1;
        else if((gOCvorzeichen == 0) && (open <= 22))
            open += 1;
        m = 0;
    }

    SREG = sreg;
}

int main(void)
{
    DDRD = 0x78;//0x60;                                // 0110 0000
    DDRC = 0x00;
    DDRB = 0x06;

    // JOYSTICK-VAR
    uint8_t xA1JS;                              // 124
    uint8_t yA1JS;                              // 127
    uint8_t rotation;
    uint8_t grapper;
    uint8_t grapperOPEN;

    // CALCULATIONS-POS
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
    uint8_t winkelAUS;
    float gyro;
    float alphaTempGrad;
    float alphaTGrad;
    uint16_t alpha01;
    uint16_t alpha02;
    counter = 205;//255;

    char str[] = "";

    // PWM
    TCCR0A = 0xA3;
    TCCR0B = 0x05;

    TCCR1A = 0xA2;
    TCCR1B = 0x1B;
	TCNT1H = 0x00;
	TCNT1L = 0x00;

    // TIMER-INTERRUPT FOR DELAY
    TCCR2B = 0x0D;                              //Teiler 1024 und WGM22 auf 1
    TIMSK2 |= (1<<OCIE2B);
    OCR2B = 250;        //165

    init_usart();
    sei();

    while(1)
    {
    // PD0 READ
        ADMUX = 0x60;                           // AVcc als Referenzspannung; Wert linksb?ndig; Kanal ADC0
        ADCSRA |= 0x07;                         // Teiler 128: 16MHz/128=125kHz
        ADCSRA |= (1<<ADEN);                    // ADC-Modul aktivieren
                                                // alternativ ADCSRA = 0b10000111
        ADCSRA |= (1<<ADSC);                    // Wandlung starten
        while (ADCSRA & (1<<ADSC));             // warten auf Ende der Wandlung
        yA1JS = ADCH;                           // Pos0 = 124

    // PD1 READ
        ADMUX = 0x61;                           // Kanal ADC1
        ADCSRA |= 0x07;
        ADCSRA |= (1<<ADEN);

        ADCSRA |= (1<<ADSC);
        while (ADCSRA & (1<<ADSC));
        xA1JS = ADCH;

    // PD3 READ
        ADMUX = 0x63;                           // Kanal ADC3
        ADCSRA |= 0x07;
        ADCSRA |= (1<<ADEN);

        ADCSRA |= (1<<ADSC);
        while (ADCSRA & (1<<ADSC));
        rotation = ADCH;

    // PD4 READ
        ADMUX = 0x64;                           // Kanal ADC4
        ADCSRA |= 0x07;
        ADCSRA |= (1<<ADEN);

        ADCSRA |= (1<<ADSC);
        while (ADCSRA & (1<<ADSC));
        grapper = ADCH;

    // PD5 READ
        ADMUX = 0x65;                           // Kanal ADC5
        ADCSRA |= 0x07;
        ADCSRA |= (1<<ADEN);

        ADCSRA |= (1<<ADSC);
        while (ADCSRA & (1<<ADSC));
        grapperOPEN = ADCH;

    // X-ACHSE
        if(xA1JS > 140)
        {
            xdelay = 255-xA1JS;
            xvorzeichen = 1;                    // Neg.
            enable = 1;
            enablex = 1;
        }
        else if(xA1JS < 110)
        {
            xdelay = xA1JS;
            xvorzeichen = 0;
            enable = 1;
            enablex = 1;
        }
        else if((xA1JS < 140) && (xA1JS > 110))
            enablex = 0;

    // Y-ACHSE
        if(yA1JS > 140)
        {
            ydelay = 255-yA1JS;
            yvorzeichen = 1;                    // Neg.
            enable = 1;
            enabley = 1;
        }
        else if(yA1JS < 110)
        {
            ydelay = yA1JS;
            yvorzeichen = 0;
            enable = 1;
            enabley = 1;
        }
        else if((yA1JS < 140) && (yA1JS > 110))
            enabley = 0;

    // ROTATION
        if(rotation > 140)
        {
            rotationdelay = 255-rotation;
            PORTD &= ~(1<<PD3);                    // Neg.
            enable = 1;
            enableRotate = 1;
            //counter++;
        }
        else if(rotation < 110)
        {
            rotationdelay = rotation;
            PORTD |= (1<<PD3);
            enable = 1;
            enableRotate = 1;
            //counter--;
        }
        else if((rotation < 140) && (rotation > 110))
            enableRotate = 0;

    // GRAPPER
        if(grapper > 140)
        {
            gdelay = 255-grapper;
            gvorzeichen = 1;
            enable = 1;
            enableg = 1;
        }
        else if(grapper < 110)
        {
            gdelay = grapper;
            gvorzeichen = 0;
            enable = 1;
            enableg = 1;
        }
        else if((grapper < 140) && (grapper > 110))
            enableg = 0;

    // GRAPPER OPEN/CLOSE
        if(grapperOPEN > 140)
        {
            gOCdelay = 255-grapperOPEN;
            gOCvorzeichen = 1;                    // Neg.
            enable = 1;
            enablegOC = 1;
        }
        else if(grapperOPEN < 110)
        {
            gOCdelay = grapperOPEN;
            if((grapperOPEN <= 25) && (grapperOPEN >= 15))
                gOCdelay = grapperOPEN-15;
            gOCvorzeichen = 0;
            enable = 1;
            enablegOC = 1;
        }
        else if((grapperOPEN < 140) && (grapperOPEN > 110))
            enablegOC = 0;

        else
            enable = 0;

    // CONTROLL SPEED
        xdelay = ((xdelay*2)/15)+3;
        ydelay = ((ydelay*2)/15)+3;     //30
        rotationdelay = ((rotationdelay*2)/31)+4;
        gdelay = ((gdelay*2)/15)+4;
        gOCdelay = (gOCdelay*5)+15;

    // CALCULATIONS - INVERTED KINEMATICS
        hypo = (posX*posX) + (posY*posY);
        hypo = sqrt(hypo);

        alpha = (acos(hypo/320));                // in RAD
        beta = M_PI - (2*alpha);          // atan(1)*4 == PI
        alphaTemp = atan(posY/posX);
        alphaTotal = alphaTemp + alpha;
        alphaTGrad = alpha;
        alphaTempGrad = alphaTemp;

    // RAD TO DRG
        alphaTGrad = (alphaTGrad * 180)/(M_PI);
        alpha01 = alphaTGrad;
        alphaTempGrad = (alphaTempGrad * 180)/(M_PI);
        alpha02 = alphaTempGrad;
        betaGrad = (beta * 180)/(M_PI) * 10;
        betaCalc = betaGrad;
        alphaGrad = (alphaTotal *180)/(M_PI) * 10;
        alphaCalc = alphaGrad;

    // DRG TO PULSE WIDTH FOR SERVO MOTORS
        alphaCalc = ((1850*alphaCalc)/900) + 1000;
        alphaPWM = alphaCalc;
        betaCalc = 5000-((betaCalc*2000)/900);
        betaPWM = betaCalc;

    // GRAPPER
        gyro = 360-winkelg-(90-alpha02)-alpha01;
        winkelAUS = ((gyro-90)/6)+10;

    // AUSGABE
        FrequencyPWM(50, betaPWM, 'A');          // mittleres Glied beta: 180: 1000 - 90: 3000 - -90: 4500
        FrequencyPWM(50, alphaPWM, 'B');         // unteres Glied alpha: 90+: 4000 - 90: 2850 - 0: 1000
        OCR0A = winkelAUS;
        OCR0B = open;
        //Greifer 10 offen / 23 zu
//        itoa(grapperOPEN,str,10);
//        _puts(str);
//        _delay_ms(10);
    }
    return 0;
}


//        itoa(posX,str,10);
//        _puts(str);
//        _delay_ms(10);
