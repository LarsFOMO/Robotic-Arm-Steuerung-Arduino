#include <avr/io.h>



void init_usart(void)

{
    UBRR0L = 103;                                //(16000000ul/(16ul*9600ul))-1;       //Baudrate 9600
    UBRR0H = 0;
    UCSR0B |= (1<<TXEN0) | (1<<RXEN0) | (1<<RXCIE0);           // Sender und Empfänger aktivieren
    UCSR0C |= (1<<UCSZ01) | (1<<UCSZ00);         // 8-Bit Übertragung

                                                // Asynchrone Übertragung (UMSEL00=0 und UMSEL01=0);
}


void _putch(unsigned char ch)
{
    while(!(UCSR0A & (1<<UDRE0)));
    UDR0 = ch;
}

void _puts(char st[])

{
    uint8_t i=0;
    while(st[i])
        _putch(st[i++]);

    _putch(13);   // CR
    _putch(10);   // NL/LF

}

unsigned char _getch(void)
{
    while(!(UCSR0A & (1<<RXC0)));
    return UDR0;
}

void _gets(unsigned char* st)
{
    do
    {
        *st=_getch();
    } while (*st++!=10);
    *st=0;
}
