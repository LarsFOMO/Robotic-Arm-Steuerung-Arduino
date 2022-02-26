#ifndef UART_H_INCLUDED
#define UART_H_INCLUDED

void init_usart(void);
void _putch(unsigned char ch);
void _puts(char st[]);
unsigned char _getch(void);
void _gets(unsigned char* st);

#endif // UART_H_INCLUDED
