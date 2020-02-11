#ifndef __usart_H
#define __usart_H
#include "stm32f10x.h"
#include <stdio.h>
void usart_init(void);

int fputc(int ch,FILE *f);

void usartinit(u32 rate,u8 sub,u8 pri);
void code_end(void);
#endif
