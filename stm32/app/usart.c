#include "usart.h"
#include <stdio.h>
u8  uread[7],unum=0;
extern vs8 xoffdeal,yoffdeal;
vu8 upload_flag=0;

void usartinit(u32 rate,u8 sub,u8 pri)
{  
	GPIO_InitTypeDef GPIO_usart;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA,ENABLE);
	
	
	

  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 

  NVIC_InitStructure.NVIC_IRQChannel =USART1_IRQn; 

  NVIC_InitStructure.NVIC_IRQChannelSubPriority = sub; 
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =pri;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 

  NVIC_Init(&NVIC_InitStructure); 

	
  GPIO_usart.GPIO_Pin=GPIO_Pin_9;
	GPIO_usart.GPIO_Speed=GPIO_Speed_10MHz; 
	GPIO_usart.GPIO_Mode=GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA,&GPIO_usart);
	
	
	GPIO_usart.GPIO_Pin = GPIO_Pin_10;
  GPIO_usart.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_usart);
	
	USART_InitStructure.USART_BaudRate =rate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl =USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_Init(USART1, &USART_InitStructure);
	USART_Cmd(USART1, ENABLE);
  USART_ITConfig(USART1, USART_IT_RXNE,ENABLE);
	
}

void USART1_IRQHandler() 
{ 	 
    if(USART_GetFlagStatus(USART1,USART_IT_RXNE)==1) 
   {               
     uread[unum] = USART_ReceiveData(USART1);
     if(unum == 0&&uread[0] !=0x65) {unum=0;USART_ClearITPendingBit(USART1, USART_IT_RXNE);return;} 
		   unum++;
		 if(unum>=7)
		 {
		   if(uread[1]==0x01)
			 {
			   switch (uread[2])
				 {
					 case 0x0c: yoffdeal-=1;
					            break;
					 case 0x0e: yoffdeal+=1;
					            break; 
					 case 0x0d: xoffdeal-=1;
					            break;
					 case 0x0f: xoffdeal+=1;
					            break;
					 default:break;
				 }	
   if(xoffdeal>125||xoffdeal<-125)xoffdeal=0;
   if(yoffdeal>125||yoffdeal<-125)yoffdeal=0;				 
			 }
			 else if(uread[1]==0x02)
			 {
			   switch (uread[2])
				 {
					 case 0x02: upload_flag=1;
					            break;
					 default:break;
				 }	 
			 }
			  unum=0;	 
		 }			
   }

}    


int fputc(int ch,FILE *f)
{
  USART_SendData(USART1,(unsigned char)ch);
	while(USART_GetFlagStatus(USART1, USART_FLAG_TXE)!=SET);
	return ch;
}
void code_end()
{
USART_SendData(USART1,0xff);
	while(USART_GetFlagStatus(USART1, USART_FLAG_TXE)!=SET);
	USART_SendData(USART1,0xff);
	while(USART_GetFlagStatus(USART1, USART_FLAG_TXE)!=SET);
	USART_SendData(USART1,0xff);
	while(USART_GetFlagStatus(USART1, USART_FLAG_TXE)!=SET);
}
