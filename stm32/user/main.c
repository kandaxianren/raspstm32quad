
#include "includes.h"

int main()
{	
  u8 rcdata[20];
	u8 data[20], i;
		u16 thro;
		vs8 yaw;
		vs16 pitch,roll;
		extern __IO u16 ADC1result[4];
		extern vu8 upload_flag;
		extern s8 xoffx,yoffx;
		extern vs8 xoffz,yoffz,xoffdeal,yoffdeal;

	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //…Ë÷√NVIC÷–∂œ∑÷◊È2:2Œª«¿’º”≈œ»º∂£¨2ŒªœÏ”¶”≈œ»º∂
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO|
	RCC_APB2Periph_SPI1|RCC_APB2Periph_ADC1|
	RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB,ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	
  delay_init();	    	 //—” ±∫Ø ˝≥ı ºªØ
  TIM4_Init(9,7199);//Tout£®“Á≥ˆ ±º‰£©=£®ARR+1)(PSC+1)/Tclk =10*7200/72000000s=1ms
  ADxinit();DMAinit();
	usartinit(115200,2,2);
  delay_us(500);
  NRF24L01_Init();    		//≥ı ºªØNRF24L01 

	while(NRF24L01_Check())
	{
		printf("NRF24L01 Error\r\n");
 		delay_ms(1000);
	}
       //printf("NRF24L01 OK\r\n");
   delay_ms(2000);
  NRF24L01_RX_Mode();
   delay_ms(1000);delay_ms(1000);
		while(1)
		{	
		
			NRF24L01_RX_Mode(); 
			while(NRF24L01_RxPacket(rcdata)!=RX_OK);

			RX_data(rcdata);
			
			
			NRF24L01_TX_Mode();
			thro=(float)(ADC1result[1]-2048)/2048*thro_max;
			//thro=sqrt(thro*thro_max);
			if(ADC1result[0]<=2050)
			{
				yaw=((float)ADC1result[0]/2000*yaw_range_zuo)-yaw_range_zuo;
			}
			else if(ADC1result[0]>=2100)
			{
				ADC1result[0]-=2100;
				yaw=((float)ADC1result[0]/2100)*yaw_range;
			}
			else{yaw=0;}
			if(ADC1result[3]<=2050)
			{
				pitch=((float)ADC1result[3]/2050*pitch_range-pitch_range);
			}
			else if(ADC1result[3]>=2120)
			{
				ADC1result[3]-=2100;
				pitch=(float)ADC1result[3]/2120*pitch_range;
			} 
			else{pitch=0;}
			if(ADC1result[2]<=2000)
			{
				roll=roll_range-(float)ADC1result[2]/2000*roll_range;
			}
			else if(ADC1result[2]>=2100)
			{
				ADC1result[2]-=2100;
				roll=(float)ADC1result[2]/2100*-roll_range;
			}
			else{roll=0;}
			data[0]=fly_dat;//rccode;
			data[1]=thro>>8;data[2]=thro&0x00ff;
			data[3]=yaw;data[4]=pitch>>8;	data[5]=pitch&0x00ff;
			data[6]=roll>>8;data[7]=roll&0x00ff;
			data[8]=data[1]^data[2]^data[3]^data[4]^data[5]^data[6]^data[7];
			if(upload_flag==1)
			{
				upload_flag=0;data[9]=upcode;
				data[10]=xoffz;data[11]=xoffx;data[12]=xoffdeal;
				data[13]=yoffz;data[14]=yoffx;data[15]=yoffdeal;
				xoffdeal=0;yoffdeal=0;
			}
			i=0;
			while(NRF24L01_TxPacket(data)!=TX_OK);
			
				
			


    }




}

/*
{
if(i>16) break;
}
*/











/*
void  TX_data(void)
{
		
		thro=(float)ADC1result[1]/4090*thro_max;
		thro=sqrt(thro*thro_max);
	
	 if(ADC1result[0]<=2050)
	   {
			 

			 yaw=((float)ADC1result[0]/2000*yaw_range_zuo)-yaw_range_zuo;
			 
	   }
	 else if(ADC1result[0]>=2100)
	 {
		 ADC1result[0]-=2100;
	   yaw=((float)ADC1result[0]/2100)*yaw_range;

	 }
	 else{yaw=0;}
		if(ADC1result[3]<=2050)
	   {
			 pitch=((float)ADC1result[3]/2050*pitch_range-pitch_range);
	   }
	 else if(ADC1result[3]>=2120)
	 {
		 ADC1result[3]-=2100;
	   pitch=(float)ADC1result[3]/2120*pitch_range;
	 } 
	  else{pitch=0;}
	 if(ADC1result[2]<=2000)
	   {
		
			roll=roll_range-(float)ADC1result[2]/2000*roll_range;
	   }
	 else if(ADC1result[2]>=2100)
	 {
		 ADC1result[2]-=2100;
	   roll=(float)ADC1result[2]/2100*-roll_range;
	 }
    else{roll=0;}
		data[0]=fly_dat;//rccode;
		data[1]=thro>>8;data[2]=thro&0x00ff;
		data[3]=yaw;data[4]=pitch>>8;	data[5]=pitch&0x00ff;
		data[6]=roll>>8;data[7]=roll&0x00ff;
		data[8]=data[1]^data[2]^data[3]^data[4]^data[5]^data[6]^data[7];
		if(upload_flag==1)
		{
		upload_flag=0;data[9]=upcode;
			data[10]=xoffz;data[11]=xoffx;data[12]=xoffdeal;
			data[13]=yoffz;data[14]=yoffx;data[15]=yoffdeal;
			xoffdeal=0;yoffdeal=0;
		}
		  //WirelessTx_handler(0,data,PACKET_LENGTH);
		
		
}

*/

#pragma import(__use_no_semihosting)  
_sys_exit(int x)  
{  
x = x;  
}  
struct __FILE  
{  
int handle;  
/* Whatever you require here. If the only file you are using is */  
/* standard output using printf() for debugging, no file handling */  
/* is required. */  
};  
/* FILE is typedefí d in stdio.h. */  
FILE __stdout; 




