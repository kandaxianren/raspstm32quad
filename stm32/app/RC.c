#include "24l01.h"
#include "spi.h"
#include "RC.h"
#include "usart.h"
#define fly_dat 0xab
u8 rcdata[20];
vs16 xoff,yoff;
vs8 xoffdeal=0,yoffdeal=0;
s8 xoffx,yoffx;
s8 xoffz,yoffz;
void RX_data(u8 *rcdata)
{
	u8 m1,m2,m3,m4,pitchx,rollx;//PITCH 等-0.x无法正常显示带修复
	s8 pitchz,rollz,az_v_h;
	u8 dis_taget,distance,dis_thro,az_v_l;
//WirelessRx_handler(rcdata,PACKET_LENGTH);
//u8 rcdata[20]={fly_dat,1,2,3,3,4,2,17,7,2,2,4,5,6,8,2,1,44};
	//while(NRF24L01_RxPacket(rcdata)!=RX_OK){}
if(rcdata[0]==fly_dat)
 {
   m1=rcdata[1]; m2=rcdata[2]; m3=rcdata[3]; m4=rcdata[4];
	 
	 xoffz=rcdata[5]; xoffx=rcdata[6]; yoffz=rcdata[7]; yoffx=rcdata[8];
	 
	 pitchz=rcdata[9]; pitchx=rcdata[10];rollz=rcdata[11];rollx=rcdata[12];
	 if(xoffz<0||xoffx<0){	 xoff=((xoffz*10+xoffx)-xoffdeal);}
	 else{ xoff=xoffz*10+xoffx+xoffdeal;}
	 if(yoffz<0||yoffx<0){	 yoff=((yoffz*10+yoffx)-yoffdeal);}
	 else{ yoff=yoffz*10+yoffx+yoffdeal;}
  
	 dis_taget=rcdata[13];   distance=rcdata[14];
	 dis_thro=rcdata[15];
	 az_v_h=rcdata[16]; az_v_l=rcdata[17];
	 printf("j1.val=%u",m1);code_end();	 printf("j2.val=%u",m2);code_end();	 
	 printf("j3.val=%u",m3);code_end();	 printf("j4.val=%u",m4);code_end();
	 
	  printf("t0.txt=\"%d\"",yoff);code_end();printf("t1.txt=\"%d\"",xoff);code_end();
	 
	  printf("t2.txt=\"%d.%u\"",pitchz,pitchx);code_end();printf("t3.txt=\"%d.%u\"",rollz,rollx);code_end();
	 
     printf("t12.txt=\"%d\"",dis_taget);code_end();printf("t13.txt=\"%d\"",distance);code_end();
	 printf("t15.txt=\"%u\"",dis_thro);code_end();
	  printf("t17.txt=\"%d.%u\"",az_v_h,az_v_l);code_end();
	 
 }	

}





