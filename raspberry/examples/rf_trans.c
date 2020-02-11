#include <string.h>
#include <stdint.h>
#include <errno.h>
#include <RF24/nrf24.h>
#include <RF24/bcm2835.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <nRF24L01.h>
#include <math.h>
#include <time.h>
#include <MotionSensor.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <linux/types.h>
#include <fcntl.h>

#define PCA9685_SUBADR1 0x2
#define PCA9685_SUBADR2 0x3
#define PCA9685_SUBADR3 0x4
#define PCA9685_MODE1 0x0
#define PCA9685_PRESCALE 0xFE
#define LED0_ON_L 0x6
#define LED0_ON_H 0x7
#define LED0_OFF_L 0x8
#define LED0_OFF_H 0x9
#define ALLLED_ON_L 0xFA
#define ALLLED_ON_H 0xFB
#define ALLLED_OFF_L 0xFC
#define ALLLED_OFF_H 0xFD

#define yaw_range 120
#define pitch_range 32
#define roll_range 32

#define MS5611_ADDRESS 0x77
#define CONV_D1_256   0x40
#define CONV_D1_512   0x42
#define CONV_D1_1024  0x44
#define CONV_D1_2048  0x46
#define CONV_D1_4096  0x48
#define CONV_D2_256   0x50
#define CONV_D2_512   0x52
#define CONV_D2_1024  0x54
#define CONV_D2_2048  0x56
#define CONV_D2_4096  0x58
#define CMD_ADC_READ  0x00
#define CMD_PROM_READ 0xA0
#define OSR_256      1000 //us
#define OSR_512      2000 //us
#define OSR_1024     3000 //us
#define OSR_2048     5000 //us
#define OSR_4096     10000 //us
#define alpha 0.96
#define beta 0.96
#define gamma 0.96
#define SEA_LEVEL_PRESSURE 1023.20


unsigned int PROM_read(int DA, char PROM_CMD)
{
	uint16_t ret = 0;
	uint8_t r8b[] = { 0, 0 };

	if (write(DA, &PROM_CMD, 1) != 1){
		printf("read set reg Failed to write to the i2c bus.\n");
	}

	if (read(DA, r8b, 2) != 2){
		printf("Failed to read from the i2c bus.\n");
	}

	ret = r8b[0] * 256 + r8b[1];

	return ret;
}

long CONV_read(int DA, char CONV_CMD)
{
	long ret = 0;
	uint8_t D[] = { 0, 0, 0 };

	int  h;
	char zero = 0x0;

	if (write(DA, &CONV_CMD, 1) != 1) {
		printf("write reg 8 bit Failed to write to the i2c bus.\n");
	}

	usleep(OSR_4096);

	if (write(DA, &zero, 1) != 1) {
		printf("write reset 8 bit Failed to write to the i2c bus.\n");
	}

	h = read(DA, &D, 3);

	if (h != 3) {
		printf("Failed to read from the i2c bus %d.\n", h);

	}

	ret = D[0] * (unsigned long)65536 + D[1] * (unsigned long)256 + D[2];

	return ret;
}
int altitudefd;
int openMs5611()
{
	if ((altitudefd = open("/dev/i2c-1", O_RDWR)) < 0){
		printf("Failed to open the bus.\n");
		return -1;
	}
	if (ioctl(altitudefd, I2C_SLAVE, MS5611_ADDRESS) < 0){
		printf("Failed to acquire bus access and/or talk to slave.\n");
		return -1;
	}
	return 0;
}
float Altitude(int fd)
{
	int i;
	//int fd;
	uint16_t C[7];
	uint32_t D1;
	uint32_t D2;
	char RESET = 0x1E;
	int64_t dT;
	int32_t TEMP;
	int64_t OFF;
	int64_t SENS;
	int32_t P;
	double Temparature, fltd_Temparature;
	double Pressure, fltd_Pressure;

	float Altitude, pre_Altitude;
	int roc, fltd_roc;

	long curSampled_time = 0;
	long prevSampled_time = 0;
	float Sampling_time, prevSampling_time;
	struct timespec spec;

	/*if ((fd = open("/dev/i2c-1", O_RDWR)) < 0){
		printf("Failed to open the bus.\n");
		return -1;
	}

	if (ioctl(fd, I2C_SLAVE, MS5611_ADDRESS) < 0){
		printf("Failed to acquire bus access and/or talk to slave.\n");
		return -1;
	}*/

	if (write(fd, &RESET, 1) != 1) {
		printf("write reg 8 bit Failed to write to the i2c bus.\n");
	}

	usleep(1000);

	for (i = 0; i < 7; i++){
		usleep(1000);

		C[i] = PROM_read(fd, CMD_PROM_READ + (i * 2));
		//printf("C[%d] = %d\n", i, C[i]);
	}


		clock_gettime(CLOCK_MONOTONIC, &spec);
		curSampled_time = round(spec.tv_nsec / 1.0e6);

		prevSampling_time = Sampling_time;
		Sampling_time = (float)curSampled_time - (float)prevSampled_time;

		if (Sampling_time < 0) // to prevent negative sampling time
			Sampling_time = prevSampling_time;

		D1 = CONV_read(fd, CONV_D1_4096);
		D2 = CONV_read(fd, CONV_D2_4096);

		dT = D2 - (uint32_t)C[5] * pow(2, 8);
		TEMP = (2000 + (dT * (int64_t)C[5] / pow(2, 23)));

		OFF = (int64_t)C[2] * pow(2, 16) + (dT*C[4]) / pow(2, 7);
		SENS = (int32_t)C[1] * pow(2, 15) + dT*C[3] / pow(2, 8);

		/*
		SECOND ORDER TEMPARATURE COMPENSATION
		*/
		if (TEMP < 2000) // if temperature lower than 20 Celsius 
		{
			int32_t T1 = 0;
			int64_t OFF1 = 0;
			int64_t SENS1 = 0;

			T1 = pow((double)dT, 2) / 2147483648;
			OFF1 = 5 * pow(((double)TEMP - 2000), 2) / 2;
			SENS1 = 5 * pow(((double)TEMP - 2000), 2) / 4;

			if (TEMP < -1500) // if temperature lower than -15 Celsius 
			{
				OFF1 = OFF1 + 7 * pow(((double)TEMP + 1500), 2);
				SENS1 = SENS1 + 11 * pow(((double)TEMP + 1500), 2) / 2;
			}

			TEMP -= T1;
			OFF -= OFF1;
			SENS -= SENS1;
		}


		P = ((((int64_t)D1*SENS) / pow(2, 21) - OFF) / pow(2, 15));

		//Temparature = (double)TEMP / (double)100;
		Pressure = (double)P / (double)100;

		if (prevSampled_time == 0)
		{
			//fltd_Temparature = Temparature;
			fltd_Pressure = Pressure;
		}

		//fltd_Temparature = alpha * fltd_Temparature + (1 - alpha) * Temparature;
		fltd_Pressure = beta * fltd_Pressure + (1 - beta) * Pressure;

		//printf("Temparature : %.2f C", fltd_Temparature);
		//printf("  Pressure : %.2f mbar", fltd_Pressure);

		Altitude = 44330.0f * (1.0f - pow((double)fltd_Pressure / (double)SEA_LEVEL_PRESSURE, 0.1902949f));

		if (prevSampled_time == 0)
		{
			pre_Altitude = Altitude;
		}

		roc = (int)(100000 * (Altitude - pre_Altitude) / Sampling_time);

		if (prevSampled_time == 0)
		{
			fltd_roc = roc;
		}

		fltd_roc = gamma * fltd_roc + (1 - gamma) * roc;

		pre_Altitude = Altitude;

		//printf("Altitude : %.2f   Rate of Climb : %d cm/s\n", Altitude, roc);

		prevSampled_time = curSampled_time;
		return Altitude;
}

short Motor[4];
float pitch,roll,pitch_last,roll_last;
volatile char yaw,yaw_last;
unsigned short thro=0,thro_last,ad;
volatile unsigned char hold_high_flag=0;
float YawLock_ANGLE = 0;

#define Moto_PwmMax 900
int fd;
int pca9685_init(unsigned char addr)
{
	int pca9685;
	pca9685 = wiringPiI2CSetup(addr);
	{ 
		double T = 2500; //周期，单位是us
		unsigned char prescale;
		double osc_clock = 25000000;
		unsigned char oldmode, newmode;
		T /= 0.915; 
		T /= 1000000; 
		prescale = (unsigned char)(osc_clock/4096*T - 1);

		oldmode = wiringPiI2CReadReg8(pca9685, PCA9685_MODE1);
		newmode = (oldmode&0x7f) | 0x10;
		wiringPiI2CWriteReg8(pca9685, PCA9685_MODE1, newmode);
		wiringPiI2CWriteReg8(pca9685, PCA9685_PRESCALE, prescale);
		oldmode &= 0xef; 
		wiringPiI2CWriteReg8(pca9685, PCA9685_MODE1, oldmode);
		delay(0.005);
		wiringPiI2CWriteReg8(pca9685, PCA9685_MODE1, oldmode | 0xa1);
	}
	return pca9685;
}
void pca9685_setmk(int fd, int num, int mk)	
{
	unsigned int ON, OFF;
	ON = 0; 
	OFF = (unsigned int)((((double)mk)/2500 * 4096)*1.0067114);

	wiringPiI2CWriteReg16(fd, LED0_ON_L+4*num, ON);
	wiringPiI2CWriteReg16(fd, LED0_OFF_L+4*num, OFF);
}
void MotorPwmFlash(int MOTO1_PWM,int MOTO2_PWM,int MOTO3_PWM,int MOTO4_PWM)
{
	if(MOTO1_PWM>=Moto_PwmMax)	MOTO1_PWM = Moto_PwmMax;
	if(MOTO2_PWM>=Moto_PwmMax)	MOTO2_PWM = Moto_PwmMax;
	if(MOTO3_PWM>=Moto_PwmMax)	MOTO3_PWM = Moto_PwmMax;
	if(MOTO4_PWM>=Moto_PwmMax)	MOTO4_PWM = Moto_PwmMax;
	if(MOTO1_PWM<=0)	MOTO1_PWM = 0;
	if(MOTO2_PWM<=0)	MOTO2_PWM = 0;
	if(MOTO3_PWM<=0)	MOTO3_PWM = 0;
	if(MOTO4_PWM<=0)	MOTO4_PWM = 0;//限定输入不能小于0，大于999

	pca9685_setmk(fd, 0, MOTO1_PWM+1000);
	pca9685_setmk(fd, 1, MOTO2_PWM+1000);
	pca9685_setmk(fd, 2, MOTO3_PWM+1000);
	pca9685_setmk(fd, 3, MOTO4_PWM+1000);
}
#define u8  unsigned char
#define    s8  char 
#define TX_OK   0x20
#define RX_OK   0x40
typedef struct
{
    float P;
    float I;
    float D;
    float Desired;
    float Error;
    float PreError;
    double Integ;
    float Deriv;
    float Output;
	
}PID_Typedef;
typedef struct float_xyz
{
    float X;
    float Y;
    float Z;
    
}F_XYZ;
typedef struct float_angle
{
    float Roll;
    float Pitch;
    float Yaw;
}F_ANGLE;

PID_Typedef pitch_angle_PID;	//角度环的PID
PID_Typedef pitch_rate_PID;		//角速率环的PID

PID_Typedef roll_angle_PID;
PID_Typedef roll_rate_PID;

PID_Typedef yaw_angle_PID;
PID_Typedef yaw_rate_PID;

PID_Typedef high_v_PID;
F_XYZ RC_ANGLE,Gyro;	
F_ANGLE MPU_ANGLE;



volatile char YawLockState =0;
float Thr=0,Rool=0,Pitch=0,Yaw=0;
#define dt 0.005//5ms
#define Integ_max 15//20/********改为90***********/
#define angel_offset 1.5 //2  油门倾角补偿
#define xoffset -3    //FULL-0.45  12.2向右偏 -0.3 新电池11.5右偏旧电池12.6左偏
#define yoffset 0       //FULL2垂直起飞   11.8 4.5

void PID_INIT(void) 
{
	float x_rp=0.4,x_rd=1.2;//d0.35 i 0.15
	float x_ap=4,x_ai=3,x_ad=1;//p5
	float y_rp=0.45,y_rd=1.8;//d=2 p0.2
	float y_ap=4,y_ai=3,y_ad=1;//p7.5
		
	pitch_angle_PID.P = y_ap;
	pitch_angle_PID.I = y_ai;
	pitch_angle_PID.D = y_ad;

	pitch_rate_PID.P  =y_rp; 
	pitch_rate_PID.I  = 0; 
	pitch_rate_PID.D  =y_rd; 
////////////////////////////////////////////
	roll_angle_PID.P = x_ap;
	roll_angle_PID.I = x_ai;
	roll_angle_PID.D = x_ad;

	roll_rate_PID.P  = x_rp;
	roll_rate_PID.I  = 0; 
	roll_rate_PID.D  = x_rd; 
///////////////////////////////////////////
	yaw_angle_PID.P = 3;//1 2
	yaw_angle_PID.I = 0;//
	yaw_angle_PID.D = 0;
	
	yaw_rate_PID.P  = 3;//5 4 3
	yaw_rate_PID.I  = 0; 
	yaw_rate_PID.D  = 0;
}
void PID_Series(PID_Typedef * PID,float target,float measure)
{
	PID->Error=target-measure;
	printf("target:%f  measure:%f",target,measure);
	if(thro>=150)	
	{	PID->Integ+=PID->Error*dt;
		if(PID->Integ>Integ_max){PID->Integ=Integ_max;}
		if(PID->Integ<-Integ_max){PID->Integ=-Integ_max;}
	}
	else{PID->Integ=0;}

	PID->Deriv=PID->Error-PID->PreError;
	if(PID->Deriv>25){PID->Deriv=25;}
	if(PID->Deriv<-25){PID->Deriv=-25;}
	PID->Output=PID->P*PID->Error+PID->I*PID->Integ+PID->D*PID->Deriv;
	printf("PID:   P:%f   I:%f  D:%f  \n",PID->Error,PID->Integ ,PID->Deriv);
	PID->PreError=PID->Error;
	
}
void PID_yaw(PID_Typedef * PID,float target,float measure)
{

	PID->Error=target-measure;
	PID->Output=PID->P*PID->Error+PID->I*PID->Integ+PID->D*PID->Deriv;
	PID->PreError=PID->Error;
}

void control(void)
{  
	float ex,ey,thro_add;


    RC_ANGLE.X =roll ;//xoffset+roll;
    RC_ANGLE.Y =pitch+0.5;// yoffset+pitch;
    RC_ANGLE.Z = yaw;
	printf("CONTROLL FUNCTION data:   yaw:%d  pitch:%f   roll:%f  thro:%d   \n",yaw, pitch,roll,thro);
 /*  if((hold_high_flag==1)&&(thro>=150))
	{thro=high_v_PID.Output;}	  //定高开启油门给PID控制     */
	PID_Series(&pitch_angle_PID,RC_ANGLE.Y,ypr[PITCH]); //串级PID
	PID_Series(&pitch_rate_PID,pitch_angle_PID.Output,gyro[1]);//Gyro.Y  DMP角速度	 //串级PID
	//外环控 制。输入为角度,输出为角速度。PID->Output作为内环的输入
	
	PID_Series(&roll_angle_PID,roll,ypr[ROLL]); //串级PID
     PID_Series(&roll_rate_PID,roll_angle_PID.Output,gyro[0]); //串级PID
    if(RC_ANGLE.Z>20|RC_ANGLE.Z<-20)
    {
        YawLock_ANGLE =yaw;
	}
    else
    {			
		YawLock_ANGLE = ypr[YAW];  
	}
    

	//PID_yaw(&yaw_angle_PID,YawLock_ANGLE ,ypr[YAW]);    
	PID_Series(&yaw_angle_PID,YawLock_ANGLE ,ypr[YAW]);    
//gyroqu[2]=-gyroqu[2];
	//PID_Series(&yaw_rate_PID,yaw_angle_PID.Output ,gyroqu[2]);

	//PID_yaw(&yaw_rate_PID,yaw_angle_PID.Output ,gyroqu[2]);//DMP_DATA数据
    

    Pitch = pitch_rate_PID.Output;
    Rool  = roll_rate_PID.Output;
    Yaw   = yaw_angle_PID.Output;//yaw_rate_PID.Output; 
	
#define yaw_max 200		//设置偏航油门上限
    if(Yaw>yaw_max){Yaw=yaw_max;}
	else if(Yaw<-yaw_max){Yaw=-yaw_max;}	
	
		if(thro<50){Motor[0]=0;Motor[1]=0;Motor[2]=0;Motor[3]=0;
		goto NORC;
	}//油门信号过低不启动
	ex=fabs(RC_ANGLE.X);ey=fabs(RC_ANGLE.Y);//油门倾角补偿
	thro_add=(ex+ey)*angel_offset;
	thro+=thro_add;  
    Motor[2] = (int16_t)(thro+Pitch -Rool-Yaw );    //M3  
    Motor[0] = (int16_t)(thro-Pitch +Rool-Yaw);    //M1
    Motor[3] = (int16_t)(thro +Pitch +Rool+Yaw);    //M4 
    Motor[1] = (int16_t)( thro-Pitch -Rool+Yaw );    //M2  thro 
NORC:    MotorPwmFlash(Motor[0],Motor[1],Motor[2],Motor[3]);  
	printf("PID_angle.OUT data:    thro:%d  Pitch:%f   Rool:%f  Yaw:%f  \n",thro,pitch_angle_PID.Output ,roll_angle_PID.Output,yaw_angle_PID.Output);
	printf("PID_rate.OUT data:    thro:%d  Pitch:%f   Rool:%f  Yaw:%f  \n",thro,Pitch ,Rool,Yaw);
	printf("MotorFlash data:    %d  %d   %d  %d  \n",Motor[0],Motor[1],Motor[2],Motor[3]);
	printf("Angle data:  ROLL:%f   PITCH:%f  YAW:%f   \n",ypr[ROLL], ypr[PITCH],ypr[YAW]);
	printf("gyro data:    roll:%d      pitch:%d   yaw%d  \n",gyroqu[0], gyroqu[1],gyroqu[2]);
}


const uint8_t addresses_recv[] =  {0x34,0x43,0x10,0x10,0x01};
const uint8_t addresses_trans[]  = {0x34,0x43,0x10,0x10,0x02};
 
// init counter
unsigned long count = 0;
 int t;
void setup(void)
{
    // init radio for writing on channel 0
	// Refer to RF24.h or nRF24L01 DS for settings
	nrf24_init(RPI_V2_GPIO_P1_15, BCM2835_SPI_CS0, BCM2835_SPI_SPEED_1MHZ);
	nrf24_begin();
	/*
	nrf24_enableDynamicPayloads();
	nrf24_setAutoAck(1);
	nrf24_setRetries(15,15);
	nrf24_setDataRate(RF24_2MBPS);
	nrf24_setPALevel(RF24_PA_MAX);
	nrf24_setChannel(40);
	nrf24_setCRCLength(RF24_CRC_16);
	*/
	//NRF24L01_CE=0;
	bcm2835_gpio_write(BCM2835_SPI_CS0, HIGH);
	bcm2835_gpio_write(RPI_V2_GPIO_P1_15, LOW);
	
  	write_register( EN_AA,0x01);     
  	write_register( EN_RXADDR,0x01); 
	write_register( RF_CH,40);	      
	write_register( RF_SETUP,0x0f); 
	bcm2835_gpio_write(RPI_V2_GPIO_P1_15, HIGH);   
  	//NRF24L01_Write_Buf( TX_ADDR,(u8*)TX_ADDRESS,TX_ADR_WIDTH);
  	 
	//write_register_bytes(RX_ADDR_P0,addresses_trans,5); 

	//nrf24_openWritingPipe(addresses_trans);
	//nrf24_openReadingPipe(1,addresses_recv);
	//nrf24_startListening();

	//
	// Dump the configuration of the rf unit for debugging
	//
	//nrf24_printDetails();
	
	printf("Output below : \n");
	sleep(1);
       
}

TX_MODE()
{
	bcm2835_gpio_write(RPI_V2_GPIO_P1_15, LOW);	
	write_register_bytes(TX_ADDR,addresses_trans,5); 
	write_register( SETUP_RETR,0x1a);
  	write_register( CONFIG,0x0e);  
	bcm2835_gpio_write(RPI_V2_GPIO_P1_15, HIGH);
}

RC_MODE(void)
{
	bcm2835_gpio_write(RPI_V2_GPIO_P1_15, LOW);
	write_register_bytes(RX_ADDR_P0,addresses_recv,5); 
	write_register(RX_PW_P0,32);
	write_register( CONFIG,0x0f);  	
	bcm2835_gpio_write(RPI_V2_GPIO_P1_15, HIGH);
}


uint8_t recv_pipe = 0;
char outBuffer[100] = {0};
char tx(void)
{
    // 32 bytes is maximum payload
    //memset(outBuffer,0, sizeof outBuffer);
	u8 txdata[20];

	unsigned short mot[4];
	unsigned char m1,m2,m3,m4,pitchx,rollx;
	s8 xoffz,yoffz,xoffx,yoffx,pitchz,rollz;
	if(Motor[0]<=0)	Motor[0]= 0;
	if(Motor[1]<=0)	Motor[1]= 0;
	if(Motor[2]<=0)	Motor[2]=0;
	if(Motor[3]<=0)	Motor[3]= 0;
	mot[0]=Motor[0];mot[1]=Motor[1];mot[2]=Motor[2];mot[3]=Motor[3];
	m1=mot[0]*0.1;	m2=mot[1]*0.1;	m3=mot[2]*0.1;	m4=mot[3]*0.1;

	
	pitchz=(s8)ypr[PITCH];	rollz=(s8)ypr[ROLL];
	pitchx=(u8)((ypr[PITCH]-(float)pitchz)*10);	rollx=(u8)((ypr[ROLL]-(float)rollz)*10);
	txdata[0]=0xab;
	txdata[1]=m1;	txdata[2]=m2;	txdata[3]=m3;	txdata[4]=m4;
	
	txdata[5]=xoffz;txdata[6]=xoffx;txdata[7]=yoffz;txdata[8]=yoffx;
	
	txdata[9]=pitchz;txdata[10]=pitchx;txdata[11]=rollz;txdata[12]=rollx;
	/*if(hold_high_flag==1)
	{
		txdata[13]= (u8)high_dis_PID.Desired;  
		txdata[14]=(s8)MS5611_Altitude; 
//		 txdata[15]=(u8)(high_v_PID.Output*0.1);
		txdata[15]=(u8)(fly_thro*0.1);
	}*/
	//txdata[14]=(char)Altitude(altitudefd);
//	txdata[16]=(s8)MS5611_v;
//	txdata[17]=(u8)(fabs(MS5611_v-(s8)MS5611_v)*100);
	txdata[16]=(s8)ypr[YAW];
	txdata[17]=(u8)(fabs(ypr[YAW]-(s8)ypr[YAW])*100);
	
	//printf("%02X\n",get_status());
	
	
	bcm2835_gpio_write(RPI_V2_GPIO_P1_15, LOW);
	write_register_bytes(RX_ADDR_P0,addresses_trans,5); 
	write_payload( txdata, sizeof txdata,0 ? W_TX_PAYLOAD_NO_ACK : W_TX_PAYLOAD ) ; 
	 //nrf24_send(txdata, sizeof txdata);
	write_register( STATUS,0xff);  
	bcm2835_gpio_write(RPI_V2_GPIO_P1_15, HIGH);
	
	//printf("%02X\n",get_status());
	 usleep(400);
	if(get_status()&TX_OK)
	{
		return TX_OK;
	}
	return 0xff;
	
}

char rc(void)
{
	 short pit,rol;
	char rx[100]= {0};

    if(nrf24_available(NULL)){
        nrf24_read(&rx, 100);
		write_register(FLUSH_RX,0xff);
       // printf("receiving data:    %d   %d   %d   %d   %d   %d   %d %d\n", rx[0], rx[1], rx[2],rx[3], rx[4], rx[5],rx[6], rx[7], rx[8]);
	}


	if(	((rx[8]==(rx[1]^rx[2]^rx[3]^rx[4]^rx[5]^rx[6]^rx[7]))&&rx[0])==0x01)
	{
		
		//rcnum=0;			safeland_flag=0;			
		thro=rx[1]<<8|rx[2];
		printf("THE THRO IS %d\n",thro);
		if(thro<50){thro=0;}
		
		if(thro>900){thro=thro_last;}
		else{thro_last=thro;}
		yaw=rx[3];
		pit=(rx[4]<<8)|rx[5];
		rol=(rx[6]<<8)|rx[7];
		pitch=(float)pit*0.1;
		roll=(float)rol*0.1;
		printf("control recv data:   yaw:%d  pitch:%f   roll:%f  thro:%d   \n",yaw, pitch,roll,thro);
		if(yaw>yaw_range||yaw<-yaw_range){yaw=0;}
		if(pitch>pitch_range||pitch<-pitch_range){pitch=0;}
		if(roll>roll_range||roll<-roll_range){roll=0;}	
		pitch_last=pitch;roll_last=roll;yaw_last=yaw;
		// printf("control pross data:    %d  %d   %d  %d \n",yaw, pitch,roll,thro);
	}
		else{thro=thro_last;yaw=yaw_last;pitch=pitch_last;roll=roll_last;	
	}
	 //printf("control pross data:    %d  %f   %f   %d \n",yaw, pitch,roll,thro);
	if(get_status()&RX_OK)
	{
		return RX_OK; 
	}	   
	return 0xff;
}

int main(int argc, char** argv) 
{
	fd =pca9685_init(0x40);
	/*pca9685_setmk(fd,0,1100);
	pca9685_setmk(fd,1,1100);
	pca9685_setmk(fd,2,1100);
	pca9685_setmk(fd,3,1100);
	sleep(3);
	/*pca9685_setmk(fd,0,1000);
	pca9685_setmk(fd,1,1000);
	pca9685_setmk(fd,2,1000);
	pca9685_setmk(fd,3,1000);
	sleep(3);
	pca9685_setmk(fd,0,1000);
	pca9685_setmk(fd,1,1000);
	pca9685_setmk(fd,2,1000);
	pca9685_setmk(fd,3,1000);	*/
	PID_INIT() ;
    setup();
	ms_open();
	openMs5611();
    while(1)
	{
		t=0;
		usleep(2000);
		TX_MODE();
		usleep(2000);
		ms_update();
		while(tx()!=TX_OK)
		{
			t++;
			if(t>16) break;
		}
		printf("send ok\n\n\n");
		//usleep(10000);
		
		t=0;
		usleep(2000);
		RC_MODE();
		usleep(2000);
		while(rc()!=RX_OK)
		{
			t++;
			if(t>16) break;
		}
		printf("recive ok\n");
		

		//MotorPwmFlash(100,100,100,100);
		
		control();
	}
	 
    return 0;
}
