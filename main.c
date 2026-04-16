
 #include "stm32f4xx.h"
 #include <stdio.h>
 #include <stdlib.h>
 #include <math.h>
 #include <string.h>
 #include "ctype.h"
 
 
 #include "config_encoder.h"
 #include "config_uart.h"
 #include "config_pwm.h" 
 #include "Elec_Compass.h"  
 #include "lcd.h"

#define PI 3.14159265
#define SPEED_LIMIT 100
#define NUM_OMNI (PI / 180.0)


vu32 var_i=0;
u8 i=0;
vu32 tick_time_gu32=0;
vs16 _value_compass;
vu32 time;
u8 value_speed=240;
int16_t SPEED = 0;

int16_t SPEED_mong_muon;
int16_t SPEED_QUAY = 1;

 float Huong_tinh_tien;
 vs16 Huong_xoay;
 vs16 Toc_do_xoay;
 vs16 SPEED_tinhtien;
 vs16 Beta;
int16_t xoay;
u8 Value_vantocgoc;
u8 Value_Giatoc;
__IO u8 Value_giatoc_pwm = 1;


char Command_compass;
#define MAX_STRLEN 100 // this is the maximum string length of our string in characters
char received_string[MAX_STRLEN+1]; // this will hold the recieved string
char str_rev[MAX_STRLEN];
char str[MAX_STRLEN];

u8 JoysticXL;
u8 JoysticYL;
u8 JoysticXR;
u8 JoysticYR;


#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */


/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void Delay(__IO uint32_t nCount)
{
  while(nCount--)
  {
  }
}

 //XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
 
u8 Vgetchar1(void)
{
//	while((USART1->SR & USART_FLAG_RXNE) == (uint16_t)RESET); 
//	return USART1->DR;
	
	while(USART_GetFlagStatus(USART1,USART_FLAG_RXNE)==RESET);
	return USART_ReceiveData(USART1);
	
}
//XXXXXXXXXXXXXXXXXXXXXXXXXXXX
void send_uart2_char(u8 ch)
{
	USART_SendData(USART2, (uint8_t) ch);
	while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
}
//xxxxxxxxxxxx
void Vsend_char(u8 ch)
{
		USART_SendData(USART1, (uint8_t) ch);
		while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
		{};		
}

//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
void send_string_uart(u8 *p,u8 num_uart)
{
	
	
	if(num_uart==1)
	{
		while(*p)
		{
			USART_SendData(USART1, (uint8_t) *p++);
			while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
			{};	
		}
	}
	
	else if(num_uart==2)
	{
		while(*p)
		{
			USART_SendData(USART2, (uint8_t) *p++);
			while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET)
			{};	
		}
	}
	
	else if(num_uart==3)
	{
		while(*p)
		{
			USART_SendData(USART3, (uint8_t) *p++);
			while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET)
			{};	
		}
	}
	
	
	else if(num_uart==4)
	{
		while(*p)
		{
			USART_SendData(UART4, (uint8_t) *p++);
			while (USART_GetFlagStatus(UART4, USART_FLAG_TC) == RESET)
			{};	
		}
	}
	
}
//xxxxxxxxxxxx


void delay_ms_stick(u32 del)
{
	u32 a;
	a=del+tick_time_gu32;
	while(tick_time_gu32<a);
	
}
//xxxxxxxxxxxxxx
	u8 getchar_u8=0;
//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
/*
Ham con dieu khien 3 banh :
Cac tham so can truyen gom : Huong tinh tien cua Robot(Alpha), Goc xoay cua Robot(Beta), Van toc xoay, van toc tinh tien
Vi du : 
Robot di thang huong 90 do khong xoay : Da_huong(90,0,0,10);
Robot di lui huong 90 do khong xoay : Da_huong(-90,0,0,10);

De Robot xoay 1 goc Beta so voi goc toa do thi trong qua trinh di chuyen ta truyen gia tri Beta va toc do di chuyen cho robot :
Khai bao mot bien Huong xoay(co dau) : trong qua trinh Robot di chuyen ta chi gan gia tri cho bien nay Robot se xoay the huong ta muon

	Da_huong(-90,Huong_xoay,10,10);
	

//*/
int16_t huongxoay ;
int i1,i2,i3,i4;
u8 value_pwm1,value_pwm2,value_pwm3,value_pwm4,value_dir=0;
 
		vs32 sai_so_goc, MotoA, MotoB, MotoC, MotoD, MotoA_Ref, MotoB_Ref, MotoC_Ref, MotoD_Ref,
			MotoA1, MotoB1, MotoC1, MotoD1;
		volatile float GocA, GocB, GocC, GocD, value_pwmA, value_pwmB, value_pwmC, value_pwmD;
		volatile float  value_cosA, value_cosB, value_cosC, value_cosD;

vs16 value_compass;

__IO float offsetMotor = 6;
		
void Da_huong_4banh(float Huong_tinh_tien_alpha, vs16 beta , u8 value_locity, u8 SPEED_TINH_TIEN, u8 VALUE_GIATOC_PWM)
	{
	
    s16 temp = 0; 
		float tamf;
		xoay = beta;
		Value_vantocgoc = value_locity;
		SPEED_mong_muon = SPEED_TINH_TIEN;
		value_compass = _value_compass;
		
		sai_so_goc = value_compass/10 - Beta;
		
		GocA = Huong_tinh_tien_alpha + value_compass/10 - 135 - 90;		
		GocB = Huong_tinh_tien_alpha + value_compass/10 + 135 - 90;		
		GocC = Huong_tinh_tien_alpha + value_compass/10 + 45 - 90;		
		GocD = Huong_tinh_tien_alpha + value_compass/10 - 45 - 90;
		
		
//		GocA = Huong_tinh_tien_alpha - 45;		
//		GocB = Huong_tinh_tien_alpha + 45;		
//		GocC = Huong_tinh_tien_alpha + 135;		
//		GocD = Huong_tinh_tien_alpha - 135;
		
		
		value_cosA = cosf(GocA * (tamf=NUM_OMNI));
		value_cosB = cosf(GocB * (tamf=NUM_OMNI));
		value_cosC = cosf(GocC * (tamf=NUM_OMNI));
		value_cosD = cosf(GocD * (tamf=NUM_OMNI));
		
		MotoA_Ref = SPEED * value_cosA + SPEED_QUAY * sai_so_goc;
		MotoB_Ref = SPEED * value_cosB + SPEED_QUAY * sai_so_goc;
		MotoC_Ref = SPEED * value_cosC + SPEED_QUAY * sai_so_goc;
		MotoD_Ref = SPEED * value_cosD + SPEED_QUAY * sai_so_goc;
		
	temp = MotoA_Ref/10;

	if(temp==0)
	{
		temp = 1;
	}
	if(temp<0)
	{
		temp = -temp;
	}
		if(MotoA > MotoA_Ref) 
		{
				MotoA -= temp;
				if(MotoA <= MotoA_Ref) MotoA = MotoA_Ref;
		}
		else 
		{
				MotoA += temp;
				if(MotoA >= MotoA_Ref )MotoA = MotoA_Ref;
		}
		
temp = MotoB_Ref/10;
		
	if(temp==0)
	{
		temp = 1;
	}
	if(temp<0)
	{
		temp = -temp;
	}	
	
		if(MotoB > MotoB_Ref) 
		{
				MotoB-=temp;
				if(MotoB <= MotoB_Ref) MotoB = MotoB_Ref;
		}
		else 
		{
				MotoB+=temp;
				if(MotoB >= MotoB_Ref )MotoB = MotoB_Ref;
		}
		
temp = MotoC_Ref/10;
		
	if(temp==0)
	{
		temp = 1;
	}
	if(temp<0)
	{
		temp = -temp;
	}
	
		if(MotoC > MotoC_Ref) 
		{
				MotoC-=temp;
				if(MotoC <= MotoC_Ref) MotoC = MotoC_Ref;
		}
		else 
		{
				MotoC+=temp;
				if(MotoC >= MotoC_Ref )MotoC = MotoC_Ref;
		}

temp = MotoD_Ref/10;
		
	if(temp==0)
	{
		temp = 1;
	}
	if(temp<0)
	{
		temp = -temp;
	}
	
		if(MotoD > MotoD_Ref) 
		{
				MotoD-=temp;
				if(MotoD <= MotoD_Ref) MotoD = MotoD_Ref;
		}
		else 
		{
				MotoD+=temp;
				if(MotoD >= MotoD_Ref )MotoD = MotoD_Ref;
		}
		
	if(MotoA > 0) {
		DIR1_SET;
		MotoA1=MotoA;
		i1=1;
	}
	else{
		DIR1_RESET;
			MotoA1=-MotoA;
		i1=0;
	}
	
///////////////////////	
	
	if(MotoB > 0){
		DIR2_SET;
		MotoB1=MotoB;
		i2=1;
	}
	else{
		DIR2_RESET;
			MotoB1=-MotoB;
		i2=0;
	}
	
//////////////////	
	
	if(MotoC > 0){
		DIR3_SET;
		MotoC1=MotoC;
		i3=1;
	}
	else{
		DIR3_RESET;
		MotoC1=-MotoC;
		i3=0;
	}
	
////////////////////	
	
	if(MotoD > 0){
		DIR4_SET;
		MotoD1=MotoD;
		i4=1;
	}
	else{
		DIR4_RESET;
		MotoD1=-MotoD;
		i4=0;
	}
		
	
	
			value_pwmA = MotoA1 + offsetMotor;
			value_pwmB = MotoB1 + offsetMotor;
			value_pwmC = MotoC1 + offsetMotor;
			value_pwmD = MotoD1 + offsetMotor;
	
	
	if(SPEED == 0){
		PWM1 = 0;
		PWM2 = 0;
		PWM3 = 0;
		PWM4 = 0;
		
	}
	else {
		PWM1 = ((value_pwmA>SPEED_LIMIT)?value_pwmA=SPEED_LIMIT:value_pwmA);
		PWM2 = ((value_pwmB>SPEED_LIMIT)?value_pwmB=SPEED_LIMIT:value_pwmB);
		PWM3 = ((value_pwmC>SPEED_LIMIT)?value_pwmC=SPEED_LIMIT:value_pwmC);
		PWM4 = ((value_pwmD>SPEED_LIMIT)?value_pwmD=SPEED_LIMIT:value_pwmD);
	}		
	

}

u8 show_lcd;
#define max_buff 10
u8 buff_game[max_buff];
u8 cnt;
u16 value_game;
u16 value_game_Joytic_Left;
u16 value_game_Joytic_Right;
u8 ktra;

void Gamepad_da_huong(){
		switch(value_game){
			case 0xefff :{
				show_lcd = 'A';
				Huong_tinh_tien = 90;
				SPEED_tinhtien = 100;
				Toc_do_xoay = 10;
				Value_Giatoc=5;
				break;
			}
			case 0xdfff :{
				show_lcd = 'B';
				SPEED_mong_muon = 0;
				SPEED_mong_muon = 20;
				Huong_tinh_tien = 0;
				break;
			}
			case 0xbfff :{
				show_lcd = 'C';
				Huong_tinh_tien = -90;
				break;
			}
			case 0x7fff :{
				Huong_tinh_tien = 180;
				show_lcd = 'D';
				break;
			}
			case 0xffef :{
				Huong_xoay = 60;
				show_lcd = 'I';
				break;
			}		
			case 0xffdf :{
				Huong_xoay = -60;
				show_lcd = 'J';
				break;
			}
			case 0xff7f :{
				
				show_lcd = 'K';
				break;
			}
			case 0xffbf :{
				
				Huong_xoay = 0;
				break;
			}
			case 0xfeff :{
				
				show_lcd = 'Z';
				break;
			}		
			case 0xfffb :{
				
				show_lcd = '0';
				break;
			}
			case 0xffeb :{
				
				show_lcd = '1';
				break;
			}
			case 0xffdb :{
				
				show_lcd = '2';
				break;
			}
			case 0xffbb :{
				
				show_lcd = '3';
				break;
			}
			case 0xff7b :{
				
				show_lcd = '4';
				break;
			}

			case 0xfffe :{
				
				show_lcd = '5';
				break;
			}
			case 0xffee :{
				
				show_lcd = '6';
				break;
			}
			case 0xffde :{
				
				show_lcd = '7';
				break;
			}
			case 0xffbe :{
				
				show_lcd = '8';
				break;
			}
			case 0xff7e :{
				
				show_lcd = '9';
				break;
			}	

			case 0xfff7 :{
				
				show_lcd = 'X';
				break;
			}
			case 0xfffd :{
				
				SPEED_tinhtien = 0;
				break;
			}				
		}
}


vu32 test1;

RCC_ClocksTypeDef ClockSys;
vs32 TestencoderTimer3;
vs32 TestencoderTimer2;
vs32 TestencoderTimer4;
vs32 TestencoderTimer1;

uint8_t byte_h;
uint8_t byte_l;
u8 send_a;

int16_t Huong, chieuquay, speed_tinh_tien;

int main(void)
{

	
	char buffer[50];
  char mang[100];
	char mang_tam[3];
	
	config_uart1(57600);
//config_uart2(115200);
	config_uart3(115200);
	config_uart4(115200);
	Config_timer67();
	ConfigEncoder();
	config_PWM();
				
	config_compass();
	lcd_Init();
	//config_uart2_dma(115200);


	
	
   if (SysTick_Config(SystemCoreClock / 1000))  // reload value = 1000 --> systick 1ms
   { 
     /* Capture error */ 
     while (1);
   }
 // mang="12354";
	strcpy(mang,"123456"); 
	 while(++send_a < 5){
	 	Command_compass = 'a';
		delay_ms(100);
	 }

//ClearEncoder(1);	ClearEncoder(2); ClearEncoder(3);ClearEncoder(4); 
	Command_compass = 'z';

	 
	 

//while(1){
//PWM1=value_pwm;
//	PWM2=value_pwm;
//	PWM3=value_pwm;
//	PWM4=value_pwm;
//}	 

	 
	while (1)
	{  
lcd_Goto(0,0);lcd_Print_Data((u8 *)"THEGIOICHIP.COM.VN ");
lcd_Goto(1,0);lcd_Print_Data((u8 *)"MAIN ROBOT 2017");		
	

//sprintf(buffer,"%3.1f  %3.1f  %3.1f   ",MotoA,MotoB,MotoC);	
//lcd_Goto(2,0);lcd_Print_Data((u8 *)buffer);
sprintf(buffer,"%5d",_value_compass);
		
lcd_Goto(3,5 );lcd_Print_Data((u8 *)buffer);
//		
//		sprintf(mang, "%2d %2d %2d", SPEED_mong_muon, SPEED_QUAY, Beta);
//		lcd_Goto(3,12 );lcd_Print_Data((u8 *)mang);
	
		
			//RCC_GetClocksFreq(&ClockSys);
			//test1=ClockSys.HCLK_Frequency;

//		TestencoderTimer2=ReadEncoder(2);
//		TestencoderTimer3=ReadEncoder(3);
//		TestencoderTimer4=ReadEncoder(4);
//		TestencoderTimer1=ReadEncoder(1);
//		sprintf(mang,"E4=%i ----- E3=%i------E2=%i ----- E1=%i\n\r",TestencoderTimer4,TestencoderTimer3,TestencoderTimer2,TestencoderTimer1);


		Gamepad_da_huong();
		lcd_Goto(2,10);lcd_Data_Write(show_lcd);

//		Joystic_control();
		//Beta = value_compass = Joytic_control;
		
	}
}