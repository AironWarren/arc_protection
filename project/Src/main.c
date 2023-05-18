/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2021 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
//#include "bsp.h"
#include "mb.h"
#include "user_mb_app.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
	RTC_TimeTypeDef sTime;
  RTC_DateTypeDef sDate;

parametrs par;
srtc rtc;
int reload = 0, dance = 0;
volatile float value[4]={0,};
volatile uint32_t timer1=0; 
//volatile uint16_t ADC_1_1,ADC_1_2,ADC_2_1,ADC_2_2,ADC_3_1,ADC_3_2, Current_1,ADC_Test_1_1,ADC_Test_1_2,ADC_Test_2_1,ADC_Test_2_2,ADC_Test_3_1,ADC_Test_3_2,Opto_test=1;
int cnt = 0, tools = 0x8,may = 0;
int min_ok = 0,hui = 0;
uint16_t devAddr = (0x50 << 1);
uint16_t memaddress_of_the = 0x0900;

int cnt30 = 0, cnt30_rez = 0;

int cnt_light_1 = 0, cnt_light_2 = 0, cnt_light_3 = 0,cnt_light_next_1 = 0,cnt_light_next_2 = 0,cnt_light_next_3 = 0;
int light_without_current_1 = 0,light_without_current_2 = 0,light_without_current_3 = 0;
int P_T_1_1 =0,P_T_2_1 =0,P_T_3_1 =0,P_T_1_2 =0,P_T_2_2 =0,P_T_3_2 =0;
int processing_time1_1,processing_time1_2,processing_time2_1,processing_time2_2,processing_time3_1,processing_time3_2;
volatile int Error_Buff[48];
volatile uint16_t adc[4] = {0,}; 
int reset,g=0,h=500,k=0,t=500,o=500,cnt_1;
unsigned int Opto_test,Sr_1=0,Sr_2=0,Sr_3=0;
int S1=0,Disc_entrance=0,ARC_test=0,Disc_current_on,level_is_exceeded;
int variable=0,in=0,quantity=0x8,j=0,am=0,change_point = 0,score = 0;
int address_of_the_last_element[2];
volatile uint16_t u;
int Addres[8],cntReadRTC;
int i,N,Banning_exits=0,reason;
int malfunction_1,malfunction_2,malfunction_3,malfunction_4;
int Answer =0;
int ARC=0,Vau=0,Mau=0;
int step_1=1,step_2=1,step_3=1;
int MS20,MS1,MS5,MKS40,MKS400,MIN1;
int a;
int ARC_1,ARC_2,ARC_3;
int Channel_1,Channel_2,Channel_3;
int c,q;
unsigned int Pipe_1_1,Pipe_1_2,Pipe_2_1,Pipe_2_2,Pipe_3_1,Pipe_3_2,Pipe_1,Pipe_2,Pipe_3,Pipe_4,Pipe_5,Pipe_6;
unsigned int Transition_1_1,Transition_1_2,Transition_1_3,Transition_2_1,Transition_2_2,Transition_2_3,Transition_3_1,Transition_3_2,Transition_3_3,Transition_1,Transition_2,Transition_3,Transition_4,Transition_5,Transition_6;
unsigned int ADC_Test_1_1,ADC_Test_1_2,ADC_Test_2_1,ADC_Test_2_2,ADC_Test_3_1,ADC_Test_3_2;
int Test_error_1,Test_error_2,Test_error_3;
int Sensor_Disconection_1,Sensor_Disconection_2,Sensor_Disconection_3;
int ADC_1,ADC_2,ADC_3;
unsigned int Current;
int Start_ADC_1 = 0,Start_ADC_2 = 0,Start_ADC_3 = 0,Light_ADC_1 = 0,Light_ADC_2 = 0,Light_ADC_3 = 0,Start_Light_1,Night_ADC_1 = 0,Night_ADC_2 = 0,Night_ADC_3 = 0;
int dark_1 = 0,light_1 = 0,arc_1 = 0,dark_2 = 0,light_2 = 0,arc_2 = 0,dark_3 = 0,light_3 = 0,arc_3 = 0,light_out_1=0,light_out_2=0,light_out_3=0;
int darck_out_1 = 0,darck_out_2 = 0, darck_out_3 = 0;
unsigned int GP2,GP3;
int flash_1,flash_2,flash_3;
int a = 30, b = 30, c = 30;
volatile int Z1 = 0,Z2 = 0,Z3 = 0;
int correction = 0;
int qwe1 = 0, qwe2 = 0, qwe3 = 0;
uint16_t transition[] ={0,};
uint16_t cnt15ms = 0;
uint16_t Current_start_up = 0;
//Для работы с временем RTC

volatile int r,l,number,mumber;

int cnt40=0;
int A1;
volatile int dat[6]={0,};
volatile int Data[8]={0,};

volatile int dat_time[6]={0,};
volatile int Data_Time[6]={0,},test_45=0;
volatile uint8_t Flag_ADC;

// Для работы с EEPROM
int inputs=0,s=0,p=1;
volatile uint16_t memAddr;
HAL_StatusTypeDef status;

// Для работы с UART
char test[4];

char		masBoot[7];
short		indexBoot;

char trans_str[64] = {0,};
volatile uint8_t flag = 0; // флажок
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
__IO eMBErrorCode err;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	//------------------------------------------------------
	//Перенос вектора необходим для того чтобы программа могла вызывать прерывания, так как в начале у нас записан бутлоудер
	//проверьте Flash->Config Flesh Tools->Target должно быть 0x8004000
	//-------------------------------------------------------
	SCB->VTOR = 0x80004000;
	BKP->DR42 = 0;
	
	//------------------------------------------------------
	//Данная запись говорит о том что, контроллер не будет выполнять свои функции, если питающее ео напряжение станет меньше 2.9 В
	//-------------------------------------------------------
	//PWR->CR |= (PWR_CR_PVDE_Msk|PWR_CR_PLS_Msk);//porog 2.9
	//HAL_PWR_EnablePVD();
	
	
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  MX_GPIO_Init();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_RTC_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_PWR_EnableBkUpAccess(); //Разрешение использование Flash памяти контроллера 
	//PVD_Config();
	Read_Memory(); //Функция читает всю EEPROM и помещает в usSRegHoldBuf в буфер карты памяти Modbus
	Rules(); //функция установки уставок и адреса записи ошибок
	

	//HAL_Delay(1000);

  HAL_TIM_Base_Start_IT(&htim7);
	HAL_ADCEx_Calibration_Start(&hadc1);
	HAL_TIM_Base_Start_IT(&htim6);
	


	eMBInit(MB_RTU,0x11,0,38400,MB_PAR_NONE);
	eMBEnable();

	
	//Установка года, месяца, даты
	sDate.WeekDay = (BKP->DR1) & 0xff;
	sDate.Date = (BKP->DR1 >> 8) & 0xff;
	sDate.Month = (BKP->DR2) & 0xff;
	sDate.Year = (BKP->DR2 >> 8) & 0xff;
	 
	hrtc.DateToUpdate.Year  = sDate.Year;//RTC_Bcd2ToByte(sDate.Year);
	hrtc.DateToUpdate.Month = sDate.Month;//RTC_Bcd2ToByte(sDate.Month);
	hrtc.DateToUpdate.Date  = sDate.Date;//RTC_Bcd2ToByte(sDate.Date);

	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_2,GPIO_PIN_SET);

	
	//	for(i=0;i<10000000;i++); //подумать 
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		
		//СВЕТОВАЯ ИНДИКАЦИЯ ПРИ НАРУШЕНИИ ЦЕЛОСТНОСТИ ОПТИЧЕСКОГО ВОЛОКНА
		//---------------------------------------------------------------------------------------------------------------

		if(Test_error_1 == 1 && ARC_1 == 0) //нарушена целостность оптоволокна первого канала, световая идентификация  
		{
				HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE,GPIO_PIN_6,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);	
				HAL_GPIO_WritePin(GPIOE,GPIO_PIN_1,GPIO_PIN_RESET);
		
			/*HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOE,GPIO_PIN_8,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOE,GPIO_PIN_7,GPIO_PIN_RESET);	
				HAL_GPIO_WritePin(GPIOE,GPIO_PIN_1,GPIO_PIN_RESET);*/
		}
		if(Test_error_2 == 1 && ARC_2 == 0) //нарушена целостность оптоволокна второго канала, световая идентификация  
		{
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_SET);	
				HAL_GPIO_WritePin(GPIOE,GPIO_PIN_1,GPIO_PIN_RESET);
		}
		if(Test_error_3 == 1 && ARC_3 == 0) //нарушена целостность оптоволокна третьего канала, световая идентификация  
		{
				HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOE,GPIO_PIN_8,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOE,GPIO_PIN_7,GPIO_PIN_RESET);	
				HAL_GPIO_WritePin(GPIOE,GPIO_PIN_1,GPIO_PIN_RESET);
					
			/*HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE,GPIO_PIN_6,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);	
				HAL_GPIO_WritePin(GPIOE,GPIO_PIN_1,GPIO_PIN_RESET);*/
		}
		
		//---------------------------------------------------------------------------------------------------------------
		//СВЕТОВАЯ ИНДИКАЦИЯ ПРИ ДУГЕ, ЗАСВЕТЕ, ЗАСВЕТЕ ПОСЛЕ ДУГИ 
		//---------------------------------------------------------------------------------------------------------------
		
		if(usSRegHoldBuf[289] == 1) //пуск по току включен или нет 
		{
			if(level_is_exceeded == 1) //есть пуск по току или нет
			{
				if(Channel_1 == 1) //канал первый 
				{
					malfunction_1++; //счетчик идикации ошибки, переменное свечение светодиода 
					if(ARC_1 == 1) //свет на первом канале 
					{
						if(Light_ADC_1 == 0) //дуга, а потом свет пропал 
						{
				
							HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,GPIO_PIN_SET);
							HAL_GPIO_WritePin(GPIOE,GPIO_PIN_6,GPIO_PIN_SET);
							HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);

						/*HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9,GPIO_PIN_RESET);
							HAL_GPIO_WritePin(GPIOE,GPIO_PIN_8,GPIO_PIN_SET);
							HAL_GPIO_WritePin(GPIOE,GPIO_PIN_7,GPIO_PIN_SET);*/
						}
						else
						{
							if(Night_ADC_1 == 0) //дуга, а потом свет остался 
							{

								HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,GPIO_PIN_RESET);
								HAL_GPIO_WritePin(GPIOE,GPIO_PIN_6,GPIO_PIN_SET);
								HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);

							/*HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9,GPIO_PIN_SET);
								HAL_GPIO_WritePin(GPIOE,GPIO_PIN_8,GPIO_PIN_SET);
								HAL_GPIO_WritePin(GPIOE,GPIO_PIN_7,GPIO_PIN_RESET);*/
							}
							else //свет пропал 
							{  

									HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,GPIO_PIN_SET);
									HAL_GPIO_WritePin(GPIOE,GPIO_PIN_6,GPIO_PIN_SET);
									HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);

/*									HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9,GPIO_PIN_RESET);
									HAL_GPIO_WritePin(GPIOE,GPIO_PIN_8,GPIO_PIN_SET);
									HAL_GPIO_WritePin(GPIOE,GPIO_PIN_7,GPIO_PIN_SET);*/
							}
						}
					}
					if(light_without_current_1 == 1 && ARC_1 == 0) //есть ток но до этого был свет, а теперь света нет
					{

						HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,GPIO_PIN_RESET);
						HAL_GPIO_WritePin(GPIOE,GPIO_PIN_6,GPIO_PIN_SET);
						HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);

					/*HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9,GPIO_PIN_SET);
						HAL_GPIO_WritePin(GPIOE,GPIO_PIN_8,GPIO_PIN_SET);
						HAL_GPIO_WritePin(GPIOE,GPIO_PIN_7,GPIO_PIN_RESET);*/

					}
				}
				if(Channel_2 == 1) //второй канал 
				{
					malfunction_2++; 
					if(ARC_2 == 1) //свет на втором канале
					{
						if(Light_ADC_2 == 0) //была дуга, после чего свет пропал 
						{
							HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);
							HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_SET);
							HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_RESET);
						}
						else
						{
							if(Night_ADC_2 == 0) //была дуга но свет остался 
							{
								HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);
								HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_SET);
								HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_SET);
							}
							else //была дуга свет был, но потом пропал 
							{
								HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);
								HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_SET);
								HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_RESET);
							}
						}
					}
				  if(light_without_current_2 == 1 && ARC_2 == 0) //был свет без тока, а при приходи тока света нет 
					{
						HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);
						HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_SET);
						HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_SET);
					}
				}
				if(Channel_3 == 1) // третий канад
				{
					malfunction_3++;
					if(ARC_3 == 1) //был свет 
					{
						if(Light_ADC_3 == 0) //была дуга после чего свет пропал 
						{

							HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9,GPIO_PIN_RESET);
							HAL_GPIO_WritePin(GPIOE,GPIO_PIN_8,GPIO_PIN_SET);
							HAL_GPIO_WritePin(GPIOE,GPIO_PIN_7,GPIO_PIN_SET);

						/*HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,GPIO_PIN_SET);
							HAL_GPIO_WritePin(GPIOE,GPIO_PIN_6,GPIO_PIN_SET);
							HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);*/
						}
						else
						{
							if(Night_ADC_3 == 0) //была дуга, после чего свет остался 
							{
							/*HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,GPIO_PIN_RESET);
								HAL_GPIO_WritePin(GPIOE,GPIO_PIN_6,GPIO_PIN_SET);
								HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);*/

								HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9,GPIO_PIN_SET);
								HAL_GPIO_WritePin(GPIOE,GPIO_PIN_8,GPIO_PIN_SET);
								HAL_GPIO_WritePin(GPIOE,GPIO_PIN_7,GPIO_PIN_RESET);

							}
							else //была дуга, был свет, но потом свет пропал 
							{
							/*HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9,GPIO_PIN_RESET);
								HAL_GPIO_WritePin(GPIOE,GPIO_PIN_8,GPIO_PIN_SET);
								HAL_GPIO_WritePin(GPIOE,GPIO_PIN_7,GPIO_PIN_SET);*/

								HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9,GPIO_PIN_RESET);
								HAL_GPIO_WritePin(GPIOE,GPIO_PIN_8,GPIO_PIN_SET);
								HAL_GPIO_WritePin(GPIOE,GPIO_PIN_7,GPIO_PIN_SET);
	
							}
						}
					}
					if(light_without_current_3 == 1 && ARC_3 == 0) //был свет но небыло тока, ток появился а свет пропал 
					{
					/*HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,GPIO_PIN_RESET);
						HAL_GPIO_WritePin(GPIOE,GPIO_PIN_6,GPIO_PIN_SET);
						HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);*/

						HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9,GPIO_PIN_SET);
						HAL_GPIO_WritePin(GPIOE,GPIO_PIN_8,GPIO_PIN_SET);
						HAL_GPIO_WritePin(GPIOE,GPIO_PIN_7,GPIO_PIN_RESET);

					}
				}
			}
			else
			{
				if(Channel_1 == 1) //первый канад 
				{
					if(ARC_1 == 1) //ток есть
					{
						if(light_without_current_1 == 0)
						{
							malfunction_1++; //счетчик идикации ошибки, переменное свечение светодиода
							HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,GPIO_PIN_SET);
							HAL_GPIO_WritePin(GPIOE,GPIO_PIN_6,GPIO_PIN_SET);
							HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);
						}
						else
						{
							malfunction_1++;
							HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,GPIO_PIN_RESET);
							HAL_GPIO_WritePin(GPIOE,GPIO_PIN_6,GPIO_PIN_SET);
							HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);
						}	
					}
					if(light_without_current_1 == 1 && ARC_1 == 0) //свет есть, а тока нет 
					{
							malfunction_1++;
						/*HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9,GPIO_PIN_SET);
							HAL_GPIO_WritePin(GPIOE,GPIO_PIN_8,GPIO_PIN_SET);
							HAL_GPIO_WritePin(GPIOE,GPIO_PIN_7,GPIO_PIN_RESET);*/

							HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,GPIO_PIN_RESET);
							HAL_GPIO_WritePin(GPIOE,GPIO_PIN_6,GPIO_PIN_SET);
							HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);
					}
				}
				if(Channel_2 == 1) //второй канал 
				{
					if(ARC_2 == 1)
					{
						if(light_without_current_2 == 0)
						{
							malfunction_2++;
							HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);
							HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_SET);
							HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_RESET);
						}
						else
						{
							malfunction_2++;
							HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);
							HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_SET);
							HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_SET);
						}
					}
					if(light_without_current_2 == 1 && ARC_1 == 0) //свет есть, а тока нет 
					{
							malfunction_2++;
							HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);
							HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_SET);
							HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_SET);
					}
				}
				if(Channel_3 == 1) //третий канал 
				{
					if(ARC_3 == 1)
					{
						if(light_without_current_3 == 0)
						{
							malfunction_3++;
							HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9,GPIO_PIN_RESET);
							HAL_GPIO_WritePin(GPIOE,GPIO_PIN_8,GPIO_PIN_SET);
							HAL_GPIO_WritePin(GPIOE,GPIO_PIN_7,GPIO_PIN_SET);
						}
						else
						{
							malfunction_3++;
/*						HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,GPIO_PIN_RESET);
							HAL_GPIO_WritePin(GPIOE,GPIO_PIN_6,GPIO_PIN_SET);
							HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);*/
							HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9,GPIO_PIN_SET);
							HAL_GPIO_WritePin(GPIOE,GPIO_PIN_8,GPIO_PIN_SET);
							HAL_GPIO_WritePin(GPIOE,GPIO_PIN_7,GPIO_PIN_RESET);
						}
					}
					if(light_without_current_3 == 1 && ARC_1 == 0) //свет есть, а тока нет 
					{
							malfunction_3++;
						/*HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,GPIO_PIN_RESET);
							HAL_GPIO_WritePin(GPIOE,GPIO_PIN_6,GPIO_PIN_SET);
							HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);*/
							HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9,GPIO_PIN_SET);
							HAL_GPIO_WritePin(GPIOE,GPIO_PIN_8,GPIO_PIN_SET);
							HAL_GPIO_WritePin(GPIOE,GPIO_PIN_7,GPIO_PIN_RESET);
						
					}
				}
			}
		}
		else //нет пуска по току
		{
			if(Channel_1 == 1) //первый канал 
			{
				malfunction_1++;
				if(ARC_1 == 1) //есть свет 
				{
					if(Light_ADC_1 == 0) //дуга была, а свет пропала
					{
					/*HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9,GPIO_PIN_RESET);
						HAL_GPIO_WritePin(GPIOE,GPIO_PIN_8,GPIO_PIN_SET);
						HAL_GPIO_WritePin(GPIOE,GPIO_PIN_7,GPIO_PIN_SET);*/
						
						HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,GPIO_PIN_SET);
						HAL_GPIO_WritePin(GPIOE,GPIO_PIN_6,GPIO_PIN_SET);
						HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);

					}
					else
					{
						if(Night_ADC_1 == 0) // дуга была, но свет остался 
						{
							/*HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9,GPIO_PIN_SET);
								HAL_GPIO_WritePin(GPIOE,GPIO_PIN_8,GPIO_PIN_SET);
								HAL_GPIO_WritePin(GPIOE,GPIO_PIN_7,GPIO_PIN_RESET);*/
							
								HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,GPIO_PIN_RESET);
								HAL_GPIO_WritePin(GPIOE,GPIO_PIN_6,GPIO_PIN_SET);
								HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);

						}
							else
								{ // дуга была, свет был, после чего свет пропал
								/*HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9,GPIO_PIN_RESET);
									HAL_GPIO_WritePin(GPIOE,GPIO_PIN_8,GPIO_PIN_SET);
									HAL_GPIO_WritePin(GPIOE,GPIO_PIN_7,GPIO_PIN_SET);*/
								
									HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,GPIO_PIN_SET);
									HAL_GPIO_WritePin(GPIOE,GPIO_PIN_6,GPIO_PIN_SET);
									HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);

							}
						}
					}
				}
				if(Channel_2 == 1) //второй канал 
			{
				malfunction_2++;
				if(ARC_2 == 1) //свет был 
				{
					if(Light_ADC_2 == 0) //была дуга, после чего свет пропал 
					{
						HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);
						HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_SET);
						HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_RESET);
					}
					else
					{
						if(Night_ADC_2 == 0) //дуга была, после чего свет остался 
						{
								HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);
								HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_SET);
								HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_SET);
						}
						else //дуга была,после чего свет остался, а потом свет пропал 
						{
							HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);
							HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_SET);
							HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_RESET);
						}
					}
				}
			}
			if(Channel_3 == 1) //третий канал 
			{
				malfunction_3++;
				if(ARC_3 == 1) //был свет 
				{
					if(Light_ADC_3 == 0) //была дуга, после чего свет пропал 
					{
					/*HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,GPIO_PIN_SET);
						HAL_GPIO_WritePin(GPIOE,GPIO_PIN_6,GPIO_PIN_SET);
						HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);*/

						HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9,GPIO_PIN_RESET);
						HAL_GPIO_WritePin(GPIOE,GPIO_PIN_8,GPIO_PIN_SET);
						HAL_GPIO_WritePin(GPIOE,GPIO_PIN_7,GPIO_PIN_SET);

					}
					else
					{
						if(Night_ADC_3 == 0) //была дуга, после чего свет остался
						{
							/*HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,GPIO_PIN_RESET);
								HAL_GPIO_WritePin(GPIOE,GPIO_PIN_6,GPIO_PIN_SET);
								HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);*/

								HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9,GPIO_PIN_SET);
								HAL_GPIO_WritePin(GPIOE,GPIO_PIN_8,GPIO_PIN_SET);
								HAL_GPIO_WritePin(GPIOE,GPIO_PIN_7,GPIO_PIN_RESET);

						}
						else //была дуга, после чего свет остался, а потом свет пропал 
						{
								/*HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,GPIO_PIN_SET);
									HAL_GPIO_WritePin(GPIOE,GPIO_PIN_6,GPIO_PIN_SET);
									HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);*/
							
									HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9,GPIO_PIN_RESET);
									HAL_GPIO_WritePin(GPIOE,GPIO_PIN_8,GPIO_PIN_SET);
									HAL_GPIO_WritePin(GPIOE,GPIO_PIN_7,GPIO_PIN_SET);

								}
							}
						}
					}
				}
		
		//---------------------------------------------------------------------------------------------------------------
		//ЧТОБЫ МЕГАТЬ СВЕТОДИОДОМ НЕИСПРАВНОСТИ		
		//---------------------------------------------------------------------------------------------------------------
							
		if(malfunction_1 == 0xFFFF||malfunction_2==0xFFFF||malfunction_3==0xFFFF) //чтобы мегал светодиод неисправности 
		{
			malfunction_1=0;
			malfunction_2=0;
			malfunction_3=0;
			HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_1);
		}
		
		//---------------------------------------------------------------------------------------------------------------
		//ЧТОБЫ МЕГАТЬ СВЕТОДИОДОМ ДИСКРЕТНЫЕ ВХОДЫ ЗАБЛАКИРОВАННЫ		
		//---------------------------------------------------------------------------------------------------------------
		
		if(Banning_exits == 1)
		{
			malfunction_4++;
		}
		else
		{
			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_0,GPIO_PIN_RESET);
		}
		if(malfunction_4 == 0xFFFF)
		{
			malfunction_4=0;
			HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_0);
		}
		
		//---------------------------------------------------------------------------------------------------------------
		//ЕСЛИ БЫЛ ВВЕДЕН КОД, ТО МЫ ЗАПИСЫВАЕМ ЗНАЧЕНИЕ НОВЫХ УСТАВОК И РЕЖИМ РАБОТЫ ДЗ В ЭНЕРГОНЕЗАВИСИМУЮ ПАМЯТЬ
		//---------------------------------------------------------------------------------------------------------------
		
		if(usSRegHoldBuf[299] == 123) 
		{
			int schet=0;
			const float setpoint_1[8] = {usSRegHoldBuf[280],usSRegHoldBuf[281],usSRegHoldBuf[282],usSRegHoldBuf[283],usSRegHoldBuf[284],
			usSRegHoldBuf[285],usSRegHoldBuf[286],usSRegHoldBuf[287]};
			const float setpoint_2[6] = {usSRegHoldBuf[288],usSRegHoldBuf[289],usSRegHoldBuf[290],usSRegHoldBuf[274],usSRegHoldBuf[275],usSRegHoldBuf[276]};

			float fata_1[8],fata_2[8],fata_3[1];
			uint16_t devAddr = (0x50 << 1);
			uint16_t memAddrsetpoint = 0x0800;
			
			HAL_StatusTypeDef status;

			HAL_I2C_Mem_Write(&hi2c1, devAddr, memAddrsetpoint, I2C_MEMADD_SIZE_16BIT,
        (uint8_t*)setpoint_1, sizeof(setpoint_1), HAL_MAX_DELAY);
			
					for(;;) { // wait...
					status = HAL_I2C_IsDeviceReady(&hi2c1, devAddr, 1,
                                       HAL_MAX_DELAY);
					if(status == HAL_OK)
							break;
				}
					
				HAL_I2C_Mem_Read(&hi2c1, devAddr, memAddrsetpoint, I2C_MEMADD_SIZE_16BIT,
					(uint8_t*)fata_1, sizeof(fata_1), HAL_MAX_DELAY);

						memAddrsetpoint+=0x0020;
				
				HAL_I2C_Mem_Write(&hi2c1, devAddr, memAddrsetpoint, I2C_MEMADD_SIZE_16BIT,
        (uint8_t*)setpoint_2, sizeof(setpoint_2), HAL_MAX_DELAY);
					
			for(;;) { // wait...
					status = HAL_I2C_IsDeviceReady(&hi2c1, devAddr, 1,
                                       HAL_MAX_DELAY);
					if(status == HAL_OK)
							break;
				}
			
				HAL_I2C_Mem_Read(&hi2c1, devAddr, memAddrsetpoint, I2C_MEMADD_SIZE_16BIT,
					(uint8_t*)fata_2, sizeof(fata_2), HAL_MAX_DELAY);
			 
				memAddrsetpoint = 0x0800;
				
				usSRegHoldBuf[299]=0;
		
		}
		
		//---------------------------------------------------------------------------------------------------------------
		//ЕСЛИ БЫЛ ВВЕДЕН КОД, ТО МЫ УСТАНАВЛИВАЕМ ВРЕМЯ
		//---------------------------------------------------------------------------------------------------------------

		if(usSRegHoldBuf[298] == 123)
		{
			rtc.hours = usSRegHoldBuf[291]; 
			rtc.minuts = usSRegHoldBuf[292]; 
			rtc.seconds = usSRegHoldBuf[293]; 
			rtc.month = usSRegHoldBuf[294]; 
			rtc.date = usSRegHoldBuf[295]; 
			rtc.years = usSRegHoldBuf[296]; 
			
			rtc.nc = 1;

			/** Initialize RTC and set the Time and Date 
			*/

			usSRegHoldBuf[298]=0;
			/* USER CODE BEGIN RTC_Init 2 */
		}
		
		//---------------------------------------------------------------------------------------------------------------
		//СБРОС НАСТРОЕК НА НАСТРОЙКИ ПО УМОЛЧАНИЮ
		//---------------------------------------------------------------------------------------------------------------
		
		if(usSRegHoldBuf[297] == 123) 
		{	
			reset = 1;
			Rules();
			reset = 0;
			usSRegHoldBuf[297]=0;
		}
		
		//---------------------------------------------------------------------------------------------------------------
		//КАЛИБРОВКА УСТАВОК 
		//---------------------------------------------------------------------------------------------------------------
		
		if(usSRegHoldBuf[273] == 123)
		{
			//if(usSRegHoldBuf[260]<=0x96)
			if(usSRegHoldBuf[260]<0xC8)
				usSRegHoldBuf[274] =	Z1 = 0x96 - usSRegHoldBuf[260];
			if(usSRegHoldBuf[261]<0xC8)
				usSRegHoldBuf[275] =	Z2 = 0x96 - usSRegHoldBuf[261];
			if(usSRegHoldBuf[262]<0xC8)	
				usSRegHoldBuf[276] =	Z3 = 0x96 - usSRegHoldBuf[262];
					
			usSRegHoldBuf[273] = 0;
		}
		
		//---------------------------------------------------------------------------------------------------------------
		//СЧЕТЧИК ДЛЯ ОБНОВЛЕНИЯ ВРЕМЕНИ 
		//---------------------------------------------------------------------------------------------------------------
		
		if(cntReadRTC++ > 100) //счетчик для обнавления времени 
		{
			cntReadRTC = 0;
		}
		
		//---------------------------------------------------------------------------------------------------------------
		//УСТАНОВКА ВРЕМЕНИ 
		//---------------------------------------------------------------------------------------------------------------
		
		if(rtc.nc) //установка времени 
		{
			rtc.nc = 0;
			setRTC(&rtc);
		}
		else if(cntReadRTC == 1)	//чтения для потоснного обновления и не пропадания времени 
		{
			readRTC(&rtc);
		}

		//---------------------------------------------------------------------------------------------------------------
		//ЕСЛИН НАПРЯЖЕННИЕ ПИТАНИЯ БУДЕТ МЕНЬШЕ 4,5 В, мы запрещаем работу дискретных выходов 
		//---------------------------------------------------------------------------------------------------------------
		
		/*if(value[3] < 2800)
		{
			Banning_exits = 1;
		}
		else if(Banning_exits != 1)
		{
			Banning_exits = 0;
		}*/
	}
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV5;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.Prediv1Source = RCC_PREDIV1_SOURCE_PLL2;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL2.PLL2State = RCC_PLL2_ON;
  RCC_OscInitStruct.PLL2.PLL2MUL = RCC_PLL2_MUL8;
  RCC_OscInitStruct.PLL2.HSEPrediv2Value = RCC_HSE_PREDIV2_DIV5;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

    /**Configure the Systick interrupt time 
    */
  __HAL_RCC_PLLI2S_ENABLE();

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* RTC init function */
static void MX_RTC_Init(void)
{

  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef DateToUpdate;

    /**Initialize RTC Only 
    */
  hrtc.Instance = RTC;
  if(HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1) != 0x32F2){
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initialize RTC and set the Time and Date 
    */
  /*sTime.Hours = 1;
  sTime.Minutes = 0;
  sTime.Seconds = 0;

  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  DateToUpdate.WeekDay = RTC_WEEKDAY_MONDAY;
  DateToUpdate.Month = RTC_MONTH_JANUARY;
  DateToUpdate.Date = 1;
  DateToUpdate.Year = 0;

  if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BIN) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    HAL_RTCEx_BKUPWrite(&hrtc,RTC_BKP_DR1,0x32F2);*/
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM6 init function */
static void MX_TIM6_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 720;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 5;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM7 init function */
static void MX_TIM7_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 720;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 50;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 38400;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_6 
                          |GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5|GPIO_PIN_7|GPIO_PIN_9|GPIO_PIN_0 
                          |GPIO_PIN_1, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_0|GPIO_PIN_3|GPIO_PIN_6 
                          |GPIO_PIN_7|GPIO_PIN_8, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE2 PE3 PE4 PE5 
                           PE6 PE7 PE8 PE9 
                           PE0 PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9 
                          |GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PC13 PC0 PC2 PC3 
                           PC6 PC7 PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_3 
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PE10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */


//---------------------------------------------------------------------------------------------------------------
//СЧИТЫВАНИЕ ЗНАЧЕНИЙ АЦП
//---------------------------------------------------------------------------------------------------------------
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
   if(hadc->Instance == ADC1)
   {

		HAL_ADC_Stop_DMA(&hadc1);

		usSRegHoldBuf[260] = value[0]=(((uint16_t)adc[0]*9900)/(4026*45));
				
		usSRegHoldBuf[261] = value[1]=(((uint16_t)adc[1]*9900)/(4026*45));
			
		usSRegHoldBuf[262] = value[2]=(((uint16_t)adc[2]*9900)/(4026*45));
			
		//value[3]=(((uint16_t)adc[3]*9900)/(4026*45));
		value[3]=(uint16_t)adc[3];
			
		usSRegHoldBuf[260]	+=	Z1;
		usSRegHoldBuf[261]	+=	Z2;
		usSRegHoldBuf[262]	+=	Z3;				
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == TIM7) //прерывание происходит по истечению 1мс (счетчик считает 1мс после чего случаеться это прерывание)
  {
	/*	 
	 if(cnt30_rez == 0)
		 cnt30++;
		 
		 if(cnt30==500)
			 cnt30_rez = 1;
		 
	if(cnt30_rez==1)*/
		 
		//---------------------------------------------------------------------------------------------------------------
		//ПРОВЕРКА НА ЗАПРОС ПЕРЕПРОГРАММИРОВАНИЯ УСТРОЙТВА 
		//---------------------------------------------------------------------------------------------------------------
		 
		checkBoot();

		if(PWR->CSR != 0x0)
			HAL_NVIC_SystemReset();
		
		HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_2); //меняем значение ножки для получения сигнала разрешения 

 		 cnt++;
		 cnt40++;
		
		 if(cnt40 == 2)
		 {
			 cnt40 =0;
			 Disc_current(); //функция для получения значений дискретных входов 
		 }
		 
		 if(cnt>=0xC8) //запись новых значения адреса EEPROM куда будут записываться ошибки о произощедщих событиях (bilo 0x64)
		 {
			cnt=0;
			if(address_of_the_last_element[1] != Addres[1])
			{
				address_of_the_last_element[0]=memAddr;
				address_of_the_last_element[1]=Addres[1];
				HAL_I2C_Mem_Write(&hi2c1, devAddr, memaddress_of_the, I2C_MEMADD_SIZE_16BIT,
					(uint8_t*)address_of_the_last_element, sizeof(address_of_the_last_element), HAL_MAX_DELAY);
			}
		 }
		
		
//*		if(cnt30_rez == 1)
//		{
		if(S1 < 0x7D0) //усллвия выбора режима работы ДЗ - тестовый режим или режим определения дуги (3e8)
		{
			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_4,GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_SET);

			S1++;
			Opto_test = 0;
			if(S1>1)
			{
				ARC_test = 1; //флаг стоит на протежении 1с
			}	
		}
		else
		{
				HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE,GPIO_PIN_4,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_RESET);
			
				Opto_test = 1; //флаг стоит на протяжении 1мс
				ARC_test = 0;
				S1 = 0;
		}
	//}
	 }
	 if(htim->Instance == TIM6) //прерывание происходит по истечению 40мкс (счетчик досчитывает и вызывает прерывание)
	{
	//	if(cnt30_rez == 1)
//		{
		HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adc, 4); //запуск опроса о значениях АЦП по DMA
			  
		if(Opto_test == 1)
		{
			if(reload == 0)
				Test_Opto(); //тестовый режим
		}
		if(ARC_test == 1)
		{
			if(reload == 0)
				Test_Arc(); //режим искания дуги 
		}
	}
//}
	
	if(htim->Instance == TIM3) //вызываеться при работе с RS-485
	{
		/*if(in==1)
		{
			change_point = 1;
			HAL_TIM_Base_Stop(&htim6);
			HAL_TIM_Base_Stop(&htim7);
			HAL_ADC_Stop_DMA(&hadc1);
		}
		in=1;*/
		#if MB_MASTER_RTU_ENABLED > 0 || MB_MASTER_ASCII_ENABLED > 0   
			(void) pxMBMasterPortCBTimerExpired();
		#endif
		#if MB_SLAVE_RTU_ENABLED>0 || MB_SLAVE_ASCII_ENABLED>0   
			(void )pxMBPortCBTimerExpired(  );
		#endif
		//HAL_TIM_Base_Start_IT(&htim6);
		//HAL_TIM_Base_Start_IT(&htim7);

	}
}
//---------------------------------------------------------------------
//Прерывания которое вызывается при уменьшении напряжения питания
//---------------------------------------------------------------------
/*
void HAL_PWR_PVDCallback(void)
{
	HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_1);
	HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_0);
}
*/
void Rules() //Фуникция для записи уставок из памяти при перезагрузки и первом запуске 
{
	
		float fata_1[8],fata_2[8];

		uint16_t devAddr = (0x50 << 1);
		uint16_t memAddrsetpoint = 0x0800;
		uint16_t memaddress_of_the = 0x0900;
    // HAL expects address to be shifted one bit to the left
    HAL_StatusTypeDef status;

    // Hint: try to comment this line
		
		for(memAddrsetpoint;memAddrsetpoint<0x0840;memAddrsetpoint+=0x0020)
		{
			if(i == 0)
				HAL_I2C_Mem_Read(&hi2c1, devAddr, memAddrsetpoint, I2C_MEMADD_SIZE_16BIT,
					(uint8_t*)fata_1, sizeof(fata_1), HAL_MAX_DELAY);
			else
				HAL_I2C_Mem_Read(&hi2c1, devAddr, memAddrsetpoint, I2C_MEMADD_SIZE_16BIT,
					(uint8_t*)fata_2, sizeof(fata_2), HAL_MAX_DELAY);
			i++;
		}
		i=0;
		memAddrsetpoint = 0x0800;
		if(reset == 1)
			fata_2[2] = 0;
		if(fata_2[2] == 0)
		{
			usSRegHoldBuf[280] =	40; //уставки
			usSRegHoldBuf[281] =	50;
			usSRegHoldBuf[282] =	40;
			usSRegHoldBuf[283] =	50;
			usSRegHoldBuf[284] =	40;
			usSRegHoldBuf[285] =	50; //мб 150
			usSRegHoldBuf[286] =	135;
			usSRegHoldBuf[287] =	135;
			usSRegHoldBuf[288] =	135;
			usSRegHoldBuf[289] =	1; //пуск по току
			usSRegHoldBuf[290] =  1;	//условие того что уставки по умолчанию больше не запишуться
			
			Z1 = usSRegHoldBuf[274] = 0;
			Z2 = usSRegHoldBuf[275] = 0;
			Z3 = usSRegHoldBuf[276] = 0;
			const float setpoint_1[8] = {usSRegHoldBuf[280],usSRegHoldBuf[281],usSRegHoldBuf[282],usSRegHoldBuf[283],usSRegHoldBuf[284],
			usSRegHoldBuf[285],usSRegHoldBuf[286],usSRegHoldBuf[287]};
			const float setpoint_2[8] = {usSRegHoldBuf[288],usSRegHoldBuf[289],usSRegHoldBuf[290],usSRegHoldBuf[274],usSRegHoldBuf[275],usSRegHoldBuf[276]};
			
			HAL_I2C_Mem_Write(&hi2c1, devAddr, memAddrsetpoint, I2C_MEMADD_SIZE_16BIT,
        (uint8_t*)setpoint_1, sizeof(setpoint_1), HAL_MAX_DELAY);
			
					for(;;) { // wait...
					status = HAL_I2C_IsDeviceReady(&hi2c1, devAddr, 1,
                                       HAL_MAX_DELAY);
					if(status == HAL_OK)
							break;
				}
				
			memAddrsetpoint+=0x0020;
				
			HAL_I2C_Mem_Write(&hi2c1, devAddr, memAddrsetpoint, I2C_MEMADD_SIZE_16BIT,
        (uint8_t*)setpoint_2, sizeof(setpoint_2), HAL_MAX_DELAY);
					
			for(;;) { // wait...
					status = HAL_I2C_IsDeviceReady(&hi2c1, devAddr, 1,
                                       HAL_MAX_DELAY);
					if(status == HAL_OK)
							break;
				}
			memAddrsetpoint+=0;
		}		
		else
			{	
				usSRegHoldBuf[280] =	fata_1[0]; //уставки
				usSRegHoldBuf[281] =	fata_1[1];
				usSRegHoldBuf[282] =	fata_1[2];
				usSRegHoldBuf[283] =	fata_1[3];
				usSRegHoldBuf[284] =	fata_1[4];
				usSRegHoldBuf[285] =	fata_1[5];
				usSRegHoldBuf[286] =	fata_1[6];
				usSRegHoldBuf[287] =	fata_1[7];
				usSRegHoldBuf[288] =	fata_2[0];
				usSRegHoldBuf[289] =	fata_2[1]; //пуск по току
				usSRegHoldBuf[290] =   1;
				Z1 = usSRegHoldBuf[274] = fata_2[3];
				Z2 = usSRegHoldBuf[275] = fata_2[4];
				Z3 = usSRegHoldBuf[276] = fata_2[5];
			}
			
			
			// указываем начальный адрес с какого заполнять историю ошибок 	
		HAL_I2C_Mem_Read(&hi2c1, devAddr, memaddress_of_the, I2C_MEMADD_SIZE_16BIT,
        (uint8_t*)Addres, sizeof(Addres), HAL_MAX_DELAY);
					
			if(reset == 1)
				Addres[1] = 0;
			if(Addres[1]>=1) //запись с какого адреса начать записывать ошибки
		{
			memAddr = Addres[0];
		}
		else
		{
			memAddr = 0x0100;
		}
		address_of_the_last_element[0]=memAddr;
		address_of_the_last_element[1]=Addres[1];
}


void Record() //функция для записи ошибок
{
		const float wmsg[] = {N,reason,Data[2],Data[1],Data[0],Data[3],Data[5],Data[4]};

    float rmsg[8];
		int rmsg_1[2];
    // HAL expects address to be shifted one bit to the left
    uint16_t devAddr = (0x50 << 1);
		uint16_t memaddress_of_the = 0x0900;
		Addres[1]++;
		if(Addres[1]>1) // запись нового адреса куда записывать ошибки
		{
			memAddr+=sizeof(wmsg);
		}
		if(Addres[1]==0x1F)
		{
			Addres[1] = 1;
			memAddr = 0x0100;
		}
    HAL_StatusTypeDef status;
		
				
    // Hint: try to comment this line
    HAL_I2C_Mem_Write(&hi2c1, devAddr, memAddr, I2C_MEMADD_SIZE_16BIT,
        (uint8_t*)wmsg, sizeof(wmsg), HAL_MAX_DELAY);

   
		if(Addres[1]>=1) //увеличиваем значения куда запишим в адресную карту Modbus
		{
			for(i=0;i<Addres[1];i++)
			{
				quantity+=0x8;
			}
			i=0;
			for(i=1;i<Addres[1];i++)
			{
				variable+=0x8;
			}
			i=0;
		}
		for(variable;variable < quantity; variable++)
		{
			usSRegHoldBuf[variable]= wmsg[j];
			j++;
		}
		rmsg[0] = 0;rmsg[1] = 0;rmsg[2] = 0;rmsg[3] = 0; rmsg[4] = 0; rmsg[5] = 0; rmsg[6] = 0; rmsg[7] = 0;
		j=0;i=0;variable = 0;quantity = 0;
}
void Time() //получения времени и дальнейшей записи в память EEPROM 
{
		HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN); 
		HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
	

		Data[0] = sTime.Seconds;//RTC_Bcd2ToByte(sTime.Seconds);
		Data[1] = sTime.Minutes;//RTC_Bcd2ToByte(sTime.Minutes);
		Data[2] = sTime.Hours;//RTC_Bcd2ToByte(sTime.Hours);
	
		Data[3] = sDate.Date;//RTC_Bcd2ToByte(sDate.Date);
		Data[4] = sDate.Year;//RTC_Bcd2ToByte(sDate.Year);
		Data[5] = sDate.Month;//RTC_Bcd2ToByte(sDate.Month);
				
		Record();
}

void Read_Memory() //функция которая считывает всю память 
{
		float rmsg[8];
    // HAL expects address to be shifted one bit to the left
    uint16_t devAddr = (0x50 << 1);
		uint16_t memaddress_of_the_memory = 0x0100;
		
		HAL_StatusTypeDef status;
		
		 // Hint: try to comment this line
	
		for(memaddress_of_the_memory;memaddress_of_the_memory<500;memaddress_of_the_memory+=0x0020)
		{
			HAL_I2C_Mem_Read(&hi2c1, devAddr, memaddress_of_the_memory, I2C_MEMADD_SIZE_16BIT,
        (uint8_t*)rmsg, sizeof(rmsg), HAL_MAX_DELAY);
			
			for(variable;variable < quantity; variable++)
			{
				usSRegHoldBuf[variable]= rmsg[j];
				j++;
			}
			j=0;
			quantity+=0x8;
		}
		variable = 0;
		quantity = 0x8;
}

void readRTC(srtc *rtc) // функция чтения времени и записи его в регистр энерго независимой памяти контроллера 
{	
		  HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
		  HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
			rtc->seconds =  sTime.Seconds;
			rtc->minuts =  sTime.Minutes;
			rtc->hours =  sTime.Hours;
			rtc->date =  sDate.Date;
			rtc->month =  sDate.Month;
			rtc->years =  sDate.Year;
	
	 BKP->DR1 = (sDate.Date << 8) | (sDate.WeekDay);
	 BKP->DR2 = (sDate.Year << 8) | (sDate.Month);
}

void setRTC(srtc *rtc) //установка времени
{	 
			sTime.Seconds = rtc->seconds;
			sTime.Minutes = rtc->minuts;
			sTime.Hours = rtc->hours;
			sDate.Date = rtc->date;
			sDate.Month = rtc->month;
			sDate.Year = rtc->years;
		  HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
		  HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
}

/*
static uint8_t RTC_Bcd2ToByte(uint8_t Value) //функция не нужна но переводит из bcd в bin
{
  uint32_t tmp = 0;
  tmp = ((uint8_t)(Value & (uint8_t)0xF0) >> (uint8_t)0x4) * 10;
  return (tmp + (Value & (uint8_t)0x0F));
}*/

void Test_Opto()
{
	
				cnt_1++;
				// первый канал
				if(min_ok!=1)
				{
					if(cnt_1<20)
					{
						if((usSRegHoldBuf[260]	- Z1)<h)
							h = usSRegHoldBuf[260]	- Z1;
					}
					else{
						if(cnt_1==20)
							usSRegHoldBuf[270]=h;
							h = 500;
					}
			
					if(cnt_1<20)
					{
						if((usSRegHoldBuf[261] - Z2)<o)
							o = usSRegHoldBuf[261]	- Z2;
					}
					else{
						if(cnt_1==20)
							usSRegHoldBuf[271]=o;
							o = 500;
					}
			
					if(cnt_1<20)
					{
						if((usSRegHoldBuf[262] - Z3)<t)
							t = usSRegHoldBuf[262] - Z3;
					}
					else{
						if(cnt_1==20)
							usSRegHoldBuf[272]=t;
						t = 500;
						cnt_1 = 0;
						min_ok = 1;
					}
				}
				
				ADC_Test_1_1=ADC_Test_1_1<<1; //сдвиг влево 
				ADC_Test_1_2=ADC_Test_1_2<<1; //сдвиг влево
				if(Sensor_Disconection_1 == 0) // условие того что датчик подключен
				{
					if((usSRegHoldBuf[260] - Z1)>usSRegHoldBuf[280]) // проверка на неисправность в оптическом канале 
					{
						ADC_Test_1_2=ADC_Test_1_2|0x0001;
					}
					else
					{
						ADC_Test_1_1=ADC_Test_1_1&0xFFFE;
						ADC_Test_1_2=ADC_Test_1_2&0xFFFE;					
					}
				}
				if((usSRegHoldBuf[260]	-	Z1)>usSRegHoldBuf[281])	// проверка подключен ли датчик или нет
				{
					ADC_Test_1_1=ADC_Test_1_1|0x0001;
					ADC_Test_1_2=ADC_Test_1_2&0xFFFE;
				}
				else
					ADC_Test_1_1=ADC_Test_1_1&0xFFFE;
					
			  Transition_1_1=ADC_Test_1_1;
				Transition_1_2=ADC_Test_1_2;
				
				Pipe_1_1 = 0;
				Pipe_1_2 = 0;	
				for(i=0; i<8; i++) { // Считаем колличество единиц если единиц будет определенное количество значит есть нарушение целосности или датчик не подключен5
					c = Transition_1_1&1;
					if(c==1)
					{
						if(Pipe_1_1 < 8)
							Pipe_1_1++;
					}
					else
					{
						if(Pipe_1_1 > 0)
							Pipe_1_1--;
					}
					Transition_1_1 >>= 1;
				}
				
				for(i=0; i<8; i++) {
					c = Transition_1_2&1;
					if(c==1)
						{
						if(Pipe_1_2 < 8)
							Pipe_1_2++;
						}
					else
					{
						if(Pipe_1_2 > 0)
							Pipe_1_2--;
					}
					Transition_1_2 >>= 1;
				}
				
				// второй канал 
				ADC_Test_2_1=ADC_Test_2_1<<1; 
				ADC_Test_2_2=ADC_Test_2_2<<1;
				if(Sensor_Disconection_2 == 0)
				{
					if((usSRegHoldBuf[261] - Z2)>usSRegHoldBuf[282]) 
					{
						ADC_Test_2_2=ADC_Test_2_2|0x0001;
					}
					else
					{
						ADC_Test_2_1=ADC_Test_2_1&0xFFFE;
						ADC_Test_2_2=ADC_Test_2_2&0xFFFE;
					}
				}
				if((usSRegHoldBuf[261] - Z2)>usSRegHoldBuf[283])
				{
					ADC_Test_2_1=ADC_Test_2_1|0x0001;
					ADC_Test_2_2=ADC_Test_2_2&0xFFFE;
				}
				else
					ADC_Test_2_1=ADC_Test_2_1&0xFFFE;
				
			  Transition_2_1=ADC_Test_2_1;
				Transition_2_2=ADC_Test_2_2;
				Pipe_2_1 = 0;
				Pipe_2_2 = 0;
				
				for(i=0; i<8; i++) {
					c = Transition_2_1&1;
					if(c==1)
					{
						if(Pipe_2_1 < 8)
							Pipe_2_1++;
					}
					else
					{
						if(Pipe_2_1 > 0)
							Pipe_2_1--;
					}
					Transition_2_1 >>= 1;
				}
				
				for(i=0; i<8; i++) {
					c = Transition_2_2&1;
					if(c==1)
					{
						if(Pipe_2_2 < 8)
							Pipe_2_2++;
					}
					else
					{
						if(Pipe_2_2 > 0)
							Pipe_2_2--;
					}
					Transition_2_2 >>= 1;
				}
				
				// третий канал
				ADC_Test_3_1=ADC_Test_3_1<<1; 
				ADC_Test_3_2=ADC_Test_3_2<<1;
				if(Sensor_Disconection_3 == 0)
				{
					if((usSRegHoldBuf[262]	-	Z3)>usSRegHoldBuf[284]) 
					{
						ADC_Test_3_2=ADC_Test_3_2|0x0001;
					}
					else
					{
						ADC_Test_3_1=ADC_Test_3_1&0xFFFE;
						ADC_Test_3_2=ADC_Test_3_2&0xFFFE;
					}
				}
				
				if((usSRegHoldBuf[262] - Z3)>usSRegHoldBuf[285])
					{
						ADC_Test_3_1=ADC_Test_3_1|0x0001;
						ADC_Test_3_2=ADC_Test_3_2&0xFFFE;
					}
					else
						ADC_Test_3_1=ADC_Test_3_1&0xFFFE;
					
			  Transition_3_1=ADC_Test_3_1;
				Transition_3_2=ADC_Test_3_2;
				
				Pipe_3_1 = 0;
				Pipe_3_2 = 0;
				for(i=0; i<8; i++) {
					c = Transition_3_1&1;
					if(c==1)
					{
						if(Pipe_3_1 < 8)
							Pipe_3_1++;
					}
					else
					{
						if(Pipe_3_1 > 0)
							Pipe_3_1--;
					}
					Transition_3_1 >>= 1;
				}
				
				for(i=0; i<8; i++) {
					c = Transition_3_2&1;
					if(c==1)
					{
						if(Pipe_3_2 < 8)
							Pipe_3_2++;
					}
					else
					{
						if(Pipe_3_2 > 0)
							Pipe_3_2--;
					}
					Transition_3_2 >>= 1;
				}
				if(ARC_1==0) //условие того что дуги по этому каналу небыло 
					{
						if(Sensor_Disconection_1 == 0) // датчик подключен
						{
							if(Pipe_1_2 >=6) // если единиц больше или равно 6 значит событие произошло
							{
								if(Test_error_1 == 0) // чтобы запись ошибки о плохой пропускной способности записалась 1 раз
								{
									Test_error_1 = 1;
									N=1; //номер канал
									reason = 2; //номер ошибки
									HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,GPIO_PIN_SET);
									Time();	//запись о неисправности с определением даты и времени в энергонезависимую память 
								}
							}
						}
					}
					if(Pipe_1_1 >= 6) //проверка подключен датчик или нет
					{
						Sensor_Disconection_1 = 1;
						HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,GPIO_PIN_SET);
						HAL_GPIO_WritePin(GPIOE,GPIO_PIN_6,GPIO_PIN_SET);
						HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);
				/*		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9,GPIO_PIN_SET);
						HAL_GPIO_WritePin(GPIOE,GPIO_PIN_8,GPIO_PIN_SET);
						HAL_GPIO_WritePin(GPIOE,GPIO_PIN_7,GPIO_PIN_SET);*/

					}
					else //на фотодиод приходит свет
					{
						Sensor_Disconection_1 = 0;
					/*	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9,GPIO_PIN_SET);
						HAL_GPIO_WritePin(GPIOE,GPIO_PIN_8,GPIO_PIN_RESET);
						HAL_GPIO_WritePin(GPIOE,GPIO_PIN_7,GPIO_PIN_SET);*/
						
						HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,GPIO_PIN_SET);
						HAL_GPIO_WritePin(GPIOE,GPIO_PIN_6,GPIO_PIN_RESET);
						HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);
						
					}
					//второй канал
					if(ARC_2==0)
					{
						if(Sensor_Disconection_2 == 0)
						{
							if(Pipe_2_2 >=6)
							{
								if(Test_error_2 == 0)
								{
									Test_error_2 = 1;
									N=2;
									reason = 2;
									HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,GPIO_PIN_SET);
									Time();	
								}
							}	
						}
					}
					if(Pipe_2_1 >= 6)
					{
						Sensor_Disconection_2 = 1;
						HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);
						HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_SET);
						HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_SET);
					}
					else
					{
						Sensor_Disconection_2 = 0;
						HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);
						HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_RESET);
						HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_SET);
					}
					
					//третий канал
					if(ARC_3==0)
					{
						if(Sensor_Disconection_3 == 0)
						{
							if(Pipe_3_2 >=6)
							{
								if(Test_error_3 == 0)
								{
									Test_error_3 = 1;
									N=3;
									reason = 2;
									HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,GPIO_PIN_SET);
									Time();	
								}
							}
						}
					}						
					if(Pipe_3_1 >= 6)
					{
						
						HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9,GPIO_PIN_SET);
						HAL_GPIO_WritePin(GPIOE,GPIO_PIN_8,GPIO_PIN_SET);
						HAL_GPIO_WritePin(GPIOE,GPIO_PIN_7,GPIO_PIN_SET);
					/*	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,GPIO_PIN_SET);
						HAL_GPIO_WritePin(GPIOE,GPIO_PIN_6,GPIO_PIN_SET);
						HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);*/
						Sensor_Disconection_3 = 1;
	
					}
					else
					{
						Sensor_Disconection_3 = 0;
						HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9,GPIO_PIN_SET);
						HAL_GPIO_WritePin(GPIOE,GPIO_PIN_8,GPIO_PIN_RESET);
						HAL_GPIO_WritePin(GPIOE,GPIO_PIN_7,GPIO_PIN_SET);
					/*HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,GPIO_PIN_SET);
						HAL_GPIO_WritePin(GPIOE,GPIO_PIN_6,GPIO_PIN_RESET);
						HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);		*/				
					}					
			}


void Test_Arc()
{
		/*
				cnt_1++;
				// первый канал
				if(min_ok!=1)
				{
					if(cnt_1<2000)
					{
						if((usSRegHoldBuf[260]	- Z1)<h)
							h = usSRegHoldBuf[260]	- Z1;
					}
					else{
						if(cnt_1==2000)
							usSRegHoldBuf[270]=h;
							h = 500;
					}
			
					if(cnt_1<2000)
					{
						if((usSRegHoldBuf[261] - Z2)<o)
							o = usSRegHoldBuf[261]	- Z2;
					}
					else{
						if(cnt_1==2000)
							usSRegHoldBuf[271]=o;
							o = 500;
					}
			
					if(cnt_1<2000)
					{
						if((usSRegHoldBuf[262] - Z3)<t)
							t = usSRegHoldBuf[262] - Z3;
					}
					else{
						if(cnt_1==2000)
							usSRegHoldBuf[272]=t;
						t = 500;
						cnt_1 = 0;
						min_ok = 1;
					}
				}
				*/
	
	
	
				min_ok=0;
				ADC_1=ADC_1<<1; //сдвиг влево
				if(usSRegHoldBuf[260]<usSRegHoldBuf[286]) // проверка на дугу  
					ADC_1 = ADC_1|0x0001;
				else
					ADC_1=ADC_1&0xFFFE;
			  Transition_1=ADC_1;
				Pipe_1 = 0;
				for(i=0; i<8; i++) {
					c = Transition_1&1;
					if(c==1)
					{
						if(Pipe_1 < 8)
							Pipe_1++;
					}
					else
					{
						if(Pipe_1 > 0)
							Pipe_1--;
					}
					Transition_1 >>= 1;
				}
				
				//второй канал
				ADC_2=ADC_2<<1; 
				if(usSRegHoldBuf[261]<usSRegHoldBuf[287]) 
					ADC_2 = ADC_2|0x0001;
				else
					ADC_2=ADC_2&0xFFFE;
				
			  Transition_2=ADC_2;
				Pipe_2 = 0;
				for(i=0; i<8; i++) {
					c = Transition_2&1;
					if(c==1)
					{
						if(Pipe_2 < 8)
							Pipe_2++;
					}
					else
					{
						if(Pipe_2 > 0)
							Pipe_2--;
					}
					Transition_2 >>= 1;
				}
				
				//третий канал
				ADC_3=ADC_3<<1; 
				if(usSRegHoldBuf[262]<usSRegHoldBuf[288]) 
					ADC_3 = ADC_3|0x0001;
				else
					ADC_3=ADC_3&0xFFFE;
				
			  Transition_3=ADC_3;
				Pipe_3 = 0;
				for(i=0; i<8; i++) {
					c = Transition_3&1;
					if(c==1)
					{
						if(Pipe_3 < 8)
							Pipe_3++;
					}
					else
					{
						if(Pipe_3 > 0)
							Pipe_3--;
					}
					Transition_3 >>= 1;
				}

				if(usSRegHoldBuf[289] == 1) //проверяем выбран ли режим пуска по току 
				{
					if(level_is_exceeded == 1) //есть сигнла на дискретном входе ток или нет
					{
						if(Pipe_1 >=6) //проверка количества единиц для определения дуги
						{
							Channel_1 = 1; //канал 1 
							light_1++;
							if(ARC_1 == 0) //чтобы в память записалось всего 1 раз
							{
								ARC_1 = 1;
								if(Banning_exits == 0)
								{
									HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_RESET);
									usSRegHoldBuf[263] = usSRegHoldBuf[260];
//								HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_SET);
								}
								HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,GPIO_PIN_SET);
							}
							arc_1 = 1;
							if(light_1 == 0xC350) //счетчик для определения засвета 
							{
								Night_ADC_1 = 0;
								dark_1 = 0;

								if(Light_ADC_1 == 0) //засвет
								{
									Light_ADC_1 = 1;
									N = 1;
									reason = 3;
									HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,GPIO_PIN_SET);
									Time();
								}
							}
						}
						else
						{
							if(arc_1 == 1)
							{
								dark_1++;
								if(dark_1 == 0xC350) //убираем засвет (светодиоды возвращаются к дуга) 
								{
									Night_ADC_1 = 1;
									light_1 = 0;
									arc_1 = 0;
								}
							}
						}	
						if(Pipe_2 >=6)
						{
							Channel_2 = 1;
							light_2++;
							if(ARC_2 == 0)
							{
								ARC_2 = 1;
								if(Banning_exits == 0)
								{
									HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);
									usSRegHoldBuf[264] = usSRegHoldBuf[261];
								}
								HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,GPIO_PIN_SET);
							}
							arc_2 = 1;
							if(light_2 == 0xC350)
							{
								Night_ADC_2 = 0;
								dark_2 = 0;
								if(Light_ADC_2 == 0)
								{
									Light_ADC_2 = 1;
									N=2;
									reason = 3;
									HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,GPIO_PIN_SET);
									Time();
								}
							}
						}
						else
						{
							if(arc_2 == 1)
							{
								dark_2++;
								if(dark_2 == 0xC350)
								{
									Night_ADC_2 = 1;
									light_2 = 0;	
									arc_2 = 0;
								}
							}
						}								
						if(Pipe_3 >=6)
						{
							Channel_3 = 1;	
							light_3++;
							if(ARC_3 == 0)
							{
								ARC_3 = 1;
								if(Banning_exits == 0)
								{
										HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_RESET);
										usSRegHoldBuf[265] = usSRegHoldBuf[262];
//									HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_SET);
								}
								HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,GPIO_PIN_SET);
							}
							arc_3 = 1;
							if(light_3 >= 0xC350)
							{
								Night_ADC_3=0;
								dark_3 = 0;
								if(Light_ADC_3 == 0)
								{
									Light_ADC_3 = 1;
									N = 3;
									reason = 3;
									HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,GPIO_PIN_SET);
									Time();
								}
							}
						}
						else
						{
							if(arc_3 == 1)
							{
								dark_3++;
								if(dark_3 >= 0xC350) //мб тут
								{
									light_3 = 0;
									Night_ADC_3 = 1;
									arc_3 = 0;
								}
							}
						}							
					}
					else
					{
						if(Pipe_1 >=6) //проверка количества единиц для определения дуги
						{
							flash_1 = 1;
						}
						else
						{
							flash_1 = 0;
						}
						if(flash_1 == 1)
						{
							Channel_1 = 1; //канал 1 
							darck_out_1 = 0;
							light_out_1++;
							if(light_out_1 == 0xC350)
							{
								if(light_without_current_1 == 0) //засвет
								{
									light_without_current_1 = 1;
									N = 1;
									reason = 3;
									HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,GPIO_PIN_SET);
									Time();
								}
							}
						}
						else
						{
							if(light_without_current_1 == 1)
							{
								light_out_1 =0;
								darck_out_1++;
								if(darck_out_1 == 0xC350)
									light_without_current_1 = 0;
							}	
						}
						if(Pipe_2 >=6)
						{
							flash_2 = 1;
						}
						else
							flash_2 = 0;
						if(flash_2 == 1)
						{
							Channel_2 = 1;
							darck_out_2=0;
							light_out_2++;
							if(light_out_2 == 0xC350)
							{
								if(light_without_current_2 == 0)
								{
									light_without_current_2 = 1;
									N=2;
									reason = 3;
									HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,GPIO_PIN_SET);
									Time();
								}
							}
						}
						else
						{
							if(light_without_current_2 == 1)
							{
								light_out_2 =0;
								darck_out_2++;
								if(darck_out_2 == 0xC350)
									light_without_current_2 = 0;
							}	
						}
						if(Pipe_3 >=6)
						{
							flash_3 = 1;
						}
						else
							flash_3 = 0;
						if(flash_3 == 1)
						{
							Channel_3 = 1;	
							darck_out_3 = 0;
							light_out_3++;
							if(light_out_3 >= 0xC350)
							{
								if(light_without_current_3 == 0)
								{
									light_without_current_3 = 1;
									N = 3;
									reason = 3;
									HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,GPIO_PIN_SET);
									Time();
								}
							}
						}	
						else
						{
							if(light_without_current_3 == 1)
							{
								light_out_3 =0;
								darck_out_3++;
								if(darck_out_3 == 0xC350)
									light_without_current_3 = 0;
							}	
						}
					}
					if(ARC_1 == 1)
					{
						if(P_T_1_1 == 0)
							if(processing_time1_1++ >=0x3E8)
							{
								N=1;
								reason = 1;
								Time();
								processing_time1_1 = 0;
								P_T_1_1 = 1;
							}
					}
					if(ARC_2 == 1)
					{
						if(P_T_2_1 == 0)
							if(processing_time2_1++ >=0x3E8)
							{
								N=2;
								reason = 1;
								Time();
								processing_time2_1 = 0;
								P_T_2_1 = 1;
							}
					}
					if(ARC_3 == 1)
					{
						if(P_T_3_1 == 0)
							if(processing_time3_1++ >=0x3E8)
							{
								N=3;
								reason = 1;
								Time();
								processing_time3_1 = 0;
								P_T_3_1 = 1;
							}
					}
				}
				else //режим пуска по току не выбран 
				{
					//первый канал 
					if(Pipe_1 >= 6)
					{
						Channel_1 = 1;
						light_1++; //смотрим остаеться ли свет на протяжение определенного времени 
						if(ARC_1 == 0) //дуга
						{
							ARC_1=1;
							if(Banning_exits == 0)
							{
									HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_RESET);
//								HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_SET);
							}	
							HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,GPIO_PIN_SET);							
						}
						arc_1 = 1;
						if(light_1 == 0xC350)
						{
							Night_ADC_1 = 0;
							dark_1 = 0;
							if(Light_ADC_1 == 0) //засвет
							{
								Light_ADC_1 = 1;
								N = 1;
								reason = 3;
								HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,GPIO_PIN_SET);
								Time();
							}
						}
					}
					else
					{
						if(arc_1 == 1)
						{
							dark_1++;
							if(dark_1 == 0xC350) //убираем засвет 
							{
								Night_ADC_1 = 1;
								light_1 = 0;
								arc_1 = 0;
							}
						}
					}	
					//второй канал
					if(Pipe_2 >= 6)
					{
						Channel_2 = 1;
						light_2++;
						if(ARC_2 == 0)
						{
							ARC_2=1;
							if(Banning_exits == 0)
							{
								HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);
							}
							HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,GPIO_PIN_SET);
						}
						arc_2 = 1;
						if(light_2 == 0xC350)
						{
							Night_ADC_2 = 0;
							dark_2 = 0;

							if(Light_ADC_2 == 0)
							{
								Light_ADC_2 = 1;
								N=2;
								reason = 3;
								HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,GPIO_PIN_SET);
								Time();
							}
						}
					}
					else
					{
						if(arc_2 == 1)
						{
							dark_2++;
							if(dark_2 == 0xC350)
							{
								Night_ADC_2 = 1;
						  	light_2 = 0;	
								arc_2 = 0;
							}
						}
					}	
					
					//третий канал
					if(Pipe_3 >= 6)
					{
						Channel_3 = 1;
						light_3++;
						if(ARC_3 == 0)
						{
							ARC_3=1;
							if(Banning_exits == 0)
							{
									HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_RESET);
//								HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_SET);
							}
							HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,GPIO_PIN_SET);
						}
						arc_3 = 1;
						if(light_3 >= 0xC350)
						{
							Night_ADC_3=0;
							dark_3 = 0;
							if(Light_ADC_3 == 0)
							{
								Light_ADC_3 = 1;
								N = 3;
								reason = 3;
								HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,GPIO_PIN_SET);
								Time();
							}
						}
					}
					else
					{
						if(arc_3 == 1)
						{
							dark_3++;
							if(dark_3 >= 0xC350) //мб тут
							{
								light_3 = 0;
                Night_ADC_3 = 1;
								arc_3 = 0;
							}
						}
					}
					if(ARC_1 == 1)
					{
						if(P_T_1_1 == 0)
							if(processing_time1_1++ >=0x3E8)
							{
								N=1;
								reason = 1;
								Time();
								processing_time1_1 = 0;
								P_T_1_1 = 1;
							}
					}
					if(ARC_2 == 1)
					{
						if(P_T_2_1 == 0)
							if(processing_time2_1++ >=0x3E8)
							{
								N=2;
								reason = 1;
								Time();
								processing_time2_1 = 0;
								P_T_2_1 = 1;
							}
					}
					if(ARC_3 == 1)
					{
						if(P_T_3_1 == 0)
							if(processing_time3_1++ >=0x3E8)
							{
								N=3;
								reason = 1;
								Time();
								processing_time3_1 = 0;
								P_T_3_1 = 1;
							}
					}
				}				
}

void Disc_current()
{
	
				//Current=Current<<1;
				if(usSRegHoldBuf[289] == 1)
				{
					/*if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_9) == 0) //условие того была ли использован этот дискретный вход или нет 
						Current=Current|0x0001;
					else
						Current=Current&0xFFFE;
					
					Transition_4=Current;
					Pipe_4=0;
					cnt15ms++;
					if(cnt15ms == 20)
					{
						for(i=0; i<15; i++) 
						{
							c = Transition_4&1;
							if(c==1)
							{
								if(Pipe_4<21)
								Pipe_4++;
							}
							//else
							//{
							//	if(Pipe_4 > 0)
							//	Pipe_4--;
							//}
							Transition_4 >>= 1;
						}
						if(Pipe_4>=6)
							level_is_exceeded = 1;
							//Current_start_up = 1;
						else
							level_is_exceeded = 0;
							//Current_start_up = 0;
						cnt15ms = 0;
					}*/
					cnt15ms++;
					if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_9) == 0) //условие того была ли использован этот дискретный вход или нет 
						Current_start_up++;
						//Current=Current|0x0001;
					if(cnt15ms == 20)
					{
							if(Current_start_up >= 6)
								level_is_exceeded = 1;
							else
								level_is_exceeded = 0;
							
							cnt15ms = 0;
							Current_start_up = 0;
					}
				}
				
			GP2=GP2<<1;
			if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_8)==0) //Обработка кнопки запрет дескретных выходов
				GP2=GP2|0x0001;
			else
				GP2=GP2&0xFFFE;
			
			Transition_5=GP2;
			Pipe_5 = 0;
			for(i=0; i<16; i++) {
					c = Transition_5&1;
					if(c==1)
					{
						if(Pipe_5<16)
						Pipe_5++;
					}
					else
					{
						if(Pipe_5 > 0)
						Pipe_5--;
					}
					Transition_5 >>= 1;
				}
		
	  	GP3=GP3<<1;
			if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_9)==0) //Обработка кнопки сброс 
				GP3=GP3|0x0001;
			else
				GP3=GP3&0xFFFE;
			
			Transition_6=GP3;
			Pipe_6 =0;
			for(i=0; i<16; i++) {
					c = Transition_6&1;
					if(c==1)
					{
						if(Pipe_6<16)
						Pipe_6++;
					}
					else
					{
						if(Pipe_6 > 0)
						Pipe_6--;
					}
					Transition_6 >>= 1;
				}

				if(Pipe_5 >= 14 || value[3] < 2800) //выходы запрешены или нет (условия выполняются при получении количества единиц = или болше 14)
				{
					Banning_exits = 1;
				}
				else
				{
					Banning_exits = 0;
				}
				
				if(Pipe_6 >= 14) // сброс, тоже самое 
				{
					if(ARC_1 == 1||ARC_2 == 1 || ARC_3 == 1|| Pipe_6>=14)
					{
					reload = 1;
					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOE,GPIO_PIN_1,GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOE,GPIO_PIN_0,GPIO_PIN_SET);
					Banning_exits = 0;
					level_is_exceeded = 0;
					Test_error_1 = 0;
					Test_error_2 = 0;
					Test_error_3 = 0;
					Channel_1 = 0;
					Channel_2 = 0;
					Channel_3 = 0;
					ARC_1=0;
					ARC_2=0;
					ARC_3=0;
					Sr_1=0;
					Sr_2=0;
					Sr_3=0;
					Light_ADC_1=0;
					Light_ADC_2=0;
					Light_ADC_3=0;
					Pipe_1_1=0;
					Pipe_1_2=0;
					Pipe_2_1=0;
					Pipe_2_2=0;
					Pipe_3_1=0;
					Pipe_3_2=0;
					Pipe_1=0;
					Pipe_2=0;
					Pipe_3=0;
					Pipe_4=0;
					Pipe_5=0;
					Pipe_6=0;
					Night_ADC_1=0;
					Night_ADC_2=0;
					Night_ADC_3=0;
					light_without_current_1 = 0;
					light_without_current_2 = 0;
					light_without_current_3 = 0;
					flash_1 = 0;
					flash_2 = 0;
					flash_3 = 0;
					P_T_1_1 = 0;
					P_T_2_1 = 0;
					P_T_3_1 = 0;

				HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOE,GPIO_PIN_6,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);	


				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_SET);	


				HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOE,GPIO_PIN_8,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOE,GPIO_PIN_7,GPIO_PIN_SET);	
			}
				}
				else
					reload = 0;
							
				/*
				if(Pipe_4 >= 6) //ТОК на входе есть или нет 
					level_is_exceeded = 1;
				else
					level_is_exceeded = 0;	*/
//	}			
}
void	checkBoot(void)
{
	if(	 masBoot[0] == 'b' && \
		 masBoot[1] == 'o' && \
		 masBoot[2] == 'o' && \
		 masBoot[3] == 't' && \
		 masBoot[4] == '1' && \
		 masBoot[5] == '0' && \
		 masBoot[6] == '7')
	 {
			BKP->DR42 = 0x55aa;
			NVIC_SystemReset();
	 }
}
void Reset (void)
{

}
/*
static void PVD_Config(void)
{
    PWR_PVDTypeDef sConfigPVD = {0,};
    sConfigPVD.PVDLevel = PWR_PVDLEVEL_7; // 2.8V
    sConfigPVD.Mode = PWR_PVD_MODE_IT_FALLING; // прерывание при падении и при превышении
    HAL_PWR_ConfigPVD(&sConfigPVD); // конфигурируем
    HAL_PWR_EnablePVD(); // активируем PVD
}*/
void Setpoints()
{
		
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
