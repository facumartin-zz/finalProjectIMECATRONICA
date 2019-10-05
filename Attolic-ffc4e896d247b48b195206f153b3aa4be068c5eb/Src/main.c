/* USER CODE BEGIN Header */
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
  * COPYRIGHT(c) 2019 STMicroelectronics
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdlib.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
#define FALSE 0
#define TRUE 1

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */
void triangle_wave(int freq, int min, int max);
void sin_wave(int A,int F);
void const_vel(int A,int F);
void Polling_UART();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
double const clock=16000000;
double const htim3_Prescaler = 1600;
int const htim3_Period = 9;
double const htim4_Prescaler = 1;
int const htim4_Period = 200;
int const pulsosporRevolucion=5000;
int const mmporRevolucion=10;

int estado=0;
int posActual=0;
int posMax=0;
int posCentral=0;
volatile int velocidades[1000];
volatile int velocidadesPulsos[1000];
volatile int posiciones[1000];
volatile int posicionesPulsos[1000];
volatile int posicionesActPulsos[1000];
volatile int periodos[1000];
// variables para comunicacion UART2
uint8_t rx_index_UART2;
_Bool OK_UART2;


uint8_t rx_buffer_UART2[50];
char bufferTX_UART2[20];
uint8_t bufferRX_UART2[50];
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	OK_UART2 = FALSE;

	rx_index_UART2 = 0;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */

  //HAL_TIM_Base_Start_IT(&htim4);
  //HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);


  //HAL_TIM_PWM_Start_IT(&htim4,TIM_CHANNEL_1);

  //HAL_TIM_Base_Start_IT(&htim3);


  char buffer[16];
  char txbuffer[10];
  HAL_UART_Transmit(&huart2, "OK ", 3, 200);
  char in[1]={'N'};
  int counter=0;
  int position=0;
  int maxposition;
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
  TIM4->ARR=1000;       //desborde de tiempo de pwm en timer 4.
  __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_1,500); //match de comparación en timer 4, siempre tiene que ser la mitad de ARR.

  estado=2; //forzado
  sin_wave(10,10);
  //const_vel(100,1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		//char info[50];
		//sprintf(info, "estado: %d \n",estado);
		//HAL_UART_Transmit(&huart2, (uint8_t*)info, strlen(info), 200);
		//estado=2; para debug comunicación-desarrollo. Saltea homming y fines de carrera.
	  if ((estado==2||estado==3)){
		  Polling_UART();

		  //const_vel(100,10);
	  }
		  if((estado==2 || estado==3)&&((HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_9) == GPIO_PIN_SET)||(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_13) == GPIO_PIN_SET))){
		  		estado=5; //error
		  		HAL_TIM_PWM_Stop_IT(&htim4,TIM_CHANNEL_1);
		  		 }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
				}


  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /**Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = htim3_Prescaler;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = htim3_Period;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = htim4_Prescaler;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 200;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */
  TIM4->ARR=10000;
  __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_1,5000);
  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_3|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC0 PC2 PC4 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB3 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PD9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PD14 PD3 PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_3|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void triangle_wave(int freq, int min,int max){
/*

	  const int half_period = 5;
	  double    amplitude   = 0.75;

	  int i = 0;
	  while(1)
	  {+
	    if(!(i++ % half_period))
	    {
	      amplitude = -amplitude;
	    }

	    printf("%d\t, %12.7f\n", i, amplitude);
	  }
		//y = abs((x++ % 6) - 3); to oscilate between 0 and 3, an period 6.
		int period=100/freq;
		float delta=((max-min)/period)*-1;
		int sign=1;
		char info[50];
		sprintf(info, "Triangular, freq: %d, min: %d, max: %d \n",freq, min,max);
		HAL_UART_Transmit(&huart2, (uint8_t*)info, strlen(info), 200);
		velocidades[0]=min;

		for (int i=1;i<10000;i++){
		if ((i+4) % period ==0)
		{
			delta=-delta;
		}
		velocidades[i]= (abs(velocidades[i-1])+delta)*sign;
		if (abs(velocidades[i])==max){
			sign=-sign;
		}
		}
		HAL_TIM_PWM_Start_IT(&htim4,TIM_CHANNEL_1);
		estado=3;*/
		}



void sin_wave(int A,int F){
	//y = 3 * sin((float)x / 10); oscilating between 3 and -3 period 20pi.
	//const float CICLE=(3.14*2)/110;
	char info[50];
	double deltaT=(htim3_Prescaler*(htim3_Period+1))/clock;  //porque tiene que cambiar al cuarto
	//float period=0.01;
	sprintf(info, "senoidal,Amplitud:%d ,Frecuencia: %d\n",A,F);
	HAL_UART_Transmit(&huart2, (uint8_t*)info, strlen(info), 200);
	for (int i=0;i<1000;i++){
		velocidades[i]=(int)((double)A*2.0*M_PI*(double)F*cos((2.0*(double)M_PI*(double)F*(double)i*(double)deltaT)));
		posiciones[i]=(int)((double)A*sin(2.0*(double)M_PI*(double)F*(double)i*(double)deltaT));
		velocidadesPulsos[i]=(int)((double)velocidades[i]*(double)pulsosporRevolucion/(double)mmporRevolucion);
		posicionesPulsos[i]=(int)((double)posiciones[i]*(double)pulsosporRevolucion/(double)mmporRevolucion);
		periodos[i]=(int)(1.0/((double)velocidadesPulsos[i]*(double)htim4_Prescaler/(double)clock));
		if (abs(periodos[i])>65535 && periodos[i-1]>0){
			periodos[i]=65535;
		}
		if (abs(periodos[i])>65535 && periodos[i-1]<0){
			periodos[i]=-65535;
		}
	}

	estado=3;
	TIM4->ARR=abs(periodos[0]);
	//__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_1,((abs(periodos[actualVel]))/2)-1);
	TIM4->CCR1=(abs(periodos[0]/2))-1;
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_PWM_Start_IT(&htim4,TIM_CHANNEL_1);
}


void const_vel(int A,int F){
	char info[50];
	int moduloVelocidad=(int)(1/((A*F/2*pulsosporRevolucion/mmporRevolucion)*htim4_Prescaler/clock));
	//double period=10000/(100*freq);
	double deltaT=(htim3_Prescaler*(htim3_Period+1))/clock;
	int period=(1/deltaT)/(F*2);
	sprintf(info, "Constante,vel:%d ,periodo: %d\n",A,F);
	HAL_UART_Transmit(&huart2, (uint8_t*)info, strlen(info), 200);
	int sign=1;
	for (int i=0;i<1000;i++){
		if((i%period)==0){
			sign=-sign;
			}
		periodos[i]=moduloVelocidad*sign;
}
	estado=3;
	HAL_TIM_PWM_Start_IT(&htim4,TIM_CHANNEL_1);
}


// Lectura de la uart
void Polling_UART() {
	int Frecuencia;
	int Amplitud;
	uint8_t rx_data_UART[13];
	if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE) == SET) { // preguntar por byte disponible en buffer
		HAL_UART_Receive(&huart2, rx_data_UART, 13, HAL_MAX_DELAY); // leer 1 byte
		if (rx_data_UART[0] == ':') {
			HAL_UART_Transmit(&huart2, (uint8_t*) rx_data_UART, 13, HAL_MAX_DELAY);

			if (rx_data_UART[1]=='t'){ // En este switch solo observo el segundo byte
						/*Frecuencia=((int)(rx_data_UART[2]-'0'))*100+((int)(rx_data_UART[3]-'0'))*10+(int)(rx_data_UART[4]-'0');
						Amplitud=((int)(rx_data_UART[5]-'0'))*100+((int)(rx_data_UART[6]-'0'))*10+(int)(rx_data_UART[7]-'0');
						Amplitud=((int)(rx_data_UART[8]-'0'))*100+((int)(rx_data_UART[9]-'0'))*10+(int)(rx_data_UART[10]-'0');
						char info[50];
						sprintf(info, "Triangular, freq: %d, min: %d, max: %d \n",freq, min, max);
						triangle_wave(freq,min,max);
						HAL_UART_Transmit(&huart2, (uint8_t*)info, strlen(info), 200);*/
					}
			else if(rx_data_UART[1]=='s'){
						Amplitud=((int)(rx_data_UART[2]-'0'))*10000+((int)(rx_data_UART[3]-'0'))*1000+((int)(rx_data_UART[4]-'0'))*100+((int)(rx_data_UART[5]-'0'))*10+((int)(rx_data_UART[6]-'0'));
						Frecuencia=((int)(rx_data_UART[7]-'0'))*100+((int)(rx_data_UART[8]-'0'))*10+((int)(rx_data_UART[9]-'0'));
						//max=((int)(rx_data_UART[8]-'0'))*100+((int)(rx_data_UART[9]-'0'))*10+(int)(rx_data_UART[10]-'0');
						//char info[50];
						//sprintf(info, "Senoidal,vel:%d ,freq: %d\n",vel, freq);
						sin_wave(Amplitud,Frecuencia);
						//HAL_UART_Transmit(&huart2, (uint8_t*)info, strlen(info), 200);
					}
			else if(rx_data_UART[1]=='c'){
									Amplitud=((int)(rx_data_UART[2]-'0'))*10000+((int)(rx_data_UART[3]-'0'))*1000+((int)(rx_data_UART[4]-'0'))*100+((int)(rx_data_UART[5]-'0'))*10+((int)(rx_data_UART[6]-'0'));
									Frecuencia=((int)(rx_data_UART[7]-'0'))*100+((int)(rx_data_UART[8]-'0'))*10+((int)(rx_data_UART[9]-'0'));
									char info[50];
									sprintf(info, "Constante,vel:%d ,freq: %d\n",Amplitud, Frecuencia);
									const_vel(Amplitud,Frecuencia);

									HAL_UART_Transmit(&huart2, (uint8_t*)info, strlen(info), 200);
								}
			else if(rx_data_UART[1]=='h'){
									estado=0;
											}
			else if(rx_data_UART[1]=='v'){
				HAL_UART_Transmit(&huart2, "Velocity: ", 10, 200);
					for(int i=0;i<100;i++){
						char buffer[16];
						HAL_UART_Transmit(&huart2, "valor de i:", 11, 200);
						HAL_UART_Transmit(&huart2, (uint8_t*)buffer, sprintf(buffer, "%d", i), 200);
						HAL_UART_Transmit(&huart2, " , ", 3, 200);
						HAL_UART_Transmit(&huart2, (uint8_t*)buffer, sprintf(buffer, "%d", periodos[i]), 200);
						HAL_UART_Transmit(&huart2, " \n", 2, 200);

			}
			}
		//procesarUart(huart2, rx_data_UART, 0);

		}
	}
	if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_ORE) == SET) {
		__HAL_UART_CLEAR_OREFLAG(&huart2);
	}

}









/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
