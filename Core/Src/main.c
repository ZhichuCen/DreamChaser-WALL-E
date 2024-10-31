/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "valuepack.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define MAX_SPEED 100
#define RATIO_TO_ARR_MAGIC_NUMBER 150
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

//const uint8_t BUFFER_SIZE=32;
//uint8_t rx_buffer[VALUEPACK_BUFFER_SIZE];
extern unsigned char vp_rxbuff[VALUEPACK_BUFFER_SIZE];
RxPack rx_pack_ptr;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void WheelControl(GPIO_TypeDef* DIR1_Port, uint16_t DIR1_Pin, GPIO_TypeDef* DIR2_Port, uint16_t DIR2_Pin, TIM_HandleTypeDef* htim, uint32_t channel, int16_t speed);
void SetMecanumWheels(int16_t speedLF, int16_t speedRF, int16_t speedLB, int16_t speedRB);
int16_t abs16_t(int16_t x);
void MoveMecanumWheels(int16_t vx, int16_t vy, int16_t omega);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



//-100<=speed<=100
void WheelControl(GPIO_TypeDef* DIR1_Port, uint16_t DIR1_Pin, GPIO_TypeDef* DIR2_Port, uint16_t DIR2_Pin, TIM_HandleTypeDef* htim, uint32_t channel, int16_t speed) {
    if (speed > 0) {
        HAL_GPIO_WritePin(DIR1_Port, DIR1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(DIR2_Port, DIR2_Pin, GPIO_PIN_RESET);
    } else if (speed < 0) {
        HAL_GPIO_WritePin(DIR1_Port, DIR1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(DIR2_Port, DIR2_Pin, GPIO_PIN_SET);
        speed = -speed; 
    } else {
        
        HAL_GPIO_WritePin(DIR1_Port, DIR1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(DIR2_Port, DIR2_Pin, GPIO_PIN_RESET);
    }
    __HAL_TIM_SET_COMPARE(htim, channel, speed); 
}


/*
int16_t debug_1=0;
int16_t debug_2=0;
int16_t debug_3=0;
int16_t debug_4=0;
*/
void SetMecanumWheels(int16_t speedLF, int16_t speedRF, int16_t speedLB, int16_t speedRB) {
		/*
		debug_1=speedLF;
		debug_2=speedRF;
		debug_3=speedLB;
		debug_4=speedRB;*/
		
    WheelControl(MOTOR_1_A_GPIO_Port, MOTOR_1_A_Pin, MOTOR_1_B_GPIO_Port, MOTOR_1_B_Pin, &htim3, TIM_CHANNEL_1, speedLF);  
    WheelControl(MOTOR_2_A_GPIO_Port, MOTOR_2_A_Pin, MOTOR_2_B_GPIO_Port, MOTOR_2_B_Pin, &htim3, TIM_CHANNEL_2, speedRF);
		WheelControl(MOTOR_3_A_GPIO_Port, MOTOR_3_A_Pin, MOTOR_3_B_GPIO_Port, MOTOR_3_B_Pin, &htim3, TIM_CHANNEL_3, speedLB);
		WheelControl(MOTOR_4_A_GPIO_Port, MOTOR_4_A_Pin, MOTOR_4_B_GPIO_Port, MOTOR_4_B_Pin, &htim3, TIM_CHANNEL_4, speedRB);
}

int16_t abs16_t(int16_t x){
	return (x>0)?x:-x;
}



void MoveMecanumWheels(int16_t vx, int16_t vy, int16_t omega) {
    int16_t speedLF = vy + vx - omega;
    int16_t speedRF = vy - vx + omega;
    int16_t speedLB = vy - vx - omega;
    int16_t speedRB = vy + vx + omega;

    // Find the maximum speed among the four wheels
    int16_t maxSpeed = abs16_t(speedLF);
    if (abs16_t(speedRF) > maxSpeed) maxSpeed = abs16_t(speedRF);
    if (abs16_t(speedLB) > maxSpeed) maxSpeed = abs16_t(speedLB);
    if (abs16_t(speedRB) > maxSpeed) maxSpeed = abs16_t(speedRB);

    // If the maximum speed exceeds 100, scale all wheel speeds proportionally
    if (maxSpeed > MAX_SPEED) {
        /*debug_1=speedLF = speedLF * MAX_SPEED / maxSpeed;
        debug_2=speedRF = speedRF * MAX_SPEED / maxSpeed;
        debug_3=speedLB = speedLB * MAX_SPEED / maxSpeed;
        debug_4=speedRB = speedRB * MAX_SPEED / maxSpeed;*/
			
				speedLF = speedLF * MAX_SPEED / maxSpeed;
        speedRF = speedRF * MAX_SPEED / maxSpeed;
        speedLB = speedLB * MAX_SPEED / maxSpeed;
        speedRB = speedRB * MAX_SPEED / maxSpeed;
    }

    // Set the speed for the mecanum wheels
    SetMecanumWheels(-speedLF, speedRF, -speedLB, speedRB);
}

void SetSingleServo(TIM_HandleTypeDef* htim, uint32_t channel, int16_t ratio){
	__HAL_TIM_SetCompare(htim,channel,ratio+100);
}

void SetServos(int16_t ratio1,int16_t ratio2,int16_t ratio3,int16_t ratio4){
	SetSingleServo(&htim4,TIM_CHANNEL_1,ratio1);
	SetSingleServo(&htim4,TIM_CHANNEL_2,ratio2);
	SetSingleServo(&htim4,TIM_CHANNEL_3,ratio3);
	SetSingleServo(&htim4,TIM_CHANNEL_4,ratio4);
}

/*
void MoveForward(int speed) {
    SetMecanumWheels(speed, speed, speed, speed);
}

void MoveBackward(int speed) {
    SetMecanumWheels(-speed, -speed, -speed, -speed);
}

void MoveLeft(int speed) {
    SetMecanumWheels(-speed, speed, speed, -speed);
}

void MoveRight(int speed) {
    SetMecanumWheels(speed, -speed, -speed, speed);
}

void RotateClockwise(int speed) {
    SetMecanumWheels(speed, -speed, speed, -speed);
}

void RotateCounterClockwise(int speed) {
    SetMecanumWheels(-speed, speed, -speed, speed);
}

*/

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if(huart==&huart1){

			//readValuePack(&rx_pack_ptr);
			//MoveMecanumWheels(rx_pack_ptr.shorts[0],rx_pack_ptr.shorts[1],rx_pack_ptr.shorts[2]);
			}
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim==&htim2){
		
		readValuePack(&rx_pack_ptr);
		MoveMecanumWheels(rx_pack_ptr.shorts[0],rx_pack_ptr.shorts[1],-rx_pack_ptr.shorts[2]);
		SetServos(rx_pack_ptr.shorts[3],rx_pack_ptr.shorts[4],rx_pack_ptr.shorts[5],rx_pack_ptr.shorts[6]);
			//SetMecanumWheels(rx_pack_ptr.shorts[0],rx_pack_ptr.shorts[1],rx_pack_ptr.shorts[2],rx_pack_ptr.shorts[3]);
	}
}



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);
	
	HAL_TIM_Base_Start_IT(&htim2);
	
	HAL_UART_Receive_DMA(&huart1,vp_rxbuff,VALUEPACK_BUFFER_SIZE);
	
	





  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
