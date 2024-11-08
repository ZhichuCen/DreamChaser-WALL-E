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
#define RASP_BUFFER_SIZE 128

// reset position
// range:-100~100
#define SERVO_1_RESET_POSITION 0
#define SERVO_2_RESET_POSITION 100
#define SERVO_3_RESET_POSITION 100

#define CLAW_CLOSED_POSITION 35
#define CLAW_OPEN_POSITION 100

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// here is BLE vars
_Bool RED0_BLE1 = 0;
_Bool BOOL1 = 0;
_Bool BOOL2 = 0;
_Bool BOOL3 = 0;
_Bool BOOL4 = 0;
_Bool BOOL5 = 0;
_Bool BOOL6 = 0;
_Bool CLAW = 0;
_Bool ResetPosition = 0;
_Bool StartAuto = 0;
_Bool Man1 = 0;
_Bool Man2 = 0;
_Bool Man3 = 0;
_Bool Man4 = 0;

int8_t OBJ = 0; // 0-none, 1-garbage, 2-mine, 3-meteroite
int8_t overall_byte = 0;
int8_t X_axis = 0;
int8_t Y_axis = 0;
int8_t ROTATION = 0;
int8_t S1 = 0;
int8_t S2 = 0;
int8_t S3 = 0;
int8_t S4 = 0;

int8_t servo_control_1 = SERVO_1_RESET_POSITION;
int8_t servo_control_2 = SERVO_2_RESET_POSITION;
int8_t servo_control_3 = SERVO_3_RESET_POSITION;
int8_t servo_control_4 = CLAW_OPEN_POSITION;

// char target_arr[6];

// debug DEBUG Debug--------------
char target_arr[6] = {'Y', 'X', 'Z', 'X', 'Y', 'Z'};
char validate_num_list[2];
_Bool *full_block_ptr[6] = {&BOOL1, &BOOL2, &BOOL3, &BOOL4, &BOOL5, &BOOL6};
_Bool finished_receiving_target = 0;
unsigned short rasp_last_index = 0;
unsigned short rasp_this_index = 0;
uint16_t rasp_index = 0;

// uint8_t auto_count=0;
// uint8_t auto_last=0;

uint8_t auto_status_flag = 0;
_Bool still_have_bonus_left = 0;
_Bool still_have_normal_left = 0;

// const uint8_t BUFFER_SIZE=32;
// uint8_t rx_buffer[VALUEPACK_BUFFER_SIZE];
extern unsigned char vp_rxbuff[VALUEPACK_BUFFER_SIZE];
unsigned char rasp_buff[RASP_BUFFER_SIZE];
RxPack rx_pack_ptr;

_Bool started_read_rasp = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void WheelControl(GPIO_TypeDef *DIR1_Port, uint16_t DIR1_Pin, GPIO_TypeDef *DIR2_Port, uint16_t DIR2_Pin, TIM_HandleTypeDef *htim, uint32_t channel, int16_t speed);
void SetMecanumWheels(int16_t speedLF, int16_t speedRF, int16_t speedLB, int16_t speedRB);
int16_t abs16_t(int16_t x);
void MoveMecanumWheels(int16_t vx, int16_t vy, int16_t omega);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//-100<=speed<=100
void WheelControl(GPIO_TypeDef *DIR1_Port, uint16_t DIR1_Pin, GPIO_TypeDef *DIR2_Port, uint16_t DIR2_Pin, TIM_HandleTypeDef *htim, uint32_t channel, int16_t speed)
{
  if (speed > 0)
  {
    HAL_GPIO_WritePin(DIR1_Port, DIR1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(DIR2_Port, DIR2_Pin, GPIO_PIN_RESET);
  }
  else if (speed < 0)
  {
    HAL_GPIO_WritePin(DIR1_Port, DIR1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DIR2_Port, DIR2_Pin, GPIO_PIN_SET);
    speed = -speed;
  }
  else
  {

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
void SetMecanumWheels(int16_t speedLF, int16_t speedRF, int16_t speedLB, int16_t speedRB)
{
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

int16_t abs16_t(int16_t x)
{
  return (x > 0) ? x : -x;
}

void MoveMecanumWheels(int16_t vx, int16_t vy, int16_t omega)
{
  int16_t speedLF = vy + vx - omega;
  int16_t speedRF = vy - vx + omega;
  int16_t speedLB = vy - vx - omega;
  int16_t speedRB = vy + vx + omega;

  // Find the maximum speed among the four wheels
  int16_t maxSpeed = abs16_t(speedLF);
  if (abs16_t(speedRF) > maxSpeed)
    maxSpeed = abs16_t(speedRF);
  if (abs16_t(speedLB) > maxSpeed)
    maxSpeed = abs16_t(speedLB);
  if (abs16_t(speedRB) > maxSpeed)
    maxSpeed = abs16_t(speedRB);

  // If the maximum speed exceeds 100, scale all wheel speeds proportionally
  if (maxSpeed > MAX_SPEED)
  {
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

void SetSingleServo(TIM_HandleTypeDef *htim, uint32_t channel, int16_t ratio)
{
  __HAL_TIM_SetCompare(htim, channel, ratio + RATIO_TO_ARR_MAGIC_NUMBER);
}

void SetServos(int16_t ratio1, int16_t ratio2, int16_t ratio3, int16_t ratio4)
{

  ratio2 = ratio2 * 0.8 + 20;
	ratio3 = ratio3 * 0.75 + 25;

  SetSingleServo(&htim4, TIM_CHANNEL_1, ratio1);
  SetSingleServo(&htim4, TIM_CHANNEL_2, ratio2);
  SetSingleServo(&htim4, TIM_CHANNEL_3, ratio3);
  SetSingleServo(&htim4, TIM_CHANNEL_4, ratio4);
}

void GetData(void)
{
  readValuePack(&rx_pack_ptr);

  RED0_BLE1 = rx_pack_ptr.bools[0];
  BOOL1 = rx_pack_ptr.bools[1];
  BOOL2 = rx_pack_ptr.bools[2];
  BOOL3 = rx_pack_ptr.bools[3];
  BOOL4 = rx_pack_ptr.bools[4];
  BOOL5 = rx_pack_ptr.bools[5];
  BOOL6 = rx_pack_ptr.bools[6];
  CLAW = rx_pack_ptr.bools[7];
  ResetPosition = rx_pack_ptr.bools[8];
  StartAuto = rx_pack_ptr.bools[9];
  Man1 = rx_pack_ptr.bools[10];
  Man2 = rx_pack_ptr.bools[11];
  Man3 = rx_pack_ptr.bools[12];
  Man4 = rx_pack_ptr.bools[13];

  OBJ = rx_pack_ptr.bytes[0]; // 0-none, 1-garbage, 2-mine, 3-meteroite
  overall_byte = rx_pack_ptr.bytes[1];
  X_axis = rx_pack_ptr.bytes[2];
  Y_axis = rx_pack_ptr.bytes[3];
  ROTATION = rx_pack_ptr.bytes[4];
  S1 = rx_pack_ptr.bytes[5];
  S2 = rx_pack_ptr.bytes[6];
  S3 = rx_pack_ptr.bytes[7];
  S4 = rx_pack_ptr.bytes[8];
}

void swap_arr(uint8_t a, uint8_t b)
{
  char temp = target_arr[a];
  target_arr[a] = target_arr[b];
  target_arr[b] = temp;
}

_Bool validate_data()
{

  int8_t my_validate_value = 0;
  for (int i = 0; i < 6; i++)
  {
    if (target_arr[i] == 'Z')
    {
      my_validate_value |= (1 << i);
    }
  }

  int8_t received_validate_value = 0;

  received_validate_value += (validate_num_list[0] - 0x30) * 10;
  received_validate_value += validate_num_list[1] - 0x30;

  if (my_validate_value == received_validate_value)
  {
    return 1;
  }
  else
  {
    return 0;
  }
}

void ReadRaspData(void)
{

  if (!finished_receiving_target)
  {
    // rasp_this_index = RASP_BUFFER_SIZE - (&huart3)->hdmarx->Instance->CNDTR;
    rasp_index = 0;
    for (; rasp_index < RASP_BUFFER_SIZE; rasp_index++)
    {

      if (rasp_buff[rasp_index] == 'K')
      {

        for (int j = 0; j < 6; j++)
        {
          target_arr[j] = rasp_buff[rasp_index++];
        }
        validate_num_list[0] = rasp_index++;
        validate_num_list[1] = rasp_index++;
        break;
      }
    }
    if (rasp_index > RASP_BUFFER_SIZE - 10)
    {
      rasp_index = 0;
    }
    if (validate_data())
    {

      if (RED0_BLE1 == 0)
      {
        swap_arr(0, 1);
        swap_arr(2, 3);
        swap_arr(4, 5);
      }

      // memset(rasp_buff,0,sizeof(rasp_buff));
      finished_receiving_target = 1;
    }
  }
}

void MoveLeftorRight(char left_or_right)
{
  if (left_or_right == 'L')
  {
    MoveMecanumWheels(-100, 0, 0);
    HAL_Delay(500);
    MoveMecanumWheels(0, 0, 0);
  }
  else if (left_or_right == 'R')
  {
    MoveMecanumWheels(100, 0, 0);
    HAL_Delay(500);
    MoveMecanumWheels(0, 0, 0);
  }
}

void MoveForwardorBackward(char for_or_back)
{
  switch (for_or_back)
  {
  case 'F':
    MoveMecanumWheels(0, 100, 0);
    HAL_Delay(1000);
    MoveMecanumWheels(0, 0, 0);
		HAL_Delay(1000);
    break;
  case 'B':
		HAL_Delay(1000);
    MoveMecanumWheels(0, -100, 0);
    HAL_Delay(2000);
    MoveMecanumWheels(0, 0, 0);

    break;
  default:
    break;
  }
}

void MoveUporDown(int8_t which_floor)
{

  switch (which_floor)
  {
  case 0:
    servo_control_2 = 83;
    servo_control_3 = 81;
    break;
  case 1:
    servo_control_2 = 4;
    servo_control_3 = 27;
    break;
  case 2:
    servo_control_2 = -42;
    servo_control_3 = -32;
    break;
  default:
    break;
  }
  SetServos(servo_control_1, servo_control_2, servo_control_3, servo_control_4);
}

void found_and_goto_put(int8_t target_index)
{

  MoveUporDown(target_index / 2);
  SetServos(servo_control_1, servo_control_2, servo_control_3, servo_control_4);
	servo_control_4 = CLAW_OPEN_POSITION;
	
	HAL_Delay(500);

  if ((target_index + 1) % 2 == 1)
  {
    MoveLeftorRight('L');
  }
  else
  {
    MoveLeftorRight('R');
  }

  MoveForwardorBackward('F');
	

  servo_control_4 = CLAW_OPEN_POSITION;
  SetServos(servo_control_1, servo_control_2, servo_control_3, servo_control_4);


	
  MoveForwardorBackward('B');
}

void none_left(void)
{
  MoveMecanumWheels(0, 0, 100);
  HAL_Delay(500);
  MoveMecanumWheels(0, 0, -100);
  HAL_Delay(500);
  MoveMecanumWheels(0, 0, 0);
}

void NoBonus(void)
{
  still_have_normal_left = 0;
  int8_t current_normal = -1;
  for (int i = 0; i < 6; i++)
  {
    if ((target_arr[i] == 'Y') && (!*full_block_ptr[i]))
    {
      still_have_bonus_left = 1;
      current_normal = i;
    }
  }

  if (still_have_normal_left)
  {
    found_and_goto_put(current_normal);
  }
  else
  {
    if (OBJ == 1 || OBJ == 0)
    {
      int8_t current_garbage = -1;
      _Bool still_have_garbage_left = 0;
      for (int i = 0; i < 6; i++)
      {
        if ((target_arr[i] == 'X') && (!*full_block_ptr[i]))
        {
          still_have_garbage_left = 1;
          current_garbage = i;
        }
      }
      if (still_have_garbage_left)
      {
        found_and_goto_put(current_garbage);
      }
    }
    else
    {
      none_left();
    }
  }
}

void AutoPut(void)
{
  int8_t current_bonus = -1;
  still_have_bonus_left = 0;
  int8_t full_count = 0;
  int8_t only_1_left = -1;

  for (int8_t i = 0; i < 6; i++)
  {
    if (*full_block_ptr[i])
    {
      full_count++;
    }
    else
    {
      only_1_left = i;
    }
  }

  if (full_count == 5)
  {
    found_and_goto_put(only_1_left);
  }
  else
  {

    for (int i = 0; i < 6; i++)
    {
      if ((target_arr[i] == 'Z') && (!*full_block_ptr[i]))
      {
        still_have_bonus_left = 1;
        current_bonus = i;
      }
    }

    if (still_have_bonus_left)
    {
      found_and_goto_put(current_bonus);
    }
    else
    {
      NoBonus();
    }
  }
  auto_status_flag = 2;

  // auto_count++;7
}

void CalculateServos(void)
{
  servo_control_1 = S1;
  servo_control_2 = S2;
  servo_control_3 = S3;
}

void Respond_to_commands(void)
{
  if (StartAuto)
  {
    if (auto_status_flag == 0)
    {
      auto_status_flag = 1;
      AutoPut();
    }
  }
  else if (auto_status_flag == 2 && StartAuto == 0 && CLAW == 0)
  {
    auto_status_flag = 0;
  }
  else
  {
    if (ResetPosition)
    {
      servo_control_1 = SERVO_1_RESET_POSITION;
      servo_control_2 = SERVO_2_RESET_POSITION;
      servo_control_3 = SERVO_3_RESET_POSITION;
      servo_control_4 = (CLAW) ? CLAW_CLOSED_POSITION : CLAW_OPEN_POSITION;
      SetServos(servo_control_1, servo_control_2, servo_control_3, servo_control_4);
    }
    else
    {
      CalculateServos();

      servo_control_4 = (CLAW) ? CLAW_CLOSED_POSITION : CLAW_OPEN_POSITION;

      // Manual Override :P be careful
      if (Man1)
      {
        servo_control_1 = S1;
      }

      if (Man2)
      {
        servo_control_2 = S2;
      }

      if (Man3)
      {
        servo_control_3 = S3;
      }

      if (Man4)
      {
        servo_control_4 = S4;
      }

      SetServos(servo_control_1, servo_control_2, servo_control_3, servo_control_4);
    }
    MoveMecanumWheels(X_axis, Y_axis, -ROTATION);
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /*if(huart==&huart1){

      //readValuePack(&rx_pack_ptr);
      //MoveMecanumWheels(rx_pack_ptr.shorts[0],rx_pack_ptr.shorts[1],rx_pack_ptr.shorts[2]);
      }*/
  if (huart == &huart3)
  {
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim2)
  {

    // readValuePack(&rx_pack_ptr);
    GetData();
    Respond_to_commands();

    if (!started_read_rasp)
    {
      started_read_rasp = 1;
      ReadRaspData();
      started_read_rasp = 0;
    }

    // SetMecanumWheels(rx_pack_ptr.shorts[0],rx_pack_ptr.shorts[1],rx_pack_ptr.shorts[2],rx_pack_ptr.shorts[3]);
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
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

  HAL_TIM_Base_Start_IT(&htim2);

  HAL_UART_Receive_DMA(&huart1, vp_rxbuff, VALUEPACK_BUFFER_SIZE);
  HAL_UART_Receive_DMA(&huart3, rasp_buff, RASP_BUFFER_SIZE);

  SetServos(SERVO_1_RESET_POSITION, SERVO_2_RESET_POSITION, SERVO_3_RESET_POSITION, CLAW_OPEN_POSITION);

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
