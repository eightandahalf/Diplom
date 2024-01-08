/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* Hardware and starter kit includes. */
#include "stm32f4xx.h"
#include <string.h> /* memset */
#include <unistd.h> /* close */
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct STRCT_freq_process {
	// FP_ - Frequency process
	uint32_t FP_frequency_counter;
	uint32_t FP_front_counter;
	uint32_t FP_period;
	uint32_t FP_frequency;
	uint32_t FP_timeout_flag;
} STRCT_freq_process;

typedef struct STRCT_lcd_process {
	// LCDPR_ - LCD PRINT
	// accelerometrs values
	int16_t LCDPR_act_acc_value;
	int16_t LCDPR_prev_acc_value;
	int16_t LCDPR_acc_max_value;
	int16_t LCDPR_acc_min_value;
	float LCDPR_acc_amplitude;
	float LCDPR_prev_acc_amplitude;

	// frequency values
	uint32_t LCDPR_frequency_counter;
	uint32_t LCDPR_period;
	float LCDPR_frequency;
	float LCDPR_prev_freqency_value;

	// frequency values from imu
	uint32_t LCDPR_frequency_counter_from_imu_values;
	uint32_t LCDPR_period_from_imu_values;
	float LCDPR_frequency_from_imu_values;

	// flags
	uint8_t LCDPR_freq_flag;
	int16_t LCDPR_ampl_flag;
	uint8_t LCDPR_phase_flag;

	// buffers
	char LCDPR_sec_print_buff[16];
	char LCDPR_print_buff[8];

	// phase
	float LCDPR_phase;
	float LCDPR_phase_crockodil;
	float LCDPR_prev_phase_value;

} STRCT_lcd_process;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define mainDELAY_LOOP_COUNT   ( 0xfffff )
#define mainDELAY_LOOP_COUNT   ( 0x7D0 )
//#define mainDELAY_LOOP_COUNT   ( 0xBB8 )

#define MPU6050_ADDR 0xD0

#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_XOUT_H_REG 0x43
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75
#define DLPF_CFG_REG 0x1A
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define TIMCLOCK   84000000
#define PRESCALAR  84-1

uint32_t IC_Val1 = 0;
uint32_t IC_Val2 = 0;
uint32_t Difference = 0;
int Is_First_Captured = 0;

// GLOBAL VARIABLES, THAT WILL CONTAIN NEXT VALUES:
// actual frequency
float GLBL_frequency = 0;
// actual value of accelerometr
int16_t GLBL_act_acc_value = 0;
// frequency counter
uint32_t GLBL_frequency_counter =  0;
// timeout flag of driver working or not working
uint16_t driver_off_flag = 0;
/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
	{
		driver_off_flag = 0;
		if (Is_First_Captured==0) // if the first rising edge is not captured
		{
			IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // read the first value
			Is_First_Captured = 1;  // set the first captured as true

			GLBL_frequency_counter = 0;
		}

		else   // If the first rising edge is captured, now we will capture the second edge
		{
			IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);  // read second value

			if (IC_Val2 > IC_Val1)
			{
				Difference = IC_Val2-IC_Val1;
			}

			else if (IC_Val1 > IC_Val2)
			{
				Difference = (0xffffffff - IC_Val1) + IC_Val2;
			}

			float refClock = TIMCLOCK/(PRESCALAR);

			GLBL_frequency = refClock/Difference;

//			if((refClock/Difference) > 100)
//				;
//			else
//				GLBL_frequency = refClock/Difference;


			//printf("frequency = %.5f\n", frequency*1000);

			__HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter
			Is_First_Captured = 0; // set it back to false

			GLBL_frequency_counter++;
		}
	}
}

void MPU6050_Init (void)
{
	uint8_t check;
	uint8_t Data;

	// check device ID WHO_AM_I

	HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR,WHO_AM_I_REG,1, &check, 1, 1000);

	if (check == 104)  // 0x68 will be returned by the sensor if everything goes well
	{
		// internal accelerometr delay to zero
		Data = 0;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, DLPF_CFG_REG, 1, &Data, 1, 1000);

		// power management register 0X6B we should write all 0's to wake the sensor up
		Data = 0;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, 1000);

		// Set DATA RATE of 1KHz by writing SMPLRT_DIV register
		Data = 0x07;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, 1000);

		// Set accelerometer configuration in ACCEL_CONFIG Register
		// XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> ± 2g
		Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, 1000);

		// Set Gyroscopic configuration in GYRO_CONFIG Register
		// XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> ± 250 °/s
		Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, 1000);
	}
}

void mpu6050_read_accel(void)
{
	// RA_ - READ ACCELEROMETR

	// accelerometrs values
	int16_t RA_Accel_X_RAW = 0;
	float RA_Ax;
	uint8_t RA_Rec_Data[2];

	HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, RA_Rec_Data, 2, 1000);

	RA_Accel_X_RAW = (int16_t)(RA_Rec_Data[0] << 8 | RA_Rec_Data [1]);

	RA_Ax = RA_Accel_X_RAW/16384.0;

	GLBL_act_acc_value = RA_Ax*1000;

	if (GLBL_frequency_counter > 0){
			GLBL_frequency_counter++;
		}
}

void lcd1602_print(STRCT_lcd_process* obj_lcd_process)
{
	// LCDPR_ - LCD PRINT

	driver_off_flag++;
	if(driver_off_flag > 200)
	{
		GLBL_frequency = 0;
		GLBL_frequency_counter = 0;
	} else if (GLBL_frequency_counter > 0){
		GLBL_frequency_counter++;
	}

	obj_lcd_process->LCDPR_act_acc_value = GLBL_act_acc_value;
	obj_lcd_process->LCDPR_frequency_counter = GLBL_frequency_counter;
	obj_lcd_process->LCDPR_frequency = GLBL_frequency;

	// there were a hard fault because here can be dividing of 360*phase_crockodil to period, and
	// period shouldn't be a zero
	if(obj_lcd_process->LCDPR_prev_acc_value > 0 && obj_lcd_process->LCDPR_act_acc_value <= 0)
	{
		obj_lcd_process->LCDPR_phase_crockodil = obj_lcd_process->LCDPR_frequency_counter;
		obj_lcd_process->LCDPR_phase = 360*(obj_lcd_process->LCDPR_phase_crockodil) / (obj_lcd_process->LCDPR_frequency) / 100;

		(obj_lcd_process->LCDPR_phase_flag)++;
		if(obj_lcd_process->LCDPR_phase_flag > 6 && (driver_off_flag > 200 || (obj_lcd_process->LCDPR_frequency_counter > 0 && obj_lcd_process->LCDPR_phase < 600)))
		{
			// phase
			HD44780_SetCursor(8, 0);
			memset(obj_lcd_process->LCDPR_print_buff, ' ', sizeof(obj_lcd_process->LCDPR_print_buff));
			HD44780_PrintStr (obj_lcd_process->LCDPR_print_buff);
			HD44780_SetCursor(8, 0);
			sprintf(obj_lcd_process->LCDPR_print_buff,"Ph = %u", (unsigned int)obj_lcd_process->LCDPR_phase);
			HD44780_PrintStr (obj_lcd_process->LCDPR_print_buff);

			obj_lcd_process->LCDPR_phase_flag = 0;
		}

		obj_lcd_process->LCDPR_frequency_counter = 0;
		obj_lcd_process->LCDPR_prev_phase_value = obj_lcd_process->LCDPR_phase;
	}

	obj_lcd_process->LCDPR_freq_flag++;
	if(obj_lcd_process->LCDPR_freq_flag > 200)
	{
		HD44780_SetCursor(0, 0);
		memset(obj_lcd_process->LCDPR_print_buff, ' ', sizeof(obj_lcd_process->LCDPR_print_buff));
		HD44780_PrintStr (obj_lcd_process->LCDPR_print_buff);
		HD44780_SetCursor(0, 0);
		sprintf(obj_lcd_process->LCDPR_print_buff,"F = %u", (unsigned int)obj_lcd_process->LCDPR_frequency);
		HD44780_PrintStr (obj_lcd_process->LCDPR_print_buff);

		obj_lcd_process->LCDPR_freq_flag = 0;
	}

	obj_lcd_process->LCDPR_prev_freqency_value = obj_lcd_process->LCDPR_frequency;

	obj_lcd_process->LCDPR_ampl_flag++;
	if(obj_lcd_process->LCDPR_ampl_flag < 200)
	{
		if(obj_lcd_process->LCDPR_act_acc_value >= obj_lcd_process->LCDPR_acc_max_value)
			obj_lcd_process->LCDPR_acc_max_value = obj_lcd_process->LCDPR_act_acc_value;
		if(obj_lcd_process->LCDPR_act_acc_value < obj_lcd_process->LCDPR_acc_min_value)
			obj_lcd_process->LCDPR_acc_min_value = obj_lcd_process->LCDPR_act_acc_value;
	}
	else
	{
		obj_lcd_process->LCDPR_acc_amplitude = obj_lcd_process->LCDPR_acc_max_value - obj_lcd_process->LCDPR_acc_min_value;

		HD44780_SetCursor(0, 1);
		memset(obj_lcd_process->LCDPR_print_buff, ' ', sizeof(obj_lcd_process->LCDPR_print_buff));
		HD44780_PrintStr (obj_lcd_process->LCDPR_print_buff);
		HD44780_SetCursor(0, 1);
		sprintf(obj_lcd_process->LCDPR_print_buff,"A = %u", (unsigned int)obj_lcd_process->LCDPR_acc_amplitude);
		HD44780_PrintStr (obj_lcd_process->LCDPR_print_buff);

		obj_lcd_process->LCDPR_prev_acc_amplitude = obj_lcd_process->LCDPR_acc_amplitude;

		obj_lcd_process->LCDPR_ampl_flag = 0;
		obj_lcd_process->LCDPR_acc_max_value = obj_lcd_process->LCDPR_act_acc_value;
		obj_lcd_process->LCDPR_acc_min_value = obj_lcd_process->LCDPR_act_acc_value;
	}

	obj_lcd_process->LCDPR_prev_acc_value = obj_lcd_process->LCDPR_act_acc_value;

}

void FRRTS_IMU_PROCESS( void *pvParameters )
{
	volatile uint32_t ul; /* volatile to ensure ul is not optimized away. */
	/* As per most tasks, this task is implemented in an infinite loop. */
	for( ;; )
	{
		mpu6050_read_accel();

		/* Delay for a period. */
		for( ul = 0; ul < mainDELAY_LOOP_COUNT; ul++ )
		{
			/* This loop is just a very crude delay implementation. There is
			nothing to do in here. Later examples will replace this crude
			loop with a proper delay/sleep function. */
		}
	}
}

void FRRTS_LCD_PRINT( void *pvParameters )
{
	volatile uint32_t ul; /* volatile to ensure ul is not optimized away. */
	/* As per most tasks, this task is implemented in an infinite loop. */

	STRCT_lcd_process obj_lcd_process = {0};

	for( ;; )
	{
		lcd1602_print(&obj_lcd_process);

		/* Delay for a period. */
		for( ul = 0; ul < mainDELAY_LOOP_COUNT; ul++ )
		{
			/* This loop is just a very crude delay implementation. There is
			nothing to do in here. Later examples will replace this crude
			loop with a proper delay/sleep function. */
		}
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

/*
 * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	ЕСЛИ ДАТЧИК ОБОРОТОВ ВЫДАЕТ СЛИШКОМ МНОГО GARBAGE ЗНАЧЕНИЙ ПРИРАСЧЕТЕ ЧАСТОТЫ,
	ТО СЛЕДУЕТ ПОПРОБОВАТЬ ПОКРУТИТЬ ПОТЕНЦИОМЕТР НА САМОМ ДАТЧИКЕ. Т.К. 03.01.2024
	ПОЛУЧИЛОСЬ РЕШИТЬ ЭТУ ПРОБЛЕМУ ТАКИМ ОБРАЗОМ/

	ЕСЛИ НЕ РАБОТАЕТ IMU, ТО ОТСОЕДИНИТЬ ПРОВОД ЗЕМЛИ ОТ ИМЮШКИ НА КОНТРОЛЛЕРЕ И
	ВСТАИТЬ ЗАНОВО

	CUBE IDE МОЖЕТ РУГАТЬСЯ НА ЭТИ СТРОЧКИ КОД:
		if(driver_off_flag > 200)
	{
		GLBL_frequency = 0;
	}

	ЕСЛИ НАЧИНАЕТ РУГАТЬ, ПОПЫТАТЬСЯ СНЧАЛА ИХ ЗАКОММЕНТИРОВАТЬ, ЗАПУСТИТЬ ПРОЕКТ,
	ОНА ДОЛЖЕН ЗАПУСТИТЬСЯ, ПОСЛЕ ЭТОГО ОСТАНОВИТЬСЯ РАБОТУ ПРОГРАММЫ, РАЗКОММЕН-
	ТИРОВАТЬ ЭТИ СТРОЧКИ КОДА И ЗАПУСТИТЬ ПРОГРАММУ ЗАНОВОS

 * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
*/

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
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	MPU6050_Init();
	// Функция инициализации дисплея
	HD44780_Init(2);
	HD44780_Clear();
	HD44780_SetCursor(3,0);
	HD44780_PrintStr("WELCOME TO");
	HD44780_SetCursor(3,1);
	HD44780_PrintStr("BALANCING");
	HAL_Delay(2000);

	HD44780_Clear();

	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);

  /* Create one of the two tasks. Note that a real application should check
   the return value of the xTaskCreate() call to ensure the task was created
   successfully. */

   xTaskCreate(FRRTS_IMU_PROCESS, /* Pointer to the function that implements the task. */
   "FRRTS_IMU_PROCESS",/* Text name for the task. This is to facilitate debugging only. */
   1000, /* Stack depth - small microcontrollers will use much less stack than this. */
   NULL, /* This example does not use the task parameter. */
   1, /* This task will run at priority 1. */
   NULL ); /* This example does not use the task handle. */

   /* Create the other task in exactly the same way and at the same priority. */
   xTaskCreate(FRRTS_LCD_PRINT, "FRRTS_LCD_PRINT", 1000, NULL, 1, NULL );
   /* Start the scheduler so the tasks start executing. */
   vTaskStartScheduler();
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
