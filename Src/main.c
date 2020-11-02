/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DigitDisplay_ms     1 /* ms each digit is kept on */
#define PressBPSwicthTime   1000 /* if user keep bp press more that this mean swicth mode else rooll over use c&se in mode */
#define MAX_DEV 2

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// global vars only for debugging with STMStudio
VL6180x_RangeData_t Range[MAX_DEV];
VL6180x_RangeData_t RangeDB1;
VL6180x_RangeData_t RangeDB2;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void WaitMilliSec(int ms);

VL6180xDev_t theVL6180xDev;
struct MyVL6180Dev_t BoardDevs[MAX_DEV] = { 
                                        [0]= { .DevID = 0 }, 
                                        [1]= { .DevID = 1 }, 
                                    };

VL6180xDev_t theVL6180xDev = &BoardDevs[0];


/**
 * VL6180x CubeMX F401 multiple device i2c implementation
 */

#define i2c_bus      (&hi2c1)
#define def_i2c_time_out 100


int VL6180x_I2CWrite(VL6180xDev_t dev, uint8_t *buff, uint8_t len) {
    int status;
    status = HAL_I2C_Master_Transmit(i2c_bus, dev->I2cAddr, buff, len, def_i2c_time_out);
    if (status) {
        HAL_I2C_MspInit(&hi2c1);
    }
    return status? -1 : 0;
}

int VL6180x_I2CRead(VL6180xDev_t dev, uint8_t *buff, uint8_t len) {
    int status;
    status = HAL_I2C_Master_Receive(i2c_bus, dev->I2cAddr, buff, len, def_i2c_time_out);
    if (status) {
        HAL_I2C_MspInit(&hi2c1);
    }

    return status? -1 : 0;
}

void WaitMilliSec(int ms) {
    HAL_Delay(ms); /* it's milli sec  cos we do set systick to 1KHz */
}



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  

  int status;
  int i;
  int n_dev=2;
  int PresentDevMask;
  int nPresentDevs;
  int PresentDevIds[MAX_DEV];
  int nReady;
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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */


  /* detect presence and initialize devices i2c address  */
  /*set device i2c address for dev[i] = 0x52+(i+1)*2 */
  PresentDevMask = 0;
  nPresentDevs = 0;
  //strcpy(DisplayStr,"TLBR");
  for (i = 0; i <= n_dev - 1; i++) { //(i = n_dev - 1; i >= 0; i--)
      int FinalI2cAddr;
      uint8_t id;
      if (i==0)
        HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_11);
      if (i==1)
        HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_12);
        
      /* unreset device that wake up at default i2c addres 0x52 */
      WaitMilliSec(2);    /* at least 400usec before to acces device */
      BoardDevs[i].I2cAddr = 0x52;
      /* to detect device presence try to read it's dev id */
      status = VL6180x_RdByte(&BoardDevs[i], IDENTIFICATION_MODEL_ID, &id);
      if (status) {
          /* these device is not present skip init and clear it's letter on string */
          /*we can add here some debug prints*/
          //BoardDevs[i].Present = 0;
          //DisplayStr[i]=' ';
          continue;
      }
      /* device present only */
      BoardDevs[i].Present = 1;
      PresentDevMask |= 1 << i;
      PresentDevIds[nPresentDevs]=i;
      nPresentDevs++;
      status = VL6180x_InitData(&BoardDevs[i]);

      FinalI2cAddr = 0x52 + ((i+1) * 2);
      if (FinalI2cAddr != 0x52) {
          status = VL6180x_SetI2CAddress(&BoardDevs[i], FinalI2cAddr);
          if( status ){
              //HandleError("VL6180x_SetI2CAddress fail");
          }
          BoardDevs[i].I2cAddr = FinalI2cAddr;
      }

      WaitMilliSec(1);
      status = VL6180x_RdByte(&BoardDevs[i], IDENTIFICATION_MODEL_ID, &id);
      WaitMilliSec(1);
      status= VL6180x_Prepare(&BoardDevs[i]);
      if( status<0 ){
          //HandleError("VL6180x_Prepare fail");
      }
      /* Disable Dmax computation */
      VL6180x_DMaxSetState(&BoardDevs[i], 0);

    }
    HAL_Delay(30);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    VL6180xDev_t dev;
    
    // kick off measure on all device 
    for( i=0; i<nPresentDevs; i++){
        dev =  BoardDevs + PresentDevIds[i];
        status = VL6180x_RangeStartSingleShot(dev);
        if( status<0 ){
            //TODO: print error : debug
        }
        dev->Ready=0;
    }

    // wait for all present device to have a measure  
    nReady=0;
    
    do{
        //DISP_ExecLoopBody();
        for( i=0; i<nPresentDevs; i++){
            dev =  BoardDevs + PresentDevIds[i];
            if( !dev->Ready ){
                status = VL6180x_RangeGetMeasurementIfReady(dev, &Range[i]);
                if (i==0)
                  RangeDB1 = Range[i];
                if (i==1)
                  RangeDB2 = Range[i];
                if( status == 0 ){
                    if(Range[i].errorStatus == DataNotReady)
                        continue;
                    // New measurement ready 
                    dev->Ready=1;
                    nReady++;
                } else {
                    //HandleError("VL6180x_RangeStartSingleShot fail"); //TODO: print error : debug  
                }
            }
        }
    }
    while( nReady<nPresentDevs);
    
    HAL_Delay(10);
      
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
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
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin PA11 PA12 */
  GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
