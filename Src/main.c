/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
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
#include "stm32l1xx_hal.h"
#include "i2c.h"
#include "rtc.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "spirit1_appli.h"

#define TX_BUFFER_SIZE   20
#define RX_BUFFER_SIZE   96
#define RX_MODE 0
#define TX_MODE 1

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
extern volatile SpiritStatus g_xStatus;

uint8_t TxLength = TX_BUFFER_SIZE;
uint8_t RxLength = 0;
uint8_t aTransmitBuffer[TX_BUFFER_SIZE] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,\
                                                                16,17,18,19,20};
uint8_t aReceiveBuffer[RX_BUFFER_SIZE] = {0x00};
uint8_t mode = TX_MODE;

uint8_t regs[] = {
           0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
	       0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0xff, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f,
   	       0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
   	       0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x3b, 0x3c, 0x3d, 0x3e, 0x3f,
   	       0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4a, 0x4b, 0x4c, 0x4d, 0x4e, 0x4f,
   	       0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
		   0xff, 0xff, 0xff, 0xff, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6a, 0x6b, 0X6c, 0x6d, 0x6e, 0x6f,
   	       0x70, 0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7a, 0x7b, 0x7c, 0x7d, 0x7e, 0x7f,
   	       0x80, 0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8a, 0x8b, 0x8c, 0x8d, 0x8e, 0x8f,
		   0x90, 0x91, 0x92, 0x93, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x9e, 0x9f,
		   0xff, 0xa1, 0xff, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7, 0xa8, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
		   0xff, 0xff, 0xb2, 0xff, 0xb4, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff

		};
char message[64];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/


/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

void Spirit1_Configure() {
	  HAL_Spirit1_Init();
	  P2P_Init();
	  uint8_t aux;
	  aux = 0x03;
	  SpiritSpiWriteRegisters(0x10, 1, &aux);
	  aux = 0x01;
	  SpiritSpiWriteRegisters(0x17, 1, &aux);
	  aux = 0x00;
	  SpiritSpiWriteRegisters(0x18, 1, &aux);
}

void Configure_GPIO() {
	  MX_GPIO_Init();
	  HAL_I2C_MspInit(&hi2c1);
	  HAL_SPI_MspInit(&hspi1);
	  HAL_UART_MspInit(&huart1);
}

void Enter_Low_Power(int wakeUpSecs) {

	  RadioEnterShutdown();

	  HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, wakeUpSecs, RTC_WAKEUPCLOCK_CK_SPRE_16BITS);

	  SystemPower_Config();

	  MCU_Enter_StopMode();

}

void Exit_Low_Power() {
	  HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
	  Configure_GPIO();
	  RadioExitShutdown();
	  Spirit1_Configure();

}

void SystemPower_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure = {0};

  /* Enable Ultra low power mode */
  HAL_PWREx_EnableUltraLowPower();

  /* Enable the fast wake up from Ultra low power mode */
  HAL_PWREx_EnableFastWakeUp();

  /* Enable GPIOs clock */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();


  /* Configure all GPIO port pins in Analog Input mode (floating input trigger OFF) */
  GPIO_InitStructure.Pin = GPIO_PIN_All;
  GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
  HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);
  HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);
  HAL_GPIO_Init(GPIOE, &GPIO_InitStructure);
  HAL_GPIO_Init(GPIOH, &GPIO_InitStructure);

  GPIO_InitStructure.Pin = GPIO_PIN_0  |                GPIO_PIN_2  | GPIO_PIN_3  |
		                   GPIO_PIN_4  | GPIO_PIN_5   | GPIO_PIN_6  | GPIO_PIN_7  |
						   GPIO_PIN_8  | GPIO_PIN_9   | GPIO_PIN_10 | GPIO_PIN_11 |
						   GPIO_PIN_12 | GPIO_PIN_13  | GPIO_PIN_14 | GPIO_PIN_15 ;
  GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);


  /* Disable GPIOs clock */
  __HAL_RCC_GPIOA_CLK_DISABLE();
  __HAL_RCC_GPIOB_CLK_DISABLE();
  __HAL_RCC_GPIOC_CLK_DISABLE();
  __HAL_RCC_GPIOD_CLK_DISABLE();
  __HAL_RCC_GPIOE_CLK_DISABLE();
  __HAL_RCC_GPIOH_CLK_DISABLE();


}

void dump_regs()
{
	  /* Dump registers */
	  uint8_t val;
	  char buf[8];

	  for (int r = 0; r < 12*16; r++) {
		  if (regs[r] != 0xff) {
			  val = 0xff;
			  RadioSpiReadRegisters(regs[r], 1, &val);
			  sprintf(buf, "%02x=%02x\r\n", regs[r], val);
			  HAL_UART_Transmit(&huart1, buf, 7, 1000);
		  }
	  }

}
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_RTC_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
  HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  HAL_GPIO_WritePin(GPIO0_GPIO_Port, GPIO0_Pin, GPIO_PIN_RESET);

  uint8_t rbuf[2] = { 0xaa, 0xbb };

  HAL_StatusTypeDef res = HAL_TIMEOUT;
  uint8_t addr = 0x00;
  while ((res != HAL_OK) && (addr < 128)) {
	   res =  HAL_I2C_IsDeviceReady(&hi2c1, addr << 1, 2, 1000);
	   HAL_Delay(200);
	   addr += 1;

  }

  if ((res == HAL_OK) && (addr < 0x46))
  {
		for (int i = 0; i < 2000; i++) {
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
			HAL_Delay(50);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
			HAL_Delay(50);

		}
  }

//  res = HAL_TIMEOUT;
//  while (res != HAL_OK) {
//	  res = HAL_I2C_Mem_Read(&hi2c1, 0x80, 0x11, 1, rbuf, 1, 5000);
//	  HAL_Delay(1000);
//  }


  if (mode == TX_MODE) {
		while (1) {
			HAL_Spirit1_Init();
			for (int i = 0; i < 1; i++) {
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
				HAL_Delay(50);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
				HAL_Delay(50);

			}


			Enter_Low_Power(3);
			Exit_Low_Power();
			//dump_regs();

			AppliFrame_t txFrame;
		    txFrame.Cmd = LED_TOGGLE;
		    txFrame.CmdLen = 0x01;
		    txFrame.Cmdtag = 8;
		    txFrame.CmdType = 2;
		    txFrame.DataBuff = aTransmitBuffer;
		    txFrame.DataLen = 5;

		    AppliSendBuff(&txFrame, txFrame.DataLen);
		    HAL_UART_Transmit(&huart1, "Sent\n", 8, 1000);

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

		}

  }
  else
  {
	  Spirit1_Configure();

	  while (1) {

		  uint8_t cRxlen = 0;
	      AppliReceiveBuff(aReceiveBuffer, cRxlen);

	  }
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  __PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
