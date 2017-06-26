/**
  ******************************************************************************
  * @file    Templates/Src/main.c
  * @author  MCD Application Team
  * @version V1.2.5
  * @date    17-February-2017
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
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
#include "ili9488.h"
#include "stlogo.h"
#include "i2c.h"
#include "voc.h"

extern FontDef_t Font_7x10;
extern FontDef_t Font_11x18;
extern FontDef_t Font_16x26;

extern unsigned char logo[];

/** @addtogroup STM32F4xx_HAL_Examples
  * @{
  */

/** @addtogroup Templates
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define I2C_TIMEOUT     10000

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* UART handler declaration */
UART_HandleTypeDef UartHandle;
UART_HandleTypeDef UartHandle_CO2;
/* SPI handler declaration */
SPI_HandleTypeDef SpiHandle;
/* I2C handler declaration */
I2C_HandleTypeDef I2cHandle;

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Error_Handler(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
#ifdef __GNUC__
/* With GCC, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
int __io_putchar(int ch)
#else
int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART1 and Loop until the end of transmission */
  HAL_UART_Transmit(&UartHandle, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}


#define HIH6130_I2C_ADDRESS        (0x27 << 1)

uint8_t HIH6130_Read(float* humidity, float* temperature)
{
  uint8_t res = 0;
  uint8_t buf[4];

  memset(buf, 0, sizeof(buf));
  res = I2C_Write(HIH6130_I2C_ADDRESS, NULL, 0);
  printf("I2C W - %d\r\n", res);
  HAL_Delay(100);
  res = I2C_Read(HIH6130_I2C_ADDRESS, buf, sizeof(buf));
  printf("H/T R - %d\r\n", res);
  printf("Data: ");
  for (int i = 0; i < sizeof(buf); i++) {
    printf("%02x ", buf[i]);
  }
  printf("\r\n");
  uint16_t h = (((uint16_t)buf[0]) << 8) | buf[1];
  uint16_t t = ((((uint16_t)buf[2]) << 8) | buf[3]) / 4;
  float rh = (float)h * 6.10e-3;
  float rt = (float)t * 1.007e-2 - 40.0;
  printf("H: %.1f, T: %.1f\r\n", rh, rt);

  *humidity = rh;
  *temperature = rt;

  return 0;
}

uint8_t S8_Read(uint16_t *c)
{
  uint8_t cmd[8] = {0xFE, 0x04, 0x00, 0x03, 0x00, 0x01, 0xD5, 0xC5};
  uint8_t rcv[8];
  memset(rcv, 0, sizeof(rcv));
  HAL_UART_Transmit(&UartHandle_CO2, cmd, 8, 0xFFFF);
  HAL_UART_Receive(&UartHandle_CO2, rcv, 7, 0xFFFF);
  for (int i = 0; i < 7; i++) {
    printf("0x%02x ", rcv[i]);
  }
  printf("\r\n");
  uint16_t co2 = rcv[3] << 8 | rcv[4];
  printf("CO2: %d\r\n", co2);

  return 0;
}

void DispBasic(void)
{
//  ILI9488_DrawRectangle(10, 100, 310, 400, 0xFFFF00);
  ILI9488_DrawFilledRectangle(10, 100, 310, 300, 0xFFFF00);
  ILI9488_Puts(64, 120, "VOC:", &Font_16x26, 0x000000, 0xFFFFFF);
  ILI9488_Puts(64, 150, "CO2:", &Font_16x26, 0x000000, 0xFFFFFF);
  ILI9488_Puts(64, 180, "HUM:", &Font_16x26, 0x000000, 0xFFFFFF);
  ILI9488_Puts(64, 210, "TEM:", &Font_16x26, 0x000000, 0xFFFFFF);
}

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{

  /* STM32F4xx HAL library initialization:
       - Configure the Flash prefetch, Flash preread and Buffer caches
       - Systick timer is configured by default as source of time base, but user
             can eventually implement his proper time base source (a general purpose
             timer for example or other time source), keeping in mind that Time base
             duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and
             handled in milliseconds basis.
       - Low Level Initialization
     */
  HAL_Init();

  /* Configure the system clock to 168 MHz */
  SystemClock_Config();


  /* Add your application code here
     */
  /*##-1- Configure the UART peripheral ######################################*/
  /* Put the USART peripheral in the Asynchronous mode (UART Mode) */
  /* UART1 configured as follow:
      - Word Length = 8 Bits
      - Stop Bit = One Stop bit
      - Parity = None
      - BaudRate = 115200 baud
      - Hardware flow control disabled (RTS and CTS signals) */
  UartHandle.Instance          = USARTx;

  UartHandle.Init.BaudRate     = 115200;
  UartHandle.Init.WordLength   = UART_WORDLENGTH_8B;
  UartHandle.Init.StopBits     = UART_STOPBITS_1;
  UartHandle.Init.Parity       = UART_PARITY_NONE;
  UartHandle.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  UartHandle.Init.Mode         = UART_MODE_TX_RX;
  UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;

  if (HAL_UART_Init(&UartHandle) != HAL_OK)
  {
    Error_Handler();
  }

  printf("Hello STM32F407\r\n");

  /*##-1- Configure the UART peripheral ######################################*/
  /* Put the USART peripheral in the Asynchronous mode (UART Mode) */
  /* UART1 configured as follow:
      - Word Length = 8 Bits
      - Stop Bit = One Stop bit
      - Parity = None
      - BaudRate = 9600 baud
      - Hardware flow control disabled (RTS and CTS signals) */
  UartHandle_CO2.Instance          = USART_CO2;

  UartHandle_CO2.Init.BaudRate     = 9600;
  UartHandle_CO2.Init.WordLength   = UART_WORDLENGTH_8B;
  UartHandle_CO2.Init.StopBits     = UART_STOPBITS_1;
  UartHandle_CO2.Init.Parity       = UART_PARITY_NONE;
  UartHandle_CO2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  UartHandle_CO2.Init.Mode         = UART_MODE_TX_RX;
  UartHandle_CO2.Init.OverSampling = UART_OVERSAMPLING_16;

  if (HAL_UART_Init(&UartHandle_CO2) != HAL_OK)
  {
    Error_Handler();
  }

#if 0
  HAL_UART_Transmit(&UartHandle_CO2, (uint8_t *)"UART1\r\n", 7, 0xFFFF);
  while (1) {
    uint8_t buf[8];
    memset(buf, 0, sizeof(buf));
    HAL_UART_Receive(&UartHandle_CO2, buf, 1, 0xFFFF);
    if (buf[0]) {
      printf("%c", buf[0]);
    }
  }
#endif

#if 0
  // Test Sensair
  while (1) {
    uint8_t cmd[8] = {0xFE, 0x04, 0x00, 0x03, 0x00, 0x01, 0xD5, 0xC5};
    uint8_t rcv[8];
    memset(rcv, 0, sizeof(rcv));
    HAL_UART_Transmit(&UartHandle_CO2, cmd, 8, 0xFFFF);
    HAL_UART_Receive(&UartHandle_CO2, rcv, 7, 0xFFFF);
    for (int i = 0; i < 7; i++) {
      printf("0x%02x ", rcv[i]);
    }
    printf("\r\n");
    uint16 co2 = buf[3] << 8 | buf[4];
    printf("CO2: %d\r\n", co2);
    HAL_Delay(1000);
  }
#endif

  /*##-1- Configure the SPI peripheral #######################################*/
  /* Set the SPI parameters */
  SpiHandle.Instance               = SPIx;

  SpiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  SpiHandle.Init.Direction         = SPI_DIRECTION_2LINES;
  SpiHandle.Init.CLKPhase          = SPI_PHASE_1EDGE;
  SpiHandle.Init.CLKPolarity       = SPI_POLARITY_LOW;
  SpiHandle.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
  SpiHandle.Init.CRCPolynomial     = 7;
  SpiHandle.Init.DataSize          = SPI_DATASIZE_8BIT;
  SpiHandle.Init.FirstBit          = SPI_FIRSTBIT_MSB;
  SpiHandle.Init.NSS               = SPI_NSS_SOFT;
  SpiHandle.Init.TIMode            = SPI_TIMODE_DISABLE;
  SpiHandle.Init.Mode              = SPI_MODE_MASTER;

  if(HAL_SPI_Init(&SpiHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

#if 0
  // Test LCD
  ILI9488_Init();

  ILI9488_Puts(0, 0, "Honeywell Connected Air Stat", &Font_16x26, 0x000000, 0xFFFFFF);
  ILI9488_Puts(160, 240, "Hello 9488", &Font_11x18, 0x000000, 0xFFFFFF);
  ILI9488_Puts(0, 400, "320RGB x 480 Resolution and 16.7M-color", &Font_7x10, 0x000000, 0xFFFFFF);
  ILI9488_Puts(0, 60, "Honeywell", &Font_16x26, 0xFF0000, 0xFFFFFF);
  ILI9488_Puts(0, 90, "Honeywell", &Font_16x26, 0x00FF00, 0x0007FF);
  ILI9488_Puts(0, 120, "Honeywell", &Font_16x26, 0x0000FF, 0x000000);

  ILI9488_DrawBitmap(0, 0, stlogo);
#endif

  /*##-1- Configure the I2C peripheral ######################################*/
  I2cHandle.Instance             = I2Cx;

  I2cHandle.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
  I2cHandle.Init.ClockSpeed      = 100000;
  I2cHandle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  I2cHandle.Init.DutyCycle       = I2C_DUTYCYCLE_16_9;
  I2cHandle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  I2cHandle.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
  I2cHandle.Init.OwnAddress1     = (0x5A << 1);
  I2cHandle.Init.OwnAddress2     = 0xFE;

  if(HAL_I2C_Init(&I2cHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

#if 1
  ILI9488_Init();
  ILI9488_Puts(88, 0, "Honeywell", &Font_16x26, 0xFF0000, 0xFFFFFF);
  ILI9488_Puts(16, 30, "Connected Air Stat", &Font_16x26, 0x0000FF, 0xFFFFFF);
  ILI9488_DrawBitmap(70, 10, logo);
  DispBasic();
  while (1) {
    float h, t;
    uint16_t co2, voc;

    Get_VocData(&co2, &voc);
    HIH6130_Read(&h, &t);
    S8_Read(&co2);

    char str[32];
    memset(str, 0, sizeof(str));
    sprintf(str, "%dppm", voc);
    ILI9488_Puts(160, 120, str, &Font_16x26, 0x000000, 0xFFFFFF);
    sprintf(str, "%dppm", co2);
    ILI9488_Puts(160, 150, str, &Font_16x26, 0x000000, 0xFFFFFF);
    sprintf(str, "%.1f%%", h);
    ILI9488_Puts(160, 180, str, &Font_16x26, 0x000000, 0xFFFFFF);
    sprintf(str, "%.1f", t);
    ILI9488_Puts(160, 210, str, &Font_16x26, 0x000000, 0xFFFFFF);

    HAL_Delay(2000);
  }
#endif

  /* Infinite loop */
  while (1)
  {
  }
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 168000000
  *            HCLK(Hz)                       = 168000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 8000000
  *            PLL_M                          = 8
  *            PLL_N                          = 336
  *            PLL_P                          = 2
  *            PLL_Q                          = 7
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 5
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();

  /* The voltage scaling allows optimizing the power consumption when the device is
     clocked below the maximum system frequency, to update the voltage scaling value
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /* STM32F405x/407x/415x/417x Revision Z devices: prefetch is supported  */
  if (HAL_GetREVID() == 0x1001)
  {
    /* Enable the Flash prefetch */
    __HAL_FLASH_PREFETCH_BUFFER_ENABLE();
  }
}
/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  /* User may add here some code to deal with this error */
  while(1)
  {
  }
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
