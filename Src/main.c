/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
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
#include "stm32f4xx_hal.h"
#include "dma.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "math.h"
#include "arm_math.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define FFT_AC_COUPLING_HZ  1000.0f

arm_rfft_fast_instance_f32 S;
float FFT_SampleRate;
int32_t FFT_inp_int32[PDM_SAMPLE_SIZE] = {0};
float FFT_inp[PDM_SAMPLE_SIZE] = {0.0f};
float FFT_oup[PDM_SAMPLE_SIZE] = {0.0f};
float FFT_mag[PDM_SAMPLE_SIZE/2] = {0.0f};
float FFT_dB[PDM_SAMPLE_SIZE/2] = {0.0f};
float FFT_frq[PDM_SAMPLE_SIZE/2] = {0.0f};
float FFT_window[PDM_SAMPLE_SIZE] = {0.0f};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_SPI1_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
    printf("\r\n***** Program start *****\r\n");

    // Initialize
    needs_CopyPDMbits = false;
    initializeCicFilterStruct(/* Order = */3, DECIMATION_M, &PDM_st);
    startPDM();
    HAL_NVIC_DisableIRQ(DMA2_Stream0_IRQn);
    HAL_NVIC_DisableIRQ(SPI1_IRQn);


    // FFT init
    float SPI1_BaudRate = (float)SystemCoreClock / 1.0f / 32.0f;
    FFT_SampleRate = SPI1_BaudRate / DECIMATION_M;

    // Hanning window
    const float tmp = 2.0f * M_PI / (float) PDM_SAMPLE_SIZE;
    for (uint32_t i = 0; i < PDM_SAMPLE_SIZE; i++)
        *(FFT_window + i) = 0.5f - 0.5f * arm_cos_f32((float) i * tmp);

    for (uint32_t i = 0; i < PDM_SAMPLE_SIZE / 2; i++)
        *(FFT_frq + i) = (float) i * (float) FFT_SampleRate / (float) PDM_SAMPLE_SIZE;

    arm_rfft_fast_init_f32(&S, PDM_SAMPLE_SIZE);

    HAL_Delay(1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
    {
        HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
        HAL_NVIC_EnableIRQ(SPI1_IRQn);
        isCalled_PDM_DMA_Callback = false;
        needs_CopyPDMbits = true;
        while (isCalled_PDM_DMA_Callback == false);

        // Software CIC Filter
        executeCicFilter((uint8_t*) PDM_RawBits, PDM_SAMPLE_SIZE * DECIMATION_M,
                (int32_t*) PDM_Filtered_int32, &PDM_st);

        // Data copy
        for (uint32_t i = 0; i < PDM_SAMPLE_SIZE; i++)
            FFT_inp[i] = (float) PDM_Filtered_int32[i];

        // Windowing
        arm_mult_f32(FFT_inp, FFT_window, FFT_inp, PDM_SAMPLE_SIZE);

        // Execute FFT
        arm_rfft_fast_f32(&S, FFT_inp, FFT_oup, 0);

        // calculate magnitude
        arm_cmplx_mag_f32(FFT_oup, FFT_mag, PDM_SAMPLE_SIZE / 2);

        // Normalization of magnitude
        arm_scale_f32(FFT_mag, 1.0f / sqrtf((float) PDM_SAMPLE_SIZE), FFT_mag, PDM_SAMPLE_SIZE / 2);

        // AC coupling
        for (uint32_t i = 0; i < PDM_SAMPLE_SIZE / 2; i++)
        {
            if (*(FFT_frq + i) < FFT_AC_COUPLING_HZ)
                FFT_mag[i] = 1.0f;
            else
                break;
        }

        float inv_dB_base_mag = 1.0f / 1.0f;
        for (uint32_t i = 0; i < PDM_SAMPLE_SIZE / 2; i++)
            FFT_dB[i] = 10.0f * log10f(FFT_mag[i] * inv_dB_base_mag);

        // calc max mag
        float mag_max, frq_max;
        uint32_t maxIndex;
        arm_max_f32(FFT_mag, PDM_SAMPLE_SIZE / 2, &mag_max, &maxIndex);
        frq_max = *(FFT_frq + maxIndex);

        printf("\r\nSampleRate=%d, frq_max = %.1f, mag_max = %f\r\n", (int) FFT_SampleRate, frq_max, mag_max);
        for (uint32_t i = 0; i < PDM_SAMPLE_SIZE / 2; i++)
        {
            printf("%.1f\t%f\r\n", FFT_frq[i], FFT_dB[i]);
        }

        //while(1);
        HAL_NVIC_DisableIRQ(DMA2_Stream0_IRQn);
        HAL_NVIC_DisableIRQ(SPI1_IRQn);
        HAL_Delay(1000);
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 400;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

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
