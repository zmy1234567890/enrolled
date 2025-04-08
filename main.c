/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "arm_math.h"  // CMSIS DSP

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define ADC_BUF_SIZE 1024  // FFT 输入大小（必须是 2 的幂）
#define SAMPLING_RATE 112903  // 实际采样率，单位 Hz
float magnitude[FFT_SIZE / 2];//// 幅度数组
float window[FFT_SIZE];//窗口数组

uint32_t adc_buf[ADC_BUF_SIZE];  // ADC DMA 缓冲区
float input_f32[ADC_BUF_SIZE];   // FFT 输入缓冲区
float output_f32[ADC_BUF_SIZE];  // FFT 计算结果
arm_rfft_fast_instance_f32 fft_instance;// 定义 FFT 结构体
char message[128];               // 串口发送缓冲区

typedef struct {
    float frequency;  // 主频
    float amplitude;  // 幅值
    char waveType[10];  // 波形类型
} SignalInfo;

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	if(hadc == &hadc1){


		uint32_t adc_vrefint = values[2];//获取内部参考电压

		// 计算实际 VDD
		float vdd = (3.3f * (*VREFINT_CAL_ADDR)) / adc_vrefint;

		        // 计算电压（修正 ADC 结果）
		        for (int i = 0; i < 2; i++) {
		            input_f32[i] = ((float)values[i]) * (vdd / 4096.0f);
		        }

		        process_signal(); // 进行 FFT 和分析
	}
}

//高贵的窗口函数
//float window[FFT_SIZE];

// 1. 生成 Hanning 窗口
void generate_hanning_window() {
    for (int i = 0; i < FFT_SIZE; i++) {
        window[i] = 0.5f * (1.0f - cosf(2 * PI * i / (FFT_SIZE - 1)));
    }
}

// 2. 对原始数据加窗，填充为复数输入
void apply_window_and_prepare_fft(float* input, float* output, float* window) {
    for (int i = 0; i < FFT_SIZE; i++) {
        output[i] = input[i] * window[i];
    }
}

void calculate_magnitude() {
    for (int i = 0; i < FFT_SIZE / 2; i++) {
        float real = output_signal[2 * i];       // 实部
        float imag = output_signal[2 * i + 1];   // 虚部
        magnitude[i] = sqrtf(real * real + imag * imag);  // 计算幅度
    }
}

float find_max_frequency() {
    uint32_t maxIndex;
    float maxValue;

    // 找到最大幅度和对应的索引
    arm_max_f32(magnitude, FFT_SIZE / 2, &maxValue, &maxIndex);

    // 计算主频
    float frequency = (float)maxIndex * (SAMPLE_RATE / FFT_SIZE);
    return frequency;
}

void process_signal() {
    // **执行 FFT**
	
  apply_hanning_window(input, FFT_SIZE);
    arm_rfft_fast_init_f32(&fft_instance, FFT_SIZE);
    arm_rfft_fast_f32(&fft_instance, input_signal, output_signal, 0);

    // **计算幅度谱**
    calculate_magnitude();

    // **计算主频**
    float main_frequency = find_main_frequency();

    // **计算幅度**
    float rms, peak;
    calculate_amplitude(&rms, &peak);

    // **识别波形**
    const char* waveform = identify_waveform();

    // **输出结果**
    printf("主频: %.2f Hz\n", main_frequency);
    printf("均方根幅度: %.2f V\n", rms);
    printf("峰值: %.2f V\n", peak);
    printf("波形类型: %s\n", waveform);
}

void calculate_amplitude(float* rms, float* peak) {//求幅度
    float sum_sq = 0.0f;
    float max_val = 0.0f;

    for (int i = 0; i < FFT_SIZE; i++) {
        sum_sq += input_signal[i] * input_signal[i];  // 计算平方和
        if (fabs(input_signal[i]) > max_val) {
            max_val = fabs(input_signal[i]);  // 找最大值
        }
    }

    *rms = sqrtf(sum_sq / FFT_SIZE);  // 计算均方根
    *peak = max_val;                  // 计算峰值
}

// **4. 识别波形类型**
/*const char* identify_waveform() {
    int harmonic_count = 0;
    float base_amp = magnitude[1];  // 基波幅度
    float threshold = base_amp * 0.1f;  // 设定阈值

    // 计算谐波分量
    for (int i = 2; i < FFT_SIZE / 2; i++) {
        if (magnitude[i] > threshold) {
            harmonic_count++;
        }
    }

    if (harmonic_count == 0) {
        return "正弦波";
    } else if (harmonic_count > 5) {
        return "方波";
    } else {
        return "三角波";
    }
}
*/
 /*const char* identify_waveform() {
    int fundamental_index = 1;
    float max_magnitude = magnitude[1];
    
    // 1. 寻找最大幅值对应的索引（主频索引）
    for (int i = 2; i < FFT_SIZE / 2; i++) {
        if (magnitude[i] > max_magnitude) {
            max_magnitude = magnitude[i];
            fundamental_index = i;
        }
    }
    
    // 2. 计算基波频率
    float fundamental_freq = (fundamental_index * SAMPLE_RATE) / FFT_SIZE;
    
    // 3. 设定谐波判定阈值（基波的10%）
    float threshold = max_magnitude * 0.1f;
    int harmonic_count = 0;
    
    // 4. 计算谐波个数（从2倍频开始判断）
    for (int i = 2 * fundamental_index; i < FFT_SIZE / 2; i += fundamental_index) {
        if (magnitude[i] > threshold) {
            harmonic_count++;
        }
    }
    
    // 5. 波形判断
    if (harmonic_count == 0) {
        return "正弦波"; // 只有基波，没有显著谐波
    } else if (harmonic_count > 5) {
        return "方波"; // 方波的谐波较多
    } else {
        return "三角波"; // 三角波的谐波衰减较快
    }
}*/
/*const char* identify_waveform() {
    int harmonic_count = 0;

    // 1. 找到基波索引（跳过直流分量）
    int base_index = 1;
    float base_amp = magnitude[1];

    for (int i = 2; i < FFT_SIZE / 2; i++) {
        if (magnitude[i] > base_amp) {
            base_amp = magnitude[i];
            base_index = i;
        }
    }

    float threshold = base_amp * 0.1f;

    // 2. 判断谐波（2~5次谐波是否大于阈值）
    for (int i = 2; i <= 5; i++) {
        int idx = base_index * i;
        if (idx >= FFT_SIZE / 2) break;
        if (magnitude[idx] > threshold) {
            harmonic_count++;
        }
    }

    // 3. 识别波形
    if (harmonic_count == 0) {
        return "正弦波";
    } else if (harmonic_count >= 3) {
        return "方波";
    } else {
        return "三角波";
    }
}
*/
const char* identify_waveform() {
    int base_index = 0;
	int max_magnitude;
	// 1. 寻找最大幅值对应的索引（主频索引）
    for (int i = 2; i < FFT_SIZE / 2; i++) {
        if (magnitude[i] > max_magnitude) {
            max_magnitude = magnitude[i];
            base_index = i;
        }
    }
    float base = magnitude[base_index];

    float total_energy = 0.0f;
    float harmonic_energy = 0.0f;

    for (int i = 1; i < size / 2; i++) {
        float mag_sq = magnitude[i] * magnitude[i];
        total_energy += mag_sq;
        if (i != base_index) {
            harmonic_energy += mag_sq;
        }
    }

    float ratio = harmonic_energy / total_energy; // 能量占比

    if (ratio < 0.05f) {
        return "正弦波";
    } else if (ratio > 0.2f) {
        return "方波";
    } else if (ratio > 0.08f && ratio < 0.18f) {
        return "三角波";
    } else {
        return "未知波形";
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
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  HAL_ADC_Start_DMA(&hadc1,(uint32_t*)values,sizeof(values)/sizeof(uint16_t));

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_112CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
