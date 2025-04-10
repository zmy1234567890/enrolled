#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include <math.h>


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "OLED.h"
#include <stdio.h>
#include <string.h>
#include "arm_math.h" 
#include "usart.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */



#define ADC_BUF_SIZE 1024  // FFT 输入大小（必须是 2 的幂）
#define SAMPLING_RATE 26667  // 实际采样率，单位 Hz
#define FFT_SIZE 1024
#define SAMPLE_RATE 26667
#define k 3.846
#define M_PI 3.1415926
#define kADC 1086
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
int max_index = 0;
float window[FFT_SIZE];
float phase1,phase2;
float amplitude_ratio;
float rms1, peak1;
float rms2, peak2;
int STATE=0;
uint16_t check1[5]={0,0,0,0,0};
uint16_t check2[5]={0,0,0,0,0};
float magnitude1[FFT_SIZE / 2];//// 幅度数组
float magnitude2[FFT_SIZE / 2];
uint16_t adc_buf1[ADC_BUF_SIZE]={0,0};  // ADC DMA 缓冲区
uint16_t adc_buf2[ADC_BUF_SIZE];
float input11_f32[ADC_BUF_SIZE];   // FFT 输入缓冲区
float input12_f32[ADC_BUF_SIZE]; 
float output1_f32[ADC_BUF_SIZE];  // FFT 计算结果
float input21_f32[ADC_BUF_SIZE];  
float input22_f32[ADC_BUF_SIZE]; 
float output2_f32[ADC_BUF_SIZE]; 
arm_rfft_fast_instance_f32 fft_instance;// 定义 FFT 结构体
char message[128];               // 串口发送缓冲区
typedef struct {
    float frequency;  // 主频
    float amplitude;  // 幅值
    char waveType[10];  // 波形类型
} SignalInfo;
SignalInfo signal1;
SignalInfo signal2;
float values[3];
float main_frequency2;//主频
float main_frequency1;
const char* waveform1;
const char* waveform2;
float phase_diff;
float thd;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
//全局变量

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void generate_hanning_window() {
    for (int i = 0; i < FFT_SIZE; i++) {
        window[i] = 0.5f * (1.0f - cosf(2 * PI * i / (FFT_SIZE - 1)));
    }
}
void apply_window_and_prepare_fft(float* input, float* output, float* window) {
    for (int i = 0; i < FFT_SIZE; i++) {
        output[i] = input[i] * window[i];
    }
}

float find_max_frequency1() {
    uint32_t maxIndex;
    float maxValue;

    // 找到最大幅度和对应的索引
    arm_max_f32(magnitude1, FFT_SIZE / 2, &maxValue, &maxIndex);

    // 计算主频
    float frequency = (float)maxIndex * (SAMPLE_RATE / FFT_SIZE);
    return frequency;
}
float find_max_frequency2() {
    uint32_t maxIndex;
    float maxValue;

    // 找到最大幅度和对应的索引
    arm_max_f32(magnitude2, FFT_SIZE / 2, &maxValue, &maxIndex);

    // 计算主频
    float frequency = (float)maxIndex * (SAMPLE_RATE / FFT_SIZE);
    return frequency;
}
const char* identify_waveform1() {
    int fundamental_index = 1;
    float max_magnitude = magnitude1[1];
    
    // 1. 寻找最大幅值对应的索引（主频索引）
    for (int i = 2; i < FFT_SIZE / 2; i++) {
        if (magnitude1[i] > max_magnitude) {
            max_magnitude = magnitude1[i];
            fundamental_index = i;
        }
    }
    
    // 2. 计算基波频率
    float fundamental_freq = (fundamental_index * SAMPLE_RATE) / FFT_SIZE;
    
    // 3. 设定谐波判定阈值（基波的10%）
    float threshold = max_magnitude * 0.3f;
    int harmonic_count = 0;
    
    // 4. 计算谐波个数（从2倍频开始判断）
    for (int i = 2 * fundamental_index; i < FFT_SIZE / 2; i += fundamental_index) {
        if (magnitude1[i] > threshold) {
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
}
const char* identify_waveform2() {
    int harmonic_count = 0;

    // 1. 找到基波索引（跳过直流分量）
    int base_index = 1;
    float base_amp = magnitude2[1];

    for (int i = 2; i < FFT_SIZE / 2; i++) {
        if (magnitude2[i] > base_amp) {
            base_amp = magnitude2[i];
            base_index = i;
        }
    }

    float threshold = base_amp * 0.05f;

    // 2. 判断谐波（2~9次谐波是否大于阈值）
    for (int i = 2; i <= 9; i++) {
        int idx = base_index * i;
        if (idx >= FFT_SIZE / 2) break;
        if (magnitude2[idx] > threshold) {
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

void calculate_amplitude1(float* rms, float* peak) {//求幅度
    float sum_sq = 0.0f;
    float max_val = 0.0f;

    for (int i = 0; i < FFT_SIZE; i++) {
        sum_sq +=  input12_f32[i] *  input12_f32[i];  // 计算平方和
        if (fabs( (float)input12_f32[i]) > max_val) {
            max_val = fabs((float)input12_f32[i]);  // 找最大值
        }
    }

    *rms = sqrtf(sum_sq / FFT_SIZE);  // 计算均方根
    *peak = max_val/kADC;                  // 计算峰值
}
void calculate_amplitude2(float* rms, float* peak) {//求幅度
    float sum_sq = 0.0f;
    float max_val = 0.0f;

    for (int i = 0; i < FFT_SIZE; i++) {
        sum_sq +=  input22_f32[i] *  input22_f32[i];  // 计算平方和
        if (fabs( (float)input22_f32[i]) > max_val) {
            max_val = fabs((float)input22_f32[i]);  // 找最大值
        }
    }

    *rms = sqrtf(sum_sq / FFT_SIZE);  // 计算均方根
    *peak = max_val/kADC;                  // 计算峰值
}
void calculate_magnitude1() {// 计算幅度
    for (int i = 0; i < FFT_SIZE / 2; i++) {
        float real = output1_f32[2 * i];       // 实部
        float imag = output1_f32[2 * i + 1];   // 虚部
        magnitude1[i] = sqrtf(real * real + imag * imag);  // 计算幅度
    }
}
void calculate_magnitude2() {// 计算幅度
    for (int i = 0; i < FFT_SIZE / 2; i++) {
        float real = output2_f32[2 * i];       // 实部
        float imag = output2_f32[2 * i + 1];   // 虚部
        magnitude2[i] = sqrtf(real * real + imag * imag);  // 计算幅度
    }
}
float find_main_frequency1(void) {// 计算 FFT 结果中的主频率
    float max_magnitude = 0.0f;
    max_index = 0;

    // 遍历 FFT 结果，找到最大幅值对应的索引
    for (int i = 1; i < FFT_SIZE / 2; i++) { // 只需要遍历一半 (对称性)
        float real = output1_f32[2 * i];     // 取实部
        float imag = output1_f32[2 * i + 1]; // 取虚部
        float magnitude = sqrt(real * real + imag * imag); // 计算幅值

        if (magnitude > max_magnitude) {
            max_magnitude = magnitude;
            max_index = i;
        }
    }

    // 计算主频率
    float main_frequency = (max_index * SAMPLE_RATE) / FFT_SIZE;
    return main_frequency;
}
float find_main_frequency2(void) {// 计算 FFT 结果中的主频率
    float max_magnitude = 0.0f;
    max_index = 0;

    // 遍历 FFT 结果，找到最大幅值对应的索引
    for (int i = 1; i < FFT_SIZE / 2; i++) { // 只需要遍历一半 (对称性)
        float real = output2_f32[2 * i];     // 取实部
        float imag = output2_f32[2 * i + 1]; // 取虚部
        float magnitude = sqrt(real * real + imag * imag); // 计算幅值

        if (magnitude > max_magnitude) {
            max_magnitude = magnitude;
            max_index = i;
        }
    }

    // 计算主频率
    float main_frequency = (max_index * SAMPLE_RATE) / FFT_SIZE;
    return main_frequency;
}
void process_signal1() {
    // **执行 FFT**
    arm_rfft_fast_init_f32(&fft_instance, FFT_SIZE);
    arm_rfft_fast_f32(&fft_instance, input11_f32,output1_f32, 0);

    // **计算幅度谱**
    calculate_magnitude1();

    // **计算主频**
    main_frequency1 =k* find_main_frequency1();

    // **计算幅度**
 
    calculate_amplitude1(&rms1, &peak1);

    // **识别波形**
    waveform1 = identify_waveform1();

    // **输出结果**
    printf("n0.val=%dV\n\xff\xff\xff", (int)main_frequency1);
    printf("峰值: %.2f V\n", peak1);
    printf("波形类型: %s\n", waveform1);
}

void process_signal2() {
    // **执行 FFT**
    arm_rfft_fast_init_f32(&fft_instance, FFT_SIZE);
	  
    arm_rfft_fast_f32(&fft_instance, input21_f32,output2_f32, 0);

    // **计算幅度谱**
    calculate_magnitude2();

    // **计算主频**
    main_frequency2 = k*find_main_frequency2();

    // **计算幅度**
   
    calculate_amplitude2(&rms2, &peak2);

    // **识别波形**
    waveform2 = identify_waveform2();

    // **输出结果**
   printf("n1.val=%d\xff\xff\xff", (int)main_frequency2);
    printf("均方根幅度: %.2f V\n", rms2);
    printf("峰值: %.2f V\n", peak2);
    printf("波形类型: %s\n", waveform2);
}
void trans1(){
	for(int i=0;i<1024;i++){
	input11_f32[i]=(float)adc_buf1[i];
	input12_f32[i]=(float)adc_buf1[i];
		
}
	}
void trans2(){
	for(int i=0;i<1024;i++){
	input21_f32[i]=(float)adc_buf2[i];
	input22_f32[i]=(float)adc_buf2[i];
		
  }
}
float calculate_amplitude_ratio(float *magnitude1, float *magnitude2, int maxIndex) {
    float amplitude1 = magnitude1[maxIndex];  // 信号1的幅度
    float amplitude2 = magnitude2[maxIndex];  // 信号2的幅度
    
    // 计算幅度比值
    if (amplitude2 != 0) {
        return amplitude1 / amplitude2;  // 返回幅度比值
    } else {
        return 0;  // 如果信号2的幅度为0，返回0（避免除零错误）
    }
}
float calculate_phase_diff(float *signal1, float *signal2, int maxIndex) {
    // 计算信号1和信号2在主频位置的相位
   phase1 = atan2(signal1[2 * maxIndex + 1], signal1[2 * maxIndex]);
   phase2 = atan2(signal2[2 * maxIndex + 1], signal2[2 * maxIndex]);
    
    // 计算相位差
    float phase_diff = phase1 - phase2;
    
    // 保证相位差在[-π, π]范围内
    if (phase_diff > M_PI) {
        phase_diff -= 2 * M_PI;
    } else if (phase_diff < -M_PI) {
        phase_diff += 2 * M_PI;
    }
    
    return phase_diff;  // 返回相位差
}

float calculate_thd(const float* magnitude, int fundamental_index) {
    float v1 = magnitude[fundamental_index];
    float sum = 0.0f;
    for (int i = 2; i <= 6; i++) {
        int idx = i * fundamental_index;
        if (idx < FFT_SIZE / 2) {
            sum += magnitude[idx] * magnitude[idx];
        }
    }
    return sqrtf(sum) / v1 * 100.0f;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){ //回调函数
	trans2();
	trans1();
	for(int i=0;i<FFT_SIZE;i++){
	printf("%f,%f\n",input11_f32[i],input21_f32[i]);
	}
	apply_window_and_prepare_fft(input12_f32,input11_f32,window);
	apply_window_and_prepare_fft(input22_f32,input21_f32,window);
	process_signal2();
  process_signal1();
	phase_diff=calculate_phase_diff(output1_f32,output2_f32,max_index);
	amplitude_ratio=calculate_amplitude_ratio(magnitude1,magnitude2, max_index);
	/*if(hadc == &hadc1&&STATE==0){
    if(check1[4]<=10){//当CH1没有信号，重新检测
		HAL_ADC_Start_DMA(&hadc1,(uint32_t*)check1,5);
		}
    if(check1[4]>10){
			HAL_ADC_Start_DMA(&hadc2,(uint32_t*)check2,5);//当CH1有信号输入，检查是否有信号进入CH2
		}  
	}
	
	if(hadc == &hadc2&&STATE==0){//CH1有信号后走到这一步
     if(check2[4]>=10){//CH2也有信号走向状态2，进行发展指标等判断
			 STATE=2;
		 }
		 else{//CH2无信号，走向单一信号的判断
		  STATE=1;
		 }
	}
	if(hadc == &hadc1&&STATE==1){//单一信号判断

    STATE=3;
		uint32_t adc_vrefint = (uint32_t)3.3f;; //values[2];//获取内部参考电压

		// 计算实际 VDD

		        // 计算电压（修正 ADC 结果）
		        for (int i = 0; i < FFT_SIZE; i++) {
		            input1_f32[i] = ((float)adc_buf1[i]) * (3.3f / 4096.0f);//要改要改要改！！！！！！！！！！！！！！！
		        }

		        process_signal1(); // 进行 FFT 和分析
	}
	if(hadc == &hadc1){
	//CH2
	for (int i = 0; i < FFT_SIZE; i++) {
		            input2_f32[i] = ((float)adc_buf2[i]) * (3.3f / 4096.0f);//要改要改要改！！！！！！！！！！！！！！！
		        }

		        process_signal2(); 
	}
	if((hadc == &hadc1||hadc == &hadc2)&&STATE==2){
	STATE=4;
	}*/
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
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_ADC2_Init();
	
	
  /* USER CODE BEGIN 2 */
	
	HAL_ADC_Start_DMA(&hadc2,(uint32_t*)adc_buf2,1024);
		HAL_ADC_Start_DMA(&hadc1,(uint32_t*)adc_buf1,1024);
	HAL_TIM_Base_Start(&htim2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	if(STATE==1){
	HAL_ADC_Start_DMA(&hadc1,(uint32_t*)adc_buf1,1024);
	}
	if(STATE==2){
	HAL_ADC_Start_DMA(&hadc1,(uint32_t*)adc_buf1,1024);
	HAL_ADC_Start_DMA(&hadc2,(uint32_t*)adc_buf2,1024);
	}
	if(STATE==3){
	process_signal1();
	}
	if(STATE==4){
	//此处是拓展指标代码
	}
	
	
		

//		printf("hello");
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
