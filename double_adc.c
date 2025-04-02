#include "stm32f4xx_hal.h"
#include "arm_math.h"

#define FFT_SIZE 1024  // FFT 采样点数

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;

uint16_t adc1_buffer[FFT_SIZE];  // 存储 ADC1 采样数据
uint16_t adc2_buffer[FFT_SIZE];  // 存储 ADC2 采样数据
float input_signal1[FFT_SIZE];   // ADC1 处理后的浮点数据
float input_signal2[FFT_SIZE];   // ADC2 处理后的浮点数据

void SystemClock_Config(void);
void MX_ADC1_Init(void);
void MX_ADC2_Init(void);
void MX_DMA_Init(void);

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_DMA_Init();
    MX_ADC1_Init();
    MX_ADC2_Init();
    
    // 启动 ADC + DMA
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc1_buffer, FFT_SIZE);
    HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adc2_buffer, FFT_SIZE);

    while (1) {
        // 处理 ADC 数据（转换为浮点数）
        for (int i = 0; i < FFT_SIZE; i++) {
            input_signal1[i] = (float)adc1_buffer[i];
            input_signal2[i] = (float)adc2_buffer[i];
        }
        
        // 进行 FFT 分析
        arm_rfft_fast_instance_f32 fft_instance;
        arm_rfft_fast_init_f32(&fft_instance, FFT_SIZE);
        float output_signal1[FFT_SIZE];
        float output_signal2[FFT_SIZE];
        arm_rfft_fast_f32(&fft_instance, input_signal1, output_signal1, 0);
        arm_rfft_fast_f32(&fft_instance, input_signal2, output_signal2, 0);
        
        // 其他数据处理...
    }
