#include "arm_math.h"
#include "arm_const_structs.h"
#include "stdio.h"
#include "math.h"

#define FFT_SIZE        1024
#define SAMPLE_RATE     26000
#define kADC            1086.0f  // 根据ADC量程计算，ADC值 / kADC = 电压值
#define DEBUG_PRINT_INPUT 0
#define HARMONIC_THRESHOLD 0.1

extern uint16_t ADC_DMA1[FFT_SIZE];
extern uint16_t ADC_DMA2[FFT_SIZE];

// FFT输入输出
float input11_f32[FFT_SIZE];   // channel 1 原始
float input12_f32[FFT_SIZE*2];   // channel 1 加窗
float input21_f32[FFT_SIZE];   // channel 2 原始
float input22_f32[FFT_SIZE*2];   // channel 2 加窗
float output1_f32[FFT_SIZE * 2];
float output2_f32[FFT_SIZE * 2];

// 幅度数组（复用后半部分）
float magnitude1[FFT_SIZE / 2];
float magnitude2[FFT_SIZE / 2];

float window[FFT_SIZE];

float main_frequency1, main_frequency2;
float amplitude_peak1, amplitude_peak2;
float amplitude_ratio, phase_diff;
float thd1, thd2;

int max_index = 0;
char* wave1 = NULL;
char* wave2 = NULL;

void generate_hanning_window(void) {
    for (int i = 0; i < FFT_SIZE; i++) {
        window[i] = 0.5f - 0.5f * cosf(2 * PI * i / (FFT_SIZE - 1));
    }
}
void trans1(void) {
    for (int i = 0; i < FFT_SIZE; i++) {
        input11_f32[i] = (float)(ADC_DMA1[i]);
    }
}

void trans2(void) {
    for (int i = 0; i < FFT_SIZE; i++) {
        input21_f32[i] = (float)(ADC_DMA2[i]);
    }
}

// signal 为原始实数输入（如 ADC 转 float 的结果）
// fft_input_complex 为目标数组大小必须是 FFT_SIZE * 2（实虚交错）
void prepare_fft_input(float* fft_input_complex, float* signal, float* window) {
    for (int i = 0; i < FFT_SIZE; i++) {
        float sample = signal[i] * window[i];  // 应用窗函数
        fft_input_complex[2 * i]     = sample; // 实部
        fft_input_complex[2 * i + 1] = 0.0f;    // 虚部清零
    }
}

float find_main_frequency(float* fft_output, int* index) {
    float max_magnitude = 0.0f;
    int max_idx = 0;

    for (int i = 1; i < FFT_SIZE / 2; i++) {
        float real = fft_output[2 * i];
        float imag = fft_output[2 * i + 1];
        float magnitude = real * real + imag * imag;

        if (magnitude > max_magnitude) {
            max_magnitude = magnitude;
            max_idx = i;
        }
    }

    if (index != NULL) *index = max_idx;
    return ((float)max_idx * SAMPLE_RATE) / FFT_SIZE;
}


float calculate_vpp(uint16_t* adc_buf) {
    uint16_t max = 0, min = 0xFFFF;
    for (int i = 0; i < FFT_SIZE; i++) {
        if (adc_buf[i] > max) max = adc_buf[i];
        if (adc_buf[i] < min) min = adc_buf[i];
    }
    return (float)(max - min) / kADC;  // kADC 是 ADC 量化因子，转电压
}


float calculate_phase_diff(float* fft_output1, float* fft_output2, int index) {
    float real1 = fft_output1[2 * index];
    float imag1 = fft_output1[2 * index + 1];
    float real2 = fft_output2[2 * index];
    float imag2 = fft_output2[2 * index + 1];

    float phase1 = atan2f(imag1, real1);
    float phase2 = atan2f(imag2, real2);

    float diff = phase1 - phase2;

    if (diff > PI) diff -= 2 * PI;
    if (diff < -PI) diff += 2 * PI;

    return diff * (180.0f / PI); // 转换为角度
}

float calculate_amplitude_ratio(float* mag1, float* mag2, int index) {
    float amp1 = mag1[index];
    float amp2 = mag2[index];
    return amp1 / amp2;
}

float calculate_thd(float* magnitude, int fundamental_index) {
    float thd_sum = 0.0f;
    float fundamental = magnitude[fundamental_index];

    for (int i = 2; i <= 7; i++) {
        int idx = fundamental_index * i;
        if (idx < FFT_SIZE / 2) {
            thd_sum += magnitude[idx] * magnitude[idx];
        }
    }

    return sqrtf(thd_sum) / fundamental * 100.0f; // 百分比
}
const char* detect_waveform(float32_t* magnitude,int index) {
    // 选择基波
    
    float32_t base_amplitude = magnitude[index];  // 基波的幅度
    uint32_t harmonic_count = 0;                       // 谐波计数

    // 判断谐波和基波幅度的比例，计算超出阈值的谐波数量
    for (uint32_t i = 2; i < FFT_SIZE / 2 && harmonic_count < MAX_HARMONICS; i++) {
        // 只考虑与基波频率的整数倍接近的谐波，简化处理
        if (magnitude[i] > base_amplitude * HARMONIC_THRESHOLD) {
            harmonic_count++;
        }
    }

    // 根据谐波数目判断波形类型
    if (harmonic_count == 0) {
        return "Sine";  // 如果没有显著的谐波，认为是正弦波
    } else if (harmonic_count == 1) {
        return "Triangle";  // 如果只有一个显著谐波，认为是三角波
    } else if (harmonic_count > 1) {
        return "Square";    // 如果有多个显著谐波，认为是方波
    }

    return "Unknown";  // 如果无法判断，返回未知
}

void process_signal1(void) {
    // 执行 FFT
    arm_cfft_f32(&arm_cfft_sR_f32_len1024, input12_f32, 0, 1);  

    // 计算幅度谱
    arm_cmplx_mag_f32(input12_f32, magnitude1, FFT_SIZE / 2);

    // 主频分析：通过计算幅度谱的最大值索引，来得到主频
    main_frequency1 = find_main_frequency(input12_f32, &max_index);

    // Vpp 计算与波形识别
    amplitude_peak1 = calculate_peak(magnitude1) ;
    wave1 = (char*)detect_waveform(magnitude1, max_index);
}

void process_signal2(void) {
    // 执行 FFT
    arm_cfft_f32(&arm_cfft_sR_f32_len1024, input22_f32, 0, 1);

    // 计算幅度谱
    arm_cmplx_mag_f32(input22_f32, magnitude2, FFT_SIZE / 2);

    // 主频分析：通过计算幅度谱的最大值索引，来得到主频
    main_frequency2 = find_main_frequency(input22_f32, NULL);

    // Vpp 计算
    amplitude_peak2 = calculate_peak(magnitude2);

    // THD 计算
    thd = calculate_thd(magnitude2, max_index);

    // 波形识别
    wave2 = (char*)detect_waveform(magnitude2, max_index);
}



void analyze_signals(void) {
    trans1();
    trans2();

    apply_window_and_prepare_fft(input12_f32, input11_f32, window);
    apply_window_and_prepare_fft(input22_f32, input21_f32, window);

    process_signal1();
    process_signal2();

    phase_diff = calculate_phase_diff(output1_f32, output2_f32, max_index);
    amplitude_ratio = calculate_amplitude_ratio(magnitude1, magnitude2, max_index);

}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    if (DEBUG_PRINT_INPUT) {
        for (int i = 0; i < FFT_SIZE; i++) {
            printf("%f, %f\n", input11_f32[i], input21_f32[i]);
        }
    }

    analyze_signals();  // 核心分析
}
int main(void) {
    HAL_Init();
    SystemClock_Config();

    MX_DMA_Init();
    MX_ADC1_Init();
    MX_ADC2_Init();
    MX_USART1_UART_Init();  // 用于 printf 输出
    MX_TIMx_Init();         // 若用定时采样
    generate_hanning_window();

    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_DMA1, FFT_SIZE);
    HAL_ADC_Start_DMA(&hadc2, (uint32_t*)ADC_DMA2, FFT_SIZE);

    while (1) {
        // 可加入延时或状态处理
    }
}
