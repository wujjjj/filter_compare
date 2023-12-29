/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "can.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdarg.h>
#include "stdio.h"
#include "string.h"
#include "pid.h"
#include "bsp_can.h"
#include "CAN_receive.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
int16_t myspeed_rpm1;
int16_t mygiven_current1;
int16_t myspeed_rpm2;
int16_t mygiven_current2;

fp32 motor1_speed,motor2_speed;
	
pid_type_def motor_pid_data;

uint8_t input[10];

float SET_SPEED = 0.15f;


int16_t values[6];
const float FACTOR_ADC = 0.80586;
//移动平均值滤波算法
#define WINDOW_SIZE 5
int16_t moving_average_filter(int16_t new_data)
{
    static int16_t window[WINDOW_SIZE] = {0};
    static int index = 0;
    int32_t sum = 0;

    // 将新的数据点添加到窗口中
    window[index] = new_data;

    // 计算窗口内所有数据点的和
    for (int i = 0; i < WINDOW_SIZE; i++) {
        sum += window[i];
    }
    // 更新窗口索引
    index = (index + 1) % WINDOW_SIZE;
    // 返回平均值
    return (int16_t)(sum / WINDOW_SIZE);
}

//中值滤波算法
int16_t median_filter(int16_t new_data)
{
    static int16_t window[WINDOW_SIZE] = {0};
    static int index = 0;
    // 将新的数据点添加到窗口中
    window[index] = new_data;
    // 对窗口内的数据点进行排序
    for (int i = 0; i < WINDOW_SIZE - 1; i++) {
        for (int j = i + 1; j < WINDOW_SIZE; j++) {
            if (window[j] < window[i]) {
                // 交换数据点位置
                int16_t temp = window[i];
                window[i] = window[j];
                window[j] = temp;
            }
        }
    }
    // 更新窗口索引
    index = (index + 1) % WINDOW_SIZE;
    // 返回中间值
    return window[WINDOW_SIZE / 2];
}

//无限脉冲响应滤波
//慢慢变小，但是永远不会消失，就是无限冲激响应
#define FILTER_ORDER 2
typedef struct {
    float b[FILTER_ORDER + 1];
    float a[FILTER_ORDER + 1];
    float x[FILTER_ORDER + 1];
    float y[FILTER_ORDER + 1];
} IIRFilter;
void iir_filter_init(IIRFilter* filter)
{
    // 初始化滤波器系数
    filter->b[0] = 0.5;
    filter->b[1] = 0.3;
    filter->b[2] = 0.2;

    filter->a[0] = 1.0;
    filter->a[1] = -0.5;
    filter->a[2] = 0.1;
    // 初始化输入和输出缓冲区
    for (int i = 0; i <= FILTER_ORDER; i++) {
        filter->x[i] = 0.0;
        filter->y[i] = 0.0;
    }
}
float iir_filter_process(IIRFilter* filter, float input)
{
    // 更新输入缓冲区
    for (int i = FILTER_ORDER; i > 0; i--) {
        filter->x[i] = filter->x[i - 1];
    }
    filter->x[0] = input;
    // 计算输出
    float output = 0.0;
    for (int i = 0; i <= FILTER_ORDER; i++) {
        output += filter->b[i] * filter->x[i];
    }
    for (int i = 1; i <= FILTER_ORDER; i++) {
        output -= filter->a[i] * filter->y[i];
    }
    // 更新输出缓冲区
    for (int i = FILTER_ORDER; i > 0; i--) {
        filter->y[i] = filter->y[i - 1];
    }
    filter->y[0] = output;
    return output;
}

//有限脉冲响应滤波
typedef struct {
    float h[FILTER_ORDER + 2];
    float x[FILTER_ORDER + 1];
} FIRFilter;
void fir_filter_init(FIRFilter* filter)
{
    // 初始化滤波器系数
    filter->h[0] = 0.1;
    filter->h[1] = 0.2;
    filter->h[2] = 0.3;
    filter->h[3] = 0.2;
    // 初始化输入缓冲区
    for (int i = 0; i <= FILTER_ORDER; i++) {
        filter->x[i] = 0.0;
    }
}
float fir_filter_process(FIRFilter* filter, float input)
{
    // 更新输入缓冲区
    for (int i = FILTER_ORDER; i > 0; i--) {
        filter->x[i] = filter->x[i - 1];
    }
    filter->x[0] = input;
    // 计算输出
    float output = 0.0;
    for (int i = 0; i <= FILTER_ORDER; i++) {
        output += filter->h[i] * filter->x[i];
    }
    return output;
}

//卡尔曼滤波算法
typedef struct {
    float x; // 状态变量
    float P; // 状态协方差
    float Q; // 系统噪声协方差
    float R; // 测量噪声协方差
} KalmanFilter;
void kalman_filter_init(KalmanFilter* filter, float initial_state_variance, float system_noise_variance, float measurement_noise_variance)
{
    filter->x = 0.0;
    filter->P = initial_state_variance;
    filter->Q = system_noise_variance;
    filter->R = measurement_noise_variance;
}
float kalman_filter_process(KalmanFilter* filter, float measurement)
{
    // 预测步骤
    float x_pred = filter->x;
    float P_pred = filter->P + filter->Q;
    // 更新步骤
    float K = P_pred / (P_pred + filter->R);
    filter->x = x_pred + K * (measurement - x_pred);
    filter->P = (1.0 - K) * P_pred;
    return filter->x;
}


IIRFilter filter;
FIRFilter filter3;
KalmanFilter filter2;

void usart_printf(const char *fmt,...)
{
    static uint8_t tx_buf[256] = {0};
    static va_list ap;
    static uint16_t len;
    va_start(ap, fmt);
    len = vsprintf((char *)tx_buf, fmt, ap);
    va_end(ap);
    //usart1_tx_dma_enable(tx_buf, len);
    HAL_UART_Transmit_DMA(&huart1,tx_buf, len);
}

//发送六轴数据到串口
#define MAX_LENGTH 100
void send_values_to_serial(int16_t *value,int n)
{
    char result[MAX_LENGTH];
    char temp_str[MAX_LENGTH];
    int i;
    // 清空结果字符串
    result[0] = '\0';
    for (i = 0; i < n; i++) {
        // 将带符号的 uint16_t 数值转换为字符串
        sprintf(temp_str, "%+d", value[i]);
        // 拼接逗号和当前值的字符串
        strcat(result, temp_str);
        strcat(result, ",");
    }
    // 去除最后一个逗号
    result[strlen(result) - 1] = '\0';
    // 发送字符串到串口，这里只是一个示例
    usart_printf("%s\n", result);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart1) {
        // 解析输入数据
        uint8_t command = input[0] - '0';
        uint16_t value = (input[1] - '0') * 1000 + (input[2] - '0') * 10 + (input[3] - '0') * 10 + (input[4] - '0');

        // 根据输入数据设置参数
        switch (command) {
            case 0:
				SET_SPEED = ((float)value/100);
			break;
			
            case 1:
				//usart_printf("%d\n",value);
				filter2.P = value/10;
			break;
			
            case 2:
				//usart_printf("%d\n",value);
				filter2.Q = value/10;
			break;
			
            case 3:
				filter2.R = value/10; 
			break;
			
            default:
                break;
        }
		usart_printf("%d",command);
		usart_printf("%d\n",value);
        // 开始下一次数据接收
        HAL_UART_Receive_DMA(&huart1, input, 5);
    }
}
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

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

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
    kalman_filter_init(&filter2, 1.0, 0.1, 300);
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
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	iir_filter_init(&filter);
	fir_filter_init(&filter3);
	
  can_filter_init();
  #define KP 3600
  #define KI 0
  #define KD 1600
  #define MAX_OUT 2000.0f
  #define MAX_IOUT 100
	
  #define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN 0.000415809748903494517209f

  
  fp32 motor1_speed,motor2_speed;
  uint16_t postion;
  
  fp32 pid[3]= {KP,KI,KD};
  PID_init(&motor_pid_data,1,pid,MAX_OUT,MAX_IOUT); 
  
  HAL_UART_Receive_DMA(&huart1, input, 5);
  usart_printf("start");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	    motor1_speed = motor_chassis[0].speed_rpm*CHASSIS_MOTOR_RPM_TO_VECTOR_SEN;
	    motor2_speed = motor_chassis[1].speed_rpm*CHASSIS_MOTOR_RPM_TO_VECTOR_SEN;
	  
	    mygiven_current1 = PID_calc(&motor_pid_data,motor1_speed,SET_SPEED);
		mygiven_current2 = PID_calc(&motor_pid_data,motor2_speed,SET_SPEED);
	  
	    postion = motor_chassis[1].ecd;
	  
	  values[0] = motor2_speed*1000;//原数据
	  values[1] = moving_average_filter(values[0]);//平均值滤波
	  values[2] = median_filter(values[0]);//中值滤波
	  values[3] = iir_filter_process(&filter, (float)values[0]);//无限脉冲响应滤波
	  values[4] = fir_filter_process(&filter3, values[0]);//有限脉冲响应滤波
	  values[5] = kalman_filter_process(&filter2, values[0]);//卡尔曼滤波

	  send_values_to_serial(values,6);
	  
        CAN_cmd_chassis(mygiven_current2, mygiven_current2, 0, 0);
        //CAN_cmd_chassis(0, 0, 0, 0);
	  
        HAL_Delay(2);
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
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
