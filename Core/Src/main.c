/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Smart Car Final (All Polling - Most Stable)
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define HIGH 1
#define LOW 0
#define AUTO_STOP_DELAY 200
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t rx_data;
long unsigned int echo_time1, echo_time2, echo_time3, echo_time4;
int dist1, dist2, dist3, dist4;
volatile uint32_t last_cmd_time = 0;

volatile uint8_t is_avoiding = 0;
volatile uint8_t is_emergency = 0;
volatile uint8_t is_fire = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);

/* USER CODE BEGIN PFP */
void Avoid_Obstacle_Routine(void);
void trig1(void); long unsigned int echo1(void);
void trig2(void); long unsigned int echo2(void);
void trig3(void); long unsigned int echo3(void);
void trig4(void); long unsigned int echo4(void);
void delay_us(uint16_t us);
int LFRB(); int RFLB(); void ST(); void MB();
void Check_Light(void);
void Check_Tilt_State(void); // 전복 감시
void Check_Fire_State(void); // 화재 감시 (NEW)
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE {
    HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, 10);
    return ch;
}

// [모터 제어 함수들]
int LF(int dir){ switch(dir){ case 1: HAL_GPIO_WritePin(LFF_GPIO_Port, LFF_Pin, 1); HAL_GPIO_WritePin(LFB_GPIO_Port, LFB_Pin, 0); break; case 0: HAL_GPIO_WritePin(LFF_GPIO_Port, LFF_Pin, 0); HAL_GPIO_WritePin(LFB_GPIO_Port, LFB_Pin, 1); break; case 2: HAL_GPIO_WritePin(LFF_GPIO_Port, LFF_Pin, 0); HAL_GPIO_WritePin(LFB_GPIO_Port, LFB_Pin, 0); break; } return 0; }
int LB(int dir){ switch(dir){ case 1: HAL_GPIO_WritePin(LBF_GPIO_Port, LBF_Pin, 1); HAL_GPIO_WritePin(LBB_GPIO_Port, LBB_Pin, 0); break; case 0: HAL_GPIO_WritePin(LBF_GPIO_Port, LBF_Pin, 0); HAL_GPIO_WritePin(LBB_GPIO_Port, LBB_Pin, 1); break; case 2: HAL_GPIO_WritePin(LBF_GPIO_Port, LBF_Pin, 0); HAL_GPIO_WritePin(LBB_GPIO_Port, LBB_Pin, 0); break; } return 0; }
int RF(int dir){ switch(dir){ case 1: HAL_GPIO_WritePin(RFF_GPIO_Port, RFF_Pin, 1); HAL_GPIO_WritePin(RFB_GPIO_Port, RFB_Pin, 0); break; case 0: HAL_GPIO_WritePin(RFF_GPIO_Port, RFF_Pin, 0); HAL_GPIO_WritePin(RFB_GPIO_Port, RFB_Pin, 1); break; case 2: HAL_GPIO_WritePin(RFF_GPIO_Port, RFF_Pin, 0); HAL_GPIO_WritePin(RFB_GPIO_Port, RFB_Pin, 0); break; } return 0; }
int RB(int dir){ switch(dir){ case 1: HAL_GPIO_WritePin(RBF_GPIO_Port, RBF_Pin, 1); HAL_GPIO_WritePin(RBB_GPIO_Port, RBB_Pin, 0); break; case 0: HAL_GPIO_WritePin(RBF_GPIO_Port, RBF_Pin, 0); HAL_GPIO_WritePin(RBB_GPIO_Port, RBB_Pin, 1); break; case 2: HAL_GPIO_WritePin(RBF_GPIO_Port, RBF_Pin, 0); HAL_GPIO_WritePin(RBB_GPIO_Port, RBB_Pin, 0); break; } return 0; }

int TLF(){ RF(1); RB(1); LF(2); LB(2); return 0; }
int TRF(){ RF(2); RB(2); LF(1); LB(1); return 0; }
int TLB(){ RF(0); RB(0); LF(2); LB(2); return 0; }
int TRB(){ RF(2); RB(2); LF(0); LB(0); return 0; }
int LFRB(){ RF(0); RB(0); LF(1); LB(1); return 0; }
int RFLB(){ RF(1); RB(1); LF(0); LB(0); return 0; }

void MF() { HAL_GPIO_WritePin(LFF_GPIO_Port, LFF_Pin, 1); HAL_GPIO_WritePin(LFB_GPIO_Port, LFB_Pin, 0); HAL_GPIO_WritePin(RFF_GPIO_Port, RFF_Pin, 1); HAL_GPIO_WritePin(RFB_GPIO_Port, RFB_Pin, 0); HAL_GPIO_WritePin(LBF_GPIO_Port, LBF_Pin, 1); HAL_GPIO_WritePin(LBB_GPIO_Port, LBB_Pin, 0); HAL_GPIO_WritePin(RBF_GPIO_Port, RBF_Pin, 1); HAL_GPIO_WritePin(RBB_GPIO_Port, RBB_Pin, 0); }
void MB() { HAL_GPIO_WritePin(LFF_GPIO_Port, LFF_Pin, 0); HAL_GPIO_WritePin(LFB_GPIO_Port, LFB_Pin, 1); HAL_GPIO_WritePin(RFF_GPIO_Port, RFF_Pin, 0); HAL_GPIO_WritePin(RFB_GPIO_Port, RFB_Pin, 1); HAL_GPIO_WritePin(LBF_GPIO_Port, LBF_Pin, 0); HAL_GPIO_WritePin(LBB_GPIO_Port, LBB_Pin, 1); HAL_GPIO_WritePin(RBF_GPIO_Port, RBF_Pin, 0); HAL_GPIO_WritePin(RBB_GPIO_Port, RBB_Pin, 1); }
void ML() { HAL_GPIO_WritePin(LFF_GPIO_Port, LFF_Pin, 0); HAL_GPIO_WritePin(LFB_GPIO_Port, LFB_Pin, 0); HAL_GPIO_WritePin(RFF_GPIO_Port, RFF_Pin, 1); HAL_GPIO_WritePin(RFB_GPIO_Port, RFB_Pin, 0); HAL_GPIO_WritePin(LBF_GPIO_Port, LBF_Pin, 0); HAL_GPIO_WritePin(LBB_GPIO_Port, LBB_Pin, 0); HAL_GPIO_WritePin(RBF_GPIO_Port, RBF_Pin, 1); HAL_GPIO_WritePin(RBB_GPIO_Port, RBB_Pin, 0); }
void MR() { HAL_GPIO_WritePin(LFF_GPIO_Port, LFF_Pin, 1); HAL_GPIO_WritePin(LFB_GPIO_Port, LFB_Pin, 0); HAL_GPIO_WritePin(RFF_GPIO_Port, RFF_Pin, 0); HAL_GPIO_WritePin(RFB_GPIO_Port, RFB_Pin, 0); HAL_GPIO_WritePin(LBF_GPIO_Port, LBF_Pin, 1); HAL_GPIO_WritePin(LBB_GPIO_Port, LBB_Pin, 0); HAL_GPIO_WritePin(RBF_GPIO_Port, RBF_Pin, 0); HAL_GPIO_WritePin(RBB_GPIO_Port, RBB_Pin, 0); }
void ST() { HAL_GPIO_WritePin(LFF_GPIO_Port, LFF_Pin, 0); HAL_GPIO_WritePin(LFB_GPIO_Port, LFB_Pin, 0); HAL_GPIO_WritePin(RFF_GPIO_Port, RFF_Pin, 0); HAL_GPIO_WritePin(RFB_GPIO_Port, RFB_Pin, 0); HAL_GPIO_WritePin(LBF_GPIO_Port, LBF_Pin, 0); HAL_GPIO_WritePin(LBB_GPIO_Port, LBB_Pin, 0); HAL_GPIO_WritePin(RBF_GPIO_Port, RBF_Pin, 0); HAL_GPIO_WritePin(RBB_GPIO_Port, RBB_Pin, 0); }

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        HAL_UART_Transmit(&huart2, &rx_data, 1, 10);
        // 비상 상황이 아닐 때만 리모컨 동작
        if (is_avoiding == 0 && is_emergency == 0 && is_fire == 0)
        {
            last_cmd_time = HAL_GetTick();
            switch (rx_data) {
                case 'W': case 'w': MF(); break;
                case 'A': case 'a': ML(); break;
                case 'D': case 'd': MR(); break;
                case 'S': case 's': MB(); break;
                case 'Q': case 'q': TLF(); break;
                case 'E': case 'e': TRF(); break;
                case 'Z': case 'z': TLB(); break;
                case 'C': case 'c': TRB(); break;
                case 'R': case 'r': LFRB(); break;
                case 'T': case 't': RFLB(); break;
                default: ST(); break;
            }
        }
        HAL_UART_Receive_IT(&huart2, &rx_data, 1);
    }
}

// ===============================================
// [상태 감지 함수들] - 둘 다 폴링 방식으로 변경
// ===============================================

// 1. 전복 감지 (2초 대기)
void Check_Tilt_State(void)
{
    static uint32_t tilt_start_time = 0;

    // 기울어짐 감지 (Active High)
    if (HAL_GPIO_ReadPin(C6_GPIO_Port, C6_Pin) == GPIO_PIN_SET)
    {
        if (tilt_start_time == 0) tilt_start_time = HAL_GetTick();

        // 2초 이상 유지되면 비상 걸림
        if (HAL_GetTick() - tilt_start_time > 2000) {
            ST(); is_emergency = 1;
            HAL_GPIO_WritePin(LED_LEFT_GPIO_Port, LED_LEFT_Pin, 1);
            HAL_GPIO_WritePin(LED_RIGHT_GPIO_Port, LED_RIGHT_Pin, 1);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 500);
            printf("⚠️차량 전복됨!!\r\n");
            tilt_start_time = 0;
        }
    }
    else { tilt_start_time = 0; }
}

// 2. 화재 감지 (즉시 반응 + 연속 감시)
void Check_Fire_State(void)
{
    // 불이 나면(High) 즉시 반응
    if (HAL_GPIO_ReadPin(FLAME_GPIO_Port, FLAME_Pin) == GPIO_PIN_SET)
    {
        // 아직 화재 모드가 아니었다면 진입
        if (is_fire == 0) {
            ST(); is_fire = 1;
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 500);
            HAL_GPIO_WritePin(LED_LEFT_GPIO_Port, LED_LEFT_Pin, 1);
            HAL_GPIO_WritePin(LED_RIGHT_GPIO_Port, LED_RIGHT_Pin, 1);
            printf("🔥 화재 발생!\r\n");
        }
    }
}

// [회피 루틴]
void Avoid_Obstacle_Routine(void)
{
    is_avoiding = 1; ST(); HAL_Delay(500);
    uint32_t last_avoid_light_check = 0;

    trig3(); echo_time3 = echo3(); dist3 = (echo_time3 > 0 && echo_time3 < 23000) ? (int)(17 * echo_time3 / 100) : 999; delay_us(15000);
    trig4(); echo_time4 = echo4(); dist4 = (echo_time4 > 0 && echo_time4 < 23000) ? (int)(17 * echo_time4 / 100) : 999;

    int escape_plan = 0;
    if (dist3 <= 150 && dist4 > 150) escape_plan = 1;
    else if (dist4 <= 150 && dist3 > 150) escape_plan = 2;
    else if (dist3 > 150 && dist4 > 150) escape_plan = 3;
    else escape_plan = 4;

    printf("회피기동모드 모드: %d\r\n", escape_plan);

    while (1)
    {
        // [핵심] 회피 중에도 계속 감시
        Check_Tilt_State();
        Check_Fire_State();

        // 감지되면 탈출
        if (is_emergency == 1 || is_fire == 1) { ST(); is_avoiding = 0; break; }

        if (HAL_GetTick() - last_avoid_light_check > 500) { Check_Light(); last_avoid_light_check = HAL_GetTick(); }

        trig1(); echo_time1 = echo1(); dist1 = (echo_time1 > 0 && echo_time1 < 23000) ? (int)(17 * echo_time1 / 100) : 999; delay_us(2000);
        trig2(); echo_time2 = echo2(); dist2 = (echo_time2 > 0 && echo_time2 < 23000) ? (int)(17 * echo_time2 / 100) : 999;

        if (dist1 > 350 && dist2 > 350) { ST(); is_avoiding = 0; last_cmd_time = HAL_GetTick(); break; }

        switch (escape_plan) {
            case 1: LFRB(); break;
            case 2: RFLB(); break;
            case 3: MB(); HAL_Delay(500); escape_plan = 2; break;
            case 4: MB(); break;
        }
        HAL_Delay(100);
    }
}

// [초음파 함수]
void timer_start(void) { HAL_TIM_Base_Start(&htim2); }
void delay_us(uint16_t us) { __HAL_TIM_SET_COUNTER(&htim2, 0); while((__HAL_TIM_GET_COUNTER(&htim2)) < us); }
void trig1(void) { HAL_GPIO_WritePin(TRIG1_GPIO_Port, TRIG1_Pin, HIGH); delay_us(10); HAL_GPIO_WritePin(TRIG1_GPIO_Port, TRIG1_Pin, LOW); }
long unsigned int echo1(void) { long unsigned int echo = 0; int timeout = 0; while(HAL_GPIO_ReadPin(ECHO1_GPIO_Port, ECHO1_Pin) == LOW) { timeout++; if(timeout > 5000) return 0; } __HAL_TIM_SET_COUNTER(&htim2, 0); timeout = 0; while(HAL_GPIO_ReadPin(ECHO1_GPIO_Port, ECHO1_Pin) == HIGH) { timeout++; if(timeout > 20000) return 0; } echo = __HAL_TIM_GET_COUNTER(&htim2); return echo; }
void trig2(void) { HAL_GPIO_WritePin(TRIG2_GPIO_Port, TRIG2_Pin, HIGH); delay_us(10); HAL_GPIO_WritePin(TRIG2_GPIO_Port, TRIG2_Pin, LOW); }
long unsigned int echo2(void) { long unsigned int echo = 0; int timeout = 0; while(HAL_GPIO_ReadPin(ECHO2_GPIO_Port, ECHO2_Pin) == LOW) { timeout++; if(timeout > 5000) return 0; } __HAL_TIM_SET_COUNTER(&htim2, 0); timeout = 0; while(HAL_GPIO_ReadPin(ECHO2_GPIO_Port, ECHO2_Pin) == HIGH) { timeout++; if(timeout > 20000) return 0; } echo = __HAL_TIM_GET_COUNTER(&htim2); return echo; }
void trig3(void) { HAL_GPIO_WritePin(TRIG3_GPIO_Port, TRIG3_Pin, HIGH); delay_us(10); HAL_GPIO_WritePin(TRIG3_GPIO_Port, TRIG3_Pin, LOW); }
long unsigned int echo3(void) { long unsigned int echo = 0; int timeout = 0; while(HAL_GPIO_ReadPin(ECHO3_GPIO_Port, ECHO3_Pin) == LOW) { timeout++; if(timeout > 5000) return 0; } __HAL_TIM_SET_COUNTER(&htim2, 0); timeout = 0; while(HAL_GPIO_ReadPin(ECHO3_GPIO_Port, ECHO3_Pin) == HIGH) { timeout++; if(timeout > 40000) return 0; } echo = __HAL_TIM_GET_COUNTER(&htim2); return echo; }
void trig4(void) { HAL_GPIO_WritePin(TRIG4_GPIO_Port, TRIG4_Pin, HIGH); delay_us(10); HAL_GPIO_WritePin(TRIG4_GPIO_Port, TRIG4_Pin, LOW); }
long unsigned int echo4(void) { long unsigned int echo = 0; int timeout = 0; while(HAL_GPIO_ReadPin(ECHO4_GPIO_Port, ECHO4_Pin) == LOW) { timeout++; if(timeout > 5000) return 0; } __HAL_TIM_SET_COUNTER(&htim2, 0); timeout = 0; while(HAL_GPIO_ReadPin(ECHO4_GPIO_Port, ECHO4_Pin) == HIGH) { timeout++; if(timeout > 40000) return 0; } echo = __HAL_TIM_GET_COUNTER(&htim2); return echo; }
/* USER CODE END 0 */

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();

  /* USER CODE BEGIN 2 */
  timer_start();
  // 부저 초기화
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  HAL_Delay(500);

  is_fire = 0; is_emergency = 0;

  HAL_UART_Receive_IT(&huart2, &rx_data, 1);
  printf("\n=== SmartGuardRover 기동 ===\n");
  last_cmd_time = HAL_GetTick();
  /* USER CODE END 2 */

  /* USER CODE BEGIN WHILE */
  uint32_t last_print_time = 0;
  uint32_t last_light_check_time = 0;

  while (1)
  {
      // [0] 상태 상시 감시 (가장 중요)
      Check_Tilt_State();
      Check_Fire_State();

      // ----------------------------------------------------
      // [1] 전복 처리 (Priority 1)
      // ----------------------------------------------------
      if (is_emergency == 1)
      {
          ST();
          // 다시 바로 섰는지(복구) 확인 (20번 연속 정상이어야 해제)
          if (HAL_GPIO_ReadPin(C6_GPIO_Port, C6_Pin) == GPIO_PIN_RESET)
          {
              int clear_cnt = 0;
              for(int i=0; i<20; i++) {
                  HAL_Delay(100);
                  if(HAL_GPIO_ReadPin(C6_GPIO_Port, C6_Pin) == GPIO_PIN_SET) {
                      clear_cnt = 0; // 아직 흔들림
                      break;
                  }
                  clear_cnt++;
              }

              if(clear_cnt >= 20) {
                  is_emergency = 0; is_avoiding = 0;
                  last_cmd_time = HAL_GetTick();
                  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
                  HAL_GPIO_WritePin(LED_LEFT_GPIO_Port, LED_LEFT_Pin, 0);
                  HAL_GPIO_WritePin(LED_RIGHT_GPIO_Port, LED_RIGHT_Pin, 0);
                  printf("Tilt Recovered.\r\n");
                  continue;
              }
          }
          continue; // 전복 중이면 다른 동작 금지
      }

      // ----------------------------------------------------
      // [2] 화재 처리 (Priority 2)
      // ----------------------------------------------------
      if (is_fire == 1)
      {
          ST(); // 모터 정지

          // 불이 꺼졌는지 확인 (Active High: 꺼지면 LOW)
          if (HAL_GPIO_ReadPin(FLAME_GPIO_Port, FLAME_Pin) == GPIO_PIN_RESET)
          {
              int clear_cnt = 0;
              for(int i=0; i<20; i++) {
                  HAL_Delay(100);
                  // 도중에 다시 불(SET)이 튀면 -> 검사 중단
                  if(HAL_GPIO_ReadPin(FLAME_GPIO_Port, FLAME_Pin) == GPIO_PIN_SET) {
                      clear_cnt = 0;
                      break;
                  }
                  clear_cnt++;
              }

              if(clear_cnt >= 20) {
                  is_fire = 0;
                  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0); // 소리 OFF
                  HAL_GPIO_WritePin(LED_LEFT_GPIO_Port, LED_LEFT_Pin, 0);
                  HAL_GPIO_WritePin(LED_RIGHT_GPIO_Port, LED_RIGHT_Pin, 0);

                  is_avoiding = 0;
                  last_cmd_time = HAL_GetTick();
                  printf("Fire Cleared.\r\n");
                  continue;
              }
          }
          continue;
      }

      // ----------------------------------------------------
      // [3] 평상시 주행
      // ----------------------------------------------------
      if (HAL_GetTick() - last_light_check_time > 500) { Check_Light(); last_light_check_time = HAL_GetTick(); }
      if (is_avoiding == 1) continue;
      if (HAL_GetTick() - last_cmd_time > AUTO_STOP_DELAY) ST();

      trig1(); echo_time1 = echo1(); dist1 = (echo_time1 > 0 && echo_time1 < 23000) ? (int)(17 * echo_time1 / 100) : 999; delay_us(5000);
      trig2(); echo_time2 = echo2(); dist2 = (echo_time2 > 0 && echo_time2 < 23000) ? (int)(17 * echo_time2 / 100) : 999;

      if ((dist1 < 150 && dist1 > 0) || (dist2 < 150 && dist2 > 0)) Avoid_Obstacle_Routine();

      if (HAL_GetTick() - last_print_time > 200) { printf("D: %d, %d\r\n", dist1, dist2); last_print_time = HAL_GetTick(); }
  }
}

// [초기화 함수들]
void SystemClock_Config(void) { RCC_OscInitTypeDef R = {0}; RCC_ClkInitTypeDef C = {0}; RCC_PeriphCLKInitTypeDef P = {0}; R.OscillatorType = RCC_OSCILLATORTYPE_HSI; R.HSIState = RCC_HSI_ON; R.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT; R.PLL.PLLState = RCC_PLL_ON; R.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2; R.PLL.PLLMUL = RCC_PLL_MUL16; HAL_RCC_OscConfig(&R); C.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2; C.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK; C.AHBCLKDivider = RCC_SYSCLK_DIV1; C.APB1CLKDivider = RCC_HCLK_DIV2; C.APB2CLKDivider = RCC_HCLK_DIV1; HAL_RCC_ClockConfig(&C, FLASH_LATENCY_2); P.PeriphClockSelection = RCC_PERIPHCLK_ADC; P.AdcClockSelection = RCC_ADCPCLK2_DIV6; HAL_RCCEx_PeriphCLKConfig(&P); }
static void MX_ADC1_Init(void) { ADC_ChannelConfTypeDef s = {0}; hadc1.Instance = ADC1; hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE; hadc1.Init.ContinuousConvMode = DISABLE; hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START; hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT; hadc1.Init.NbrOfConversion = 1; HAL_ADC_Init(&hadc1); s.Channel = ADC_CHANNEL_10; s.Rank = ADC_REGULAR_RANK_1; s.SamplingTime = ADC_SAMPLETIME_1CYCLE_5; HAL_ADC_ConfigChannel(&hadc1, &s); }
static void MX_TIM2_Init(void) { TIM_ClockConfigTypeDef s = {0}; TIM_MasterConfigTypeDef m = {0}; htim2.Instance = TIM2; htim2.Init.Prescaler = 63; htim2.Init.CounterMode = TIM_COUNTERMODE_UP; htim2.Init.Period = 65535; htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE; HAL_TIM_Base_Init(&htim2); s.ClockSource = TIM_CLOCKSOURCE_INTERNAL; HAL_TIM_ConfigClockSource(&htim2, &s); m.MasterOutputTrigger = TIM_TRGO_RESET; m.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE; HAL_TIMEx_MasterConfigSynchronization(&htim2, &m); }
static void MX_TIM3_Init(void) { TIM_ClockConfigTypeDef s = {0}; TIM_MasterConfigTypeDef m = {0}; TIM_OC_InitTypeDef c = {0}; htim3.Instance = TIM3; htim3.Init.Prescaler = 63; htim3.Init.CounterMode = TIM_COUNTERMODE_UP; htim3.Init.Period = 999; htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE; HAL_TIM_Base_Init(&htim3); HAL_TIM_PWM_Init(&htim3); s.ClockSource = TIM_CLOCKSOURCE_INTERNAL; HAL_TIM_ConfigClockSource(&htim3, &s); m.MasterOutputTrigger = TIM_TRGO_RESET; m.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE; HAL_TIMEx_MasterConfigSynchronization(&htim3, &m); c.OCMode = TIM_OCMODE_PWM1; c.Pulse = 0; c.OCPolarity = TIM_OCPOLARITY_HIGH; c.OCFastMode = TIM_OCFAST_DISABLE; HAL_TIM_PWM_ConfigChannel(&htim3, &c, TIM_CHANNEL_4); HAL_TIM_MspPostInit(&htim3); }
static void MX_USART2_UART_Init(void) { huart2.Instance = USART2; huart2.Init.BaudRate = 115200; huart2.Init.WordLength = UART_WORDLENGTH_8B; huart2.Init.StopBits = UART_STOPBITS_1; huart2.Init.Parity = UART_PARITY_NONE; huart2.Init.Mode = UART_MODE_TX_RX; huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE; huart2.Init.OverSampling = UART_OVERSAMPLING_16; HAL_UART_Init(&huart2); }
static void MX_GPIO_Init(void) { GPIO_InitTypeDef g = {0}; __HAL_RCC_GPIOC_CLK_ENABLE(); __HAL_RCC_GPIOD_CLK_ENABLE(); __HAL_RCC_GPIOA_CLK_ENABLE(); __HAL_RCC_GPIOB_CLK_ENABLE(); HAL_GPIO_WritePin(GPIOC, LED_LEFT_Pin|LED_RIGHT_Pin|TRIG1_Pin, 0); HAL_GPIO_WritePin(GPIOA, LD2_Pin|TRIG2_Pin|LBF_Pin|LFB_Pin, 0); HAL_GPIO_WritePin(GPIOB, LBB_Pin|TRIG3_Pin|TRIG4_Pin|LFF_Pin|RBF_Pin|RBB_Pin|RFF_Pin|RFB_Pin, 0); g.Pin = B1_Pin; g.Mode = GPIO_MODE_IT_RISING; g.Pull = GPIO_NOPULL; HAL_GPIO_Init(B1_GPIO_Port, &g);
// [수정] Active High용 PULLDOWN, 인터럽트 제거 (입력 모드)
g.Pin = FLAME_Pin; g.Mode = GPIO_MODE_INPUT; g.Pull = GPIO_PULLDOWN; HAL_GPIO_Init(FLAME_GPIO_Port, &g);
g.Pin = C6_Pin; g.Mode = GPIO_MODE_INPUT; g.Pull = GPIO_PULLDOWN; HAL_GPIO_Init(C6_GPIO_Port, &g);
// 나머지 핀 설정
g.Pin = ECHO2_Pin; g.Mode = GPIO_MODE_INPUT; HAL_GPIO_Init(ECHO2_GPIO_Port, &g); g.Pin = LD2_Pin|TRIG2_Pin|LBF_Pin|LFB_Pin; g.Mode = GPIO_MODE_OUTPUT_PP; g.Speed = GPIO_SPEED_FREQ_LOW; HAL_GPIO_Init(GPIOA, &g); g.Pin = ECHO1_Pin|ECHO3_Pin|ECHO4_Pin; g.Mode = GPIO_MODE_INPUT; HAL_GPIO_Init(GPIOB, &g); g.Pin = LBB_Pin|TRIG3_Pin|TRIG4_Pin|LFF_Pin|RBF_Pin|RBB_Pin|RFF_Pin|RFB_Pin; g.Mode = GPIO_MODE_OUTPUT_PP; HAL_GPIO_Init(GPIOB, &g); g.Pin = LED_LEFT_Pin|LED_RIGHT_Pin|TRIG1_Pin; HAL_GPIO_Init(GPIOC, &g); HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0); HAL_NVIC_EnableIRQ(EXTI1_IRQn); HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0); HAL_NVIC_EnableIRQ(EXTI9_5_IRQn); HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0); HAL_NVIC_EnableIRQ(EXTI15_10_IRQn); }

/* USER CODE BEGIN 4 */
// [인터럽트는 비워둠]
void Check_Light(void)
{
    uint32_t adc_value = 0;
    float voltage = 0.0f;

    // ADC 변환 시작
    HAL_ADC_Start(&hadc1);

    // 변환 완료 대기 (최대 100ms)
    if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK)
    {
        // 값 읽기 (0 ~ 4095)
        adc_value = HAL_ADC_GetValue(&hadc1);

        // 전압으로 변환 (3.3V 기준)
        voltage = (adc_value * 3.3f) / 4095.0f;

        // [로직] 전압이 2.0V 미만이면 어두움 -> LED 켜기
        // 단, 화재 발생 시(is_fire=1)에는 화재 알림이 우선이므로 간섭하지 않음
        if (voltage < 2.0f && is_fire == 0)
        {
            HAL_GPIO_WritePin(LED_LEFT_GPIO_Port, LED_LEFT_Pin, 1);
            HAL_GPIO_WritePin(LED_RIGHT_GPIO_Port, LED_RIGHT_Pin, 1);
        }
        // 밝으면 LED 끄기 (화재 아닐 때만)
        else if (is_fire == 0)
        {
            HAL_GPIO_WritePin(LED_LEFT_GPIO_Port, LED_LEFT_Pin, 0);
            HAL_GPIO_WritePin(LED_RIGHT_GPIO_Port, LED_RIGHT_Pin, 0);
        }
    }

    // ADC 정지
    HAL_ADC_Stop(&hadc1);
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) { }
/* USER CODE END 4 */

void Error_Handler(void) { __disable_irq(); while (1) {} }
#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {}
#endif
