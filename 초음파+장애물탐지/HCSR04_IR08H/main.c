/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Standard Avoidance (Ultrasonic Only - Back & Turn)
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
#define AUTO_STOP_DELAY 200 // 키보드 자동 정지 시간 (ms)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t rx_data;
long unsigned int echo_time1, echo_time2, echo_time3, echo_time4;
int dist1, dist2, dist3, dist4;
volatile uint32_t last_cmd_time = 0;
volatile uint8_t is_avoiding = 0; // 0:수동, 1:회피중

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
    HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, 10);
    return ch;
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void Avoid_Obstacle_Routine(void);
void trig1(void); long unsigned int echo1(void);
void trig2(void); long unsigned int echo2(void);
void trig3(void); long unsigned int echo3(void);
void trig4(void); long unsigned int echo4(void);
void delay_us(uint16_t us);
int LFRB(); int RFLB(); void ST(); void MB();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// ==========================================
// [SECTION 1] 모터 제어 함수
// ==========================================
int LF(int dir){
    switch(dir){
        case 1: HAL_GPIO_WritePin(LFF_GPIO_Port, LFF_Pin, 1); HAL_GPIO_WritePin(LFB_GPIO_Port, LFB_Pin, 0); break;
        case 0: HAL_GPIO_WritePin(LFF_GPIO_Port, LFF_Pin, 0); HAL_GPIO_WritePin(LFB_GPIO_Port, LFB_Pin, 1); break;
        case 2: HAL_GPIO_WritePin(LFF_GPIO_Port, LFF_Pin, 0); HAL_GPIO_WritePin(LFB_GPIO_Port, LFB_Pin, 0); break;
    } return 0;
}
int LB(int dir){
    switch(dir){
        case 1: HAL_GPIO_WritePin(LBF_GPIO_Port, LBF_Pin, 1); HAL_GPIO_WritePin(LBB_GPIO_Port, LBB_Pin, 0); break;
        case 0: HAL_GPIO_WritePin(LBF_GPIO_Port, LBF_Pin, 0); HAL_GPIO_WritePin(LBB_GPIO_Port, LBB_Pin, 1); break;
        case 2: HAL_GPIO_WritePin(LBF_GPIO_Port, LBF_Pin, 0); HAL_GPIO_WritePin(LBB_GPIO_Port, LBB_Pin, 0); break;
    } return 0;
}
int RF(int dir){
    switch(dir){
        case 1: HAL_GPIO_WritePin(RFF_GPIO_Port, RFF_Pin, 1); HAL_GPIO_WritePin(RFB_GPIO_Port, RFB_Pin, 0); break;
        case 0: HAL_GPIO_WritePin(RFF_GPIO_Port, RFF_Pin, 0); HAL_GPIO_WritePin(RFB_GPIO_Port, RFB_Pin, 1); break;
        case 2: HAL_GPIO_WritePin(RFF_GPIO_Port, RFF_Pin, 0); HAL_GPIO_WritePin(RFB_GPIO_Port, RFB_Pin, 0); break;
    } return 0;
}
int RB(int dir){
    switch(dir){
        case 1: HAL_GPIO_WritePin(RBF_GPIO_Port, RBF_Pin, 1); HAL_GPIO_WritePin(RBB_GPIO_Port, RBB_Pin, 0); break;
        case 0: HAL_GPIO_WritePin(RBF_GPIO_Port, RBF_Pin, 0); HAL_GPIO_WritePin(RBB_GPIO_Port, RBB_Pin, 1); break;
        case 2: HAL_GPIO_WritePin(RBF_GPIO_Port, RBF_Pin, 0); HAL_GPIO_WritePin(RBB_GPIO_Port, RBB_Pin, 0); break;
    } return 0;
}

int TLF(){ RF(1); RB(1); LF(2); LB(2); return 0; }
int TRF(){ RF(2); RB(2); LF(1); LB(1); return 0; }
int TLB(){ RF(0); RB(0); LF(2); LB(2); return 0; }
int TRB(){ RF(2); RB(2); LF(0); LB(0); return 0; }
int LFRB(){ RF(0); RB(0); LF(1); LB(1); return 0; }
int RFLB(){ RF(1); RB(1); LF(0); LB(0); return 0; }

void MF() {
    HAL_GPIO_WritePin(LFF_GPIO_Port, LFF_Pin, 1); HAL_GPIO_WritePin(LFB_GPIO_Port, LFB_Pin, 0);
    HAL_GPIO_WritePin(RFF_GPIO_Port, RFF_Pin, 1); HAL_GPIO_WritePin(RFB_GPIO_Port, RFB_Pin, 0);
    HAL_GPIO_WritePin(LBF_GPIO_Port, LBF_Pin, 1); HAL_GPIO_WritePin(LBB_GPIO_Port, LBB_Pin, 0);
    HAL_GPIO_WritePin(RBF_GPIO_Port, RBF_Pin, 1); HAL_GPIO_WritePin(RBB_GPIO_Port, RBB_Pin, 0);
}
void MB() {
    HAL_GPIO_WritePin(LFF_GPIO_Port, LFF_Pin, 0); HAL_GPIO_WritePin(LFB_GPIO_Port, LFB_Pin, 1);
    HAL_GPIO_WritePin(RFF_GPIO_Port, RFF_Pin, 0); HAL_GPIO_WritePin(RFB_GPIO_Port, RFB_Pin, 1);
    HAL_GPIO_WritePin(LBF_GPIO_Port, LBF_Pin, 0); HAL_GPIO_WritePin(LBB_GPIO_Port, LBB_Pin, 1);
    HAL_GPIO_WritePin(RBF_GPIO_Port, RBF_Pin, 0); HAL_GPIO_WritePin(RBB_GPIO_Port, RBB_Pin, 1);
}
void ML() {
    HAL_GPIO_WritePin(LFF_GPIO_Port, LFF_Pin, 0); HAL_GPIO_WritePin(LFB_GPIO_Port, LFB_Pin, 0);
    HAL_GPIO_WritePin(RFF_GPIO_Port, RFF_Pin, 1); HAL_GPIO_WritePin(RFB_GPIO_Port, RFB_Pin, 0);
    HAL_GPIO_WritePin(LBF_GPIO_Port, LBF_Pin, 0); HAL_GPIO_WritePin(LBB_GPIO_Port, LBB_Pin, 0);
    HAL_GPIO_WritePin(RBF_GPIO_Port, RBF_Pin, 1); HAL_GPIO_WritePin(RBB_GPIO_Port, RBB_Pin, 0);
}
void MR() {
    HAL_GPIO_WritePin(LFF_GPIO_Port, LFF_Pin, 1); HAL_GPIO_WritePin(LFB_GPIO_Port, LFB_Pin, 0);
    HAL_GPIO_WritePin(RFF_GPIO_Port, RFF_Pin, 0); HAL_GPIO_WritePin(RFB_GPIO_Port, RFB_Pin, 0);
    HAL_GPIO_WritePin(LBF_GPIO_Port, LBF_Pin, 1); HAL_GPIO_WritePin(LBB_GPIO_Port, LBB_Pin, 0);
    HAL_GPIO_WritePin(RBF_GPIO_Port, RBF_Pin, 0); HAL_GPIO_WritePin(RBB_GPIO_Port, RBB_Pin, 0);
}
void ST() {
    HAL_GPIO_WritePin(LFF_GPIO_Port, LFF_Pin, 0); HAL_GPIO_WritePin(LFB_GPIO_Port, LFB_Pin, 0);
    HAL_GPIO_WritePin(RFF_GPIO_Port, RFF_Pin, 0); HAL_GPIO_WritePin(RFB_GPIO_Port, RFB_Pin, 0);
    HAL_GPIO_WritePin(LBF_GPIO_Port, LBF_Pin, 0); HAL_GPIO_WritePin(LBB_GPIO_Port, LBB_Pin, 0);
    HAL_GPIO_WritePin(RBF_GPIO_Port, RBF_Pin, 0); HAL_GPIO_WritePin(RBB_GPIO_Port, RBB_Pin, 0);
}

// ==========================================
// [SECTION 2] UART 인터럽트
// ==========================================
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        HAL_UART_Transmit(&huart2, &rx_data, 1, 10);

        if (is_avoiding == 0)
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

// ==========================================
// [SECTION 3] 심플 회피 알고리즘 (복구됨)
// ==========================================
void Avoid_Obstacle_Routine(void)
{
    is_avoiding = 1;
    ST(); // 정지
    printf("\r\n[!] Obstacle! Scanning...\r\n");
    HAL_Delay(500); // 안정화

    // ==========================================
    // 1. [판단 단계] 진입 전 딱 한 번만 결정 (Decision Locking)
    // ==========================================

    // 측면 거리 측정 (초기 판단용)
    trig3(); echo_time3 = echo3();
    dist3 = (echo_time3 > 0 && echo_time3 < 23000) ? (int)(17 * echo_time3 / 100) : 999;
    delay_us(15000);

    trig4(); echo_time4 = echo4();
    dist4 = (echo_time4 > 0 && echo_time4 < 23000) ? (int)(17 * echo_time4 / 100) : 999;

    int escape_plan = 0;
    int side_threshold = 150;

    // 작전 수립
    if (dist3 <= side_threshold && dist4 > side_threshold)
    {
        escape_plan = 1; // 우회전 작전
        printf(">> Plan: Turn RIGHT (Left Blocked)\r\n");
    }
    else if (dist4 <= side_threshold && dist3 > side_threshold)
    {
        escape_plan = 2; // 좌회전 작전
        printf(">> Plan: Turn LEFT (Right Blocked)\r\n");
    }
    else if (dist3 > side_threshold && dist4 > side_threshold)
    {
        escape_plan = 3; // 후진 후 회전 작전
        printf(">> Plan: BACK & Turn (Both Clear)\r\n");
    }
    else
    {
        escape_plan = 4; // 갇힘
        printf(">> Plan: Emergency Back (Stuck)\r\n");
    }

    // ==========================================
    // 2. [실행 단계] 모니터링은 하되, 판단은 바꾸지 않음
    // ==========================================
    while (1)
    {
        // A. 전방 거리 측정 (탈출용)
        trig1(); echo_time1 = echo1();
        dist1 = (echo_time1 > 0 && echo_time1 < 23000) ? (int)(17 * echo_time1 / 100) : 999;
        delay_us(2000);
        trig2(); echo_time2 = echo2();
        dist2 = (echo_time2 > 0 && echo_time2 < 23000) ? (int)(17 * echo_time2 / 100) : 999;

        // B. [요청하신 부분] 측면 거리 측정 (단순 모니터링용)
        // 측정은 하지만 escape_plan을 바꾸는 if문에는 넣지 않습니다!
        delay_us(2000);
        trig3(); echo_time3 = echo3();
        dist3 = (echo_time3 > 0 && echo_time3 < 23000) ? (int)(17 * echo_time3 / 100) : 999;

        delay_us(2000);
        trig4(); echo_time4 = echo4();
        dist4 = (echo_time4 > 0 && echo_time4 < 23000) ? (int)(17 * echo_time4 / 100) : 999;

        // C. 실시간 상태 출력
        printf("Running Plan %d... | Front: %d, %d | Side: %d, %d\r\n",
               escape_plan, dist1, dist2, dist3, dist4);

        // D. 탈출 조건
        if (dist1 > 350 && dist2 > 350)
        {
            ST();
            printf(">> Path Clear! Escaping Loop.\r\n");
            is_avoiding = 0;
            last_cmd_time = HAL_GetTick();
            break;
        }

        // E. 작전 수행 (센서 값이 변해도 무시하고 원래 작전 수행)
        switch (escape_plan)
        {
            case 1: LFRB(); break; // 우회전
            case 2: RFLB(); break; // 좌회전

            case 3: // 후진 후 좌회전
                // 한번 뒤로 갔으면 계속 회전만 하도록 로직 변경 (중복 후진 방지)
                MB(); HAL_Delay(500);
                escape_plan = 2; // 다음 루프부터는 좌회전(Case 2)만 수행
                break;

            case 4: MB(); break; // 후진
        }

        HAL_Delay(100); // 0.1초 대기
    }
}

// ==========================================
// [SECTION 4] 초음파 센서 함수
// ==========================================
void timer_start(void) { HAL_TIM_Base_Start(&htim2); }
void delay_us(uint16_t us) {
   __HAL_TIM_SET_COUNTER(&htim2, 0);
   while((__HAL_TIM_GET_COUNTER(&htim2)) < us);
}
void trig1(void) {
   HAL_GPIO_WritePin(TRIG1_GPIO_Port, TRIG1_Pin, HIGH);
   delay_us(10); HAL_GPIO_WritePin(TRIG1_GPIO_Port, TRIG1_Pin, LOW);
}
long unsigned int echo1(void) {
   long unsigned int echo = 0; int timeout = 0;
   while(HAL_GPIO_ReadPin(ECHO1_GPIO_Port, ECHO1_Pin) == LOW) { timeout++; if(timeout > 5000) return 0; }
   __HAL_TIM_SET_COUNTER(&htim2, 0); timeout = 0;
   while(HAL_GPIO_ReadPin(ECHO1_GPIO_Port, ECHO1_Pin) == HIGH) { timeout++; if(timeout > 20000) return 0; }
   echo = __HAL_TIM_GET_COUNTER(&htim2); return echo;
}
void trig2(void) {
   HAL_GPIO_WritePin(TRIG2_GPIO_Port, TRIG2_Pin, HIGH);
   delay_us(10); HAL_GPIO_WritePin(TRIG2_GPIO_Port, TRIG2_Pin, LOW);
}
long unsigned int echo2(void) {
   long unsigned int echo = 0; int timeout = 0;
   while(HAL_GPIO_ReadPin(ECHO2_GPIO_Port, ECHO2_Pin) == LOW) { timeout++; if(timeout > 5000) return 0; }
   __HAL_TIM_SET_COUNTER(&htim2, 0); timeout = 0;
   while(HAL_GPIO_ReadPin(ECHO2_GPIO_Port, ECHO2_Pin) == HIGH) { timeout++; if(timeout > 20000) return 0; }
   echo = __HAL_TIM_GET_COUNTER(&htim2); return echo;
}
void trig3(void) {
   HAL_GPIO_WritePin(TRIG3_GPIO_Port, TRIG3_Pin, HIGH);
   delay_us(10); HAL_GPIO_WritePin(TRIG3_GPIO_Port, TRIG3_Pin, LOW);
}
long unsigned int echo3(void) {
   long unsigned int echo = 0; int timeout = 0;
   while(HAL_GPIO_ReadPin(ECHO3_GPIO_Port, ECHO3_Pin) == LOW) { timeout++; if(timeout > 5000) return 0; }
   __HAL_TIM_SET_COUNTER(&htim2, 0); timeout = 0;
   while(HAL_GPIO_ReadPin(ECHO3_GPIO_Port, ECHO3_Pin) == HIGH) { timeout++; if(timeout > 40000) return 0; }
   echo = __HAL_TIM_GET_COUNTER(&htim2); return echo;

}
void trig4(void) {
   HAL_GPIO_WritePin(TRIG4_GPIO_Port, TRIG4_Pin, HIGH);
   delay_us(10); HAL_GPIO_WritePin(TRIG4_GPIO_Port, TRIG4_Pin, LOW);
}
long unsigned int echo4(void) {
   long unsigned int echo = 0; int timeout = 0;
   while(HAL_GPIO_ReadPin(ECHO4_GPIO_Port, ECHO4_Pin) == LOW) { timeout++; if(timeout > 5000) return 0; }
   __HAL_TIM_SET_COUNTER(&htim2, 0); timeout = 0;
   while(HAL_GPIO_ReadPin(ECHO4_GPIO_Port, ECHO4_Pin) == HIGH) { timeout++; if(timeout > 40000) return 0; }
   echo = __HAL_TIM_GET_COUNTER(&htim2); return echo;
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
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  timer_start();
  HAL_UART_Receive_IT(&huart2, &rx_data, 1);
  printf("\n=== SMART CAR STARTED (Ultrasonic Basic) ===\n");
  last_cmd_time = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t last_print_time = 0;

  while (1)
  {
      // 회피 중이면 메인 루프 패스
      if (is_avoiding == 1) continue;

      // 키보드 자동 정지
      if (HAL_GetTick() - last_cmd_time > AUTO_STOP_DELAY) ST();

      // 초음파 측정
      trig1(); echo_time1 = echo1();
      dist1 = (echo_time1 > 0 && echo_time1 < 23000) ? (int)(17 * echo_time1 / 100) : 999;

      delay_us(5000);

      trig2(); echo_time2 = echo2();
      dist2 = (echo_time2 > 0 && echo_time2 < 23000) ? (int)(17 * echo_time2 / 100) : 999;

      // [핵심] 15cm 이내 위험 감지 (둘 중 하나라도 가까우면 회피)
      if ((dist1 < 150 && dist1 > 0) || (dist2 < 150 && dist2 > 0))
      {
          Avoid_Obstacle_Routine();
      }

      if (HAL_GetTick() - last_print_time > 200) {
          printf("Dist: %d, %d mm\r\n", dist1, dist2);
          last_print_time = HAL_GetTick();
      }
  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 63;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|TRIG2_Pin|LBF_Pin|LFB_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LBB_Pin|TRIG3_Pin|TRIG4_Pin|LFF_Pin
                          |RBF_Pin|RBB_Pin|RFF_Pin|RFB_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TRIG1_GPIO_Port, TRIG1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ECHO2_Pin */
  GPIO_InitStruct.Pin = ECHO2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ECHO2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin TRIG2_Pin LBF_Pin LFB_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|TRIG2_Pin|LBF_Pin|LFB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ECHO1_Pin ECHO3_Pin ECHO4_Pin */
  GPIO_InitStruct.Pin = ECHO1_Pin|ECHO3_Pin|ECHO4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LBB_Pin TRIG3_Pin TRIG4_Pin LFF_Pin
                           RBF_Pin RBB_Pin RFF_Pin RFB_Pin */
  GPIO_InitStruct.Pin = LBB_Pin|TRIG3_Pin|TRIG4_Pin|LFF_Pin
                          |RBF_Pin|RBB_Pin|RFF_Pin|RFB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : TRIG1_Pin */
  GPIO_InitStruct.Pin = TRIG1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TRIG1_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
