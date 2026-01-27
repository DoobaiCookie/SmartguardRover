/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Smart Car + LCD Face (Removed LD2, Fixed Pins)
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
// LCD 색상 정의 (RGB565)
#define ST7735_BLACK   0x0000
#define ST7735_BLUE    0x001F
#define ST7735_RED     0xF800
#define ST7735_GREEN   0x07E0
#define ST7735_CYAN    0x07FF
#define ST7735_MAGENTA 0xF81F
#define ST7735_YELLOW  0xFFE0
#define ST7735_WHITE   0xFFFF
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

SPI_HandleTypeDef hspi1;

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

// LCD 상태 관리 (0: 초기화 전, 1: Happy, 2: Angry)
uint8_t current_face_state = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
void Avoid_Obstacle_Routine(void);
void trig1(void); long unsigned int echo1(void);
void trig2(void); long unsigned int echo2(void);
void trig3(void); long unsigned int echo3(void);
void trig4(void); long unsigned int echo4(void);
void delay_us(uint16_t us);
int LFRB(); int RFLB(); void ST(); void MB();
void Check_Light(void);
void Check_Tilt_State(void);
void Check_Fire_State(void);

// [LCD 함수]
void LCD_Init(void);
void LCD_WriteCommand(uint8_t cmd);
void LCD_WriteData(uint8_t data);
void LCD_FillScreen(uint16_t color);
void LCD_DrawRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
void Draw_Happy_Face(void);
void Draw_Angry_Face(void);
void Update_Face_Expression(void);
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

// ====================================================
// [LCD 드라이버]
// ====================================================
void LCD_WriteCommand(uint8_t cmd) {
    HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_RESET); // Command
    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET); // CS Low
    HAL_SPI_Transmit(&hspi1, &cmd, 1, 10);
    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);   // CS High
}

void LCD_WriteData(uint8_t data) {
    HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_SET);   // Data
    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET); // CS Low
    HAL_SPI_Transmit(&hspi1, &data, 1, 10);
    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);   // CS High
}

void LCD_SetWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
    LCD_WriteCommand(0x2A); LCD_WriteData(0x00); LCD_WriteData(x0); LCD_WriteData(0x00); LCD_WriteData(x1);
    LCD_WriteCommand(0x2B); LCD_WriteData(0x00); LCD_WriteData(y0); LCD_WriteData(0x00); LCD_WriteData(y1);
    LCD_WriteCommand(0x2C);
}

void LCD_FillScreen(uint16_t color) {
    LCD_SetWindow(0, 0, 127, 127);
    uint8_t data[2] = {color >> 8, color & 0xFF};
    HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);
    for(int i = 0; i < 128*128; i++) HAL_SPI_Transmit(&hspi1, data, 2, 10);
    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);
}

void LCD_DrawRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
    if((x >= 128) || (y >= 128)) return;
    if((x + w - 1) >= 128) w = 128 - x;
    if((y + h - 1) >= 128) h = 128 - y;
    LCD_SetWindow(x, y, x+w-1, y+h-1);
    uint8_t data[2] = {color >> 8, color & 0xFF};
    HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);
    for(int i=0; i<w*h; i++) HAL_SPI_Transmit(&hspi1, data, 2, 10);
    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);
}

void LCD_Init(void) {
    HAL_GPIO_WritePin(LCD_RES_GPIO_Port, LCD_RES_Pin, GPIO_PIN_RESET); HAL_Delay(50);
    HAL_GPIO_WritePin(LCD_RES_GPIO_Port, LCD_RES_Pin, GPIO_PIN_SET); HAL_Delay(50);
    LCD_WriteCommand(0x01); HAL_Delay(150);
    LCD_WriteCommand(0x11); HAL_Delay(200);
    LCD_WriteCommand(0x3A); LCD_WriteData(0x05);
    LCD_WriteCommand(0x29); HAL_Delay(10);
}

// [얼굴 그리기]
void Draw_Happy_Face(void) {
    LCD_FillScreen(ST7735_BLUE);
    LCD_DrawRect(30, 40, 20, 20, ST7735_WHITE); LCD_DrawRect(80, 40, 20, 20, ST7735_WHITE);
    LCD_DrawRect(35, 45, 10, 10, ST7735_BLACK); LCD_DrawRect(85, 45, 10, 10, ST7735_BLACK);
    LCD_DrawRect(30, 90, 10, 5, ST7735_WHITE); LCD_DrawRect(40, 95, 10, 5, ST7735_WHITE);
    LCD_DrawRect(50, 100, 28, 5, ST7735_WHITE); LCD_DrawRect(78, 95, 10, 5, ST7735_WHITE);
    LCD_DrawRect(88, 90, 10, 5, ST7735_WHITE);
}

void Draw_Angry_Face(void) {
    LCD_FillScreen(ST7735_RED);
    LCD_DrawRect(25, 30, 30, 8, ST7735_BLACK); LCD_DrawRect(75, 30, 30, 8, ST7735_BLACK);
    LCD_DrawRect(30, 45, 20, 15, ST7735_WHITE); LCD_DrawRect(80, 45, 20, 15, ST7735_WHITE);
    LCD_DrawRect(40, 90, 48, 8, ST7735_WHITE);
}

void Update_Face_Expression(void) {
    if (is_emergency == 1 || is_fire == 1) {
        if (current_face_state != 2) { Draw_Angry_Face(); current_face_state = 2; }
    } else {
        if (current_face_state != 1) { Draw_Happy_Face(); current_face_state = 1; }
    }
}

// [모터 제어]
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

void Check_Tilt_State(void) {
    static uint32_t tilt_start_time = 0;
    if (HAL_GPIO_ReadPin(C6_GPIO_Port, C6_Pin) == GPIO_PIN_SET) {
        if (tilt_start_time == 0) tilt_start_time = HAL_GetTick();
        if (HAL_GetTick() - tilt_start_time > 2000) {
            ST(); is_emergency = 1;
            HAL_GPIO_WritePin(LED_LEFT_GPIO_Port, LED_LEFT_Pin, 1);
            HAL_GPIO_WritePin(LED_RIGHT_GPIO_Port, LED_RIGHT_Pin, 1);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 500);
            printf("⚠️차량 전복됨!!\r\n");
            tilt_start_time = 0;
        }
    } else { tilt_start_time = 0; }
}

void Check_Fire_State(void) {
    if (HAL_GPIO_ReadPin(FLAME_GPIO_Port, FLAME_Pin) == GPIO_PIN_SET) {
        if (is_fire == 0) {
            ST(); is_fire = 1;
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 500);
            HAL_GPIO_WritePin(LED_LEFT_GPIO_Port, LED_LEFT_Pin, 1);
            HAL_GPIO_WritePin(LED_RIGHT_GPIO_Port, LED_RIGHT_Pin, 1);
            printf("🔥 화재 발생!\r\n");
        }
    }
}

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

    printf("회피기동모드: %d\r\n", escape_plan);

    while (1)
    {
        Check_Tilt_State(); Check_Fire_State();
        Update_Face_Expression();

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
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  timer_start();

  // LCD 초기화
  LCD_Init();
  Draw_Happy_Face();

  // 부저 초기화
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  HAL_Delay(500);

  is_fire = 0; is_emergency = 0;

  HAL_UART_Receive_IT(&huart2, &rx_data, 1);
  printf("\n=== SmartGuardRover With LCD ===\n");
  last_cmd_time = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 63;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  HAL_GPIO_WritePin(GPIOC, LED_LEFT_Pin|LED_RIGHT_Pin|TRIG1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LCD_RES_Pin|LCD_DC_Pin|TRIG2_Pin|LBF_Pin
                          |LFB_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LBB_Pin|TRIG3_Pin|TRIG4_Pin|LFF_Pin
                          |RBF_Pin|RBB_Pin|LCD_CS_Pin|RFF_Pin
                          |RFB_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : FLAME_Pin */
  GPIO_InitStruct.Pin = FLAME_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(FLAME_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_LEFT_Pin LED_RIGHT_Pin TRIG1_Pin */
  GPIO_InitStruct.Pin = LED_LEFT_Pin|LED_RIGHT_Pin|TRIG1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_RES_Pin LCD_DC_Pin TRIG2_Pin LBF_Pin
                           LFB_Pin */
  GPIO_InitStruct.Pin = LCD_RES_Pin|LCD_DC_Pin|TRIG2_Pin|LBF_Pin
                          |LFB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ECHO2_Pin */
  GPIO_InitStruct.Pin = ECHO2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ECHO2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ECHO1_Pin ECHO3_Pin ECHO4_Pin */
  GPIO_InitStruct.Pin = ECHO1_Pin|ECHO3_Pin|ECHO4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LBB_Pin TRIG3_Pin TRIG4_Pin LFF_Pin
                           RBF_Pin RBB_Pin LCD_CS_Pin RFF_Pin
                           RFB_Pin */
  GPIO_InitStruct.Pin = LBB_Pin|TRIG3_Pin|TRIG4_Pin|LFF_Pin
                          |RBF_Pin|RBB_Pin|LCD_CS_Pin|RFF_Pin
                          |RFB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : C6_Pin */
  GPIO_InitStruct.Pin = C6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(C6_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void Check_Light(void)
{
    uint32_t adc_value = 0; float voltage = 0.0f;
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK) {
        adc_value = HAL_ADC_GetValue(&hadc1); voltage = (adc_value * 3.3f) / 4095.0f;
        if (voltage < 2.0f && is_fire == 0) { HAL_GPIO_WritePin(LED_LEFT_GPIO_Port, LED_LEFT_Pin, 1); HAL_GPIO_WritePin(LED_RIGHT_GPIO_Port, LED_RIGHT_Pin, 1); }
        else if (is_fire == 0) { HAL_GPIO_WritePin(LED_LEFT_GPIO_Port, LED_LEFT_Pin, 0); HAL_GPIO_WritePin(LED_RIGHT_GPIO_Port, LED_RIGHT_Pin, 0); }
    }
    HAL_ADC_Stop(&hadc1);
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) { }
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
