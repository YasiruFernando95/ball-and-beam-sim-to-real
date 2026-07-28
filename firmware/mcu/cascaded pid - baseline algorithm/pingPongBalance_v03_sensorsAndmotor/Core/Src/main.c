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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "dist_adc.h"
#include "ssd1306.h"
#include "fonts.h"
#include "mt6816.h"
#include "encoder_speed.h"
#include "motor_driver.h"
#include "virtual_wall.h"
#include <math.h>
#include "pid.h"
#include "usb_telemetry.h"
#include <string.h>
#include <stdio.h>
#include "stm32f4xx_hal.h"   // or your MCU family header
#include "usbd_cdc_if.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
extern uint8_t CDC_Transmit(const void* Buf, uint32_t Len);
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CRC_HandleTypeDef hcrc;

I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_tx;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim10;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CRC_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM10_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define ANGLE_MIN_DEG   -25.0f
#define ANGLE_MAX_DEG    25.0f
#define MOTOR_TO_ANGLE_SIGN   (+1)
MotorDriver_HandleTypeDef motor;

EncoderSpeed_Handle_t enc;
uint16_t mt6816_zero = 8344 & 0x3FFF; //Set Zero position here
MT6816_Handle_t mt6816;

#define ADC_DMA_LEN  64
static uint16_t adc_dma_buf[ADC_DMA_LEN];
static AdcDistance_t adcDist;

// -------- Cascaded PID --------
PID_TypeDef pidPos;   // outer: ball position -> angle setpoint
PID_TypeDef pidAng;   // inner: angle -> motor command

// Hardcoded defaults (edit these)
static const float DEFAULT_POS_KP = 22.302f;
static const float DEFAULT_POS_KI = 0.0f;
static const float DEFAULT_POS_KD = 4.587f;

static const float DEFAULT_ANG_KP = 0.02115f;
static const float DEFAULT_ANG_KI = 0.02104f;
static const float DEFAULT_ANG_KD = 0.0001710f;


// Outer PID (position)
double pos_in = 0.0, pos_out = 0.0, pos_sp = 0.0;  // pos_out = ang_sp (deg)

// Inner PID (angle)
double ang_in = 0.0, ang_out = 0.0, ang_sp = 0.0;  // ang_out = motor cmd [-1..+1]

// Start gains (tune later)
static double Kp_pos = DEFAULT_POS_KP, Ki_pos = DEFAULT_POS_KI, Kd_pos = DEFAULT_POS_KD;   // d -> degrees
static double Kp_ang = DEFAULT_ANG_KP, Ki_ang = DEFAULT_ANG_KI, Kd_ang = DEFAULT_ANG_KD;   // degrees -> cmd

// Optional: outer deadband to reduce jitter near center
static const float POS_DEADBAND = 0.01f; // normalized d


int findCharPosition(const char *buffer, size_t size, char character) {
    for (size_t i = 0; i < size; ++i) {
        if (buffer[i] == character) {
            return (int)i;
        }
    }
    return -1; // '\r' not found
}

void USB_CDC_RxHandler(uint8_t *buf, uint32_t len)
{
	USB_Command_Process(buf, len);
}

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c) {
    ssd1306_I2C_TxCpltCallback(hi2c);
}

static void OLED_Render(float V, float P, float A, float R)
{
    char buf[32];

    ssd1306_Fill(Black);

    // Title
    ssd1306_SetCursor(0, 0);
    ssd1306_WriteString("Ball Beam", Font_11x18, White);

    // Second line: V
    ssd1306_SetCursor(0, 16);
    snprintf(buf, sizeof(buf), "V = %.3f V", V);
    ssd1306_WriteString(buf, Font_7x10, White);

    // Third line: P
    ssd1306_SetCursor(0, 28);
    snprintf(buf, sizeof(buf), "P = %.3f", P);
    ssd1306_WriteString(buf, Font_7x10, White);

    // Fourth line: A
    ssd1306_SetCursor(0, 40);
    snprintf(buf, sizeof(buf), "A = %.3f deg", A);
    ssd1306_WriteString(buf, Font_7x10, White);

    // Fifth line: R
    ssd1306_SetCursor(0, 52);
    snprintf(buf, sizeof(buf), "R = %.3f rad/s", R);
    ssd1306_WriteString(buf, Font_7x10, White);

    // Start non-blocking DMA flush (returns immediately)
    (void)ssd1306_UpdateScreen_DMA(&hi2c1);
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef * hspi)
{
//		absEnc_spiDone = 1;// TX-RX Done
		MT6816_SpiTxRxCpltCallback(&mt6816, hspi);
}

int32_t MT6816_GetZeroed_Int32(uint16_t raw14)
{
    // 1) Force to 14-bit
    uint16_t raw = raw14 & 0x3FFF;

    // 2) Zero with wrap in 14 bits
    uint16_t z = (raw - mt6816_zero) & 0x3FFF;   // 0..16383

    // 3) Convert to signed around 0
    //    0..8191  ->  0..8191
    //    8192..16383 -> -8192..-1
    if (z >= 8192U)
        return (int32_t)z - 16384;   // 16384 = 1 << 14
    else
        return (int32_t)z;
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
  MX_CRC_Init();
  MX_USB_DEVICE_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_TIM10_Init();
  /* USER CODE BEGIN 2 */
  //Motor driver init
  uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim3);
  MotorDriver_Init(&motor,
                   &htim3,
                   TIM_CHANNEL_1,
                   GPIOB,
                   GPIO_PIN_3,
                   period,
                   GPIO_PIN_RESET);   // "forward" = pin low
  MotorDriver_Start(&motor);

  //Timer encoder counter init
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  EncoderSpeed_Init(&enc, &htim2,
                        98u,      // counts_per_rev
                        14.0f,      // gear_ratio
                        0.001f);    // dt_s (TIM10 period)
  EncoderSpeed_SetLowPass(&enc, 2.0f);     // Hz cutoff
  HAL_TIM_Base_Start_IT(&htim10);

  //Abs enc command building
  MT6816_Init(&mt6816, &hspi1, 3);

  // Init lcd using one of the stm32HAL i2c typedefs
  if (ssd1306_Init(&hi2c1) != 0) {
//    ; //comment error handler if you plug out the board
	Error_Handler();
  }
  HAL_Delay(1000);

  ssd1306_Fill(Black);
  if (!ssd1306_IsBusy()) {
      // draw text into SSD1306_Buffer...
      ssd1306_UpdateScreen_DMA(&hi2c1);
  }

  HAL_Delay(1000);

  // Preset scaling:
  // we want distance = 0..1 over that window, inverted (near=1, far=0), with mild nonlinearity.

  AdcDist_Init(&adcDist, &hadc1,
               adc_dma_buf, ADC_DMA_LEN,
			   /* v_ref */ 3.3f,
			   /* adc_bits */ 12,
               /* v_low */ 0.44f,
               /* v_mid */ 1.73f,   // if not between low & high, it will auto-set to midpoint
               /* v_high*/ 3.0f,
               /* invert */ 0,
               /* gamma  */ 1.0f,
               /* compute_period_ms (unused) */ 0);
  AdcDist_SetFilterAlpha(&adcDist, 0.05);
//  AdcDist_SetMedianWindow(&adcDist, 5);
  AdcDist_Start(&adcDist);

  // ============ Cascaded PID ============

  g_ang_kp = DEFAULT_ANG_KP;
  g_ang_ki = DEFAULT_ANG_KI;
  g_ang_kd = DEFAULT_ANG_KD;

  g_pos_kp = DEFAULT_POS_KP;
  g_pos_ki = DEFAULT_POS_KI;
  g_pos_kd = DEFAULT_POS_KD;

  // Outer: ball position -> desired angle (deg)
  PID2(&pidPos,
       &pos_in,
       &pos_out,
       &pos_sp,
       Kp_pos, Ki_pos, Kd_pos,
       _PID_CD_REVERSE);   // REVERSE matches what you observed for ball centering

  PID_SetOutputLimits(&pidPos, ANGLE_MIN_DEG, ANGLE_MAX_DEG);
  pidPos.SampleTime = 10;                 // outer slower (50 Hz is plenty)
  PID_SetMode(&pidPos, _PID_MODE_AUTOMATIC);

  // Inner: angle -> motor command [-1..+1]
  PID2(&pidAng,
       &ang_in,
       &ang_out,
       &ang_sp,
       Kp_ang, Ki_ang, Kd_ang,
       _PID_CD_DIRECT);

  PID_SetOutputLimits(&pidAng, -1.0, 1.0);  // start limited while tuning
  pidAng.SampleTime = 3;                     // inner faster (100 Hz)
  PID_SetMode(&pidAng, _PID_MODE_AUTOMATIC);

  // Targets
  pos_sp = 0.0;
  ang_sp = 0.0;

  uint32_t last_print = HAL_GetTick();
  uint32_t last_motorStep = HAL_GetTick();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t now = HAL_GetTick();
    AdcDist_Update(&adcDist);
    MT6816_Update(&mt6816, now);

    /////////////////////////////

    if (now - last_print >= 20)
    {
      uint8_t buf[64];
	  last_print = now;
	  float v = AdcDist_GetVoltage(&adcDist);
	  float d = -(2*AdcDist_GetDistance(&adcDist)-1); //0-1, scaled to -1 to 1;

	  uint16_t A_raw;
	  MT6816_GetRaw(&mt6816, &A_raw);
	  float A = MT6816_GetZeroed_Int32(A_raw)*(360.0/16384.0);

	  float r = EncoderSpeed_GetOutputRadPerSecFilt(&enc);

	  if (!ssd1306_IsBusy()) {
	      OLED_Render(v, d, A, r);
	  }

	  int len = snprintf((char*)buf, sizeof(buf),
	      "D:%.3f,A:%.2f,W:%.3f\n",
	      d, A, r);

	  CDC_Transmit_FS(buf, (uint16_t)len);

    }
    if ((now - last_motorStep) >= 10)
    {
        last_motorStep = now;

        // ---- 1) Update ball position ----
        AdcDist_Update(&adcDist);
        float d = 2.0f * AdcDist_GetDistance(&adcDist) - 1.0f;

        // ---- 2) Read beam angle ----
        uint16_t A_raw;
        MT6816_GetRaw(&mt6816, &A_raw);
        float A = MT6816_GetZeroed_Int32(A_raw) * (360.0f / 16384.0f);

        // ---- 3) OUTER loop inputs ----
        float d_for_pid = d;
        if (fabsf(d_for_pid) < POS_DEADBAND) d_for_pid = 0.0f;

        float r = EncoderSpeed_GetOutputRadPerSecFilt(&enc);

        pos_in = (double)d_for_pid;
        pos_sp = (double)g_pos_sp;


		PID_SetTunings(&pidPos, g_pos_kp, g_pos_ki, g_pos_kd);

		// If outer gains all zero, clear memory
		if (fabsf(g_pos_kp) < 1e-6f && fabsf(g_pos_ki) < 1e-6f && fabsf(g_pos_kd) < 1e-6f)
		{
			pidPos.OutputSum = 0.0;
			pidPos.LastInput = pos_in;
			pos_out = 0.0;
		}

        PID_Compute(&pidPos);
        ang_sp = pos_out;

        if (!g_outer_loop_armed)
        {
            ang_sp = 0.0;
            // Clear outer loop integrator to prevent windup
            pidPos.OutputSum = 0.0;
            pidPos.LastInput = pos_in;
        }


        // Slew-rate limit on ang_sp (deg per 10ms tick)
        static float ang_sp_f = 0.0f;
        const float MAX_STEP_DEG = 1.0f; // try 0.5..2.0

        float target = (float)ang_sp;
        float delta  = target - ang_sp_f;

        if (delta >  MAX_STEP_DEG) delta =  MAX_STEP_DEG;
        if (delta < -MAX_STEP_DEG) delta = -MAX_STEP_DEG;

        ang_sp_f += delta;
        ang_sp = (double)ang_sp_f;


        // Safety clamp
        if (ang_sp > ANGLE_MAX_DEG) ang_sp = ANGLE_MAX_DEG;
        if (ang_sp < ANGLE_MIN_DEG) ang_sp = ANGLE_MIN_DEG;

        // ---- 6) INNER loop input ----
        ang_in = (double)A;

        // ---- 7) Update INNER tunings if changed ----
        static float last_ang_kp = -1.0f, last_ang_ki = -1.0f, last_ang_kd = -1.0f;
        if (g_ang_kp != last_ang_kp || g_ang_ki != last_ang_ki || g_ang_kd != last_ang_kd)
        {
            last_ang_kp = g_ang_kp;
            last_ang_ki = g_ang_ki;
            last_ang_kd = g_ang_kd;

            PID_SetTunings(&pidAng, g_ang_kp, g_ang_ki, g_ang_kd);

            // If inner gains all zero, clear memory + force output 0
            if (fabsf(g_ang_kp) < 1e-6f && fabsf(g_ang_ki) < 1e-6f && fabsf(g_ang_kd) < 1e-6f)
            {
                pidAng.OutputSum = 0.0;
                pidAng.LastInput = ang_in;
                ang_out = 0.0;
            }
        }

        // ---- 8) Compute INNER and drive motor ----
        if (PID_Compute(&pidAng))
        {
            float cmd = (float)ang_out;   // expected in [-1..+1] if pidAng limits are set that way

            float safe_cmd = VirtualWall_Apply(cmd,
                                               A,
                                               ANGLE_MIN_DEG,
                                               ANGLE_MAX_DEG,
                                               MOTOR_TO_ANGLE_SIGN);

            // If wall clamps to zero while we are trying to push, kill integrators & brake
            if (safe_cmd == 0.0f && cmd != 0.0f)
            {
                pidAng.OutputSum = 0.0;
                pidAng.LastInput = ang_in;

                pidPos.OutputSum = 0.0;
                pidPos.LastInput = pos_in;

                float w = EncoderSpeed_GetOutputRadPerSecFilt(&enc);
                float brake = -0.2f * w;
                if (brake >  1.0f) brake =  1.0f;
                if (brake < -1.0f) brake = -1.0f;
                MotorDriver_SetSpeed(&motor, brake);
            }
            else
            {
                MotorDriver_SetSpeed(&motor, safe_cmd);
            }
        }
        else
        {
            // Optional: if PID didn't compute this tick, keep applying last command or do nothing
            // MotorDriver_SetSpeed(&motor, last_cmd);
        }
    }


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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 4199;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 0;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 41999;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM11 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
  if (htim->Instance == TIM10)   // TIM10 update event
  {
      EncoderSpeed_Update(&enc);
      AdcDist_Update(&adcDist);
  }
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM11)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
#ifdef USE_FULL_ASSERT
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
