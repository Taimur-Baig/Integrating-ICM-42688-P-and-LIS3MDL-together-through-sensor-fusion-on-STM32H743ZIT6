/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "usart.h"
#include "gpio.h"
#include "spi.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include<stdio.h>
#include <math.h>
#include "../lib/icm42688/icm42688.h"
#include "../lib/lis3mdl/lis3mdl.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

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
static void MPU_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE BEGIN 0 */
typedef struct {
    float angle; // The calculated optimal angle (State variable)
    float bias;  // The calculated gyroscope bias
    float P[2][2]; // Error covariance matrix
} Kalman_t;

// Tuning parameters (Adjust these if the response is too slow or too twitchy)
float Q_angle = 0.001f; // Process noise variance for the accelerometer
float Q_bias  = 0.003f; // Process noise variance for the gyro bias
float R_measure = 0.03f; // Measurement noise variance (Accelerometer confidence)

static float NormalizeAngle(float angle)
{
    while (angle > 180.0f) angle -= 360.0f;
    while (angle <= -180.0f) angle += 360.0f;
    return angle;
}

void Kalman_Update(Kalman_t *Kalman, float newAngle, float newRate, float dt) {
    // Step 1: Predict state ahead
    Kalman->angle += dt * (newRate - Kalman->bias);
    Kalman->angle = NormalizeAngle(Kalman->angle);

    // Step 2: Predict error covariance ahead
    Kalman->P[0][0] += dt * (dt * Kalman->P[1][1] - Kalman->P[0][1] - Kalman->P[1][0] + Q_angle);
    Kalman->P[0][1] -= dt * Kalman->P[1][1];
    Kalman->P[1][0] -= dt * Kalman->P[1][1];
    Kalman->P[1][1] += Q_bias * dt;

    // Step 3: Calculate Kalman gain
    float S = Kalman->P[0][0] + R_measure;
    float K[2];
    K[0] = Kalman->P[0][0] / S;
    K[1] = Kalman->P[1][0] / S;

    // Step 4: Update state estimate with measurement
    float y = newAngle - Kalman->angle;
    if (y > 180.0f) y -= 360.0f;
    else if (y < -180.0f) y += 360.0f;

    Kalman->angle += K[0] * y;
    Kalman->angle = NormalizeAngle(Kalman->angle);
    Kalman->bias  += K[1] * y;

    // Step 5: Update error covariance matrix
    float P00_temp = Kalman->P[0][0];
    float P01_temp = Kalman->P[0][1];

    Kalman->P[0][0] -= K[0] * P00_temp;
    Kalman->P[0][1] -= K[0] * P01_temp;
    Kalman->P[1][0] -= K[1] * P00_temp;
    Kalman->P[1][1] -= K[1] * P01_temp;
}
/* USER CODE END 0 */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

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
  MX_SPI4_Init();
/* USER CODE BEGIN 2 */
/* USER CODE BEGIN 2 */
HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
HAL_GPIO_WritePin(MAG_CS_GPIO_Port, MAG_CS_Pin, GPIO_PIN_SET);

setvbuf(stdout, NULL, _IONBF, 0);

/* ── Hard-Iron calibration offsets ─────────────────────────────────────
 * You MUST collect these by running the calibration routine below ONCE,
 * then paste the printed values here and recompile.
 * Until then, set all to 0 and expect a constant heading error.          */
float mag_offset_x = -14.987f;   // <-- paste your measured value
float mag_offset_y = 7.7910f;   // <-- paste your measured value
float mag_offset_z = -79.9050f;   // <-- paste your measured value

// ── Kalman structures ───────────────────────────────────────────────────
Kalman_t kalman_pitch = { .angle = 0.0f, .bias = 0.0f, .P = {{0,0},{0,0}} };
Kalman_t kalman_roll  = { .angle = 0.0f, .bias = 0.0f, .P = {{0,0},{0,0}} };
Kalman_t kalman_yaw   = { .angle = 0.0f, .bias = 0.0f, .P = {{0,0},{0,0}} };

/* Initialize ICM-42688 */
ICM42688_Handle_t imu_handle;
ICM42688_Init(&imu_handle, &hspi4, IMU_CS_GPIO_Port, IMU_CS_Pin);

uint8_t who_am_i = 0;
if (ICM42688_GetWhoAmI(&imu_handle, &who_am_i) != HAL_OK || who_am_i != 0x47)
  printf("ICM WHO_AM_I = 0x%02X (ERROR)\r\n", who_am_i);
else
  printf("ICM WHO_AM_I = 0x47 (OK)\r\n");

ICM42688_SoftReset(&imu_handle);
ICM42688_ConfigureSensor(&imu_handle);

/* ── Gyro Z calibration ────────────────────────────────────────────────*/
printf("Calibrating Gyroscope... DO NOT MOVE!\r\n");
float gz_offset = 0.0f;
for (int i = 0; i < 200; i++) {          // More samples = better offset
    int16_t t, ax, ay, az, gx, gy, gz;
    if (ICM42688_ReadTempAccelGyro(&imu_handle, &t, &ax, &ay, &az, &gx, &gy, &gz) == HAL_OK)
        gz_offset += (gz / 16.4f);
    HAL_Delay(5);
}
gz_offset /= 200.0f;
printf("Gyro Z Offset: %.4f dps\r\n", gz_offset);

/* Initialize LIS3MDL */
LIS3MDL_Handle_t mag_handle;
LIS3MDL_Init(&mag_handle, &hspi4, MAG_CS_GPIO_Port, MAG_CS_Pin);

if (LIS3MDL_GetWhoAmI(&mag_handle, &who_am_i) != HAL_OK || who_am_i != 0x3D)
  printf("MAG WHO_AM_I = 0x%02X (ERROR)\r\n", who_am_i);
else
  printf("MAG WHO_AM_I = 0x3D (OK)\r\n");

LIS3MDL_ConfigureSensor(&mag_handle);

/* ════════════════════════════════════════════════════════════════════════
 * HARD-IRON CALIBRATION ROUTINE
 * Uncomment this block ONCE, rotate the board slowly through all
 * orientations for ~30 seconds, then read the printed offsets.
 * Paste them into mag_offset_x/y/z above, then re-comment this block.
 * ════════════════════════════════════════════════════════════════════════ */
/*
{
    float mx_min = 1e9f, mx_max = -1e9f;
    float my_min = 1e9f, my_max = -1e9f;
    float mz_min = 1e9f, mz_max = -1e9f;
    printf("=== HARD-IRON CAL: rotate board in all directions for 30s ===\r\n");
    uint32_t t_end = HAL_GetTick() + 30000;
    while (HAL_GetTick() < t_end) {
        float mx, my, mz;
        if (LIS3MDL_ReadMagneticField(&mag_handle, &mx, &my, &mz) == HAL_OK) {
            if (mx < mx_min){ mx_min = mx;  if (mx > mx_max) mx_max = mx;};
            if (my < my_min){ my_min = my;  if (my > my_max) my_max = my;};
            if (mz < mz_min){ mz_min = mz;  if (mz > mz_max) mz_max = mz;};
        }
        HAL_Delay(20);
    }
    printf("Paste these into your code:\r\n");
    printf("mag_offset_x = %.4f;\r\n", (mx_min + mx_max) / 2.0f);
    printf("mag_offset_y = %.4f;\r\n", (my_min + my_max) / 2.0f);
    printf("mag_offset_z = %.4f;\r\n", (mz_min + mz_max) / 2.0f);
    while(1); // Stop here — recompile after pasting offsets
}
*/

uint32_t last_tick = HAL_GetTick();
static float mag_x_filt = 0.0f, mag_y_filt = 0.0f, mag_z_filt = 0.0f;
static uint8_t mag_filter_init = 0;

printf("=== Fusion Loop Starting ===\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

   /* USER CODE BEGIN 3 */

    // ── FIXED: Measure actual elapsed time, don't assume ──────────────
    uint32_t now = HAL_GetTick();
    float dt = (now - last_tick) / 1000.0f;   // real dt in seconds
    if (dt <= 0.0f || dt > 1.0f) dt = 0.01f;  // sanity clamp
    last_tick = now;

    int16_t temp_raw, ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw;
    float mag_x_uT = 0.0f, mag_y_uT = 0.0f, mag_z_uT = 0.0f;
    uint8_t imu_ok = 0, mag_ok = 0;

    // 1. Read IMU
    if (ICM42688_ReadTempAccelGyro(&imu_handle, &temp_raw,
            &ax_raw, &ay_raw, &az_raw,
            &gx_raw, &gy_raw, &gz_raw) == HAL_OK)
        imu_ok = 1;

    // 2. Read Magnetometer
    if (LIS3MDL_ReadMagneticField(&mag_handle, &mag_x_uT, &mag_y_uT, &mag_z_uT) == HAL_OK) {
        mag_ok = 1;

        // ── Apply hard-iron correction FIRST ──────────────────────────
        mag_x_uT -= mag_offset_x;
        mag_y_uT -= mag_offset_y;
        mag_z_uT -= mag_offset_z;

        // Low-pass filter
        if (!mag_filter_init) {
            mag_x_filt = mag_x_uT;
            mag_y_filt = mag_y_uT;
            mag_z_filt = mag_z_uT;
            mag_filter_init = 1;
        } else {
            const float alpha = 0.15f;
            mag_x_filt = alpha * mag_x_uT + (1.0f - alpha) * mag_x_filt;
            mag_y_filt = alpha * mag_y_uT + (1.0f - alpha) * mag_y_filt;
            mag_z_filt = alpha * mag_z_uT + (1.0f - alpha) * mag_z_filt;
        }
        mag_x_uT = mag_x_filt;
        mag_y_uT = mag_y_filt;
        mag_z_uT = mag_z_filt;
    }

    if (imu_ok && mag_ok)
    {
        // Convert to engineering units
        float ax = (ax_raw / 4096.0f) * 9.80665f;
        float ay = (ay_raw / 4096.0f) * 9.80665f;
        float az = (az_raw / 4096.0f) * 9.80665f;
        float gx_dps = gx_raw / 16.4f;
        float gy_dps = gy_raw / 16.4f;
        float gz_dps = (gz_raw / 16.4f) - gz_offset;

        // Pitch & Roll from accelerometer
        float accel_pitch = atan2f(ay, sqrtf(ax*ax + az*az)) * 57.29578f;
        float accel_roll  = atan2f(-ax, az) * 57.29578f;

        // Kalman update for pitch & roll
        Kalman_Update(&kalman_pitch, accel_pitch, gy_dps, dt);
        Kalman_Update(&kalman_roll,  accel_roll,  gx_dps, dt);

        // Tilt compensation — CORRECTED formula
        float pitch_rad = kalman_pitch.angle * 0.01745329f;
        float roll_rad  = kalman_roll.angle  * 0.01745329f;

        float cos_p = cosf(pitch_rad), sin_p = sinf(pitch_rad);
        float cos_r = cosf(roll_rad),  sin_r = sinf(roll_rad);

        float mx_comp =  mag_x_uT * cos_p
                       + mag_y_uT * sin_r * sin_p
                       + mag_z_uT * cos_r * sin_p;

        float my_comp =  mag_y_uT * cos_r
                       - mag_z_uT * sin_r;          // ← corrected sign

        // Magnetic heading
        float mag_yaw = atan2f(-my_comp, mx_comp) * 57.29578f;
        mag_yaw = NormalizeAngle(mag_yaw);

        // Kalman update for yaw
        Kalman_Update(&kalman_yaw, mag_yaw, gz_dps, dt);

        printf("PITCH:%.2f, ROLL:%.2f, YAW_FILT:%.2f, Max:180.0, Min:-180.0\r\n",
               kalman_pitch.angle, kalman_roll.angle, kalman_yaw.angle);
    }

    HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
    HAL_Delay(10);
    /* USER CODE END 3 */
}
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 200;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
// doin this for uart
int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

/* USER CODE END 4 */

 /* MPU Configuration */

static void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

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
