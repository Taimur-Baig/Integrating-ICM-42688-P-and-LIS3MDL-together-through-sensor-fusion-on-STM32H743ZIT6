/**
  ******************************************************************************
  * @file    lis3mdl.h
  * @brief   LIS3MDL magnetometer driver header
  * @note    SPI read bit = MSB of address (bit7 = 1 to read)
  *          Auto-increment bit = bit6 (0x40) for burst access
  ******************************************************************************
  */

#ifndef LIS3MDL_H
#define LIS3MDL_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>
#include "stm32h7xx_hal_gpio.h"
#include "stm32h7xx_hal_spi.h"

/* Register Definitions ------------------------------------------------------*/
#define LIS3MDL_WHO_AM_I           0x0F
#define LIS3MDL_CTRL_REG1          0x20
#define LIS3MDL_CTRL_REG2          0x21
#define LIS3MDL_CTRL_REG3          0x22
#define LIS3MDL_CTRL_REG4          0x23
#define LIS3MDL_CTRL_REG5          0x24
#define LIS3MDL_OUTX_L             0x28
#define LIS3MDL_OUTX_H             0x29
#define LIS3MDL_OUTY_L             0x2A
#define LIS3MDL_OUTY_H             0x2B
#define LIS3MDL_OUTZ_L             0x2C
#define LIS3MDL_OUTZ_H             0x2D

/* SPI Command Bits ----------------------------------------------------------*/
#define LIS3MDL_SPI_READ           0x80U
#define LIS3MDL_SPI_AUTOINC        0x40U

/* Sensitivity at ±4 gauss: 0.14 mgauss/LSB -> 0.014 uT/LSB */
#define LIS3MDL_SENSITIVITY_4GAUSS 0.014f

/* LIS3MDL Handle Structure --------------------------------------------------*/
typedef struct {
  SPI_HandleTypeDef *spi;
  GPIO_TypeDef *cs_port;
  uint16_t cs_pin;
  float offset[3];
  float scale[3];
} LIS3MDL_Handle_t;

/* Function Prototypes -------------------------------------------------------*/
HAL_StatusTypeDef LIS3MDL_Init(LIS3MDL_Handle_t *handle,
                               SPI_HandleTypeDef *spi,
                               GPIO_TypeDef *cs_port,
                               uint16_t cs_pin);

HAL_StatusTypeDef LIS3MDL_ReadReg(LIS3MDL_Handle_t *handle,
                                  uint8_t reg,
                                  uint8_t *data);

HAL_StatusTypeDef LIS3MDL_WriteReg(LIS3MDL_Handle_t *handle,
                                   uint8_t reg,
                                   uint8_t data);

HAL_StatusTypeDef LIS3MDL_GetWhoAmI(LIS3MDL_Handle_t *handle,
                                    uint8_t *who_am_i);

HAL_StatusTypeDef LIS3MDL_ConfigureSensor(LIS3MDL_Handle_t *handle);

HAL_StatusTypeDef LIS3MDL_ReadRawMag(LIS3MDL_Handle_t *handle,
                                     int16_t *mag_x,
                                     int16_t *mag_y,
                                     int16_t *mag_z);

HAL_StatusTypeDef LIS3MDL_ReadMagneticField(LIS3MDL_Handle_t *handle,
                                            float *mag_x_uT,
                                            float *mag_y_uT,
                                            float *mag_z_uT);

void LIS3MDL_SetCalibration(LIS3MDL_Handle_t *handle,
                            const float offset[3],
                            const float scale[3]);

#ifdef __cplusplus
}
#endif

#endif /* LIS3MDL_H */
