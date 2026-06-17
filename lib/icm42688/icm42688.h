/**
  ******************************************************************************
  * @file    icm42688.h
  * @brief   ICM-42688 6-axis IMU driver header
  *          Datasheet: DS-000347 Rev 1.2
  * @note    SPI read bit = MSB of address (bit7 = 1 to read)
  *          Data registers are big-endian (high byte at lower address)
  ******************************************************************************
  */

#ifndef ICM42688_H
#define ICM42688_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>
#include "stm32h7xx_hal_gpio.h"
#include "stm32h7xx_hal_spi.h"

/* Register Definitions (§14 of DS-000347) ------------------------------------*/
#define ICM42688_WHO_AM_I           0x75  /* Expected value: 0x47 */
#define ICM42688_DEVICE_CONFIG      0x11
#define ICM42688_INT_CONFIG         0x14
#define ICM42688_FIFO_CONFIG        0x16
#define ICM42688_TEMP_DATA1         0x1D  /* Temperature MSB */
#define ICM42688_TEMP_DATA0         0x1E  /* Temperature LSB */
#define ICM42688_ACCEL_DATA_X1      0x1F  /* Accel X MSB */
#define ICM42688_ACCEL_DATA_X0      0x20  /* Accel X LSB */
#define ICM42688_ACCEL_DATA_Y1      0x21  /* Accel Y MSB */
#define ICM42688_ACCEL_DATA_Y0      0x22  /* Accel Y LSB */
#define ICM42688_ACCEL_DATA_Z1      0x23  /* Accel Z MSB */
#define ICM42688_ACCEL_DATA_Z0      0x24  /* Accel Z LSB */
#define ICM42688_GYRO_DATA_X1       0x25  /* Gyro X MSB */
#define ICM42688_GYRO_DATA_X0       0x26  /* Gyro X LSB */
#define ICM42688_GYRO_DATA_Y1       0x27  /* Gyro Y MSB */
#define ICM42688_GYRO_DATA_Y0       0x28  /* Gyro Y LSB */
#define ICM42688_GYRO_DATA_Z1       0x29  /* Gyro Z MSB */
#define ICM42688_GYRO_DATA_Z0       0x2A  /* Gyro Z LSB */
#define ICM42688_PWR_MGMT0          0x4E  /* Power management */
#define ICM42688_GYRO_CONFIG0       0x4F
#define ICM42688_ACCEL_CONFIG0      0x50

/* SPI Command Bits -------------------------------------------------------*/
#define ICM42688_SPI_READ           0x80  /* MSB = 1 for read */
#define ICM42688_SPI_WRITE          0x00  /* MSB = 0 for write */

/* ICM42688 Handle Structure -----------------------------------------------*/
typedef struct {
  SPI_HandleTypeDef *spi;      /* Pointer to SPI handle */
  GPIO_TypeDef *cs_port;       /* Chip select GPIO port */
  uint16_t cs_pin;             /* Chip select GPIO pin */
} ICM42688_Handle_t;

/* Function Prototypes ------------------------------------------------------*/

/**
  * @brief  Initialize ICM-42688 IMU over SPI
  * @param  handle: Pointer to ICM42688_Handle_t structure
  * @param  spi: Pointer to SPI_HandleTypeDef
  * @param  cs_port: Chip select GPIO port
  * @param  cs_pin: Chip select GPIO pin
  * @retval HAL_StatusTypeDef: HAL_OK if successful
  */
HAL_StatusTypeDef ICM42688_Init(ICM42688_Handle_t *handle, 
                                 SPI_HandleTypeDef *spi,
                                 GPIO_TypeDef *cs_port, 
                                 uint16_t cs_pin);

/**
  * @brief  Read single register from ICM-42688
  * @param  handle: Pointer to ICM42688_Handle_t structure
  * @param  reg: Register address
  * @param  data: Pointer to store read data
  * @retval HAL_StatusTypeDef: HAL_OK if successful
  */
HAL_StatusTypeDef ICM42688_ReadReg(ICM42688_Handle_t *handle, 
                                    uint8_t reg, 
                                    uint8_t *data);

/**
  * @brief  Write single register to ICM-42688
  * @param  handle: Pointer to ICM42688_Handle_t structure
  * @param  reg: Register address
  * @param  data: Data to write
  * @retval HAL_StatusTypeDef: HAL_OK if successful
  */
HAL_StatusTypeDef ICM42688_WriteReg(ICM42688_Handle_t *handle, 
                                     uint8_t reg, 
                                     uint8_t data);

/**
  * @brief  Soft reset the ICM-42688
  * @param  handle: Pointer to ICM42688_Handle_t structure
  * @retval HAL_StatusTypeDef: HAL_OK if successful
  */
HAL_StatusTypeDef ICM42688_SoftReset(ICM42688_Handle_t *handle);

/**
  * @brief  Configure gyro/accel and enable low-noise mode
  * @param  handle: Pointer to ICM42688_Handle_t structure
  * @retval HAL_StatusTypeDef: HAL_OK if successful
  */
HAL_StatusTypeDef ICM42688_ConfigureSensor(ICM42688_Handle_t *handle);

/**
  * @brief  Read a 14-byte burst from TEMP_DATA1: temp + accel + gyro
  * @param  handle: Pointer to ICM42688_Handle_t structure
  * @param  temp: Pointer to store raw temperature
  * @param  accel_x: Pointer to store raw accel X
  * @param  accel_y: Pointer to store raw accel Y
  * @param  accel_z: Pointer to store raw accel Z
  * @param  gyro_x: Pointer to store raw gyro X
  * @param  gyro_y: Pointer to store raw gyro Y
  * @param  gyro_z: Pointer to store raw gyro Z
  * @retval HAL_StatusTypeDef: HAL_OK if successful
  */
HAL_StatusTypeDef ICM42688_ReadTempAccelGyro(ICM42688_Handle_t *handle,
                                             int16_t *temp,
                                             int16_t *accel_x,
                                             int16_t *accel_y,
                                             int16_t *accel_z,
                                             int16_t *gyro_x,
                                             int16_t *gyro_y,
                                             int16_t *gyro_z);

/**
  * @brief  Read WHO_AM_I register (identity verification)
  * @param  handle: Pointer to ICM42688_Handle_t structure
  * @param  who_am_i: Pointer to store WHO_AM_I value
  * @retval HAL_StatusTypeDef: HAL_OK if successful
  * @note   Expected value is 0x47
  */
HAL_StatusTypeDef ICM42688_GetWhoAmI(ICM42688_Handle_t *handle, 
                                      uint8_t *who_am_i);

#ifdef __cplusplus
}
#endif

#endif /* ICM42688_H */
