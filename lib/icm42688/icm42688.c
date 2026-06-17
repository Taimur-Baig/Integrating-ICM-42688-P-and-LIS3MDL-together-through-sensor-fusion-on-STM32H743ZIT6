/**
  ******************************************************************************
  * @file    icm42688.c
  * @brief   ICM-42688 6-axis IMU driver implementation
  *          Datasheet: DS-000347 Rev 1.2
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "icm42688.h"
#include <string.h>

/* Private macros ------------------------------------------------------------*/
#define ICM42688_SPI_TIMEOUT        1000  /* SPI timeout in ms */

/**
  * @brief  Set Chip Select line
  */
static void ICM42688_CS_High(ICM42688_Handle_t *handle)
{
  HAL_GPIO_WritePin(handle->cs_port, handle->cs_pin, GPIO_PIN_SET);
}

/**
  * @brief  Clear Chip Select line
  */
static void ICM42688_CS_Low(ICM42688_Handle_t *handle)
{
  HAL_GPIO_WritePin(handle->cs_port, handle->cs_pin, GPIO_PIN_RESET);
}

/**
  * @brief  Initialize ICM-42688 IMU over SPI
  */
HAL_StatusTypeDef ICM42688_Init(ICM42688_Handle_t *handle, 
                                 SPI_HandleTypeDef *spi,
                                 GPIO_TypeDef *cs_port, 
                                 uint16_t cs_pin)
{
  if (handle == NULL || spi == NULL)
  {
    return HAL_ERROR;
  }

  /* Store SPI and CS configuration */
  handle->spi = spi;
  handle->cs_port = cs_port;
  handle->cs_pin = cs_pin;

  /* Ensure CS is high initially */
  ICM42688_CS_High(handle);
  
  /* Small delay for SPI bus to settle */
  HAL_Delay(100);

  return HAL_OK;
}

/**
  * @brief  Read single register from ICM-42688
  * @note   SPI frame format:
  *         Byte 0: [RW bit | address bits 6:0]
  *         Byte 1: Data (read from device)
  */
HAL_StatusTypeDef ICM42688_ReadReg(ICM42688_Handle_t *handle, 
                                    uint8_t reg, 
                                    uint8_t *data)
{
  HAL_StatusTypeDef status;
  uint8_t tx_buf[2];
  uint8_t rx_buf[2];

  if (handle == NULL || data == NULL)
  {
    return HAL_ERROR;
  }

  /* Prepare SPI transaction:
     TX[0] = register address with read bit (bit7 = 1)
     TX[1] = dummy byte (will receive data)
  */
  tx_buf[0] = reg | ICM42688_SPI_READ;  /* Set read bit */
  tx_buf[1] = 0x00;                      /* Dummy byte */

  /* Pull CS low */
  ICM42688_CS_Low(handle);
  
  /* SPI full-duplex transfer */
  status = HAL_SPI_TransmitReceive(handle->spi, tx_buf, rx_buf, 2, 
                                   ICM42688_SPI_TIMEOUT);

  /* Pull CS high */
  ICM42688_CS_High(handle);

  if (status == HAL_OK)
  {
    /* Data is in RX[1] */
    *data = rx_buf[1];
  }

  return status;
}

/**
  * @brief  Write single register to ICM-42688
  * @note   SPI frame format:
  *         Byte 0: [RW bit (0) | address bits 6:0]
  *         Byte 1: Data (written to device)
  */
HAL_StatusTypeDef ICM42688_WriteReg(ICM42688_Handle_t *handle, 
                                     uint8_t reg, 
                                     uint8_t data)
{
  HAL_StatusTypeDef status;
  uint8_t tx_buf[2];
  uint8_t rx_buf[2];

  if (handle == NULL)
  {
    return HAL_ERROR;
  }

  /* Prepare SPI transaction:
     TX[0] = register address with write bit (bit7 = 0)
     TX[1] = data to write
  */
  tx_buf[0] = reg & ~ICM42688_SPI_READ;  /* Clear read bit (write) */
  tx_buf[1] = data;

  /* Pull CS low */
  ICM42688_CS_Low(handle);
  
  /* SPI full-duplex transfer */
  status = HAL_SPI_TransmitReceive(handle->spi, tx_buf, rx_buf, 2, 
                                   ICM42688_SPI_TIMEOUT);

  /* Pull CS high */
  ICM42688_CS_High(handle);

  return status;
}

/**
  * @brief  Read WHO_AM_I register (identity verification)
  * @param  handle: Pointer to ICM42688_Handle_t structure
  * @param  who_am_i: Pointer to store WHO_AM_I value
  * @retval HAL_StatusTypeDef: HAL_OK if successful
  * @note   Expected value is 0x47 (from datasheet §14.58)
  */
HAL_StatusTypeDef ICM42688_SoftReset(ICM42688_Handle_t *handle)
{
  HAL_StatusTypeDef status = ICM42688_WriteReg(handle, ICM42688_DEVICE_CONFIG, 0x01);
  if (status != HAL_OK)
  {
    return status;
  }

  HAL_Delay(10);
  return HAL_OK;
}

HAL_StatusTypeDef ICM42688_ConfigureSensor(ICM42688_Handle_t *handle)
{
  HAL_StatusTypeDef status;

  status = ICM42688_WriteReg(handle, ICM42688_GYRO_CONFIG0, 0x06);
  if (status != HAL_OK) return status;

  status = ICM42688_WriteReg(handle, ICM42688_ACCEL_CONFIG0, 0x26);
  if (status != HAL_OK) return status;

  status = ICM42688_WriteReg(handle, ICM42688_PWR_MGMT0, 0x0F);
  if (status != HAL_OK) return status;

  HAL_Delay(50);
  return HAL_OK;
}

HAL_StatusTypeDef ICM42688_ReadTempAccelGyro(ICM42688_Handle_t *handle,
                                             int16_t *temp,
                                             int16_t *accel_x,
                                             int16_t *accel_y,
                                             int16_t *accel_z,
                                             int16_t *gyro_x,
                                             int16_t *gyro_y,
                                             int16_t *gyro_z)
{
  HAL_StatusTypeDef status;
  uint8_t tx_buf[15];
  uint8_t rx_buf[15];

  if (handle == NULL || temp == NULL || accel_x == NULL || accel_y == NULL || accel_z == NULL ||
      gyro_x == NULL || gyro_y == NULL || gyro_z == NULL)
  {
    return HAL_ERROR;
  }

  memset(tx_buf, 0, sizeof(tx_buf));
  tx_buf[0] = ICM42688_TEMP_DATA1 | ICM42688_SPI_READ;

  ICM42688_CS_Low(handle);
  status = HAL_SPI_TransmitReceive(handle->spi, tx_buf, rx_buf, 15, ICM42688_SPI_TIMEOUT);
  ICM42688_CS_High(handle);

  if (status != HAL_OK)
  {
    return status;
  }

  *temp     = (int16_t)((rx_buf[1] << 8) | rx_buf[2]);
  *accel_x  = (int16_t)((rx_buf[3] << 8) | rx_buf[4]);
  *accel_y  = (int16_t)((rx_buf[5] << 8) | rx_buf[6]);
  *accel_z  = (int16_t)((rx_buf[7] << 8) | rx_buf[8]);
  *gyro_x   = (int16_t)((rx_buf[9] << 8) | rx_buf[10]);
  *gyro_y   = (int16_t)((rx_buf[11] << 8) | rx_buf[12]);
  *gyro_z   = (int16_t)((rx_buf[13] << 8) | rx_buf[14]);

  return HAL_OK;
}

HAL_StatusTypeDef ICM42688_GetWhoAmI(ICM42688_Handle_t *handle, 
                                      uint8_t *who_am_i)
{
  return ICM42688_ReadReg(handle, ICM42688_WHO_AM_I, who_am_i);
}
