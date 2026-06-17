/**
  ******************************************************************************
  * @file    lis3mdl.c
  * @brief   LIS3MDL magnetometer driver implementation
  ******************************************************************************
  */

#include "lis3mdl.h"
#include <string.h>

#define LIS3MDL_SPI_TIMEOUT        1000U

static void LIS3MDL_CS_High(LIS3MDL_Handle_t *handle)
{
  HAL_GPIO_WritePin(handle->cs_port, handle->cs_pin, GPIO_PIN_SET);
}

static void LIS3MDL_CS_Low(LIS3MDL_Handle_t *handle)
{
  HAL_GPIO_WritePin(handle->cs_port, handle->cs_pin, GPIO_PIN_RESET);
}

HAL_StatusTypeDef LIS3MDL_Init(LIS3MDL_Handle_t *handle,
                               SPI_HandleTypeDef *spi,
                               GPIO_TypeDef *cs_port,
                               uint16_t cs_pin)
{
  if (handle == NULL || spi == NULL)
  {
    return HAL_ERROR;
  }

  handle->spi = spi;
  handle->cs_port = cs_port;
  handle->cs_pin = cs_pin;

  handle->offset[0] = 0.0f;
  handle->offset[1] = 0.0f;
  handle->offset[2] = 0.0f;

  handle->scale[0] = 1.0f;
  handle->scale[1] = 1.0f;
  handle->scale[2] = 1.0f;

  LIS3MDL_CS_High(handle);
  HAL_Delay(100);

  return HAL_OK;
}

HAL_StatusTypeDef LIS3MDL_ReadReg(LIS3MDL_Handle_t *handle,
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

  tx_buf[0] = reg | LIS3MDL_SPI_READ;
  tx_buf[1] = 0x00;

  LIS3MDL_CS_Low(handle);
  status = HAL_SPI_TransmitReceive(handle->spi, tx_buf, rx_buf, 2, LIS3MDL_SPI_TIMEOUT);
  LIS3MDL_CS_High(handle);

  if (status == HAL_OK)
  {
    *data = rx_buf[1];
  }

  return status;
}

HAL_StatusTypeDef LIS3MDL_WriteReg(LIS3MDL_Handle_t *handle,
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

  tx_buf[0] = reg & ~LIS3MDL_SPI_READ;
  tx_buf[1] = data;

  LIS3MDL_CS_Low(handle);
  status = HAL_SPI_TransmitReceive(handle->spi, tx_buf, rx_buf, 2, LIS3MDL_SPI_TIMEOUT);
  LIS3MDL_CS_High(handle);

  return status;
}

HAL_StatusTypeDef LIS3MDL_GetWhoAmI(LIS3MDL_Handle_t *handle,
                                    uint8_t *who_am_i)
{
  return LIS3MDL_ReadReg(handle, LIS3MDL_WHO_AM_I, who_am_i);
}

HAL_StatusTypeDef LIS3MDL_ConfigureSensor(LIS3MDL_Handle_t *handle)
{
  HAL_StatusTypeDef status;

  status = LIS3MDL_WriteReg(handle, LIS3MDL_CTRL_REG1, 0xFE);
  if (status != HAL_OK) return status;

  status = LIS3MDL_WriteReg(handle, LIS3MDL_CTRL_REG2, 0x00);
  if (status != HAL_OK) return status;

  status = LIS3MDL_WriteReg(handle, LIS3MDL_CTRL_REG3, 0x00);
  if (status != HAL_OK) return status;

  status = LIS3MDL_WriteReg(handle, LIS3MDL_CTRL_REG4, 0x0C);
  if (status != HAL_OK) return status;

  status = LIS3MDL_WriteReg(handle, LIS3MDL_CTRL_REG5, 0x40);
  if (status != HAL_OK) return status;

  HAL_Delay(20);
  return HAL_OK;
}

HAL_StatusTypeDef LIS3MDL_ReadRawMag(LIS3MDL_Handle_t *handle,
                                     int16_t *mag_x,
                                     int16_t *mag_y,
                                     int16_t *mag_z)
{
  HAL_StatusTypeDef status;
  uint8_t tx_buf[7];
  uint8_t rx_buf[7];

  if (handle == NULL || mag_x == NULL || mag_y == NULL || mag_z == NULL)
  {
    return HAL_ERROR;
  }

  memset(tx_buf, 0, sizeof(tx_buf));
  tx_buf[0] = LIS3MDL_OUTX_L | LIS3MDL_SPI_READ | LIS3MDL_SPI_AUTOINC;

  LIS3MDL_CS_Low(handle);
  status = HAL_SPI_TransmitReceive(handle->spi, tx_buf, rx_buf, 7, LIS3MDL_SPI_TIMEOUT);
  LIS3MDL_CS_High(handle);

  if (status != HAL_OK)
  {
    return status;
  }

  *mag_x = (int16_t)((rx_buf[2] << 8) | rx_buf[1]);
  *mag_y = (int16_t)((rx_buf[4] << 8) | rx_buf[3]);
  *mag_z = (int16_t)((rx_buf[6] << 8) | rx_buf[5]);

  return HAL_OK;
}

HAL_StatusTypeDef LIS3MDL_ReadMagneticField(LIS3MDL_Handle_t *handle,
                                            float *mag_x_uT,
                                            float *mag_y_uT,
                                            float *mag_z_uT)
{
  int16_t raw_x, raw_y, raw_z;
  HAL_StatusTypeDef status = LIS3MDL_ReadRawMag(handle, &raw_x, &raw_y, &raw_z);
  if (status != HAL_OK)
  {
    return status;
  }

  float x = raw_x * LIS3MDL_SENSITIVITY_4GAUSS;
  float y = raw_y * LIS3MDL_SENSITIVITY_4GAUSS;
  float z = raw_z * LIS3MDL_SENSITIVITY_4GAUSS;

  if (mag_x_uT != NULL) *mag_x_uT = (x - handle->offset[0]) * handle->scale[0];
  if (mag_y_uT != NULL) *mag_y_uT = (y - handle->offset[1]) * handle->scale[1];
  if (mag_z_uT != NULL) *mag_z_uT = (z - handle->offset[2]) * handle->scale[2];

  return HAL_OK;
}

void LIS3MDL_SetCalibration(LIS3MDL_Handle_t *handle,
                            const float offset[3],
                            const float scale[3])
{
  if (handle == NULL || offset == NULL || scale == NULL)
  {
    return;
  }

  handle->offset[0] = offset[0];
  handle->offset[1] = offset[1];
  handle->offset[2] = offset[2];

  handle->scale[0] = scale[0];
  handle->scale[1] = scale[1];
  handle->scale[2] = scale[2];
}
