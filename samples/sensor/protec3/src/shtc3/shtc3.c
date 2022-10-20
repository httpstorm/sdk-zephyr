//==============================================================================
//    S E N S I R I O N   AG,  Laubisruetistr. 50, CH-8712 Staefa, Switzerland
//==============================================================================
// Project   :  SHTC3 Sample Code (V1.0)
// File      :  shtc3.c (V1.0)
// Author    :  RFU
// Date      :  24-Nov-2017
// Controller:  STM32F100RB
// IDE       :  µVision V5.17.0.0
// Compiler  :  Armcc
// Brief     :  Sensor Layer: Implementation of functions for sensor access.
//==============================================================================

#include "shtc3.h"

typedef enum{
  READ_ID            = 0xEFC8, // command: read ID register
  SOFT_RESET         = 0x805D, // soft reset
  SLEEP              = 0xB098, // sleep
  WAKEUP             = 0x3517, // wakeup
  MEAS_T_RH_POLLING  = 0x7866, // meas. read T first, clock stretching disabled
  MEAS_T_RH_CLOCKSTR = 0x7CA2, // meas. read T first, clock stretching enabled
  MEAS_RH_T_POLLING  = 0x58E0, // meas. read RH first, clock stretching disabled
  MEAS_RH_T_CLOCKSTR = 0x5C24  // meas. read RH first, clock stretching enabled
}etCommands;

static etError SHTC3_Read2BytesAndCrc(uint16_t *data);
static etError SHTC3_CheckCrc(uint8_t data[], uint8_t nbrOfBytes,
                              uint8_t checksum);
static float SHTC3_CalcTemperature(uint16_t rawValue);
static float SHTC3_CalcHumidity(uint16_t rawValue);

static uint8_t _Address;
const struct device * i2c_dev;

//------------------------------------------------------------------------------
void SHTC3_Init(uint8_t address,
	const struct device * i2c2_dev)
        {
  _Address = address;
  i2c_dev = i2c2_dev;
}

//------------------------------------------------------------------------------
etError SHTC3_GetTempAndHumi(float *temp, float *humi){
  etError  error;        // error code
  uint16_t rawValueTemp; // temperature raw value from sensor
  uint16_t rawValueHumi; // humidity raw value from sensor

  uint8_t cmd[2];
  uint8_t bytes[6];
  cmd[0] = (MEAS_T_RH_CLOCKSTR >> 8);
  cmd[1] = (MEAS_T_RH_CLOCKSTR & 0xFF);

  if (0 == i2c_write(i2c_dev, cmd, 2, _Address))
  {
          error = NO_ERROR;
  } else
  {
          error = ACK_ERROR;
  }
  // if no error, read temperature and humidity raw values
  if(error == NO_ERROR)
  {

    if (0 == i2c_read(i2c_dev, bytes, 6, _Address))
    {
          uint8_t checksum = bytes[2];

          // verify checksum
          error |= SHTC3_CheckCrc(bytes, 2, checksum);
          // combine the two bytes to a 16-bit value
          rawValueTemp = (bytes[0] << 8) | bytes[1];

          checksum = bytes[5];

          // verify checksum
          error |= SHTC3_CheckCrc(&bytes[3], 2, checksum);
          // combine the two bytes to a 16-bit value
          rawValueHumi = (bytes[3] << 8) | bytes[4];

          // if no error, calculate temperature in °C and humidity in %RH
          if(error == NO_ERROR) {
                  *temp = SHTC3_CalcTemperature(rawValueTemp);
                  *humi = SHTC3_CalcHumidity(rawValueHumi);
          }
    } else
    {
      error = ACK_ERROR;
    }
  }

  return error;
}

//------------------------------------------------------------------------------
etError SHTC3_GetTempAndHumiPolling(float *temp, float *humi){
  etError  error;           // error code
  uint8_t  maxPolling = 20; // max. retries to read the measurement (polling)
  uint16_t rawValueTemp;    // temperature raw value from sensor
  uint16_t rawValueHumi;    // humidity raw value from sensor

  uint8_t cmd[2];
  uint8_t bytes[6];
  cmd[0] = (MEAS_T_RH_POLLING >> 8);
  cmd[1] = (MEAS_T_RH_POLLING & 0xFF);

  if (0 == i2c_write(i2c_dev, cmd, 2, _Address))
  {
          error = NO_ERROR;
  } else
  {
          error = ACK_ERROR;
  }

  k_msleep(1);

  // if no error, ...
  if(error == NO_ERROR)
  {
    // poll every 1ms for measurement ready
    while(maxPolling--) {

      // delay 1ms
      k_msleep(1);

      if (0 == i2c_read(i2c_dev, bytes, 6, _Address))
      {
          uint8_t checksum = bytes[2];
          error = NO_ERROR;

          // verify checksum
          error |= SHTC3_CheckCrc(bytes, 2, checksum);
          // combine the two bytes to a 16-bit value
          rawValueTemp = (bytes[0] << 8) | bytes[1];

          checksum = bytes[5];

          // verify checksum
          error |= SHTC3_CheckCrc(&bytes[3], 2, checksum);
          // combine the two bytes to a 16-bit value
          rawValueHumi = (bytes[3] << 8) | bytes[4];

          // if no error, calculate temperature in °C and humidity in %RH
          if(error == NO_ERROR) {
                  *temp = SHTC3_CalcTemperature(rawValueTemp);
                  *humi = SHTC3_CalcHumidity(rawValueHumi);
          }
      } else
      {
          error = ACK_ERROR;
      }

      // if no error, read temperature and humidity raw values
      if(error == NO_ERROR) {
            break;
      }
    }
  }

  return error;
}

//------------------------------------------------------------------------------
etError SHTC3_GetId(uint16_t *id)
{
  etError error; // error code

  uint8_t cmd[2];
  cmd[0] = (READ_ID >> 8);
  cmd[1] = (READ_ID & 0xFF);

  if (0 == i2c_write(i2c_dev, cmd, 2, _Address))
  {
          error = NO_ERROR;
  } else
  {
          error = ACK_ERROR;
  }

  // if no error, read ID
  if(error == NO_ERROR)
  {
    error = SHTC3_Read2BytesAndCrc(id);
  }

  return error;
}

//------------------------------------------------------------------------------
etError SHTC3_Sleep(void) {
  etError error;

  uint8_t cmd[2];
  cmd[0] = (SLEEP >> 8);
  cmd[1] = (SLEEP & 0xFF);

  if (0 == i2c_write(i2c_dev, cmd, 2, _Address))
  {
          error = NO_ERROR;
  } else
  {
          error = ACK_ERROR;
  }

  return error;
}

//------------------------------------------------------------------------------
etError SHTC3_Wakeup(void)
{
  etError error;

  uint8_t cmd[2];
  cmd[0] = (WAKEUP >> 8);
  cmd[1] = (WAKEUP & 0xFF);

  if (0 == i2c_write(i2c_dev, cmd, 2, _Address))
  {
          error = NO_ERROR;
  } else
  {
          error = ACK_ERROR;
  }

  k_usleep(100); // wait 100 us

  return error;
}

//------------------------------------------------------------------------------
etError SHTC3_SoftReset(void)
{
  etError error; // error code

  uint8_t cmd[2];
  cmd[0] = (SOFT_RESET >> 8);
  cmd[1] = (SOFT_RESET & 0xFF);

  if (0 == i2c_write(i2c_dev, cmd, 2, _Address))
  {
          error = NO_ERROR;
  } else
  {
          error = ACK_ERROR;
  }

  return error;
}

//------------------------------------------------------------------------------
static etError SHTC3_Read2BytesAndCrc(uint16_t *data){
  etError error;    // error code
  uint8_t bytes[3]; // read data array
  uint8_t checksum; // checksum byte

  // read two data bytes and one checksum byte
  if (0 == i2c_read(i2c_dev, bytes, 3, _Address))
  {
          checksum = bytes[2];

          // verify checksum
          error = SHTC3_CheckCrc(bytes, 2, checksum);
          // combine the two bytes to a 16-bit value
          *data = (bytes[0] << 8) | bytes[1];
  } else
  {
          error = ACK_ERROR;
  }

  return error;
}

//------------------------------------------------------------------------------
static etError SHTC3_CheckCrc(uint8_t data[], uint8_t nbrOfBytes,
                              uint8_t checksum){
  uint8_t bit;        // bit mask
  uint8_t crc = 0xFF; // calculated checksum
  uint8_t byteCtr;    // byte counter

  // calculates 8-Bit checksum with given polynomial
  for(byteCtr = 0; byteCtr < nbrOfBytes; byteCtr++) {
    crc ^= (data[byteCtr]);
    for(bit = 8; bit > 0; --bit) {
      if(crc & 0x80) {
        crc = (crc << 1) ^ CRC_POLYNOMIAL;
      } else {
        crc = (crc << 1);
      }
    }
  }

  // verify checksum
  if(crc != checksum) {
    return CHECKSUM_ERROR;
  } else {
    return NO_ERROR;
  }
}

//------------------------------------------------------------------------------
static float SHTC3_CalcTemperature(uint16_t rawValue){
  // calculate temperature [°C]
  // T = -45 + 175 * rawValue / 2^16
  return 175 * (float)rawValue / 65536.0f - 45.0f;
}

//------------------------------------------------------------------------------
static float SHTC3_CalcHumidity(uint16_t rawValue){
  // calculate relative humidity [%RH]
  // RH = rawValue / 2^16 * 100
  return 100 * (float)rawValue / 65536.0f;
}
