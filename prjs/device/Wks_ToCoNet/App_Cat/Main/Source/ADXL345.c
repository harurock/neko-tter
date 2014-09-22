/**************************************************************************/
/*!
    @file     Adafruit_ADXL345.cpp
    @author   K.Townsend (Adafruit Industries)
    @license  BSD (see license.txt)

    The ADXL345 is a digital accelerometer with 13-bit resolution, capable
    of measuring up to +/-16g.  This driver communicate using I2C.

    This is a library for the Adafruit ADXL345 breakout
    ----> https://www.adafruit.com/products/1231

    Adafruit invests time and resources providing this open source code,
    please support Adafruit and open-source hardware by purchasing
    products from Adafruit!

    @section  HISTORY
    
    v1.1 - Added Adafruit_Sensor library support
    v1.0 - First release
*/
/**************************************************************************/
#include <string.h>
#include <jendefs.h>
#include <AppHardwareApi.h>

#include "SMBus.h"
#include "ADXL345.h"

/**************************************************************************/
/*!
    @brief  Abstract away platform differences in Arduino wire library
*/
/**************************************************************************/
static uint8 i2cread(void) {
  uint8 dat[1];
  bSMBusSequentialRead(ADXL345_ADDRESS, 1, dat);
  return dat[0];
}

/**************************************************************************/
/*!
    @brief  Abstract away platform differences in Arduino wire library
*/
/**************************************************************************/
static void i2cwrite(uint8 x) {
  bSMBusWrite(ADXL345_ADDRESS, x, 0, NULL);
}

/**************************************************************************/
/*!
    @brief  Writes 8-bits to the specified destination register
*/
/**************************************************************************/
void vADXL345_WriteReg(uint8 reg, uint8 value) {
  bSMBusWrite(ADXL345_ADDRESS, reg, 1, &value);
}

/**************************************************************************/
/*!
    @brief  Reads 8-bits from the specified register
*/
/**************************************************************************/
uint8 u8ADXL345_ReadReg(uint8 reg) {  
  i2cwrite(reg);
  return i2cread();
}

/**************************************************************************/
/*!
    @brief  Reads 16-bits from the specified register
*/
/**************************************************************************/
int16 u16ADXL345_Read(uint8 reg) {
  uint8 dat[2];

  i2cwrite(reg);
  bSMBusSequentialRead(ADXL345_ADDRESS, 2, dat);
  return (int16)(dat[0] | (dat[1] << 8));  
}

void vADXL345_SetDataRate(bool low_power, dataRate_t dataRate)
{
  if(low_power)
    vADXL345_WriteReg(ADXL345_REG_BW_RATE, dataRate | 0x10);
  else
    vADXL345_WriteReg(ADXL345_REG_BW_RATE, dataRate);  
}

void vADXL345_SetRange(range_t range)
{
  /* Red the data format register to preserve bits */
  uint8 format = u8ADXL345_ReadReg(ADXL345_REG_DATA_FORMAT);

  /* Update the data rate */
  format &= ~0x03;
  format |= range;
  
  /* Make sure that the FULL-RES bit is enabled for range scaling */
  format |= 0x08;
  
  /* Write the register back to the IC */
  vADXL345_WriteReg(ADXL345_REG_DATA_FORMAT, format);
}

void vADXL345_GetFloatXYZ(tsFloatXYZ *xyz)
{
  memset(xyz, 0, sizeof(tsFloatXYZ));

  xyz->x = u16ADXL345_Read(ADXL345_REG_DATAX0) * ADXL345_MG2G_MULTIPLIER;
  xyz->y = u16ADXL345_Read(ADXL345_REG_DATAY0) * ADXL345_MG2G_MULTIPLIER;
  xyz->z = u16ADXL345_Read(ADXL345_REG_DATAZ0) * ADXL345_MG2G_MULTIPLIER;
}

void vADXL345_GetInt16XYZ(tsInt16XYZ *xyz)
{
  memset(xyz, 0, sizeof(tsInt16XYZ));

  xyz->x = u16ADXL345_Read(ADXL345_REG_DATAX0);
  xyz->y = u16ADXL345_Read(ADXL345_REG_DATAY0);
  xyz->z = u16ADXL345_Read(ADXL345_REG_DATAZ0);
}

void vADXL345_Sleep(void)
{
  // D4:Activity(0x10)
  vADXL345_WriteReg(ADXL345_REG_INT_ENABLE, 0x10);
  // D3:Measure(0x08) | D2:sleep(0x04)
  vADXL345_WriteReg(ADXL345_REG_POWER_CTL, 0x08 | 0x04);
}

void vADXL345_Wake(void)
{
  // D4:Activity(0x10) | D3:Inactivity(0x08)
  vADXL345_WriteReg(ADXL345_REG_INT_ENABLE, 0x18);
  // wake-up
  vADXL345_WriteReg(ADXL345_REG_POWER_CTL, 0x08);
}
