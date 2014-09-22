/**************************************************************************/
/*!
    @file     Adafruit_ADS1015.h
    @author   K. Townsend (Adafruit Industries)
    @license  BSD (see license.txt)

    This is a library for the Adafruit ADS1015 breakout board
    ----> https://www.adafruit.com/products/???

    Adafruit invests time and resources providing this open source code,
    please support Adafruit and open-source hardware by purchasing
    products from Adafruit!

    @section  HISTORY

    v1.0  - First release
*/
/**************************************************************************/
#ifndef  ADXL345_H_INCLUDED
#define  ADXL345_H_INCLUDED

#include <jendefs.h>

/*=========================================================================
    I2C ADDRESS/BITS
    -----------------------------------------------------------------------*/
    //#define ADXL345_ADDRESS                 (0x53)    // Assumes ALT address pin low
    #define ADXL345_ADDRESS                 (0x1D)    // Assumes ALT address pin low
/*=========================================================================*/

/*=========================================================================
    REGISTERS
    -----------------------------------------------------------------------*/
    #define ADXL345_REG_DEVID               (0x00)    // Device ID
    #define ADXL345_REG_THRESH_TAP          (0x1D)    // Tap threshold
    #define ADXL345_REG_OFSX                (0x1E)    // X-axis offset
    #define ADXL345_REG_OFSY                (0x1F)    // Y-axis offset
    #define ADXL345_REG_OFSZ                (0x20)    // Z-axis offset
    #define ADXL345_REG_DUR                 (0x21)    // Tap duration
    #define ADXL345_REG_LATENT              (0x22)    // Tap latency
    #define ADXL345_REG_WINDOW              (0x23)    // Tap window
    #define ADXL345_REG_THRESH_ACT          (0x24)    // Activity threshold
    #define ADXL345_REG_THRESH_INACT        (0x25)    // Inactivity threshold
    #define ADXL345_REG_TIME_INACT          (0x26)    // Inactivity time
    #define ADXL345_REG_ACT_INACT_CTL       (0x27)    // Axis enable control for activity and inactivity detection
    #define ADXL345_REG_THRESH_FF           (0x28)    // Free-fall threshold
    #define ADXL345_REG_TIME_FF             (0x29)    // Free-fall time
    #define ADXL345_REG_TAP_AXES            (0x2A)    // Axis control for single/double tap
    #define ADXL345_REG_ACT_TAP_STATUS      (0x2B)    // Source for single/double tap
    #define ADXL345_REG_BW_RATE             (0x2C)    // Data rate and power mode control
    #define ADXL345_REG_POWER_CTL           (0x2D)    // Power-saving features control
    #define ADXL345_REG_INT_ENABLE          (0x2E)    // Interrupt enable control
    #define ADXL345_REG_INT_MAP             (0x2F)    // Interrupt mapping control
    #define ADXL345_REG_INT_SOURCE          (0x30)    // Source of interrupts
    #define ADXL345_REG_DATA_FORMAT         (0x31)    // Data format control
    #define ADXL345_REG_DATAX0              (0x32)    // X-axis data 0
    #define ADXL345_REG_DATAX1              (0x33)    // X-axis data 1
    #define ADXL345_REG_DATAY0              (0x34)    // Y-axis data 0
    #define ADXL345_REG_DATAY1              (0x35)    // Y-axis data 1
    #define ADXL345_REG_DATAZ0              (0x36)    // Z-axis data 0
    #define ADXL345_REG_DATAZ1              (0x37)    // Z-axis data 1
    #define ADXL345_REG_FIFO_CTL            (0x38)    // FIFO control
    #define ADXL345_REG_FIFO_STATUS         (0x39)    // FIFO status
/*=========================================================================*/

/*=========================================================================
    REGISTERS
    -----------------------------------------------------------------------*/
    #define ADXL345_MG2G_MULTIPLIER (0.004)  // 4mg per lsb
/*=========================================================================*/

/* Used with register 0x2C (ADXL345_REG_BW_RATE) to set bandwidth */
typedef enum
{
  ADXL345_DATARATE_3200_HZ    = 0xf, // 0b1111, // 1600Hz Bandwidth   140�A IDD
  ADXL345_DATARATE_1600_HZ    = 0xe, // 0b1110, //  800Hz Bandwidth    90�A IDD
  ADXL345_DATARATE_800_HZ     = 0xd, // 0b1101, //  400Hz Bandwidth   140�A IDD
  ADXL345_DATARATE_400_HZ     = 0xc, // 0b1100, //  200Hz Bandwidth   140�A IDD
  ADXL345_DATARATE_200_HZ     = 0xb, // 0b1011, //  100Hz Bandwidth   140�A IDD
  ADXL345_DATARATE_100_HZ     = 0xa, // 0b1010, //   50Hz Bandwidth   140�A IDD
  ADXL345_DATARATE_50_HZ      = 0x9, // 0b1001, //   25Hz Bandwidth    90�A IDD
  ADXL345_DATARATE_25_HZ      = 0x8, // 0b1000, // 12.5Hz Bandwidth    60�A IDD
  ADXL345_DATARATE_12_5_HZ    = 0x7, // 0b0111, // 6.25Hz Bandwidth    50�A IDD
  ADXL345_DATARATE_6_25HZ     = 0x6, // 0b0110, // 3.13Hz Bandwidth    45�A IDD
  ADXL345_DATARATE_3_13_HZ    = 0x5, // 0b0101, // 1.56Hz Bandwidth    40�A IDD
  ADXL345_DATARATE_1_56_HZ    = 0x4, // 0b0100, // 0.78Hz Bandwidth    34�A IDD
  ADXL345_DATARATE_0_78_HZ    = 0x3, // 0b0011, // 0.39Hz Bandwidth    23�A IDD
  ADXL345_DATARATE_0_39_HZ    = 0x2, // 0b0010, // 0.20Hz Bandwidth    23�A IDD
  ADXL345_DATARATE_0_20_HZ    = 0x1, // 0b0001, // 0.10Hz Bandwidth    23�A IDD
  ADXL345_DATARATE_0_10_HZ    = 0x0  // 0b0000  // 0.05Hz Bandwidth    23�A IDD (default value)
} dataRate_t;

/* Used with register 0x31 (ADXL345_REG_DATA_FORMAT) to set g range */
typedef enum
{
  ADXL345_RANGE_16_G          = 0x3, // 0b11,   // +/- 16g
  ADXL345_RANGE_8_G           = 0x2, // 0b10,   // +/- 8g
  ADXL345_RANGE_4_G           = 0x1, // 0b01,   // +/- 4g
  ADXL345_RANGE_2_G           = 0x0  // 0b00    // +/- 2g (default value)
} range_t;

typedef struct {
  float x;
  float y;
  float z;
} tsFloatXYZ;

typedef struct {
  int16 x;
  int16 y;
  int16 z;
} tsInt16XYZ;

void vADXL345_WriteReg(uint8 reg, uint8 value);
uint8 u8ADXL345_ReadReg(uint8 reg);
int16 u16ADXL345_Read(uint8 reg);
void vADXL345_SetDataRate(bool low_power, dataRate_t dataRate);
void vADXL345_SetRange(range_t range);
void vADXL345_GetFloatXYZ(tsFloatXYZ *xyz);
void vADXL345_GetInt16XYZ(tsInt16XYZ *xyz);
void vADXL345_Sleep(void);
void vADXL345_Wake(void);

#endif  /* ADXL345_H_INCLUDED */
