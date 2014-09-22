/****************************************************************************
 * (C) TOCOS - 2012 all rights reserved
 ****************************************************************************/
#ifndef  CONFIG_H_INCLUDED
#define  CONFIG_H_INCLUDED

#if defined __cplusplus
extern "C" {
#endif

/****************************************************************************/
/***        Include Files                                                 ***/
/****************************************************************************/
#include <AppHardwareApi.h>

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/

/* Serial Configuration */
#define UART_BAUD   		115200
#define UART_PARITY_ENABLE	E_AHI_UART_PARITY_DISABLE
#define UART_PARITY_TYPE 	E_AHI_UART_ODD_PARITY // if enabled
#define UART_BITLEN		E_AHI_UART_WORD_LEN_8
#define UART_STOPBITS 		E_AHI_UART_1_STOP_BIT

/* Specify which serial port to use when outputting debug information */
#define UART_PORT_MASTER    E_AHI_UART_0 // for Coordinator
#define UART_PORT_SLAVE     E_AHI_UART_0 // for End Device

/* Specify the PAN ID and CHANNEL to be used by tags, readers and gateway */
#define APP_ID              0x67726306
#define CHANNEL             18

#define PKT_TYPE_BEACON     0
#define PKT_TYPE_ACT        1
#define PKT_TYPE_INACT      2

// Specify length of the cat data(include an alignment)
#define PKT_LEN_CAT         8

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/

typedef struct
{
  // Packet Type
  // D7  :tx source (0=Cat, 1=Station)
  // D0-6:packet type(0=beacon, 1=active, 2=inacvive)
  uint8 u8Type;
  // Acceleration values
  uint16 u16Dx,u16Dy,u16Dz;
  // Cat address
  // (only in tx form a station)
  uint32 u32CatAddr;
  // Signal strength of a cat
  // (only in tx form a station)
  uint8 u8CatLqi;
} tsPkt;

/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/

/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/

#if defined __cplusplus
}
#endif

#endif  /* CONFIG_H_INCLUDED */

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/
