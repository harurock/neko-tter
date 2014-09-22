/****************************************************************************
 * (C) Tokyo Cosmos Electric, Inc. (TOCOS) - 2013 all rights reserved.
 *
 * Condition to use:
 *   - The full or part of source code is limited to use for TWE (TOCOS
 *     Wireless Engine) as compiled and flash programmed.
 *   - The full or part of source code is prohibited to distribute without
 *     permission from TOCOS.
 *
 ****************************************************************************/

/****************************************************************************/
/***        Include files                                                 ***/
/****************************************************************************/
#include <string.h>

#include <jendefs.h>
#include <AppHardwareApi.h>

#include "utils.h"

#include "Main.h"
#include "config.h"
#include "Version.h"

#include "SMBus.h"
#include "ADXL345.h"

// DEBUG options

#include "serial.h"
#include "fprintf.h"
#include "sprintf.h"

/****************************************************************************/
/***        ToCoNet Definitions                                           ***/
/****************************************************************************/
// Select Modules (define befor include "ToCoNet.h")
//#define ToCoNet_USE_MOD_NBSCAN // Neighbour scan module
//#define ToCoNet_USE_MOD_NBSCAN_SLAVE

// includes
#include "ToCoNet.h"
#include "ToCoNet_mod_prototype.h"

#include "app_event.h"

//#define ENABLE_INT_ACT
//#define ADXL345_BYPASS_MODE
#define ADXL345_TORIGGER_MODE
#define ADXL345_TORIGGER_AND_STATE_MODE

#define PORT_INPUT1 12
#define PORT_INPUT2 13
#define PORT_OUTPUT1 18
#define PORT_OUTPUT2 19

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/

typedef struct
{
  // MAC
  uint8 u8channel;
  uint16 u16addr;
  
  // シーケンス番号
  uint32 u32Seq;

  // スリープカウンタ
  uint8 u8SleepCt;

  uint32 u32WakeStat; // wake status
  uint8 u8NwkPanic;   // panic flag
} tsAppData;

/****************************************************************************/
/***        Local Function Prototypes                                     ***/
/****************************************************************************/

static void vProcessEvCore(tsEvent *pEv, teEvent eEvent, uint32 u32evarg);

static void vInitHardware(int f_warm_start);

void vSerialInit(uint32 u32Baud, tsUartOpt *pUartOpt);
static void vHandleSerialInput(void);

/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/

/****************************************************************************/
/***        Local Variables                                               ***/
/****************************************************************************/
/* Version/build information. This is not used in the application unless we
   are in serial debug mode. However the 'used' attribute ensures it is
   present in all binary files, allowing easy identifaction... */

/* Local data used by the tag during operation */
static tsAppData sAppData;
static tsPkt sPkt;

PUBLIC tsFILE sSerStream;
tsSerialPortSetup sSerPort;

static uint8 adxl345_deviceid = 0;
static uint8 adxl345_inact = 0;

static int16 i16TxCbId = 0;

/****************************************************************************
 *
 * NAME: AppColdStart
 *
 * DESCRIPTION:
 *
 * RETURNS:
 *
 ****************************************************************************/
void cbAppColdStart(bool_t bAfterAhiInit)
{
  //static uint8 u8WkState;
  if (!bAfterAhiInit) {
    // before AHI init, very first of code.

    // Register modules
    ToCoNet_REG_MOD_ALL();
    
  } else {
    // disable brown out detect
    vAHI_BrownOutConfigure(0,//0:2.0V 1:2.3V
			   FALSE,
			   FALSE,
			   FALSE,
			   FALSE);
    
    // clear application context
    memset (&sAppData, 0x00, sizeof(sAppData));
    sAppData.u8channel = CHANNEL;

    // ToCoNet configuration
    sToCoNet_AppContext.u32AppId = APP_ID;
    sToCoNet_AppContext.u8Channel = CHANNEL;

    sToCoNet_AppContext.bRxOnIdle = TRUE;

    // others
    SPRINTF_vInit128();

    // Register
    ToCoNet_Event_Register_State_Machine(vProcessEvCore);

    // Others
    vInitHardware(FALSE);

    // do not start network here
  }
}

/****************************************************************************
 *
 * NAME: AppWarmStart
 *
 * DESCRIPTION:
 *
 * RETURNS:
 *
 ****************************************************************************/

void cbAppWarmStart(bool_t bAfterAhiInit)
{
  if (!bAfterAhiInit) {
    // before AHI init, very first of code.

    if(u8AHI_WakeTimerFiredStatus()) {
      // wake up timer
      sAppData.u32WakeStat = 0;
    } else {
      sAppData.u32WakeStat = u32AHI_DioWakeStatus();
    }
  } else {
    // Initialize hardware
    vInitHardware(TRUE);

    // do not start network here
  }
}

/****************************************************************************/
/***        Local Functions                                               ***/
/****************************************************************************/
/****************************************************************************
 *
 * NAME: vMain
 *
 * DESCRIPTION:
 *
 * RETURNS:
 *
 ****************************************************************************/
void cbToCoNet_vMain(void)
{
  /* handle uart input */
  vHandleSerialInput();
}

/****************************************************************************
 *
 * NAME: cbToCoNet_vNwkEvent
 *
 * DESCRIPTION:
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 *
 * NOTES:
 ****************************************************************************/
void cbToCoNet_vNwkEvent(teEvent eEvent, uint32 u32arg) {
  switch(eEvent) {
  case E_EVENT_TOCONET_PANIC: // have a problem in MAC layer, we should reset
    sAppData.u8NwkPanic = 1;
    vfPrintf(&sSerStream, "E_EVENT_TOCONET_PANIC" LB);
    break;
  default:
    break;
  }
}

/****************************************************************************
 *
 * NAME: cbvMcRxHandler
 *
 * DESCRIPTION:
 *
 * RETURNS:
 *
 ****************************************************************************/
void cbToCoNet_vRxEvent(tsRxDataApp *pRx) {
  // nothing to do
}

/****************************************************************************
 *
 * NAME: cbvMcEvTxHandler
 *
 * DESCRIPTION:
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 *
 * NOTES:
 ****************************************************************************/
void cbToCoNet_vTxEvent(uint8 u8CbId, uint8 bStatus) {
  // 送信完了
  if (i16TxCbId >= 0 && u8CbId == i16TxCbId) {
    // スリープを行う場合は、このイベントを持ってスリープ遷移
    ToCoNet_Event_Process(E_EVENT_APP_TX_COMPLETE, bStatus, (void*)vProcessEvCore);
  }
  return;
}

static void vReadFifo(int n)
{
  int i;
  tsInt16XYZ xyz;
  
  int16 xmin, xmax;
  int16 ymin, ymax;
  int16 zmin, zmax;
  xmin = ymin = zmin = 32767;
  xmax = ymax = zmax = -32768;

  uint32 s = u32TickCount_ms;
  for(i = 0;i < n; i++){
    vADXL345_GetInt16XYZ(&xyz);
    if(xmax < xyz.x) xmax = xyz.x;
    if(xmin > xyz.x) xmin = xyz.x;
    if(ymax < xyz.y) ymax = xyz.y;
    if(ymin > xyz.y) ymin = xyz.y;
    if(zmax < xyz.z) zmax = xyz.z;
    if(zmin > xyz.z) zmin = xyz.z;
#if 0
    vfPrintf(&sSerStream, "%d X: %d, Y: %d, Z: %d" LB, 
	     //	     i, (int16)(xyz.x * 1000), (int16)(xyz.y * 1000), (int16)(xyz.z * 1000));
	     i,  xyz.x, xyz.y, xyz.z);
#endif
  }
  sPkt.u16Dx = xmax - xmin;
  sPkt.u16Dy = ymax - ymin;
  sPkt.u16Dz = zmax - zmin;

  vfPrintf(&sSerStream, "%d %d %d" LB, sPkt.u16Dx, sPkt.u16Dy, sPkt.u16Dz);
  vfPrintf(&sSerStream, "adxl345_read_fifo %d" LB, u32TickCount_ms - s);
}

static bool_t bWaterMark(void)
{
  uint8 fifo_stat = u8ADXL345_ReadReg(ADXL345_REG_FIFO_STATUS);
  //vfPrintf(&sSerStream, "fifo_stat=%x" LB, fifo_stat);
  if(fifo_stat & 0x80){ // torigger event occur?
    if(u8ADXL345_ReadReg(ADXL345_REG_INT_SOURCE) & 0x02){ // is Watermark?
      vReadFifo(fifo_stat & 0x7f); // number of entries
      // clear trigger
      // FIFO mode: Bypass(0x00) | INT2(0x20) | sample = 31
      vADXL345_WriteReg(ADXL345_REG_FIFO_CTL,0x00 | 0x20| 31);
      // FIFO mode: Trriger(0xC0) | INT2(0x20) | sample = 31
      vADXL345_WriteReg(ADXL345_REG_FIFO_CTL,0xC0 | 0x20| 31);
      return TRUE;
    }else{
      vADXL345_Wake();
    }
  }
  return FALSE;
}

/****************************************************************************
 *
 * NAME: cbToCoNet_vHwEvent
 *
 * DESCRIPTION:
 * Process any hardware events.
 *
 * PARAMETERS:      Name            RW  Usage
 *                  u32DeviceId
 *                  u32ItemBitmap
 *
 * RETURNS:
 * None.
 *
 * NOTES:
 * None.
 ****************************************************************************/
void cbToCoNet_vHwEvent(uint32 u32DeviceId, uint32 u32ItemBitmap)
{
#ifdef ADXL345_BYPASS_MODE
  tsFloatXYZ xyz;
#endif

  switch (u32DeviceId) {
  case E_AHI_DEVICE_SYSCTRL:
    if(u32ItemBitmap & (1UL << PORT_INPUT1)){
      adxl345_inact = 1;
      vfPrintf(&sSerStream, "Int PORT1(INACT)" LB);
    }
    if(u32ItemBitmap & (1UL << PORT_INPUT2)){
      vfPrintf(&sSerStream, "Int PORT2(ACT)" LB);
    }
    break;
  case E_AHI_DEVICE_TICK_TIMER:
#ifndef ADXL345_TORIGGER_AND_STATE_MODE
#ifdef ADXL345_TORIGGER_MODE
    uint8 int_src = u8ADXL345_ReadReg(ADXL345_REG_INT_SOURCE);
    //vfPrintf(&sSerStream, "INT_SRC %x" LB, int_src);
    
    uint8 fifo_stat = u8ADXL345_ReadReg(ADXL345_REG_FIFO_STATUS);
    if(fifo_stat & 0x80){ // torigger event occur?
      //vfPrintf(&sSerStream, "FIFO_TRIG:" LB);
      if(int_src & 0x02){ // is Watermark?
	adxl345_read_fifo(fifo_stat & 0x7f); // number of entries
	// clear trigger
	// FIFO mode: Bypass(0x00) | INT2(0x20) | sample = 31
	vADXL345_WriteReg(ADXL345_REG_FIFO_CTL,0x00 | 0x20| 31);
	// FIFO mode: Trriger(0xC0) | INT2(0x20) | sample = 31
	vADXL345_WriteReg(ADXL345_REG_FIFO_CTL,0xC0 | 0x20| 31);
      }else{
	vADXL345_Wake();
      }
    }

    if(adxl345_inact){
      vfPrintf(&sSerStream, "ADXL345 going to sleep" LB);
      adxl345_inact = 0;
      vADXL345_Sleep();
    }
#endif

#ifdef ADXL345_BYPASS_MODE
    if(u32TickCount_ms % 1000 == 0) {
      vADXL345_GetFloatXYZ(&xyz);
      vfPrintf(&sSerStream, "X: %d" LB, (int16)(xyz.x * 1000));
      vfPrintf(&sSerStream, "Y: %d" LB, (int16)(xyz.y * 1000));
      vfPrintf(&sSerStream, "Z: %d" LB, (int16)(xyz.z * 1000));
    }
#endif

#endif // #ifndef ADXL345_TORIGGER_AND_STATE_MODE

    break;
  default:
    break;
  }
}

/****************************************************************************
 *
 * NAME: cbToCoNet_u8HwInt
 *
 * DESCRIPTION:
 *   called during an interrupt
 *
 * PARAMETERS:      Name            RW  Usage
 *                  u32DeviceId
 *                  u32ItemBitmap
 *
 * RETURNS:
 *                  FALSE -  interrupt is not handled, escalated to further
 *                           event call (cbToCoNet_vHwEvent).
 *                  TRUE  -  interrupt is handled, no further call.
 *
 * NOTES:
 *   Do not put a big job here.
 ****************************************************************************/
uint8 cbToCoNet_u8HwInt(uint32 u32DeviceId, uint32 u32ItemBitmap) {
  return FALSE;
}

static bool vInit345(void)
{
  /* Check connection */
  adxl345_deviceid = u8ADXL345_ReadReg(ADXL345_REG_DEVID);
  if (adxl345_deviceid != 0xE5) {
    /* No ADXL345 detected ... return false */
    //Serial.println(deviceid, HEX);
    return FALSE;
  }
  vADXL345_SetRange(ADXL345_RANGE_16_G);
  vADXL345_SetDataRate(TRUE, ADXL345_DATARATE_25_HZ);
  
#ifdef ADXL345_TORIGGER_MODE
  // 5 * 62.5 mg/LSB
  vADXL345_WriteReg(ADXL345_REG_THRESH_INACT, 5);
  // 10 * 1sec/LSB
  vADXL345_WriteReg(ADXL345_REG_TIME_INACT, 10);
  
  // 8 * 62.5 mg/LSB
  vADXL345_WriteReg(ADXL345_REG_THRESH_ACT, 0x08);
  // ACT ac | ACT_X | ACT_Y |ACT_Z
  // INACT ac | INACT_X | INACT_Y | INACT_Z
  vADXL345_WriteReg(ADXL345_REG_ACT_INACT_CTL, 0xFF);
  
  // Activity->INT2
  // Inactivity->INT1
  vADXL345_WriteReg(ADXL345_REG_INT_MAP, 0x10);
  // D4:Activity(0x10) | D3:Inactivity(0x08)
  vADXL345_WriteReg(ADXL345_REG_INT_ENABLE, 0x18);
  
  // FIFO mode: Trriger(0xC0) | INT2(0x20) | sample = 31
  vADXL345_WriteReg(ADXL345_REG_FIFO_CTL,0xC0 | 0x20| 31);
  
  // clear event
  u8ADXL345_ReadReg(ADXL345_REG_INT_SOURCE);
  // Enable measurements
  // D3:Measure(0x08) | Wakeup 8Hz (0x00)
  vADXL345_WriteReg(ADXL345_REG_POWER_CTL, 0x08);
#endif

#ifdef ADXL345_BYPASS_MODE
  // Enable measurements
  vADXL345_WriteReg(ADXL345_REG_POWER_CTL, 0x08);
#endif
  return TRUE;
}

/****************************************************************************
 *
 * NAME: vInitHardware
 *
 * DESCRIPTION:
 *
 * RETURNS:
 *
 ****************************************************************************/
static void vInitHardware(int f_warm_start)
{
  // Serial Initialize
#if 0
  // UART の細かい設定テスト
  tsUartOpt sUartOpt;
  memset(&sUartOpt, 0, sizeof(tsUartOpt));
  sUartOpt.bHwFlowEnabled = FALSE;
  sUartOpt.bParityEnabled = E_AHI_UART_PARITY_ENABLE;
  sUartOpt.u8ParityType = E_AHI_UART_EVEN_PARITY;
  sUartOpt.u8StopBit = E_AHI_UART_2_STOP_BITS;
  sUartOpt.u8WordLen = 7;

  vSerialInit(UART_BAUD, &sUartOpt);
#else
  vSerialInit(UART_BAUD, NULL);
#endif

  ToCoNet_vDebugInit(&sSerStream);
  ToCoNet_vDebugLevel(0);

  /// IOs
  vPortSetLo(PORT_OUTPUT1);
  vPortSetLo(PORT_OUTPUT2);
  vPortAsOutput(PORT_OUTPUT1);
  vPortAsOutput(PORT_OUTPUT2);

  // dio interrupt
  // INACT
  vPortAsInput(PORT_INPUT1);
  vAHI_DioInterruptEnable((1UL << PORT_INPUT1), 0); // 割り込みの登録
  vAHI_DioInterruptEdge((1UL << PORT_INPUT1), 0); // 割り込みエッジの登録
  // ACT
  vPortAsInput(PORT_INPUT2);
#ifdef ENABLE_INT_ACT
  vAHI_DioInterruptEnable((1UL << PORT_INPUT2), 0); // 割り込みの登録
  vAHI_DioInterruptEdge((1UL << PORT_INPUT2), 0); // 割り込みエッジの登録
  //vAHI_DioWakeEdge((1UL << PORT_INPUT2), 0); // 割り込みエッジ（立上がりに設定）
#endif

  // SMBUS
  vSMBusInit();

  // ADXL345
  if(!f_warm_start) vInit345();
}

/****************************************************************************
 *
 * NAME: vInitHardware
 *
 * DESCRIPTION:
 *
 * RETURNS:
 *
 ****************************************************************************/
void vSerialInit(uint32 u32Baud, tsUartOpt *pUartOpt) {
  /* Create the debug port transmit and receive queues */
  static uint8 au8SerialTxBuffer[864];
  static uint8 au8SerialRxBuffer[128];

  /* Initialise the serial port to be used for debug output */
  sSerPort.pu8SerialRxQueueBuffer = au8SerialRxBuffer;
  sSerPort.pu8SerialTxQueueBuffer = au8SerialTxBuffer;
  sSerPort.u32BaudRate = u32Baud;
  sSerPort.u16AHI_UART_RTS_LOW = 0xffff;
  sSerPort.u16AHI_UART_RTS_HIGH = 0xffff;
  sSerPort.u16SerialRxQueueSize = sizeof(au8SerialRxBuffer);
  sSerPort.u16SerialTxQueueSize = sizeof(au8SerialTxBuffer);
  sSerPort.u8SerialPort = UART_PORT_SLAVE;
  sSerPort.u8RX_FIFO_LEVEL = E_AHI_UART_FIFO_LEVEL_1;
  SERIAL_vInitEx(&sSerPort, pUartOpt);

  sSerStream.bPutChar = SERIAL_bTxChar;
  sSerStream.u8Device = UART_PORT_SLAVE;
}

/****************************************************************************
 *
 * NAME: vHandleSerialInput
 *
 * DESCRIPTION:
 *
 * PARAMETERS:      Name            RW  Usage
 *
 * RETURNS:
 *
 * NOTES:
 ****************************************************************************/
static void vHandleSerialInput(void)
{
  // nothing to do
}

static int16 i16TxMsg(uint8 u8Type) {
  tsTxDataApp tsTx;
  memset(&tsTx, 0, sizeof(tsTxDataApp));
  uint8 *q = tsTx.auData;

  sAppData.u32Seq++;

  tsTx.u32SrcAddr = ToCoNet_u32GetSerial(); // 自身のアドレス
  tsTx.u32DstAddr = 0xFFFF; // ブロードキャスト

  tsTx.bAckReq = FALSE;
  tsTx.u8Retry = 0x82; // ブロードキャストで都合３回送る
  tsTx.u8CbId = sAppData.u32Seq & 0xFF;
  tsTx.u8Seq = sAppData.u32Seq & 0xFF;
  tsTx.u8Cmd = TOCONET_PACKET_CMD_APP_DATA;

  sPkt.u8Type = u8Type;
  if(u8Type != PKT_TYPE_ACT) {
    sPkt.u16Dx = sPkt.u16Dy = sPkt.u16Dz = 0; 
  }
  memcpy(q, (void *)&sPkt, PKT_LEN_CAT);
  q += PKT_LEN_CAT;

  tsTx.u8Len = q - tsTx.auData;
  // 送信
  if (ToCoNet_bMacTxReq(&tsTx)) {
    vfPrintf(&sSerStream, "Fire Tx Msg." LB);

    return tsTx.u8CbId;
  } else {
    return -1;
  }
}

/****************************************************************************
 *
 * NAME: vProcessEvent
 *
 * DESCRIPTION:
 *
 * RETURNS:
 *
 ****************************************************************************/
static void vProcessEvCore(tsEvent *pEv, teEvent eEvent, uint32 u32evarg) {
  static uint32 t_new_stat = 0;

  switch (pEv->eState) {
  case E_STATE_IDLE:
    if (eEvent == E_EVENT_START_UP) {
      vPortSetHi(PORT_OUTPUT1);
      // ここで UART のメッセージを出力すれば安全である。
      if (u32evarg & EVARG_START_UP_WAKEUP_RAMHOLD_MASK) {
	vfPrintf(&sSerStream, LB "RAMHOLD" LB);
      }
      if (u32evarg & EVARG_START_UP_WAKEUP_MASK) {
	vfPrintf(&sSerStream, "Wake up by ");
	if(sAppData.u32WakeStat == 0){ // TIMER
	  vfPrintf(&sSerStream, "TIMER");
	}else{
	  if(sAppData.u32WakeStat & (1UL << PORT_INPUT1)){ // INACT
	    vfPrintf(&sSerStream, "INACT");
	  }
	  if(sAppData.u32WakeStat & (1UL << PORT_INPUT2)){ // ACT
	    vfPrintf(&sSerStream, "ACT");
	  }
	}
	vfPrintf(&sSerStream, LB);
      } else {
	vfPrintf(&sSerStream, "*** App_Cat %d.%02d-%d ***" LB, VERSION_MAIN, VERSION_SUB, VERSION_VAR);
	vfPrintf(&sSerStream, "*** %08x ***" LB, ToCoNet_u32GetSerial());
      }
      ToCoNet_Event_SetState(pEv, E_STATE_APP_NW_BOOT);
    }
    break;
#ifdef ADXL345_TORIGGER_AND_STATE_MODE
  case E_STATE_APP_NW_BOOT:
    if (eEvent == E_EVENT_NEW_STATE){
      vPortSetHi(PORT_OUTPUT1);
      vfPrintf(&sSerStream, "E_STATE_APP_NW_BOOT %d" LB, u32TickCount_ms & 0xFFFF);
      sAppData.u8NwkPanic = 0;
      ToCoNet_vMacStart();
    }else{
      vPortSetHi(PORT_OUTPUT2);
      if(sAppData.u8NwkPanic){
	// if we receive E_EVENT_TOCONET_PANIC, go to E_STATE_APP_SLEEP
	ToCoNet_Event_SetState(pEv, E_STATE_APP_SLEEP);
      }else{
	ToCoNet_Event_SetState(pEv, E_STATE_APP_RUNNING);
      }
    }
    break;
  case E_STATE_APP_RUNNING:
    if (eEvent == E_EVENT_NEW_STATE){
      t_new_stat = u32TickCount_ms; // for timeout
      vPortSetHi(PORT_OUTPUT1);
      vfPrintf(&sSerStream, "E_STATE_APP_RUNNING %d" LB, u32TickCount_ms & 0xFFFF);
      if(sAppData.u32WakeStat & (1UL << PORT_INPUT1)){ // INACT
	vADXL345_Sleep();
	// TX sleep beaconi
	i16TxCbId = i16TxMsg(PKT_TYPE_INACT);

	ToCoNet_Event_SetState(pEv, E_STATE_APP_WAIT_TX);
      }else if(sAppData.u32WakeStat & (1UL << PORT_INPUT2)){ // ACT
	if(bWaterMark()){
	  // TX data
	  i16TxCbId = i16TxMsg(PKT_TYPE_ACT);
	  ToCoNet_Event_SetState(pEv, E_STATE_APP_WAIT_TX);
	}else{
	  // wait Watermark
	}
      }else{
	if(bWaterMark()){
	  // TX data
	  i16TxCbId = i16TxMsg(PKT_TYPE_ACT);
	  ToCoNet_Event_SetState(pEv, E_STATE_APP_WAIT_TX);
	}else{
	  // TX only beacon
	  i16TxCbId = i16TxMsg(PKT_TYPE_BEACON);
	  ToCoNet_Event_SetState(pEv, E_STATE_APP_WAIT_TX);
	}
      }
    }else{
      vPortSetHi(PORT_OUTPUT2);
      if(bWaterMark()){
	// TX data
	i16TxCbId = i16TxMsg(PKT_TYPE_ACT);
	ToCoNet_Event_SetState(pEv, E_STATE_APP_WAIT_TX);
      }else{
	// timeout 100ms
	if(u32TickCount_ms - t_new_stat > 100){
	  vfPrintf(&sSerStream, "timeout" LB);
	  // go to E_STATE_APP_SLEEP
	  ToCoNet_Event_SetState(pEv, E_STATE_APP_SLEEP);
	}
      }
    }
    break;
  case E_STATE_APP_WAIT_TX:
    if (eEvent == E_EVENT_NEW_STATE){
      t_new_stat = u32TickCount_ms;
      vPortSetHi(PORT_OUTPUT1);
      vfPrintf(&sSerStream, "E_STATE_APP_WAIT_TX %d" LB, u32TickCount_ms & 0xFFFF);
    }else{
      if(eEvent == E_EVENT_APP_TX_COMPLETE ||
	 u32TickCount_ms - t_new_stat > 100){
	vPortSetHi(PORT_OUTPUT2);
	ToCoNet_Event_SetState(pEv, E_STATE_APP_SLEEP);
      }
    }
    break;
  case E_STATE_APP_SLEEP:
    if (eEvent == E_EVENT_NEW_STATE){
      t_new_stat = u32TickCount_ms;
      vPortSetHi(PORT_OUTPUT1);
      vfPrintf(&sSerStream, "E_STATE_APP_SLEEP %d" LB, u32TickCount_ms & 0xFFFF);
    }else{
      vPortSetHi(PORT_OUTPUT2);
      if(u32TickCount_ms - t_new_stat > 10){ // wait for serial debug write
	// x000ms, TIMER:ON, RAM:ON
	ToCoNet_vSleep(E_AHI_WAKE_TIMER_0, 3000, TRUE, FALSE);
      }
    }
    break;
#endif
  default:
    break;
  }
  vPortSetLo(PORT_OUTPUT1);
  vPortSetLo(PORT_OUTPUT2);
}

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/
