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

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/
#define PORT_INPUT1 12
#define PORT_INPUT2 13
#define PORT_OUTPUT1 18
#define PORT_OUTPUT2 19

#define MAX_CATS 5

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/

typedef struct
{
  // MAC
  uint8 u8channel;
  uint16 u16addr;
  
  // LED Counter
  uint32 u32LedCt;

  // シーケンス番号
  uint32 u32Seq;

  uint32 u32WakeStat; // wake status
  uint8 u8NwkPanic;   // panic flag
} tsAppData;

typedef struct
{
  uint32 u32CatAddr;
  uint8 u8SeqPrev;
} tsAddrSeq;

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

static tsAddrSeq sAddrSeq[MAX_CATS];

PUBLIC tsFILE sSerStream;
tsSerialPortSetup sSerPort;

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

    // init memory
    memset(&sAddrSeq, 0x00, sizeof(tsAddrSeq) * MAX_CATS);

    ToCoNet_vMacStart();
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

    ToCoNet_vMacStart();
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
  static uint16 u16seqPrev = 0xFFFF;
  tsPkt sPkt;

  memcpy(&sPkt, pRx->auData, pRx->u8Len);
  // chaeck tx source
  if(sPkt.u8Type >= 0x80){
    return; // ignore if tx source is a Station.
  }

  // check the CatAddr table
  bool_t bSendPkt = FALSE;
  int i;
  for(i = 0;i < MAX_CATS;i ++){
    if(sAddrSeq[i].u32CatAddr == pRx->u32SrcAddr){
      if(sAddrSeq[i].u8SeqPrev != pRx->u8Seq){
	sAddrSeq[i].u8SeqPrev = pRx->u8Seq;
	bSendPkt = TRUE;
      }
      break;
    }
    if(sAddrSeq[i].u32CatAddr == 0 &&
       sAddrSeq[i].u8SeqPrev == 0){
      sAddrSeq[i].u32CatAddr = pRx->u32SrcAddr;
      sAddrSeq[i].u8SeqPrev = pRx->u8Seq;
      bSendPkt = TRUE;
      break;
    }
  }
  // CatAddr table is full,we will reset the system.
  if(i == MAX_CATS){
    vAHI_SwReset();
  }

  if(bSendPkt){
    // print coming payload
    vfPrintf(&sSerStream, "[PKT Ad:%04x,Ln:%03d,Seq:%03d,Lq:%03d,Tms:%05d,",
	     pRx->u32SrcAddr,
	     pRx->u8Len+4, // Actual payload byte: the network layer uses additional 4 bytes.
	     pRx->u8Seq,
	     pRx->u8Lqi,
	     pRx->u32Tick & 0xFFFF);
    vfPrintf(&sSerStream, "Typ:%02x,X:%03d,Y:%03d,Z:%03d]" LB,
	     sPkt.u8Type, sPkt.u16Dx, sPkt.u16Dy, sPkt.u16Dz);

    tsTxDataApp tsTx;
    memset(&tsTx, 0, sizeof(tsTxDataApp));

    tsTx.u32SrcAddr = ToCoNet_u32GetSerial();
    tsTx.u32DstAddr = 0xFFFF; // broadcast

    tsTx.bAckReq = FALSE;
    tsTx.u8Retry = 0x81;
    tsTx.u8CbId = pRx->u8Seq;
    tsTx.u8Seq = pRx->u8Seq;
    tsTx.u8Len = sizeof(tsPkt);
    tsTx.u8Cmd = TOCONET_PACKET_CMD_APP_DATA;

    tsTx.u16DelayMin = 64; // 最小遅延
    tsTx.u16DelayMax = 256; // 最大遅延

    sPkt.u8Type |= 0x80;               // tx source = Station
    sPkt.u32CatAddr = pRx->u32SrcAddr;
    sPkt.u8CatLqi = pRx->u8Lqi;
    memcpy(tsTx.auData, (void *)&sPkt, tsTx.u8Len);

    if(ToCoNet_bMacTxReq(&tsTx)){
      // turn on Led a while
      sAppData.u32LedCt = u32TickCount_ms;
      vfPrintf(&sSerStream, "Fire station packet to the base." LB);
    }else{
      vfPrintf(&sSerStream, "Send packet failed." LB);
    }
  }
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
#if 0
  // 送信完了
  if (i16TxCbId >= 0 && u8CbId == i16TxCbId) {
    // スリープを行う場合は、このイベントを持ってスリープ遷移
    ToCoNet_Event_Process(E_EVENT_APP_TX_COMPLETE, bStatus, (void*)vProcessEvCore);
  }
#endif
  return;
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
  switch (u32DeviceId) {
  case E_AHI_DEVICE_SYSCTRL:
#if 0
    if(u32ItemBitmap & (1UL << PORT_INPUT1)){
      vfPrintf(&sSerStream, "Int PORT1(INACT)" LB);
    }
    if(u32ItemBitmap & (1UL << PORT_INPUT2)){
      vfPrintf(&sSerStream, "Int PORT2(ACT)" LB);
    }
#endif
    break;
  case E_AHI_DEVICE_TICK_TIMER:
    // LED BLINK
    vPortSet_TrueAsLo(PORT_OUTPUT2, u32TickCount_ms & 0x400);

    // LED ON when receive
    if (u32TickCount_ms - sAppData.u32LedCt < 300) {
      vPortSetLo(PORT_OUTPUT1);
    } else {
      vPortSetHi(PORT_OUTPUT1);
    }

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

#if 0
  // dio interrupt
  // INACT
  vPortAsInput(PORT_INPUT1);
  vAHI_DioInterruptEnable((1UL << PORT_INPUT1), 0); // 割り込みの登録
  vAHI_DioInterruptEdge((1UL << PORT_INPUT1), 0); // 割り込みエッジの登録
  // ACT
  vPortAsInput(PORT_INPUT2);
  vAHI_DioInterruptEnable((1UL << PORT_INPUT2), 0); // 割り込みの登録
  vAHI_DioInterruptEdge((1UL << PORT_INPUT2), 0); // 割り込みエッジの登録
  //vAHI_DioWakeEdge((1UL << PORT_INPUT2), 0); // 割り込みエッジ（立上がりに設定）
#endif
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
  // handle UART command
  while (!SERIAL_bRxQueueEmpty(sSerPort.u8SerialPort)) {
    int16 i16Char;

    i16Char = SERIAL_i16RxChar(sSerPort.u8SerialPort);

    vfPrintf(&sSerStream, "\n\r# [%c] --> ", i16Char);
    SERIAL_vFlush(sSerStream.u8Device);

    switch(i16Char) {
    case '>': case '.':
      /* channel up */
      sAppData.u8channel++;
      if (sAppData.u8channel > 26) sAppData.u8channel = 11;
      sToCoNet_AppContext.u8Channel = sAppData.u8channel;
      ToCoNet_vRfConfig();
      vfPrintf(&sSerStream, "set channel to %d." LB, sAppData.u8channel);
      break;

    case '<': case ',':
      /* channel down */
      sAppData.u8channel--;
      if (sAppData.u8channel < 11) sAppData.u8channel = 26;
      sToCoNet_AppContext.u8Channel = sAppData.u8channel;
      ToCoNet_vRfConfig();
      vfPrintf(&sSerStream, "set channel to %d." LB, sAppData.u8channel);
      break;

    case 'd': case 'D':
      _C {
	static uint8 u8DgbLvl;

	u8DgbLvl++;
	if(u8DgbLvl > 5) u8DgbLvl = 0;
	ToCoNet_vDebugLevel(u8DgbLvl);

	vfPrintf(&sSerStream, "set NwkCode debug level to %d." LB, u8DgbLvl);
      }
      break;

    case 't': // パケット送信してみる
      _C {
      }
      break;

    default:
      break;
    }

    vfPrintf(&sSerStream, LB);
    SERIAL_vFlush(sSerStream.u8Device);
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

  switch (pEv->eState) {
  case E_STATE_IDLE:
    if (eEvent == E_EVENT_START_UP) {
      //vPortSetHi(PORT_OUTPUT1);
      // ここで UART のメッセージを出力すれば安全である。
      if (u32evarg & EVARG_START_UP_WAKEUP_RAMHOLD_MASK) {
	vfPrintf(&sSerStream, LB "RAMHOLD" LB);
      }
      if (u32evarg & EVARG_START_UP_WAKEUP_MASK) {
	vfPrintf(&sSerStream, "Wake up by ");
	if(sAppData.u32WakeStat == 0){ // TIMER
	  vfPrintf(&sSerStream, "TIMER,");
	}else{
	  if(sAppData.u32WakeStat & (1UL << PORT_INPUT1)){ // INACT
	    vfPrintf(&sSerStream, "INACT,");
	  }
	  if(sAppData.u32WakeStat & (1UL << PORT_INPUT2)){ // ACT
	    vfPrintf(&sSerStream, "ACT,");
	  }
	}
      } else {
	vfPrintf(&sSerStream, "*** App_Cat_Station %d.%02d-%d ***" LB, VERSION_MAIN, VERSION_SUB, VERSION_VAR);
	vfPrintf(&sSerStream, "*** %08x ***" LB, ToCoNet_u32GetSerial());
      }
      //ToCoNet_Event_SetState(pEv, E_STATE_APP_NW_BOOT);
    }
    break;
  default:
    break;
  }
  //vPortSetLo(PORT_OUTPUT1);
  //vPortSetLo(PORT_OUTPUT2);
}

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/
