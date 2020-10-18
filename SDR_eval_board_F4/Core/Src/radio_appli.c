/**
******************************************************************************
* @file    radio_appli.c
* @author  CLAB
* @version V1.0.0
* @date    03-March-2017
* @brief   user file to configure S2LP transceiver.
*         
@verbatim
===============================================================================
##### How to use this driver #####
===============================================================================
[..]
This file is generated automatically by STM32CubeMX and eventually modified 
by the user

@endverbatim
******************************************************************************
* @attention
*
* <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
*   1. Redistributions of source code must retain the above copyright notice,
*      this list of conditions and the following disclaimer.
*   2. Redistributions in binary form must reproduce the above copyright notice,
*      this list of conditions and the following disclaimer in the documentation
*      and/or other materials provided with the distribution.
*   3. Neither the name of STMicroelectronics nor the names of its contributors
*      may be used to endorse or promote products derived from this software
*      without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
******************************************************************************
*/ 

/* Includes ------------------------------------------------------------------*/

#include "main.h"

#include "radio_appli.h"
#include "cube_hal.h"
#include "MCU_Interface.h" 
#include "radio_gpio.h" 
#include "fsm.h"

#if defined(X_NUCLEO_IDS01A4) || defined(X_NUCLEO_IDS01A5)
#include "SPIRIT1_Util.h"
#endif


/*User interface includes*/
#include <stdio.h>
#include <inttypes.h>
#include "retarget.h"

/** @addtogroup USER
* @{
*/

/** @defgroup S2LP_APPLI
* @brief User file to configure S2LP transceiver for desired frequency and 
* @feature.
* @{
*/

/* Private typedef -----------------------------------------------------------*/

/**
* @brief RadioDriver_t structure fitting
*/

RadioDriver_t radio_cb =
{
  .Init = Spirit1InterfaceInit,
  .GpioIrq = Spirit1GpioIrqInit,
  .RadioInit = Spirit1RadioInit,
  .SetRadioPower = Spirit1SetPower,
  .PacketConfig = Spirit1PacketConfig,
  .SetPayloadLen = Spirit1SetPayloadlength,
  .SetDestinationAddress = Spirit1SetDestinationAddress,
  .EnableTxIrq = Spirit1EnableTxIrq,
  .EnableRxIrq = Spirit1EnableRxIrq,
  .DisableIrq = Spirit1DisableIrq,
  .SetRxTimeout = Spirit1SetRxTimeout,
  .EnableSQI = Spirit1EnableSQI,
  .SetRssiThreshold = Spirit1SetRssiTH,
  .ClearIrqStatus = Spirit1ClearIRQ,
  .StartRx = Spirit1StartRx,
  .StartTx = Spirit1StartTx,
  .GetRxPacket = Spirit1GetRxPacket
};





/**
* @brief MCULowPowerMode_t structure fitting
*/
MCULowPowerMode_t MCU_LPM_cb =
{
  .McuStopMode = MCU_Enter_StopMode,
  .McuStandbyMode = MCU_Enter_StandbyMode,
  .McuSleepMode = MCU_Enter_SleepMode
}; 

/**
* @brief RadioLowPowerMode_t structure fitting
*/
RadioLowPowerMode_t Radio_LPM_cb =
{
  .RadioShutDown = RadioPowerOFF,
  .RadioStandBy = RadioStandBy,
  .RadioSleep = RadioSleep,
  .RadioPowerON = RadioPowerON
};

/**
* @brief GPIO structure fitting
*/
SGpioInit xGpioIRQ={
  SPIRIT_GPIO_IRQ, /*SPIRIT_GPIO_3*/
  SPIRIT_GPIO_MODE_DIGITAL_OUTPUT_LP,
  SPIRIT_GPIO_DIG_OUT_IRQ
};

/**
* @brief Radio structure fitting
*/
SRadioInit xRadioInit = {
  XTAL_OFFSET_PPM,
  BASE_FREQUENCY,
  CHANNEL_SPACE,
  CHANNEL_NUMBER,
  MODULATION_SELECT,
  DATARATE,
  FREQ_DEVIATION,
  BANDWIDTH
};


#if defined(USE_STack_PROTOCOL)
/**
* @brief Packet Basic structure fitting
*/
PktStackInit xStackInit={
  PREAMBLE_LENGTH,
  SYNC_LENGTH,
  SYNC_WORD,
  LENGTH_TYPE,
  LENGTH_WIDTH,
  CRC_MODE,
  CONTROL_LENGTH,
  EN_FEC,
  EN_WHITENING
};

/* LLP structure fitting */
PktStackLlpInit xStackLLPInit ={
  EN_AUTOACK,
  EN_PIGGYBACKING,
  MAX_RETRANSMISSIONS
};

/**
* @brief Address structure fitting
*/
PktStackAddressesInit xAddressInit={
  EN_FILT_MY_ADDRESS,
  MY_ADDRESS,
  EN_FILT_MULTICAST_ADDRESS,
  MULTICAST_ADDRESS,
  EN_FILT_BROADCAST_ADDRESS,
  BROADCAST_ADDRESS
};

#elif defined(USE_BASIC_PROTOCOL)

/**
* @brief Packet Basic structure fitting
*/

PktBasicInit xBasicInit={
  
  PREAMBLE_LENGTH,
  SYNC_LENGTH,
  SYNC_WORD,
#if defined(X_NUCLEO_IDS01A4) || defined(X_NUCLEO_IDS01A5)
  LENGTH_TYPE,
  LENGTH_WIDTH,
#endif
  CRC_MODE,
#if defined(X_NUCLEO_IDS01A4) || defined(X_NUCLEO_IDS01A5)
  CONTROL_LENGTH,
#endif
  EN_ADDRESS,
  EN_FEC,
  EN_WHITENING
};

/**
* @brief Address structure fitting
*/
PktBasicAddressesInit xAddressInit={
  EN_FILT_MY_ADDRESS,
  MY_ADDRESS,
  EN_FILT_MULTICAST_ADDRESS,
  MULTICAST_ADDRESS,
  EN_FILT_BROADCAST_ADDRESS,
  BROADCAST_ADDRESS
};

#endif

#ifdef CSMA_ENABLE
/**
* @brief CSMA structure fitting
*/
CsmaInit xCsmaInit={
  
  PERSISTENT_MODE_EN, /* CCA may optionally be persistent, i.e., rather than 
  entering backoff when the channel is found busy, CCA continues until the 
  channel becomes idle or until the MCU stops it.
  The thinking behind using this option is to give the MCU the possibility of 
  managing the CCA by itself, for instance, with the allocation of a 
  transmission timer: this timer would start when MCU finishes sending out 
  data to be transmitted, and would end when MCU expects that its transmission
  takes place, which would occur after a period of CCA.
  The choice of making CCA persistent should come from trading off 
  transmission latency,under the direct control of the MCU, and power 
  consumption, which would be greater due to a busy wait in reception mode.
  */
  CS_PERIOD,
  CS_TIMEOUT,
  MAX_NB, /*Retries*/
  BU_COUNTER_SEED,
  CU_PRESCALER
};
#endif 

/* Private define ------------------------------------------------------------*/
#define TIME_UP                                         0x01
#define TX_BUFFER_SIZE   20
#define RX_BUFFER_SIZE   96

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
extern  UART_HandleTypeDef huart2;


uint8_t TxLength = TX_BUFFER_SIZE;
uint8_t RxLength = 0;
uint8_t aTransmitBuffer[TX_BUFFER_SIZE] = {0x00};
uint8_t aReceiveBuffer[RX_BUFFER_SIZE] = {0x00};

int a[20] = {0};

RadioDriver_t *pRadioDriver;
MCULowPowerMode_t *pMCU_LPM_Comm;
RadioLowPowerMode_t  *pRadio_LPM_Comm;
/*Flags declarations*/
volatile FlagStatus xRxDoneFlag = RESET, xTxDoneFlag=RESET, cmdFlag=RESET;
volatile FlagStatus xStartRx=RESET, Spirit1_RX_timeout=RESET, SW_RX_timeout=RESET;
volatile FlagStatus PushButtonStatusWakeup=RESET;
volatile FlagStatus PushButtonStatusData=RESET;
FlagStatus ACK_Process, IDLE_Process, tx_value = RESET;
/*IRQ status struct declaration*/
SpiritIrqs xIrqStatus;

__IO uint32_t KEYStatusData = 0x00;
static AppliFrame_t xTxFrame, xRxFrame;
#if defined(X_NUCLEO_IDS01A4) || defined(X_NUCLEO_IDS01A5)
uint8_t DestinationAddr;
#endif
uint8_t TxFrameBuff[MAX_BUFFER_LEN] = {0x00};
uint16_t exitCounter = 0;
uint16_t txCounter = 0;
uint16_t wakeupCounter = 0;
uint16_t dataSendCounter = 0x00;

uint8_t temp_DataBuff[]={0x00};
uint8_t  dest_addr;

fsm_t* radio_fsm;

radio_select_t selectedBand;

/* Private function prototypes -----------------------------------------------*/


static int time_out_rx(fsm_t* this)
{

	return ((Spirit1_RX_timeout)||(check_Rx_count()));
}

static int tx_flag(fsm_t* this)
{
	return tx_value;
}

static int rx_flag(fsm_t* this)
{
	return xRxDoneFlag;
}


static int data_received(fsm_t* this)
{
	if(xRxFrame.Cmd == MSG_CMD) return 1;
	else return 0;
}

static int ack_received(fsm_t* this)
{
    if(xRxFrame.Cmd == ACK_OK) return 1;
    else return 0;
}

static int multicast(fsm_t* this)
{

	if ((dest_addr == MULTICAST_ADDRESS) || (dest_addr == BROADCAST_ADDRESS)) return 1;
	else return 0;

}

static int address_known(fsm_t* this)
{
	if ((dest_addr != MULTICAST_ADDRESS) && (dest_addr != BROADCAST_ADDRESS)) return 1;
	else return 0;

}

static int tx_done(fsm_t* this)
{
	return xTxDoneFlag;
}


static int ACK_confirm (fsm_t* this)
{
	return ACK_Process;
}

void EN_Rx(fsm_t* this)
{
    AppliReceiveBuff(aReceiveBuffer, RxLength);
    Spirit1_RX_timeout = RESET;
    reset_RX_count();
    BSP_LED_Toggle(LED2);

	xTxDoneFlag = RESET;
#if defined(X_NUCLEO_IDS01A4) || defined(X_NUCLEO_IDS01A5)
    RadioShieldLedOff(RADIO_SHIELD_LED);
#endif
    ACK_Process = RESET;
    IDLE_Process = SET;

    printf("\r\nDispositivo SDR preparado\r\n");
    printf("\r\nPara enviar un mensaje pulse cualquier tecla\r\n");
}

void send_data(fsm_t* this)
{
	tx_value = RESET;
	xTxFrame.Cmd = MSG_CMD;
	xTxFrame.CmdLen = 0x01;
	xTxFrame.Cmdtag = txCounter++;
	xTxFrame.CmdType = APPLI_CMD;
	xTxFrame.DataBuff = aTransmitBuffer;
	xTxFrame.DataLen = TxLength;
	AppliSendBuff(&xTxFrame, xTxFrame.DataLen);

	printf("\r\n\r\nSe ha enviado el mensaje con los siguientes campos:\r\n");
	printf("\r\nCampo CMD: %d\r\n", MSG_CMD);
	printf("\r\nCampo CMD Tag: %d\r\n", txCounter);
	printf("\r\nCampo CMD Type: %d\r\n", APPLI_CMD);
	printf("\r\nComando: %d%d%d%d%d\r\n", aTransmitBuffer[0], aTransmitBuffer[1], aTransmitBuffer[2], aTransmitBuffer[3], aTransmitBuffer[4]);

}

void read_RX_Data(fsm_t* this)
{
	uint8_t msg_length = 0;

	xRxDoneFlag = RESET;

	Spirit1GetRxPacket(aReceiveBuffer,&RxLength);
	/*rRSSIValue = Spirit1GetRssiTH();*/
	/*rRSSIValue = S2LPGetRssiTH();*/
	xRxFrame.Cmd = aReceiveBuffer[0];
	xRxFrame.CmdLen = aReceiveBuffer[1];
	xRxFrame.Cmdtag = aReceiveBuffer[2];
	xRxFrame.CmdType = aReceiveBuffer[3];
	xRxFrame.DataLen = aReceiveBuffer[4];

	/*FIXED BUG IN DATA RECEPTION*/
	for (uint8_t xIndex = 5; xIndex < RxLength; xIndex++)
	{
	  temp_DataBuff[xIndex] = aReceiveBuffer[xIndex];
	}

	printf("\r\nSe ha Recibido el siguiente mensaje:\r\n");
	printf("\r\nCampo CMD: %d\r\n", aReceiveBuffer[0]);
	printf("\r\nCampo CMD Tag: %d\r\n", aReceiveBuffer[2]);
	printf("\r\nCampo CMD Type: %d\r\n", aReceiveBuffer[3]);
	printf("\r\nComando: ");

	msg_length = 5;
	while (msg_length < RxLength)
	{
		printf("%d", aReceiveBuffer[msg_length]);
		msg_length++;
	}
	printf("\r\n");

	xRxFrame.DataBuff= temp_DataBuff;
}

void LED_ON(fsm_t* this)
{
	/*IT WILL BE NECESSARY CHANGE IT TO PCB LED*/
#if defined(X_NUCLEO_IDS01A4) || defined(X_NUCLEO_IDS01A5)
    RadioShieldLedOn(RADIO_SHIELD_LED);
#endif
    dest_addr = SpiritPktCommonGetReceivedDestAddress();
}

void LED_Toggle(fsm_t* this)
{
	uint8_t ledToggleCtr = 0;

    dest_addr = SpiritPktCommonGetReceivedDestAddress();

#if defined(X_NUCLEO_IDS01A4) || defined(X_NUCLEO_IDS01A5)
	HAL_Delay(DELAY_TX_LED_GLOW);
#endif
	for(; ledToggleCtr<5; ledToggleCtr++)
	{
		/*IT WILL BE NECESSARY CHANGE IT TO PCB LED*/
		#if defined(X_NUCLEO_IDS01A4) || defined(X_NUCLEO_IDS01A5)
		RadioShieldLedToggle(RADIO_SHIELD_LED);
		#endif
		HAL_Delay(DELAY_RX_LED_TOGGLE);
	}
#if defined(X_NUCLEO_IDS01A4) || defined(X_NUCLEO_IDS01A5)
    RadioShieldLedOff(RADIO_SHIELD_LED);
#endif
    BSP_LED_Off(LED2);

    printf("\r\nACK recibido, transmision completada.\r\n");

    ACK_Process = SET;
}

void send_ACK(fsm_t* this)
{
	  xTxFrame.Cmd = ACK_OK;
	  xTxFrame.CmdLen = 0x01;
	  xTxFrame.Cmdtag = xRxFrame.Cmdtag;
	  xTxFrame.CmdType = APPLI_CMD;
	  xTxFrame.DataBuff = aTransmitBuffer;
	  xTxFrame.DataLen = TxLength;
	  HAL_Delay(DELAY_TX_LED_GLOW);

	  printf("\r\nEnviando ACK...\r\n");

	  AppliSendBuff(&xTxFrame, xTxFrame.DataLen);
}

void clr_tx_flag(fsm_t* this)
{
	xTxDoneFlag = RESET;
}

void reset_state(fsm_t* this)
{
#if defined(X_NUCLEO_IDS01A4) || defined(X_NUCLEO_IDS01A5)
    RadioShieldLedOff(RADIO_SHIELD_LED);
#endif
    BSP_LED_Off(LED2);

    ACK_Process = RESET;
    IDLE_Process = SET;
}





static fsm_trans_t radio_states[] = {
  { SM_STATE_START_RX, 			time_out_rx,    SM_STATE_START_RX, 			EN_Rx		 },
  { SM_STATE_START_RX, 			tx_flag,        SM_STATE_SEND_DATA, 		send_data	 },
  { SM_STATE_SEND_DATA, 		tx_done,   	 	SM_STATE_START_RX,  	    EN_Rx	 	 },
  { SM_STATE_START_RX, 			rx_flag,        SM_STATE_MSG_RECEIVED, 		read_RX_Data },
  { SM_STATE_MSG_RECEIVED,      data_received,  SM_STATE_DATA_RECEIVED, 	LED_ON 		 },
  { SM_STATE_MSG_RECEIVED,      ack_received,   SM_STATE_ACK_RECEIVED, 		LED_Toggle   },
  { SM_STATE_ACK_RECEIVED,    	ACK_confirm,   	SM_STATE_START_RX, 			EN_Rx		 },
  { SM_STATE_DATA_RECEIVED,    	multicast,   	SM_STATE_START_RX, 			EN_Rx	 	 },
  { SM_STATE_DATA_RECEIVED,    	address_known,  SM_STATE_SEND_ACK, 			send_ACK 	 },
  { SM_STATE_SEND_ACK,    		tx_done,		SM_STATE_START_RX, 			EN_Rx		 },
  {-1, NULL, -1, NULL },
  };
/* Private functions ---------------------------------------------------------*/

/** @defgroup S2LP_APPLI_Private_Functions
* @{
*/

/**
* @brief  Initializes RF Transceiver's HAL.
* @param  None
* @retval None.
*/
void HAL_Radio_Init(void)
{
  pRadioDriver = &radio_cb;
  pRadioDriver->Init( ); 
}



/* @brief  S2LP P2P Process State machine
* @param  uint8_t *pTxBuff = Pointer to aTransmitBuffer
*         uint8_t cTxlen = length of aTransmitBuffer
*         uint8_t* pRxBuff = Pointer to aReceiveBuffer
*         uint8_t cRxlen= length of aReceiveBuffer
* @retval None.
*/
void P2P_Process(void)
{
	fsm_fire(radio_fsm);
}

/**
* @brief  This function handles the point-to-point packet transmission
* @param  AppliFrame_t *xTxFrame = Pointer to AppliFrame_t structure 
*         uint8_t cTxlen = Length of aTransmitBuffer
* @retval None
*/
void AppliSendBuff(AppliFrame_t *xTxFrame, uint8_t cTxlen)
{
  uint8_t xIndex = 0;
  uint8_t trxLength = 0;
  
#ifdef USE_BASIC_PROTOCOL
  SpiritPktBasicAddressesInit(&xAddressInit);
#endif  
  
  TxFrameBuff[0] = xTxFrame->Cmd;
  TxFrameBuff[1] = xTxFrame->CmdLen;
  TxFrameBuff[2] = xTxFrame->Cmdtag;
  TxFrameBuff[3] = xTxFrame->CmdType;
  TxFrameBuff[4] = xTxFrame->DataLen;
  for(; xIndex < xTxFrame->DataLen; xIndex++)
  {
    TxFrameBuff[xIndex+5] =  xTxFrame->DataBuff[xIndex];
  }

  trxLength = (xIndex+5);

  /* Spirit IRQs disable */
  Spirit1DisableIrq();
  /* Spirit IRQs enable */
  Spirit1EnableTxIrq();
  /* payload length config */
  Spirit1SetPayloadlength(trxLength);
  /* rx timeout config */
  Spirit1SetRxTimeout(RECEIVE_TIMEOUT);
  /* IRQ registers blanking */
  Spirit1ClearIRQ();
  /* destination address */
  Spirit1SetDestinationAddress(DestinationAddr);
  /* send the TX command */
  Spirit1StartTx(TxFrameBuff, trxLength);
}


/**
* @brief  This function handles the point-to-point packet reception
* @param  uint8_t *RxFrameBuff = Pointer to ReceiveBuffer
*         uint8_t cRxlen = length of ReceiveBuffer
* @retval None
*/
void AppliReceiveBuff(uint8_t *RxFrameBuff, uint8_t cRxlen)
{
  /*float rRSSIValue = 0;*/
//  SW_RX_timeout = SET;
  exitCounter = TIME_TO_EXIT_RX;
  SpiritPktBasicAddressesInit(&xAddressInit);

  Spirit1DisableIrq();
  Spirit1EnableRxIrq();

  /* payload length config */
  Spirit1SetPayloadlength(PAYLOAD_LEN);

  /* rx timeout config */
  Spirit1SetRxTimeout(RECEIVE_TIMEOUT);

  /* destination address */
  Spirit1SetDestinationAddress(DestinationAddr);
  /* IRQ registers blanking */
  Spirit1ClearIRQ();
  /* RX command */ 
  Spirit1StartRx();
}

/**
* @brief  This function initializes the protocol for point-to-point 
* communication
* @param  None
* @retval None
*/
void P2P_Init(void)
{
  DestinationAddr = DESTINATION_ADDRESS;
  pRadioDriver->GpioIrq(&xGpioIRQ);

  /*Configure 868 transceiver*/
  selectedBand.conf_868 = SET;
  selectedBand.conf_433 = RESET;
  Spirit1RadioInit(&xRadioInit);
  Spirit1SetPower(POWER_INDEX, POWER_DBM);
  Spirit1PacketConfig();
  Spirit1EnableSQI();
  SpiritQiSetRssiThresholddBm(RSSI_THRESHOLD);

  /*Configure 433 transceiver
   * It cant be tested on evaluation board*/
//  selectedBand.conf_868 = RESET;
//  selectedBand.conf_433 = SET;
//  Spirit1RadioInit(&xRadioInit);
//  Spirit1SetPower(POWER_INDEX, POWER_DBM);
//  Spirit1PacketConfig();
//  Spirit1EnableSQI();
//  SpiritQiSetRssiThresholddBm(RSSI_THRESHOLD);


  radio_fsm = fsm_new (radio_states);

  /*868MHz band as predetermined band*/
  selectedBand.conf_433 = RESET;
  selectedBand.conf_868 = SET;

}

/**
* @brief  This function initializes the STack Packet handler of S2LP
* @param  None
* @retval None
*/
void STackProtocolInit(void)
{ 
  
  
  PktStackInit xStackInit=
  {
    .xPreambleLength = PREAMBLE_LENGTH,
    .xSyncLength = SYNC_LENGTH,
    .lSyncWords = SYNC_WORD,
#if defined(X_NUCLEO_IDS01A4) || defined(X_NUCLEO_IDS01A5)
    .xFixVarLength = LENGTH_TYPE,
    .cPktLengthWidth = 8, 
#endif   
    .xCrcMode = CRC_MODE,
#if defined(X_NUCLEO_IDS01A4) || defined(X_NUCLEO_IDS01A5)
    .xControlLength = CONTROL_LENGTH,
#endif   
    .xFec = EN_FEC,
    .xDataWhitening = EN_WHITENING
  };  
  /* Radio Packet config */
  SpiritPktStackInit (&xStackInit);
}  


/*xDataWhitening;  *
* @brief  This function initializes the BASIC Packet handler of S2LP
* @param  None
* @retval None
*/
void BasicProtocolInit(void)
{
  /* RAdio Packet config */
	SpiritPktBasicInit(&xBasicInit);
}

radio_select_t bandSelect(void)
{
	return selectedBand;
}



/**
* @brief  This routine will put the radio and mcu in LPM
* @param  None
* @retval None
*/
void Enter_LP_mode(void)
{  
  pMCU_LPM_Comm = &MCU_LPM_cb;
  pRadio_LPM_Comm = &Radio_LPM_cb;
  
#if defined(MCU_SLEEP_MODE)&&defined(RF_SHUTDOWN) 
  {
    pRadio_LPM_Comm->RadioShutDown(); 
    pMCU_LPM_Comm->McuSleepMode();
  }
#elif defined(MCU_SLEEP_MODE)&&defined(RF_STANDBY) 
  {
    pRadio_LPM_Comm->RadioStandBy(); 
    pMCU_LPM_Comm->McuSleepMode();
  }
#elif defined(MCU_SLEEP_MODE)&&defined(RF_SLEEP) 
  {
    pRadio_LPM_Comm->RadioSleep();
    pMCU_LPM_Comm->McuSleepMode();
  }
#elif defined(MCU_STOP_MODE)&&defined(RF_SHUTDOWN) 
  {
    pRadio_LPM_Comm->RadioShutDown(); 
    pMCU_LPM_Comm->McuStopMode();
  }
#elif defined(MCU_STOP_MODE)&&defined(RF_STANDBY) 
  {
    pRadio_LPM_Comm->RadioStandBy(); 
    pMCU_LPM_Comm->McuStopMode();
  }
#elif defined(MCU_STOP_MODE)&&defined(RF_SLEEP) 
  {
    pRadio_LPM_Comm->RadioSleep();
    pMCU_LPM_Comm->McuStopMode();
  }
#else
  pMCU_LPM_Comm->McuSleepMode();
#endif
}

/**
* @brief  This routine wake-up the mcu and radio from LPM
* @param  None
* @retval None
*/
void Exit_LP_mode(void)
{
  pRadio_LPM_Comm = &Radio_LPM_cb;      
  pRadio_LPM_Comm->RadioPowerON();  
}

/**
* @brief  This routine puts the MCU in stop mode
* @param  None
* @retval None
*/
void MCU_Enter_StopMode(void)
{
  APPLI_EnterSTOPMode();
}

/**
* @brief  This routine puts the MCU in standby mode
* @param  None
* @retval None
*/
void MCU_Enter_StandbyMode(void)
{
  APPLI_EnterSTANDBYMode();
}

/**
* @brief  This routine puts the MCU in sleep mode
* @param  None
* @retval None
*/
void MCU_Enter_SleepMode(void)
{
  /*Suspend Tick increment to prevent wakeup by Systick interrupt. 
  Otherwise the Systick interrupt will wake up the device within 1ms (HAL time base)*/
  
  APPLI_EnterSLEEPMode();
}

/**
* @brief  This function will turn on the radio and waits till it enters the Ready state.
* @param  Param:None. 
* @retval None
*                       
*/
void RadioPowerON(void)
{
	SpiritCmdStrobeReady();
  
  do{
    /* Delay for state transition */
    for(volatile uint8_t i=0; i!=0xFF; i++);
    
    /* Reads the MC_STATUS register */
    
    SpiritRefreshStatus();
    
    
  }
  while(g_xStatus.MC_STATE!=MC_STATE_READY);
}


/**
* @brief  This function will Shut Down the radio.
* @param  Param:None. 
* @retval None
*                       
*/
void RadioPowerOFF(void)
{
  RadioEnterShutdown();
  
  
}


/**
* @brief  This function will put the radio in standby state.
* @param  None. 
* @retval None
*                       
*/
void RadioStandBy(void)
{
	SpiritCmdStrobeStandby();
}

/**
* @brief  This function will put the radio in sleep state.
* @param  None. 
* @retval None
*                       
*/
void RadioSleep(void)
{
	SpiritCmdStrobeSleep();
}

/**
* @brief  This routine updates the respective status for key press.
* @param  None
* @retval None
*/
FlagStatus Rx_flag_status(void)
{
	return xRxDoneFlag;
}


void read_data_recived(uint8_t* pRxBuff, uint8_t cRxlen)
{

}


/**
* @brief  This function handles External interrupt request. In this application it is used
*         to manage the S2LP IRQ configured to be notified on the S2LP GPIO_3.
* @param  None
* @retval None
*/
void P2PInterruptHandler(void)
{
	SpiritIrqGetStatus(&xIrqStatus);
  
  
  /* Check the SPIRIT1 TX_DATA_SENT IRQ flag */
  if(
     (xIrqStatus.IRQ_TX_DATA_SENT) 
       
#ifdef CSMA_ENABLE
       ||(xIrqStatus.IRQ_MAX_BO_CCA_REACH)
#endif
         )
  {
#ifdef CSMA_ENABLE
	SpiritCsma(S_DISABLE);
	SpiritRadioPersistenRx(S_ENABLE);	/*To comeback to RX state*/
	SpiritRadioCsBlanking(S_ENABLE);
    
    if(xIrqStatus.IRQ_MAX_BO_CCA_REACH)
    {
    	SpiritCmdStrobeSabort();
        printf("\r\nNumero de retransmisiones maximas superadas,\r\nvolviendo al estado por defecto...\r\n");

    }
    SpiritQiSetRssiThresholddBm(RSSI_THRESHOLD);
    
    
#endif
    
    xTxDoneFlag = SET;
  }
  
  /* Check the S2LP RX_DATA_READY IRQ flag */
  if((xIrqStatus.IRQ_RX_DATA_READY))
  {
    xRxDoneFlag = SET;   
  }
  
  /* Restart receive after receive timeout*/
  if (xIrqStatus.IRQ_RX_TIMEOUT)
  {
    Spirit1_RX_timeout = SET; 
    SpiritCmdStrobeRx();
    
  }
  
  /* Check the S2LP RX_DATA_DISC IRQ flag */
  if(xIrqStatus.IRQ_RX_DATA_DISC)
  {      
    /* RX command - to ensure the device will be ready for the next reception */
	  SpiritCmdStrobeRx();
    
  }  
}

/**
* @brief  SYSTICK callback.
* @param  None
* @retval None
*/
void HAL_SYSTICK_Callback(void)
{
//  if(SW_RX_timeout)
//  {
//    /*Decreament the counter to check when 3 seconds has been elapsed*/
//    exitCounter--;
//    /*3 seconds has been elapsed*/
//    if(exitCounter <= TIME_UP)
//    {
//      SW_RX_timeout = RESET;
//    }
//  }
#if defined(RF_STANDBY)
  /*Check if Push Button pressed for wakeup or to send data*/
  if(PushButtonStatusWakeup)
  {
    /*Decreament the counter to check when 5 seconds has been elapsed*/  
    wakeupCounter--;
    
    /*5seconds has been elapsed*/
    if(wakeupCounter<=TIME_UP)
    {
      /*Perform wakeup opeartion*/      
      Exit_LP_mode();
      PushButtonStatusWakeup = RESET;
      PushButtonStatusData = SET;
    }
  }
  else if(PushButtonStatusData)
  {
    dataSendCounter--;
    if(dataSendCounter<=TIME_UP)
    {
      Set_KeyStatus(SET);
      PushButtonStatusWakeup = RESET;
      PushButtonStatusData = RESET;
    }
  }
#endif
}
/**
* @}
*/

/**
* @brief GPIO EXTI callback
* @param uint16_t GPIO_Pin
* @retval None
*/

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  
#if defined(RF_STANDBY)/*if S2LP is in standby*/
#if defined(MCU_STOP_MODE)/*if MCU is in stop mode*/        
  
  /* Clear Wake Up Flag */
  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
  
  /* Configures system clock after wake-up from STOP: enable HSE, PLL and select
  PLL as system clock source (HSE and PLL are disabled in STOP mode) */
  SystemClockConfig_ExitSTOPMode(); 
#endif
#if defined(MCU_SLEEP_MODE) 
  /* Resume Tick interrupt if disabled prior to sleep mode entry*/
  HAL_ResumeTick();
#endif 
  
  /* Initialize LEDs*/
#if defined(X_NUCLEO_IDS01A4) || defined(X_NUCLEO_IDS01A5)
  RadioShieldLedInit(RADIO_SHIELD_LED);
#endif
  BSP_LED_Init(LED2);
  
  PushButtonStatusWakeup = SET;
  PushButtonStatusData = RESET;
  wakeupCounter = LPM_WAKEUP_TIME; 
  dataSendCounter = DATA_SEND_TIME;
  dataSendCounter++;
#endif
  if(GPIO_Pin==USER_BUTTON_PIN)
  {

	/*It will be an USB interrupt, not a button*/
	tx_value = SET;

	for(uint8_t i; i<TX_BUFFER_SIZE; i++)
	{
		aTransmitBuffer[i] = i*2;
	}
  } 
#if defined(X_NUCLEO_IDS01A4) || defined(X_NUCLEO_IDS01A5)
  else 
    if(GPIO_Pin==RADIO_GPIO_3_EXTI_LINE)
    {
      P2PInterruptHandler();
    }
  
#endif
  
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

	printf("\r\n Teclee el primer parametro a enviar (uint8_t): ");
    scanf("%d", &a[0]);

    while (a[0] > 255)
    {
    	printf("\r\n Debe ser tipo uint8_t. \r\n");
    	printf("\r\n Teclee el primer parametro a enviar (uint8_t): ");
    	scanf("%d", &a[0]);
    }

	printf("\r\n\r\n Teclee el segundo parametro a enviar (uint8_t): ");
    scanf("%d", &a[1]);

    while (a[1] > 255)
    {
    	printf("\r\n Debe ser tipo uint8_t.\r\n");
    	printf("\r\n\r\n Teclee el segundo parametro a enviar (uint8_t): ");
    	scanf("%d", &a[1]);
    }

	printf("\r\n\r\n Teclee el tercer parametro a enviar (uint8_t): ");
    scanf("%d", &a[2]);

    while (a[2] > 255)
    {
    	printf("\r\n Debe ser tipo uint8_t.\r\n");
    	printf("\r\n\r\n Teclee el tercer parametro a enviar (uint8_t): ");
    	scanf("%d", &a[2]);
    }

	printf("\r\n\r\n Teclee el cuarto parametro a enviar (uint8_t): ");
    scanf("%d", &a[3]);

    while (a[3] > 255)
    {
    	printf("\r\n Debe ser tipo uint8_t.\r\n");
    	printf("\r\n\r\n Teclee el cuarto parametro a enviar (uint8_t): ");
    	scanf("%d", &a[3]);
    }

	printf("\r\n\r\n Teclee el quinto parametro a enviar (uint8_t): ");
    scanf("%d", &a[4]);

    while (a[4] > 255)
    {
    	printf("\r\n Debe ser tipo uint8_t.\r\n");
    	printf("\r\n\r\n Teclee el primer parametro a enviar (uint8_t): ");
    	scanf("%d", &a[4]);
    }

    aTransmitBuffer[0] = a[0];
    aTransmitBuffer[1] = a[1];
    aTransmitBuffer[2] = a[2];
    aTransmitBuffer[3] = a[3];
    aTransmitBuffer[4] = a[4];

	tx_value = SET;

	HAL_UART_Receive_IT(&huart2, aTransmitBuffer, 1);
}
/**
* @}
*/

/**
* @}
*/

/**
* @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
