#ifndef _P2P_LIB_H_
#define _P2P_LIB_H_
#include "MCU_Interface.h" 

#if defined(X_NUCLEO_IDS01A4) || defined(X_NUCLEO_IDS01A5)
#include "SPIRIT1_Util.h"
#endif
/*---------appli.h-----*/
#if defined(X_NUCLEO_IDS01A4) || defined(X_NUCLEO_IDS01A5)
//#define       RADIO_Csma.h      SPIRIT_Csma

#endif

#if defined(X_NUCLEO_IDS01A4) || defined(X_NUCLEO_IDS01A5)

#define        HAL_Radio_Init(void)          HAL_Spirit1_Init(void)   

#define        RadioInterfaceInit             Spirit1InterfaceInit
#define        RadioGpioIrqInit               Spirit1GpioIrqInit
#define        RadioRadioInit                 Spirit1RadioInit
#define        RadioSetPower                  Spirit1SetPower
#define        RadioPacketConfig             Spirit1PacketConfig
#define        RadioSetPayloadlength         Spirit1SetPayloadlength
#define        RadioSetDestinationAddress    Spirit1SetDestinationAddress
#define        RadioEnableTxIrq              Spirit1EnableTxIrq
#define        RadioEnableRxIrq              Spirit1EnableRxIrq
#define        RadioDisableIrq               Spirit1DisableIrq
#define        RadioSetRxTimeout             Spirit1SetRxTimeout
#define        RadioEnableSQI                Spirit1EnableSQI
#define        RadioSetRssiTH                 Spirit1SetRssiTH//
#define        RadioClearIRQ                  Spirit1ClearIRQ//
#define        RadioStartRx                     Spirit1StartRx
#define        RadioStartTx                     Spirit1StartTx
#define        RadioGetRxPacket                 Spirit1GetRxPacket

#define    RADIO_GPIO_MODE_DIGITAL_OUTPUT_LP      SPIRIT_GPIO_MODE_DIGITAL_OUTPUT_LP
#define    RADIO_GPIO_DIG_OUT_IRQ                 SPIRIT_GPIO_DIG_OUT_IRQ

#define    RadioCsmaInit             CsmaInit

#define    RadioIrqs                 SpiritIrqs

#define    RadioGetReceivedDestinationAddress()       SpiritPktCommonGetReceivedDestAddress()

#define    Radio_PktStackAddressesInit                SpiritPktStackAddressesInit

#define    Radio_PktStackLlpInit                SpiritPktStackLlpInit
#define    RadioPktBasicAddressesInit                SpiritPktBasicAddressesInit

#define    RadioPktStackFilterOnSourceAddress                SpiritPktStackFilterOnSourceAddress
#define    RadioPktStackSetRxSourceMask                      SpiritPktStackSetRxSourceMask
#define    RadioPktStackSetSourceReferenceAddress            SpiritPktStackSetSourceReferenceAddress

#define    RadioPktStackInit                  SpiritPktStackInit
#define    RadioPktBasicInit                  SpiritPktBasicInit

#define    RadioCmdStrobeReady                SpiritCmdStrobeReady
#define    RadioRefreshStatus                 SpiritRefreshStatus
//#define    RadioEnterShutdown()               SpiritEnterShutdown()KG
#define    RadioCmdStrobeStandby()            SpiritCmdStrobeStandby()
#define    RadioCmdStrobeSleep()              SpiritCmdStrobeSleep()
#define    RadioCmdStrobeSabort()             SpiritCmdStrobeSabort()
#define    RadioCmdStrobeRx()                 SpiritCmdStrobeRx()

#define    RadioGpioIrqGetStatus              SpiritIrqGetStatus

#define    RadioCsma                         SpiritCsma
#define    RadioRadioPersistenRx             SpiritRadioPersistenRx
#define    RadioRadioCsBlanking              SpiritRadioCsBlanking

#define    RadioQiSetRssiThresholddBm       SpiritQiSetRssiThresholddBm


#endif  

#endif //_P2P_LIB_H_
