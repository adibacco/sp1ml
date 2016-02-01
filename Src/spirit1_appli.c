/**
******************************************************************************
* @file    spirit1_appli.c
* @author  Central Labs
* @version V1.1.0
* @date    14-Aug-2014
* @brief   user file to configure Spirit1 transceiver.
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
#include "usart.h"
#include "spirit1_appli.h"
#include "MCU_Interface.h"

/** @addtogroup USER
* @{
*/

/** @defgroup SPIRIT1_APPLI
* @brief User file to configure spirit1 tranceiver for desired frequency and 
* @feature.
* @{
*/

/* Private typedef -----------------------------------------------------------*/

/**
* @brief RadioDriver_t structure fitting
*/
RadioDriver_t spirit_cb =
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
  SPIRIT_GPIO_3,
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
  LENGTH_TYPE,
  LENGTH_WIDTH,
  CRC_MODE,
  CONTROL_LENGTH,
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


/* Private define ------------------------------------------------------------*/
#define TIME_UP                                         0x01

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
RadioDriver_t *pRadioDriver;
MCULowPowerMode_t *pMCU_LPM_Comm;
RadioLowPowerMode_t  *pRadio_LPM_Comm;
/*Flags declarations*/
volatile FlagStatus xRxDoneFlag = RESET, xTxDoneFlag=RESET, cmdFlag=RESET;
volatile FlagStatus xStartRx=RESET, rx_timeout=RESET, exitTime=RESET, rx_error=RESET;
volatile FlagStatus datasendFlag=RESET, wakeupFlag=RESET;
volatile FlagStatus PushButtonStatusWakeup=RESET;
volatile FlagStatus PushButtonStatusData=RESET;
/*IRQ status struct declaration*/
SpiritIrqs xIrqStatus;
static __IO uint32_t KEYStatusData = 0x00;
AppliFrame_t xTxFrame, xRxFrame;
uint8_t TxFrameBuff[MAX_BUFFER_LEN] = {0x00};
uint16_t exitCounter = 0;
uint16_t txCounter = 0;
uint16_t wakeupCounter = 0;
uint16_t dataSendCounter = 0x00;

/* Private function prototypes -----------------------------------------------*/

void HAL_Spirit1_Init(void);
void Data_Comm_On(uint8_t *pTxBuff, uint8_t cTxlen, uint8_t* pRxBuff, uint8_t cRxlen);
void Enter_LP_mode(void);
void Exit_LP_mode(void);
void MCU_Enter_StopMode(void);
void MCU_Enter_StandbyMode(void);
void MCU_Enter_SleepMode(void);
void RadioPowerON(void);
void RadioPowerOFF(void);
void RadioStandBy(void);
void RadioSleep(void);
void AppliSendBuff(AppliFrame_t *xTxFrame, uint8_t cTxlen);
void AppliReceiveBuff(uint8_t *RxFrameBuff, uint8_t cRxlen);
void P2P_Init(void);
void STackProtocolInit(void);
void BasicProtocolInit(void);
void P2PInterruptHandler(void);
void Set_KeyStatus(FlagStatus val);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void HAL_SYSTICK_Callback(void);

/* Private functions ---------------------------------------------------------*/

/** @defgroup SPIRIT1_APPLI_Private_Functions
* @{
*/

/**
* @brief  Initializes RF Transceiver's HAL.
* @param  None
* @retval None.
*/
void HAL_Spirit1_Init(void)
{
  pRadioDriver = &spirit_cb;
  pRadioDriver->Init( ); 
}

/**
* @brief  SPIRIT1 Data Transfer Routine.
* @param  uint8_t *pTxBuff = Pointer to aTransmitBuffer 
*         uint8_t cTxlen = length of aTransmitBuffer 
*         uint8_t* pRxBuff = Pointer to aReceiveBuffer
*         uint8_t cRxlen= length of aReceiveBuffer
* @retval None.
*/
void Data_Comm_On(uint8_t *pTxBuff, uint8_t cTxlen, uint8_t* pRxBuff, uint8_t cRxlen)
{
#define RF_STANDBY
#if defined(RF_STANDBY)
  if(wakeupFlag)
  {
    HAL_UART_Transmit(&huart1, "Receiving\n", 10, 1000);

    AppliReceiveBuff(pRxBuff, cRxlen);

    if(datasendFlag)
    {    
      datasendFlag = RESET;
      xTxFrame.Cmd = LED_TOGGLE;
      xTxFrame.CmdLen = 0x01;
      xTxFrame.Cmdtag = txCounter++;
      xTxFrame.CmdType = APPLI_CMD;
      xTxFrame.DataBuff = pTxBuff;
      xTxFrame.DataLen = cTxlen;
      
      AppliSendBuff(&xTxFrame, xTxFrame.DataLen);
      HAL_UART_Transmit(&huart1, "Sending\n", 8, 1000);


      //AppliReceiveBuff(pRxBuff, cRxlen);
    }
    
    if(cmdFlag)
    {
      cmdFlag = RESET;
      
      xTxFrame.Cmd = ACK_OK;
      xTxFrame.CmdLen = 0x01;
      xTxFrame.Cmdtag = xRxFrame.Cmdtag;
      xTxFrame.CmdType = APPLI_CMD;
      xTxFrame.DataBuff = pTxBuff;
      xTxFrame.DataLen = cTxlen;
      
      AppliSendBuff(&xTxFrame, xTxFrame.DataLen);
      HAL_Delay(DELAY_TX_LED_GLOW);
      
      RadioShieldLedOff(RADIO_SHIELD_LED);
      
      wakeupFlag = RESET;
      Enter_LP_mode();
    }
  }
  else if(datasendFlag==SET && wakeupFlag==RESET)
  {
    datasendFlag = RESET;
    Enter_LP_mode();
  }
#else
  HAL_UART_Transmit(&huart1, "Receiving\n", 10, 1000);

 AppliReceiveBuff(pRxBuff, cRxlen);

  if(KEYStatusData)
  {
      KEYStatusData = RESET;
      xTxFrame.Cmd = LED_TOGGLE;
      xTxFrame.CmdLen = 0x01;
      xTxFrame.Cmdtag = txCounter++;
      xTxFrame.CmdType = APPLI_CMD;
      xTxFrame.DataBuff = pTxBuff;
      xTxFrame.DataLen = cTxlen;
      
      AppliSendBuff(&xTxFrame, xTxFrame.DataLen);
      HAL_UART_Transmit(&huart1, "Sending\n", 8, 1000);

  }
  if(cmdFlag)
  {
    cmdFlag = RESET;
    
    xTxFrame.Cmd = ACK_OK;
    xTxFrame.CmdLen = 0x01;
    xTxFrame.Cmdtag = xRxFrame.Cmdtag;
    xTxFrame.CmdType = APPLI_CMD;
    xTxFrame.DataBuff = pTxBuff;
    xTxFrame.DataLen = cTxlen;
    
    AppliSendBuff(&xTxFrame, xTxFrame.DataLen);
    
    HAL_Delay(DELAY_TX_LED_GLOW);
    
    RadioShieldLedOff(RADIO_SHIELD_LED);
    BSP_LED_Off(LED2);
#if defined(LPM_ENABLE)
    Enter_LP_mode();
#endif
  }  
#endif
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
  pRadioDriver = &spirit_cb; 
  
  TxFrameBuff[0] = xTxFrame->Cmd;
  TxFrameBuff[1] = xTxFrame->CmdLen;
  TxFrameBuff[2] = xTxFrame->Cmdtag;
  TxFrameBuff[3] = xTxFrame->CmdType;
  TxFrameBuff[4] = xTxFrame->DataLen;
  
  for(; xIndex < cTxlen; xIndex++)
  {
    TxFrameBuff[xIndex+5] =  xTxFrame->DataBuff[xIndex];
  }
  
  
  trxLength = (xIndex+5);
  
  /* Spirit IRQs enable */
  pRadioDriver->DisableIrq();
  pRadioDriver->EnableTxIrq();
    
  /* payload length config */
  pRadioDriver->SetPayloadLen(trxLength);  
  
  /* rx timeout config */
  pRadioDriver->SetRxTimeout(RECEIVE_TIMEOUT);
  
  /* IRQ registers blanking */
  pRadioDriver->ClearIrqStatus();
  
  /* destination address */
  pRadioDriver->SetDestinationAddress(DESTINATION_ADDRESS);
  /* send the TX command */
  pRadioDriver->StartTx(TxFrameBuff, trxLength);
}


/**
* @brief  This function handles the point-to-point packet reception
* @param  uint8_t *RxFrameBuff = Pointer to ReceiveBuffer
*         uint8_t cRxlen = length of ReceiveBuffer
* @retval None
*/
void AppliReceiveBuff(uint8_t *RxFrameBuff, uint8_t cRxlen)
{
  uint8_t xIndex = 0;
  uint8_t ledToggleCtr = 0;
  /*float rRSSIValue = 0;*/
  cmdFlag = RESET;
  exitTime = SET;
  exitCounter = TIME_TO_EXIT_RX;
  pRadioDriver = &spirit_cb;
  
  /* Spirit IRQs enable */
  pRadioDriver->DisableIrq();
  pRadioDriver->EnableRxIrq();
   
  /* payload length config */

  pRadioDriver->SetPayloadLen(PAYLOAD_LEN);
  /* rx timeout config */
  pRadioDriver->SetRxTimeout(1000);
  
  /* IRQ registers blanking */
  pRadioDriver->ClearIrqStatus();
  SpiritIrqGetStatus(&xIrqStatus);

  /* RX command */
  pRadioDriver->StartRx();      
    
  /* wait for data received or timeout period occured */
  while((RESET == xRxDoneFlag)&&(RESET == rx_timeout));
  

  if(rx_timeout==SET)
  {
    rx_timeout = RESET;
  }
  else if(xRxDoneFlag) 
  {
    xRxDoneFlag=RESET;

    pRadioDriver->GetRxPacket(RxFrameBuff,&cRxlen); 
    /*rRSSIValue = Spirit1GetRssiTH();*/
    
  
    xRxFrame.Cmd = RxFrameBuff[0];
    xRxFrame.CmdLen = RxFrameBuff[1];
    xRxFrame.Cmdtag = RxFrameBuff[2];
    xRxFrame.CmdType = RxFrameBuff[3];
    xRxFrame.DataLen = RxFrameBuff[4];
  
    for (xIndex = 5; xIndex < cRxlen; xIndex++)
    {
      xRxFrame.DataBuff[xIndex] = RxFrameBuff[xIndex];
    }
  
  
    if(xRxFrame.Cmd == LED_TOGGLE)
    {
      RadioShieldLedOn(RADIO_SHIELD_LED);
      cmdFlag = SET;      
    }
    if(xRxFrame.Cmd == ACK_OK)
    {
      for(; ledToggleCtr<5; ledToggleCtr++)
      {
        RadioShieldLedToggle(RADIO_SHIELD_LED);
        HAL_Delay(DELAY_RX_LED_TOGGLE);
      }
      RadioShieldLedOff(RADIO_SHIELD_LED);
            
#if defined(LPM_ENABLE)
#if defined(RF_STANDBY)/*||defined(RF_SLEEP)*/
    wakeupFlag = RESET;
#endif
    Enter_LP_mode();
#endif
    }
  }
}

/**
* @brief  This function initializes the protocol for point-to-point 
* communication
* @param  None
* @retval None
*/
void P2P_Init(void)
{
  pRadioDriver = &spirit_cb;
   
     /* Spirit IRQ config */
  pRadioDriver->GpioIrq(&xGpioIRQ);
  
  /* Spirit Radio config */    
  pRadioDriver->RadioInit(&xRadioInit);
  
  /* Spirit Radio set power */
  pRadioDriver->SetRadioPower(POWER_INDEX, POWER_DBM);  
  
  /* Spirit Packet config */  
  pRadioDriver->PacketConfig();
  
  pRadioDriver->EnableSQI();
  
  pRadioDriver->SetRssiThreshold(RSSI_THRESHOLD);
}

/**
* @brief  This function initializes the STack Packet handler of spirit1
* @param  None
* @retval None
*/
void STackProtocolInit(void)
{
#if defined(USE_STack_PROTOCOL)
  /* Spirit Packet config */
  SpiritPktStackInit(&xStackInit);
  SpiritPktStackAddressesInit(&xAddressInit);
  SpiritPktStackLlpInit(&xStackLLPInit);
  
    /* require ack from the receiver */
  SpiritPktStackRequireAck(S_ENABLE);

  if(EN_FILT_SOURCE_ADDRESS)
  {
    SpiritPktStackFilterOnSourceAddress(S_ENABLE);
    SpiritPktStackSetRxSourceMask(SOURCE_ADDR_MASK);
    SpiritPktStackSetSourceReferenceAddress(SOURCE_ADDR_REF);
    
  }
  else
  {
    SpiritPktStackFilterOnSourceAddress(S_DISABLE);    
  }

#endif
}

/**
* @brief  This function initializes the BASIC Packet handler of spirit1
* @param  None
* @retval None
*/
void BasicProtocolInit(void)
{ 
#if defined(USE_BASIC_PROTOCOL)
  /* Spirit Packet config */
  SpiritPktBasicInit(&xBasicInit);
  SpiritPktBasicAddressesInit(&xAddressInit);
#endif
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
  
#if defined(MCU_STOP_MODE)&&defined(RF_SHUTDOWN) 
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
#elif defined(MCU_STANDBY_MODE)&&defined(RF_SHUTDOWN) 
  {
    pRadio_LPM_Comm->RadioShutDown(); 
    pMCU_LPM_Comm->McuStandbyMode();
  } 
#elif defined(MCU_STANDBY_MODE)&&defined(RF_STANDBY) 
  {
    pRadio_LPM_Comm->RadioStandBy();  
    pMCU_LPM_Comm->McuStandbyMode();
  }
#elif defined(MCU_STANDBY_MODE)&&defined(RF_SLEEP) 
  {
    pRadio_LPM_Comm->RadioSleep();
    pMCU_LPM_Comm->McuStandbyMode();
  }  
#elif defined(MCU_SLEEP_MODE)&&defined(RF_SHUTDOWN) 
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
#elif defined(MCU_STOP_MODE)
  pMCU_LPM_Comm->McuStopMode();
  
#elif defined(MCU_STANDBY_MODE)
  pMCU_LPM_Comm->McuStandbyMode();
  
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
#if 0
void MCU_Enter_StopMode(void)
{

	  HAL_SuspendTick();
	  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
	  while((PWR->CSR & (uint32_t) 0x00000001)!=0);//attesa che il WUF si azzeri (via HW)

	  HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFE);  /* Infinite loop */

	  HAL_ResumeTick();

}
#endif
/**
* @brief  This routine puts the MCU in standby mode
* @param  None
* @retval None
*/
void MCU_Enter_StandbyMode(void)
{
  HAL_PWR_EnterSTANDBYMode();  /* Infinite loop */
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
  HAL_SuspendTick();
  HAL_PWR_EnterSLEEPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);  /* Infinite loop */
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
  SpiritEnterShutdown();
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
#if 0  
  do{
    /* Delay for state transition */
    for(volatile uint8_t i=0; i!=0xFF; i++);
    
    /* Reads the MC_STATUS register */
    SpiritRefreshStatus();
  }
  while(g_xStatus.MC_STATE!=MC_STATE_STANDBY);
#endif
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
#if 0
  do{
    /* Delay for state transition */
    for(volatile uint8_t i=0; i!=0xFF; i++);
    
    /* Reads the MC_STATUS register */
    SpiritRefreshStatus();
  }
  while(g_xStatus.MC_STATE!=MC_STATE_SLEEP);
#endif
}



/**
* @brief  This function handles External interrupt request. In this application it is used
*         to manage the Spirit IRQ configured to be notified on the Spirit GPIO_3.
* @param  None
* @retval None
*/
void P2PInterruptHandler(void)
{
  SpiritIrqGetStatus(&xIrqStatus);
  
  /* Check the SPIRIT TX_DATA_SENT IRQ flag */
  if(xIrqStatus.IRQ_TX_DATA_SENT || xIrqStatus.IRQ_MAX_RE_TX_REACH)
  {
    xTxDoneFlag = SET;
  }
  
  /* Check the SPIRIT RX_DATA_READY IRQ flag */
  else if(xIrqStatus.IRQ_RX_DATA_READY)
  {
    xRxDoneFlag = SET;   
  }
  
  /* Check the SPIRIT RX_DATA_DISC IRQ flag */
  else if(xIrqStatus.IRQ_RX_DATA_DISC)
  {    
	rx_error = SET;
    /* RX command - to ensure the device will be ready for the next reception */
    if(xIrqStatus.IRQ_RX_TIMEOUT)
    {
      SpiritCmdStrobeFlushRxFifo();
      rx_timeout = SET; 
    }
  }
}

/**
  * @brief  SYSTICK callback.
  * @param  None
  * @retval None
  */
void HAL_SYSTICK_Callback(void)
{

#if defined(RF_STANDBY)
  /*Check if Push Button pressed for wakeup or to send data*/

    /*Decreament the counter to check when 5 seconds has been elapsed*/  
  wakeupCounter--;
    
    /*5seconds has been elapsed*/
  if(wakeupCounter<=TIME_UP)
  {
      /*Perform wakeup opeartion*/
      wakeupFlag = SET;
      Exit_LP_mode();
      PushButtonStatusWakeup = RESET;
      PushButtonStatusData = SET;
  }
  else if(PushButtonStatusData)
  {
    dataSendCounter--;
    if(dataSendCounter<=TIME_UP)
    {
      datasendFlag = SET;
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
* @}
*/

/**
* @}
*/

/**
* @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
