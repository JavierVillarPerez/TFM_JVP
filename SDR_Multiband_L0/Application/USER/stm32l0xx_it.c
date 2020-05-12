/**
******************************************************************************
* @file    stm32l1xx_it.c
* @author  MCD Application Team
* @version V3.3.0
* @date    21-March-2011
* @brief   Main Interrupt Service Routines.
*          This file provides template for all exceptions handler and
*          peripherals interrupt service routine.
******************************************************************************
* @copy
*
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
* TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
* DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
* FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
* CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
* <h2><center>&copy; COPYRIGHT 2015 STMicroelectronics</center></h2>
*/

/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx_it.h"
#include "cube_hal.h"
#if defined(X_NUCLEO_IDS01A4) || defined(X_NUCLEO_IDS01A5)
#include "radio_shield_config.h"
#include "radio_gpio.h"
#endif

#include "radio_appli.h"
/** @addtogroup Template_Project
* @{
*/

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/
//#if defined(X_NUCLEO_IDS01A4) || defined(X_NUCLEO_IDS01A5)
void EXTI0_1_IRQHandler(void);
void EXTI2_3_IRQHandler(void);
void EXTI4_15_IRQHandler(void);
void SysTick_Handler(void);
//#endif

/**
* @brief  This function handles NMI exception.
* @param  None
* @retval : None
*/
void NMI_Handler(void)
{
}

/**
* @brief  This function handles Hard Fault exception.
* @param  None
* @retval : None
*/
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
* @brief  This function handles Memory Manage exception.
* @param  None
* @retval : None
*/
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
* @brief  This function handles Bus Fault exception.
* @param  None
* @retval : None
*/
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
* @brief  This function handles Usage Fault exception.
* @param  None
* @retval : None
*/
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
* @brief  This function handles SVCall exception.
* @param  None
* @retval : None
*/

void DebugMon_Handler(void)
{
}
void SVC_Handler(void)
{
}


/**
* @brief  This function handles PendSVC exception.
* @param  None
* @retval : None
*/
void PendSV_Handler(void)
{
}

/**
* @brief  This function handles SysTick Handler.
* @param  None
* @retval None
*/

void SysTick_Handler(void)
{
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
}

/*******************************************************************************
* Function Name  : USB_LP_IRQHandler
* Description    : This function handles USB Low Priority interrupts  requests.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
//void USB_LP_IRQHandler(void)
//{
//  HAL_PCD_IRQHandler(&hpcd);
//}

void EXTI0_1_IRQHandler(void)
{
  /* EXTI line interrupt detected */
  if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_0) != RESET) 
  {
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
  }
  
  if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_1) != RESET) 
  {
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
  }
}

void EXTI2_3_IRQHandler(void)
{
  if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_2) != RESET) 
  {
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
  }
  if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_3) != RESET) 
  {
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);
    
  }  
}
/**
* @brief  This function handles External External line 9 to 5 interrupt request.
* @param  None
* @retval None
*/
void EXTI4_15_IRQHandler(void)
{
#if defined(X_NUCLEO_IDS01A4) || defined(X_NUCLEO_IDS01A5)
  /* EXTI line 7 interrupt detected */
  if(__HAL_GPIO_EXTI_GET_IT(RADIO_GPIO_3_EXTI_LINE) != RESET) 
  { 
    // __HAL_GPIO_EXTI_CLEAR_IT(RADIO_GPIO_3_EXTI_LINE);
    
    //P2PInterruptHandler();
    HAL_GPIO_EXTI_IRQHandler(RADIO_GPIO_3_EXTI_LINE);
  } 
  
  HAL_GPIO_EXTI_IRQHandler(KEY_BUTTON_PIN);
#endif
}



/******************************************************************************/
/*                 STM32L15x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32l1xx_lp.s).                                            */
/******************************************************************************/

/**
* @brief  This function handles PPP interrupt request.
* @param  None
* @retval : None
*/
/*void PPP_IRQHandler(void)
{
}*/

/**
* @}
*/


/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
