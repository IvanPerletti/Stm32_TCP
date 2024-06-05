/**
  ******************************************************************************
  * @file    CAN/LoopBack/stm32f4xx_it.c
  * @author  MCD Application Team
  * @version V1.0.1
  * @date    13-April-2012
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/

#include "stm32f4xx_it.h"
#include "usb_core.h"
#include "usbd_core.h"
#include "stm32f4_discovery.h"
#include "usbd_cdc_core.h"
/** @addtogroup STM32F4xx_StdPeriph_Examples
  * @{
  */

/** @addtogroup CAN_LoopBack
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern __IO uint32_t ret;


/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
extern void HardFault_Handler( void );
extern USB_OTG_CORE_HANDLE           USB_OTG_dev;
extern uint32_t USBD_OTG_ISR_Handler (USB_OTG_CORE_HANDLE *pdev);
extern void DISCOVERY_EXTI_IRQHandler(void);

#ifdef USB_OTG_HS_DEDICATED_EP1_ENABLED
extern uint32_t USBD_OTG_EP1IN_ISR_Handler (USB_OTG_CORE_HANDLE *pdev);
extern uint32_t USBD_OTG_EP1OUT_ISR_Handler (USB_OTG_CORE_HANDLE *pdev);
#endif

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/
/**
 * @brief   This function handles NMI exception.
 * @param  None
 * @retval None
 */
void NMI_Handler(void)
{
}
/* The prototype shows it is a naked function - in effect this is just an
assembly function. */
// From Joseph Yiu, minor edits by FVH
// hard fault handler in C,
// with stack frame location as input parameter
// called from HardFault_Handler in file xxx.s
void hard_fault_handler_c (unsigned int * hardfault_args)
{
  unsigned int stacked_r0;
  unsigned int stacked_r1;
  unsigned int stacked_r2;
  unsigned int stacked_r3;
  unsigned int stacked_r12;
  unsigned int stacked_lr;
  unsigned int stacked_pc;
  unsigned int stacked_psr;

  stacked_r0 = ((unsigned long) hardfault_args[0]);
  stacked_r1 = ((unsigned long) hardfault_args[1]);
  stacked_r2 = ((unsigned long) hardfault_args[2]);
  stacked_r3 = ((unsigned long) hardfault_args[3]);

  stacked_r12 = ((unsigned long) hardfault_args[4]);
  stacked_lr = ((unsigned long) hardfault_args[5]);
  stacked_pc = ((unsigned long) hardfault_args[6]);
  stacked_psr = ((unsigned long) hardfault_args[7]);

//  printf ("\n\n[Hard fault handler - all numbers in hex]\n");
//  printf ("R0 = %x\n", stacked_r0);
//  printf ("R1 = %x\n", stacked_r1);
//  printf ("R2 = %x\n", stacked_r2);
//  printf ("R3 = %x\n", stacked_r3);
//  printf ("R12 = %x\n", stacked_r12);
//  printf ("LR [R14] = %x  subroutine call return address\n", stacked_lr);
//  printf ("PC [R15] = %x  program counter\n", stacked_pc);
//  printf ("PSR = %x\n", stacked_psr);
//  printf ("BFAR = %x\n", (*((volatile unsigned long *)(0xE000ED38))));
//  printf ("CFSR = %x\n", (*((volatile unsigned long *)(0xE000ED28))));
//  printf ("HFSR = %x\n", (*((volatile unsigned long *)(0xE000ED2C))));
//  printf ("DFSR = %x\n", (*((volatile unsigned long *)(0xE000ED30))));
//  printf ("AFSR = %x\n", (*((volatile unsigned long *)(0xE000ED3C))));
//  printf ("SCB_SHCSR = %x\n", SCB->SHCSR);
  
  while (1);
}

/* The fault handler implementation calls a function called
prvGetRegistersFromStack(). */
//static void HardFault_Handler_Perletti(void)
//{
//    __asm volatile
//    (
//        " tst lr, #4                                                \n"
//        " ite eq                                                    \n"
//        " mrseq r0, msp                                             \n"
//        " mrsne r0, psp                                             \n"
//        " ldr r1, [r0, #24]                                         \n"
//        " ldr r2, handler2_address_const                            \n"
//        " bx r2                                                     \n"
//        " handler2_address_const: .word prvGetRegistersFromStack    \n"
//    );
//}
/**
 * @brief  This function handles Hard Fault exception.
 * @param  None
 * @retval None
 */
void prvGetRegistersFromStack (unsigned int * hardfault_args)
{
  unsigned int stacked_r0;
  unsigned int stacked_r1;
  unsigned int stacked_r2;
  unsigned int stacked_r3;
  unsigned int stacked_r12;
  unsigned int stacked_lr;
  unsigned int stacked_pc;
  unsigned int stacked_psr;
 
  stacked_r0 = ((unsigned long) hardfault_args[0]);
  stacked_r1 = ((unsigned long) hardfault_args[1]);
  stacked_r2 = ((unsigned long) hardfault_args[2]);
  stacked_r3 = ((unsigned long) hardfault_args[3]);
 
  stacked_r12 = ((unsigned long) hardfault_args[4]);
  stacked_lr  = ((unsigned long) hardfault_args[5]);
  stacked_pc  = ((unsigned long) hardfault_args[6]);
  stacked_psr = ((unsigned long) hardfault_args[7]);
 
//  printf ("\n\n[Hard fault handler - all numbers in hex]\n");
//  printf ("R0 = %x\n", stacked_r0);
//  printf ("R1 = %x\n", stacked_r1);
//  printf ("R2 = %x\n", stacked_r2);
//  printf ("R3 = %x\n", stacked_r3);
//  printf ("R12 = %x\n", stacked_r12);
//  printf ("LR [R14] = %x  subroutine call return address\n", stacked_lr);
//  printf ("PC [R15] = %x  program counter\n", stacked_pc);
//  printf ("PSR = %x\n", stacked_psr);
//  printf ("BFAR = %x\n", (*((volatile unsigned long *)(0xE000ED38))));
//  printf ("CFSR = %x\n", (*((volatile unsigned long *)(0xE000ED28))));
//  printf ("HFSR = %x\n", (*((volatile unsigned long *)(0xE000ED2C))));
//  printf ("DFSR = %x\n", (*((volatile unsigned long *)(0xE000ED30))));
//  printf ("AFSR = %x\n", (*((volatile unsigned long *)(0xE000ED3C))));
//  printf ("SCB_SHCSR = %x\n", SCB->SHCSR);
  
  while (1);

}

/**
 * @brief  This function handles Memory Manage exception.
 * @param  None
 * @retval None
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
 * @retval None
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
 * @retval None
 */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}


/**
 * @brief  This function handles Debug Monitor exception.
 * @param  None
 * @retval None
 */
void DebugMon_Handler(void)
{
}

//------------------------------------------------------------------------------
/**
 * @brief  This function handles EXTI0_IRQ Handler.
 * @param  None
 * @retval None
 */
void EXTI0_IRQHandler(void)
{
}
/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
/*
void PendSV_Handler(void)
{
}
*/

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
/*
void SysTick_Handler(void)
{
}
*/

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles OTG_FS_WKUP_IRQ Handler.
  * @param  None
  * @retval None
  */
/*
#ifdef USE_USB_OTG_FS
void OTG_FS_WKUP_IRQHandler(void)
{
  if(USB_OTG_dev.cfg.low_power)
  {
    *(uint32_t *)(0xE000ED10) &= 0xFFFFFFF9 ;
    SystemInit();
    USB_OTG_UngateClock(&USB_OTG_dev);
  }
  EXTI_ClearITPendingBit(EXTI_Line18);
}
#endif
*/

/**
  * @brief  This function handles OTG_HS_WKUP_IRQ Handler.
  * @param  None
  * @retval None
  */
#ifdef USE_USB_OTG_HS
void OTG_HS_WKUP_IRQHandler(void)
{
  if(USB_OTG_dev.cfg.low_power)
  {
    *(uint32_t *)(0xE000ED10) &= 0xFFFFFFF9 ;
    SystemInit();
    USB_OTG_UngateClock(&USB_OTG_dev);
  }
  EXTI_ClearITPendingBit(EXTI_Line20);
}
#endif

/**
  * @brief  This function handles OTG_xx_IRQ Handler.
  * @param  None
  * @retval None
  */
/*
#ifdef USE_USB_OTG_HS
void OTG_HS_IRQHandler(void)
#else
void OTG_FS_IRQHandler(void)
#endif
{
  USBD_OTG_ISR_Handler (&USB_OTG_dev);
}
*/

#ifdef USB_OTG_HS_DEDICATED_EP1_ENABLED
/**
  * @brief  This function handles EP1_IN Handler.
  * @param  None
  * @retval None
  */
void OTG_HS_EP1_IN_IRQHandler(void)
{
  USBD_OTG_EP1IN_ISR_Handler (&USB_OTG_dev);
}

/**
  * @brief  This function handles EP1_OUT Handler.
  * @param  None
  * @retval None
  */
void OTG_HS_EP1_OUT_IRQHandler(void)
{
  USBD_OTG_EP1OUT_ISR_Handler (&USB_OTG_dev);
}
#endif


/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/


/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
