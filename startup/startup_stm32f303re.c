#include "stdint.h"

extern uint32_t _estack;
extern uint32_t _sidata;
extern uint32_t _sdata;
extern uint32_t _edata;
extern uint32_t _sbss;
extern uint32_t _ebss;

extern int main (void);

void Reset_Handler (void);
void NMI_Handler (void)
    __attribute__ ((weak, alias ("Default_Handler")));
void HardFault_Handler (void)
    __attribute__ ((weak, alias ("Default_Handler")));
void MemManage_Handler (void)
    __attribute__ ((weak, alias ("Default_Handler")));
void BusFault_Handler (void)
    __attribute__ ((weak, alias ("Default_Handler")));
void UsageFault_Handler (void)
    __attribute__ ((weak, alias ("Default_Handler")));
void SVCall_Handler (void)
    __attribute__ ((weak, alias ("Default_Handler")));
void PendSV_Handler (void)
    __attribute__ ((weak, alias ("Default_Handler")));
void SysTick_Handler (void)
    __attribute__ ((weak, alias ("Default_Handler")));
void WWDG_IRQHandler (void)
    __attribute__ ((weak, alias ("Default_Handler")));
void TAMPER_STAMP_IRQHandler (void)
    __attribute__ ((weak, alias ("Default_Handler")));
void RTC_WKUP_IRQHandler (void)
    __attribute__ ((weak, alias ("Default_Handler")));
void FLASH_IRQHandler (void)
    __attribute__ ((weak, alias ("Default_Handler")));
void RCC_IRQHandler (void)
    __attribute__ ((weak, alias ("Default_Handler")));
void EXTI0_IRQHandler (void)
    __attribute__ ((weak, alias ("Default_Handler")));
void EXTI1_IRQHandler (void)
    __attribute__ ((weak, alias ("Default_Handler")));
void EXTI2_TS_IRQHandler (void)
    __attribute__ ((weak, alias ("Default_Handler")));
void EXTI3_IRQHandler (void)
    __attribute__ ((weak, alias ("Default_Handler")));
void EXTI4_IRQHandler (void)
    __attribute__ ((weak, alias ("Default_Handler")));
void DMA1_Channel1_IRQHandler (void)
    __attribute__ ((weak, alias ("Default_Handler")));
void DMA1_Channel2_IRQHandler (void)
    __attribute__ ((weak, alias ("Default_Handler")));
void DMA1_Channel3_IRQHandler (void)
    __attribute__ ((weak, alias ("Default_Handler")));
void DMA1_Channel4_IRQHandler (void)
    __attribute__ ((weak, alias ("Default_Handler")));
void DMA1_Channel5_IRQHandler (void)
    __attribute__ ((weak, alias ("Default_Handler")));
void DMA1_Channel6_IRQHandler (void)
    __attribute__ ((weak, alias ("Default_Handler")));
void DMA1_Channel7_IRQHandler (void)
    __attribute__ ((weak, alias ("Default_Handler")));
void ADC1_2_IRQHandler (void)
    __attribute__ ((weak, alias ("Default_Handler")));
void USB_HP_CAN_TX_IRQHandler (void)
    __attribute__ ((weak, alias ("Default_Handler")));
void USB_LP_CAN_RX0_IRQHandler (void)
    __attribute__ ((weak, alias ("Default_Handler")));
void CAN_RX1_IRQHandler (void)
    __attribute__ ((weak, alias ("Default_Handler")));
void CAN_SCE_IRQHandler (void)
    __attribute__ ((weak, alias ("Default_Handler")));
void EXTI9_5_IRQHandler (void)
    __attribute__ ((weak, alias ("Default_Handler")));
void TIM1_BRK_TIM15_IRQHandler (void)
    __attribute__ ((weak, alias ("Default_Handler")));
void TIM1_UP_TIM16_IRQHandler (void)
    __attribute__ ((weak, alias ("Default_Handler")));
void TIM1_TRG_COM_TIM17_IRQHandler (void)
    __attribute__ ((weak, alias ("Default_Handler")));
void TIM1_CC_IRQHandler (void)
    __attribute__ ((weak, alias ("Default_Handler")));
void TIM2_IRQHandler (void)
    __attribute__ ((weak, alias ("Default_Handler")));
void TIM3_IRQHandler (void)
    __attribute__ ((weak, alias ("Default_Handler")));
void TIM4_IRQHandler (void)
    __attribute__ ((weak, alias ("Default_Handler")));
void I2C1_EV_IRQHandler (void)
    __attribute__ ((weak, alias ("Default_Handler")));
void I2C1_ER_IRQHandler (void)
    __attribute__ ((weak, alias ("Default_Handler")));
void I2C2_EV_IRQHandler (void)
    __attribute__ ((weak, alias ("Default_Handler")));
void I2C2_ER_IRQHandler (void)
    __attribute__ ((weak, alias ("Default_Handler")));
void SPI1_IRQHandler (void)
    __attribute__ ((weak, alias ("Default_Handler")));
void SPI2_IRQHandler (void)
    __attribute__ ((weak, alias ("Default_Handler")));
void USART1_IRQHandler (void)
    __attribute__ ((weak, alias ("Default_Handler")));
void USART2_IRQHandler (void)
    __attribute__ ((weak, alias ("Default_Handler")));
void USART3_IRQHandler (void)
    __attribute__ ((weak, alias ("Default_Handler")));
void EXTI15_10_IRQHandler (void)
    __attribute__ ((weak, alias ("Default_Handler")));
void RTC_Alarm_IRQHandler (void)
    __attribute__ ((weak, alias ("Default_Handler")));
void USBWakeUp_IRQHandler (void)
    __attribute__ ((weak, alias ("Default_Handler")));
void TIM8_BRK_IRQHandler (void)
    __attribute__ ((weak, alias ("Default_Handler")));
void TIM8_UP_IRQHandler (void)
    __attribute__ ((weak, alias ("Default_Handler")));
void TIM8_TRG_COM_IRQHandler (void)
    __attribute__ ((weak, alias ("Default_Handler")));
void TIM8_CC_IRQHandler (void)
    __attribute__ ((weak, alias ("Default_Handler")));
void ADC3_IRQHandler (void)
    __attribute__ ((weak, alias ("Default_Handler")));
void FMC_IRQHandler (void)
    __attribute__ ((weak, alias ("Default_Handler")));
void SPI3_IRQHandler (void)
    __attribute__ ((weak, alias ("Default_Handler")));
void UART4_IRQHandler (void)
    __attribute__ ((weak, alias ("Default_Handler")));
void UART5_IRQHandler (void)
    __attribute__ ((weak, alias ("Default_Handler")));
void TIM6_DAC_IRQHandler (void)
    __attribute__ ((weak, alias ("Default_Handler")));
void TIM7_IRQHandler (void)
    __attribute__ ((weak, alias ("Default_Handler")));
void DMA2_Channel1_IRQHandler (void)
    __attribute__ ((weak, alias ("Default_Handler")));
void DMA2_Channel2_IRQHandler (void)
    __attribute__ ((weak, alias ("Default_Handler")));
void DMA2_Channel3_IRQHandler (void)
    __attribute__ ((weak, alias ("Default_Handler")));
void DMA2_Channel4_IRQHandler (void)
    __attribute__ ((weak, alias ("Default_Handler")));
void DMA2_Channel5_IRQHandler (void)
    __attribute__ ((weak, alias ("Default_Handler")));
void ADC4_IRQHandler (void)
    __attribute__ ((weak, alias ("Default_Handler")));
void COMP1_2_3_IRQHandler (void)
    __attribute__ ((weak, alias ("Default_Handler")));
void COMP4_5_6_IRQHandler (void)
    __attribute__ ((weak, alias ("Default_Handler")));
void COMP7_IRQHandler (void)
    __attribute__ ((weak, alias ("Default_Handler")));
void I2C3_EV_IRQHandler (void)
    __attribute__ ((weak, alias ("Default_Handler")));
void I2C3_ER_IRQHandler (void)
    __attribute__ ((weak, alias ("Default_Handler")));
void USB_HP_IRQHandler (void)
    __attribute__ ((weak, alias ("Default_Handler")));
void USB_LP_IRQHandler (void)
    __attribute__ ((weak, alias ("Default_Handler")));
void USB_WakeUp_RMP_IRQHandler (void)
    __attribute__ ((weak, alias ("Default_Handler")));
void TIM20_BRK_IRQHandler (void)
    __attribute__ ((weak, alias ("Default_Handler")));
void TIM20_UP_IRQHandler (void)
    __attribute__ ((weak, alias ("Default_Handler")));
void TIM20_TRG_COM_IRQHandler (void)
    __attribute__ ((weak, alias ("Default_Handler")));
void TIM20_CC_IRQHandler (void)
    __attribute__ ((weak, alias ("Default_Handler")));
void FPU_IRQHandler (void)
    __attribute__ ((weak, alias ("Default_Handler")));
void SPI4_IRQHandler (void)
    __attribute__ ((weak, alias ("Default_Handler")));
void PVD_IRQHandler (void)
    __attribute__ ((weak, alias ("Default_Handler")));
void DebugMon_Handler (void)
    __attribute__ ((weak, alias ("Default_Handler")));

__attribute__ ((section (
    ".isr_vector"))) void (*const fpn_vector[]) (void)
    = { (void (*) (void)) (&_estack),
        Reset_Handler,
        NMI_Handler,
        HardFault_Handler,
        MemManage_Handler,
        BusFault_Handler,
        UsageFault_Handler,
        0,
        0,
        0,
        0,
        SVCall_Handler,
        DebugMon_Handler,
        0,
        PendSV_Handler,
        SysTick_Handler,
        WWDG_IRQHandler,
        PVD_IRQHandler,
        TAMPER_STAMP_IRQHandler,
        RTC_WKUP_IRQHandler,
        FLASH_IRQHandler,
        RCC_IRQHandler,
        EXTI0_IRQHandler,
        EXTI1_IRQHandler,
        EXTI2_TS_IRQHandler,
        EXTI3_IRQHandler,
        EXTI4_IRQHandler,
        DMA1_Channel1_IRQHandler,
        DMA1_Channel2_IRQHandler,
        DMA1_Channel3_IRQHandler,
        DMA1_Channel4_IRQHandler,
        DMA1_Channel5_IRQHandler,
        DMA1_Channel6_IRQHandler,
        DMA1_Channel7_IRQHandler,
        ADC1_2_IRQHandler,
        USB_HP_CAN_TX_IRQHandler,
        USB_LP_CAN_RX0_IRQHandler,
        CAN_RX1_IRQHandler,
        CAN_SCE_IRQHandler,
        EXTI9_5_IRQHandler,
        TIM1_BRK_TIM15_IRQHandler,
        TIM1_UP_TIM16_IRQHandler,
        TIM1_TRG_COM_TIM17_IRQHandler,
        TIM1_CC_IRQHandler,
        TIM2_IRQHandler,
        TIM3_IRQHandler,
        TIM4_IRQHandler,
        I2C1_EV_IRQHandler,
        I2C1_ER_IRQHandler,
        I2C2_EV_IRQHandler,
        I2C2_ER_IRQHandler,
        SPI1_IRQHandler,
        SPI2_IRQHandler,
        USART1_IRQHandler,
        USART2_IRQHandler,
        USART3_IRQHandler,
        EXTI15_10_IRQHandler,
        RTC_Alarm_IRQHandler,
        USBWakeUp_IRQHandler,
        TIM8_BRK_IRQHandler,
        TIM8_UP_IRQHandler,
        TIM8_TRG_COM_IRQHandler,
        TIM8_CC_IRQHandler,
        ADC3_IRQHandler,
        FMC_IRQHandler,
        0,
        0,
        SPI3_IRQHandler,
        UART4_IRQHandler,
        UART5_IRQHandler,
        TIM6_DAC_IRQHandler,
        TIM7_IRQHandler,
        DMA2_Channel1_IRQHandler,
        DMA2_Channel2_IRQHandler,
        DMA2_Channel3_IRQHandler,
        DMA2_Channel4_IRQHandler,
        DMA2_Channel5_IRQHandler,
        ADC4_IRQHandler,
        0,
        0,
        COMP1_2_3_IRQHandler,
        COMP4_5_6_IRQHandler,
        COMP7_IRQHandler,
        0,
        0,
        0,
        0,
        0,
        I2C3_EV_IRQHandler,
        I2C3_ER_IRQHandler,
        USB_HP_IRQHandler,
        USB_LP_IRQHandler,
        USB_WakeUp_RMP_IRQHandler,
        TIM20_BRK_IRQHandler,
        TIM20_UP_IRQHandler,
        TIM20_TRG_COM_IRQHandler,
        TIM20_CC_IRQHandler,
        FPU_IRQHandler,
        0,
        0,
        SPI4_IRQHandler };

void
Reset_Handler (void)
{
  /** Copy Data from FLASH to SRAM **/
  uint32_t *pSRC = (uint32_t *)&_sidata;
  uint32_t *pDST = (uint32_t *)&_sdata;

  for (uint32_t *dataptr = (uint32_t *)pDST;
       dataptr < &_edata;)
    {
      *dataptr++ = *pSRC++;
    }
  /** Initialize BSS with ZEROES **/
  for (uint32_t *bss_ptr = (uint32_t *)&_sbss;
       bss_ptr < &_ebss;)
    {
      *bss_ptr++ = 0;
    }

  /** CALL main() **/
  main ();

  for (;;)
    {
    }
}

void
Default_Handler (void)
{
  for (;;)
    ;
}