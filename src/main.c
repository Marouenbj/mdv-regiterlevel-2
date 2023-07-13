#include "../include/mdv.h"

/* Core Peripherals */
NVIC_t *const NVICC = (NVIC_t *)0xE000E100UL;
STK_t *const STK = (STK_t *)0xE000E010UL;

/* Peripherals */
FLASH_t *const FLASH = (FLASH_t *)0x40022000UL;
RCC_t *const RCC = (RCC_t *)0x40021000UL;
GPIO_t *const GPIOA = (GPIO_t *)0x48000000UL;
TIMgp_t *const TIM2 = (TIMgp_t *)0x40000000UL;
USART_t *const USART2 = (USART_t *)0x40004400UL;
ADC_t *const ADC1 = (ADC_t *)0x50000000UL;
ADCCommon_t *const ADC12_COMMON
    = (ADCCommon_t *)0x50000300UL;

DMA_t *const DMA1 = (DMA_t *)0x40020000UL;
DMA_t *const DMA2 = (DMA_t *)0x40020400UL;

static uint8_t dma_buffer[100];
static uint8_t main_buffer[100];

fsm_t fsmachine = {
  .buf.dmabuf_ptr = dma_buffer,
  .buf.mainbuf_ptr = main_buffer,
  .data.counter = 0,
};

fsm_t *const fsm = &fsmachine;

__STATIC_FORCEINLINE void
delay_ms (uint32_t ms, uint32_t sysclk_hz)
{
  for (size_t i = 0; i < ms; i++)
    {
      for (size_t j = 0; j < (sysclk_hz / 8000); j++)
        __NOP ();
    }
}

__STATIC_FORCEINLINE void
init_gpioa (GPIO_t *gpio)
{
  gpio->MODER.MODER10 = 1;
  gpio->MODER.MODER9 = 1;
  gpio->MODER.MODER8 = 1;

  // PA2 and PA3 alternate function mode for USART2
  gpio->MODER.MODER2 = 2;
  gpio->MODER.MODER3 = 2;

  gpio->OSPEEDR.OSPEEDR1 = 1;
  gpio->OSPEEDR.OSPEEDR2 = 1;
  gpio->OSPEEDR.OSPEEDR3 = 1;

  gpio->AFRL.AFR2 = 0b111;
  gpio->AFRL.AFR3 = 0b111;
}

__STATIC_FORCEINLINE void
init_usart2 (USART_t *usart)
{
  // 8n1 - 8 data, no parity, 1 stop
  usart->CR1.M0 = 0;
  usart->CR1.M1 = 0;
  usart->CR1.PCE = 0;
  usart->CR2.STOP = 0;

  // Oversample by 16
  usart->CR1.OVER8 = 0;

  // Baud-rate: 2MBps at f_clk = 72MHz 0x24UL
  // Baud-rate: 115200 at f_clk = 72MHz 0x271UL
  usart->BRR.BRR = 0x24UL;
  usart->RTOR.RTO = 2000;
  usart->CR2.RTOEN = 1;

  // Enable USART IDLE line interrupt
  usart->CR1.RXNEIE = 1;
  usart->CR1.RTOIE = 1;

  usart->CR3.DMAR = 1;

  usart->CR1.UE = 1;
  usart->CR1.TE = 1;
  usart->CR1.RE = 1;
}

__STATIC_FORCEINLINE void
init_dma1 (DMA_t *dma, USART_t *usart,
           void *const rx_buffer)
{
  // DMA1_Channel6 - USART2_RX

  dma->CMAR6.MA = (uintptr_t)rx_buffer;
  dma->CPAR6.PA = (uintptr_t)&usart->RDR.RDR;

  dma->CNDTR6.NDT = 100;

  dma->CCR6.PL = 3;
  dma->CCR6.DIR = 0;

  dma->CCR6.PSIZE = 0;
  dma->CCR6.MSIZE = 0;

  dma->CCR6.MINC = 1;
  dma->CCR6.PINC = 0;
  dma->CCR6.CIRC = 1;

  dma->CCR6.TCIE = 1;
  dma->CCR6.HTIE = 0;
  dma->CCR6.TEIE = 1;
}

__STATIC_FORCEINLINE void
start_dma1 (DMA_t *dma)
{
  dma->CCR6.EN = 1;
}

__STATIC_FORCEINLINE void
init_adc1 (ADC_t *adc, ADCCommon_t *adc_common)
{
  adc->CR.ADEN = 0;
  // Wait until end of ADC operation
  while (adc->CR.ADEN == 1)
    __NOP ();

  if (adc->CR.ADVREGEN == 2)
    {
      adc->CR.ADVREGEN = 0b00; // intermediate state
      adc->CR.ADVREGEN = 0b01; // enabled state
    }

  delay_ms (1, 72e6);

  // ADC Calibration
  adc->CR.ADEN = 0;
  // Wait until end of ADC operation
  while (adc->CR.ADEN == 1)
    __NOP ();

  adc->CR.ADCALDIF = 0;
  adc->CR.ADCAL = 1;
  while (adc->CR.ADCAL == 1)
    __NOP ();

  delay_ms (1, 72e6);

  // Enable ADC
  adc->CR.ADEN = 1;
  while (adc->ISR.ADRDY != 1)
    __NOP ();
  delay_ms (1, 72e6);

  adc->CFGR.EXTSEL = 0b1011;
  adc->CFGR.EXTEN = 1;

  // Single regular conversion
  adc->CFGR.CONT = 0;
  // ADC resolution 10-bit
  adc->CFGR.RES = 1;

  adc->CFGR.OVRMOD = 1;

  // Regular channel, one conversion at Channel 2
  adc->SQR1.L = 0;
  adc->SQR1.SQ1 = 2;

  adc->IER.EOCIE = 1;
}

__STATIC_FORCEINLINE void
init_tim2 (TIMgp_t *tim, uint16_t psc, uint32_t arr)
{
  tim->PSC.PSC = psc;
  tim->CNT = 0;
  tim->ARR = 0;
  tim->ARR = arr;

  // Clear URS & UDIS
  tim->CR1.URS = 0;
  tim->CR1.UDIS = 0;

  tim->CR1.DIR = 0;
  tim->CR2.MMS = 3;

  // Auto-reload preload enable
  tim->CR1.ARPE = 1;
  // Enable update interrupt
  tim->DIER.UIE = 1;
}

__STATIC_FORCEINLINE void
start_adc1 (ADC_t *adc)
{
  adc->CR.ADSTART = 1;
}

__STATIC_FORCEINLINE void
start_tim2 (TIMgp_t *tim)
{
  tim->CNT = 0;
  tim->CR1.CEN = 1;
}

__STATIC_FORCEINLINE void
transfer_hw (USART_t *usart, uint16_t val)
{
  adc_data tx;
  tx.hw = val;
  for (size_t i = 0; i < 2; i++)
    {
      USART2->TDR.TDR = tx.b[i];
      while (!USART2->ISR.TC)
        __NOP ();
    }

  while (!USART2->ISR.TC)
    __NOP ();
}

__STATIC_FORCEINLINE void
init_system (FLASH_t *flash, RCC_t *rcc, STK_t *stk)
{
  flash->ACR.LATENCY = 2;
  while (flash->ACR.LATENCY != 2)
    __NOP ();
  flash->ACR.PRFTBE = 1;
  while (flash->ACR.PRFTBS != 1)
    __NOP ();

  rcc->CR.PLLON = 0;
  while (rcc->CR.PLLRDY == 1)
    __NOP ();
  __COMPILER_BARRIER ();

  rcc->CFGR.PLLSRC = 1;
  rcc->CFGR.PLLMUL = 7;
  rcc->CFGR.SW = 2;
  rcc->CFGR.HPRE = 0;
  rcc->CFGR.PPRE1 = 0b100;
  rcc->CFGR.PPRE2 = 0;
  rcc->CFGR2.ADC12PRES = 0b10000;
  rcc->CFGR3.TIM2SW = 1;
  rcc->CFGR3.USART2SW = 1;

  rcc->CR.PLLON = 1;
  while (rcc->CR.PLLRDY == 0)
    __NOP ();

  stk->LOAD.RELOAD = 7200000;
  stk->VAL.CURRENT = 0;
  stk->CTRL.CLKSOURCE = 1;
  stk->CTRL.TICKINT = 1;
  stk->CTRL.ENABLE = 0;

  rcc->APB1ENR.PWREN = 1;
  rcc->APB2ENR.SYSCFGEN = 1;
  rcc->AHBENR.IOPAEN = 1;
  rcc->APB1ENR.USART2EN = 1;
  rcc->AHBENR.DMA1EN = 1;
  rcc->AHBENR.ADC12EN = 1;
  rcc->APB1ENR.TIM2EN = 1;
}

int
main (void)
{
  init_system (FLASH, RCC, STK);
  __COMPILER_BARRIER ();

  init_gpioa (GPIOA);
  init_dma1 (DMA1, USART2, dma_buffer);
  start_dma1 (DMA1);
  init_usart2 (USART2);
  init_adc1 (ADC1, ADC12_COMMON);

  NVICC->ISER.SETENA16 = 1;
  NVICC->ISER.SETENA17 = 1;
  NVICC->ISER.SETENA18 = 1;
  NVICC->ISER.SETENA28 = 1;
  NVICC->ISER.SETENA38 = 1;
  __DSB ();

  LED_YGR (0, 0, 1);

  for (;;)
    {
      __WFI ();
      if (fsm->state.rto_detected)
        {
          fsm->state.rto_detected = false;
          __disable_irq ();
          memcpy (fsm->buf.mainbuf_ptr,
                  fsm->buf.dmabuf_ptr, 100);
          memset (fsm->buf.dmabuf_ptr, 0,
                  sizeof (uint8_t) * 100);
          command_check (fsm);
          LED_YGR (0, 0, 0);
          __enable_irq ();

          if (fsm->state.cmd_valid)
            {
              LED_GREEN = 1;
              __disable_irq ();
              USART2->CR2.RTOEN = 0;
              init_tim2 (TIM2, fsm->cmd.psc, fsm->cmd.arr);
              start_adc1 (ADC1);
              __enable_irq ();
              start_tim2 (TIM2);
              __DSB ();
              break;
            }
        }
    }

  for (;;)
    {
      if (fsm->state.sampling_instant)
        {
          fsm->state.sampling_instant = false;
          transfer_hw (USART2, fsm->data.adc_k);
          fsm->data.counter++;
          if (fsm->data.counter == fsm->cmd.count)
            fsm->state.reset = true;
        }

      if (fsm->state.reset)
        __NVIC_SystemReset ();
    }
}
