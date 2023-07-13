#ifndef PERIPH_H
#define PERIPH_H

#include "stdint.h"

typedef struct GPIO_t
{
  struct
  {
    volatile uint32_t MODER0 : 2;
    volatile uint32_t MODER1 : 2;
    volatile uint32_t MODER2 : 2;
    volatile uint32_t MODER3 : 2;
    volatile uint32_t MODER4 : 2;
    volatile uint32_t MODER5 : 2;
    volatile uint32_t MODER6 : 2;
    volatile uint32_t MODER7 : 2;
    volatile uint32_t MODER8 : 2;
    volatile uint32_t MODER9 : 2;
    volatile uint32_t MODER10 : 2;
    volatile uint32_t MODER11 : 2;
    volatile uint32_t MODER12 : 2;
    volatile uint32_t MODER13 : 2;
    volatile uint32_t MODER14 : 2;
    volatile uint32_t MODER15 : 2;
  } MODER;

  struct
  {
    volatile uint32_t OT0 : 1;
    volatile uint32_t OT1 : 1;
    volatile uint32_t OT2 : 1;
    volatile uint32_t OT3 : 1;
    volatile uint32_t OT4 : 1;
    volatile uint32_t OT5 : 1;
    volatile uint32_t OT6 : 1;
    volatile uint32_t OT7 : 1;
    volatile uint32_t OT8 : 1;
    volatile uint32_t OT9 : 1;
    volatile uint32_t OT10 : 1;
    volatile uint32_t OT11 : 1;
    volatile uint32_t OT12 : 1;
    volatile uint32_t OT13 : 1;
    volatile uint32_t OT14 : 1;
    volatile uint32_t OT15 : 1;
    const uint32_t res : 16;
  } OTYPER;
  struct
  {
    volatile uint32_t OSPEEDR0 : 2;
    volatile uint32_t OSPEEDR1 : 2;
    volatile uint32_t OSPEEDR2 : 2;
    volatile uint32_t OSPEEDR3 : 2;
    volatile uint32_t OSPEEDR4 : 2;
    volatile uint32_t OSPEEDR5 : 2;
    volatile uint32_t OSPEEDR6 : 2;
    volatile uint32_t OSPEEDR7 : 2;
    volatile uint32_t OSPEEDR8 : 2;
    volatile uint32_t OSPEEDR9 : 2;
    volatile uint32_t OSPEEDR10 : 2;
    volatile uint32_t OSPEEDR11 : 2;
    volatile uint32_t OSPEEDR12 : 2;
    volatile uint32_t OSPEEDR13 : 2;
    volatile uint32_t OSPEEDR14 : 2;
    volatile uint32_t OSPEEDR15 : 2;
  } OSPEEDR;

  struct
  {
    volatile uint32_t PUPDR0 : 2;
    volatile uint32_t PUPDR1 : 2;
    volatile uint32_t PUPDR2 : 2;
    volatile uint32_t PUPDR3 : 2;
    volatile uint32_t PUPDR4 : 2;
    volatile uint32_t PUPDR5 : 2;
    volatile uint32_t PUPDR6 : 2;
    volatile uint32_t PUPDR7 : 2;
    volatile uint32_t PUPDR8 : 2;
    volatile uint32_t PUPDR9 : 2;
    volatile uint32_t PUPDR10 : 2;
    volatile uint32_t PUPDR11 : 2;
    volatile uint32_t PUPDR12 : 2;
    volatile uint32_t PUPDR13 : 2;
    volatile uint32_t PUPDR14 : 2;
    volatile uint32_t PUPDR15 : 2;
  } PUPDR;

  struct
  {
    volatile uint32_t IDR0 : 1;
    volatile uint32_t IDR1 : 1;
    volatile uint32_t IDR2 : 1;
    volatile uint32_t IDR3 : 1;
    volatile uint32_t IDR4 : 1;
    volatile uint32_t IDR5 : 1;
    volatile uint32_t IDR6 : 1;
    volatile uint32_t IDR7 : 1;
    volatile uint32_t IDR8 : 1;
    volatile uint32_t IDR9 : 1;
    volatile uint32_t IDR10 : 1;
    volatile uint32_t IDR11 : 1;
    volatile uint32_t IDR12 : 1;
    volatile uint32_t IDR13 : 1;
    volatile uint32_t IDR14 : 1;
    volatile uint32_t IDR15 : 1;
    const uint32_t res : 16;
  } IDR;

  struct
  {
    volatile uint32_t ODR0 : 1;
    volatile uint32_t ODR1 : 1;
    volatile uint32_t ODR2 : 1;
    volatile uint32_t ODR3 : 1;
    volatile uint32_t ODR4 : 1;
    volatile uint32_t ODR5 : 1;
    volatile uint32_t ODR6 : 1;
    volatile uint32_t ODR7 : 1;
    volatile uint32_t ODR8 : 1;
    volatile uint32_t ODR9 : 1;
    volatile uint32_t ODR10 : 1;
    volatile uint32_t ODR11 : 1;
    volatile uint32_t ODR12 : 1;
    volatile uint32_t ODR13 : 1;
    volatile uint32_t ODR14 : 1;
    volatile uint32_t ODR15 : 1;
    const uint32_t res : 16;
  } ODR;

  struct
  {
    volatile uint32_t BS0 : 1;
    volatile uint32_t BS1 : 1;
    volatile uint32_t BS2 : 1;
    volatile uint32_t BS3 : 1;
    volatile uint32_t BS4 : 1;
    volatile uint32_t BS5 : 1;
    volatile uint32_t BS6 : 1;
    volatile uint32_t BS7 : 1;
    volatile uint32_t BS8 : 1;
    volatile uint32_t BS9 : 1;
    volatile uint32_t BS10 : 1;
    volatile uint32_t BS11 : 1;
    volatile uint32_t BS12 : 1;
    volatile uint32_t BS13 : 1;
    volatile uint32_t BS14 : 1;
    volatile uint32_t BS15 : 1;
    volatile uint32_t BR0 : 1;
    volatile uint32_t BR1 : 1;
    volatile uint32_t BR2 : 1;
    volatile uint32_t BR3 : 1;
    volatile uint32_t BR4 : 1;
    volatile uint32_t BR5 : 1;
    volatile uint32_t BR6 : 1;
    volatile uint32_t BR7 : 1;
    volatile uint32_t BR8 : 1;
    volatile uint32_t BR9 : 1;
    volatile uint32_t BR10 : 1;
    volatile uint32_t BR11 : 1;
    volatile uint32_t BR12 : 1;
    volatile uint32_t BR13 : 1;
    volatile uint32_t BR14 : 1;
    volatile uint32_t BR15 : 1;
  } BSRR;

  struct
  {
    volatile uint32_t LCK0 : 1;
    volatile uint32_t LCK1 : 1;
    volatile uint32_t LCK2 : 1;
    volatile uint32_t LCK3 : 1;
    volatile uint32_t LCK4 : 1;
    volatile uint32_t LCK5 : 1;
    volatile uint32_t LCK6 : 1;
    volatile uint32_t LCK7 : 1;
    volatile uint32_t LCK8 : 1;
    volatile uint32_t LCK9 : 1;
    volatile uint32_t LCK10 : 1;
    volatile uint32_t LCK11 : 1;
    volatile uint32_t LCK12 : 1;
    volatile uint32_t LCK13 : 1;
    volatile uint32_t LCK14 : 1;
    volatile uint32_t LCK15 : 1;
    volatile uint32_t LCKK : 1;
    const uint32_t res : 15;
  } LCKR;

  struct
  {
    volatile uint32_t AFR0 : 4;
    volatile uint32_t AFR1 : 4;
    volatile uint32_t AFR2 : 4;
    volatile uint32_t AFR3 : 4;
    volatile uint32_t AFR4 : 4;
    volatile uint32_t AFR5 : 4;
    volatile uint32_t AFR6 : 4;
    volatile uint32_t AFR7 : 4;
  } AFRL;

  struct
  {
    volatile uint32_t AFR8 : 4;
    volatile uint32_t AFR9 : 4;
    volatile uint32_t AFR10 : 4;
    volatile uint32_t AFR11 : 4;
    volatile uint32_t AFR12 : 4;
    volatile uint32_t AFR13 : 4;
    volatile uint32_t AFR14 : 4;
    volatile uint32_t AFR15 : 4;
  } AFRH;

  struct
  {
    volatile uint32_t BR0 : 1;
    volatile uint32_t BR1 : 1;
    volatile uint32_t BR2 : 1;
    volatile uint32_t BR3 : 1;
    volatile uint32_t BR4 : 1;
    volatile uint32_t BR5 : 1;
    volatile uint32_t BR6 : 1;
    volatile uint32_t BR7 : 1;
    volatile uint32_t BR8 : 1;
    volatile uint32_t BR9 : 1;
    volatile uint32_t BR10 : 1;
    volatile uint32_t BR11 : 1;
    volatile uint32_t BR12 : 1;
    volatile uint32_t BR13 : 1;
    volatile uint32_t BR14 : 1;
    volatile uint32_t BR15 : 1;
    const uint32_t res : 16;
  } BRR;
} GPIO_t;

typedef struct TIMgp_t
{
  struct
  {
    volatile uint32_t CEN : 1;
    volatile uint32_t UDIS : 1;
    volatile uint32_t URS : 1;
    volatile uint32_t OPM : 1;
    volatile uint32_t DIR : 1;
    volatile uint32_t CMS : 2;
    volatile uint32_t ARPE : 1;
    volatile uint32_t CKD : 2;
    const uint32_t res : 1;
    volatile uint32_t UIFREMAP : 1;
    const uint32_t res2 : 20;
  } CR1;

  struct
  {
    const uint32_t res : 3;
    volatile uint32_t CCDS : 1;
    volatile uint32_t MMS : 3;
    volatile uint32_t TI1S : 1;
    const uint32_t res2 : 24;
  } CR2;

  struct
  {
    volatile uint32_t SMS : 3;
    volatile uint32_t OCCS : 1;
    volatile uint32_t TS : 3;
    volatile uint32_t MSM : 1;
    volatile uint32_t ETF : 4;
    volatile uint32_t ETPS : 2;
    volatile uint32_t ECE : 1;
    volatile uint32_t ETP : 1;
    volatile uint32_t SMS3 : 1;
    const uint32_t res : 15;
  } SMCR;

  struct
  {
    volatile uint32_t UIE : 1;
    volatile uint32_t CC1IE : 1;
    volatile uint32_t CC2IE : 1;
    volatile uint32_t CC3IE : 1;
    volatile uint32_t CC4IE : 1;
    const uint32_t res : 1;
    volatile uint32_t TIE : 1;
    const uint32_t res2 : 1;
    volatile uint32_t UDE : 1;
    volatile uint32_t CC1DE : 1;
    volatile uint32_t CC2DE : 1;
    volatile uint32_t CC3DE : 1;
    volatile uint32_t CC4DE : 1;
    const uint32_t res3 : 1;
    volatile uint32_t TDE : 1;
    const uint32_t res4 : 17;
  } DIER;

  struct
  {
    volatile uint32_t UIF : 1;
    volatile uint32_t CC1IF : 1;
    volatile uint32_t CC2IF : 1;
    volatile uint32_t CC3IF : 1;
    volatile uint32_t CC4IF : 1;
    const uint32_t res : 1;
    volatile uint32_t TIF : 1;
    const uint32_t res2 : 1;
    volatile uint32_t UDF : 1;
    volatile uint32_t CC1OF : 1;
    volatile uint32_t CC2OF : 1;
    volatile uint32_t CC3OF : 1;
    volatile uint32_t CC4OF : 1;
    const uint32_t res3 : 19;
  } SR;

  struct
  {
    volatile uint32_t UG : 1;
    volatile uint32_t CC1G : 1;
    volatile uint32_t CC2G : 1;
    volatile uint32_t CC3G : 1;
    volatile uint32_t CC4G : 1;
    const uint32_t res : 1;
    volatile uint32_t TG : 1;
    const uint32_t res2 : 25;
  } EGR;

  struct
  {
    union
    {
      struct
      {
        volatile uint32_t CC1S : 2;
        volatile uint32_t OC1FE : 1;
        volatile uint32_t OC1PE : 1;
        volatile uint32_t OC1M : 3;
        volatile uint32_t OC1CE : 1;
        volatile uint32_t CC2S : 2;
        volatile uint32_t OC2FE : 1;
        volatile uint32_t OC2PE : 1;
        volatile uint32_t OC2M : 3;
        volatile uint32_t OC2CE : 1;
        volatile uint32_t OC1M_3 : 1;
        const uint32_t res : 7;
        volatile uint32_t OC2M_3 : 1;
        const uint32_t res2 : 7;
      } OC;
      struct
      {
        volatile uint32_t CC1S : 2;
        volatile uint32_t IC1PSC : 2;
        volatile uint32_t IC1F : 4;
        volatile uint32_t CC2S : 2;
        volatile uint32_t IC2PSC : 2;
        volatile uint32_t IC2F : 4;
        const uint32_t res : 16;
      } IC;
    };
  } CCMR1;

  struct
  {
    union
    {
      struct
      {
        volatile uint32_t CC3S : 2;
        volatile uint32_t OC3FE : 1;
        volatile uint32_t OC3PE : 1;
        volatile uint32_t OC3M : 3;
        volatile uint32_t OC3CE : 1;
        volatile uint32_t CC4S : 2;
        volatile uint32_t OC4FE : 1;
        volatile uint32_t OC4PE : 1;
        volatile uint32_t OC4M : 3;
        volatile uint32_t OC4CE : 1;
        volatile uint32_t OC3M_3 : 1;
        const uint32_t res : 7;
        volatile uint32_t OC4M_3 : 1;
        const uint32_t res2 : 7;
      } OC;
      struct
      {
        volatile uint32_t CC3S : 2;
        volatile uint32_t IC3PSC : 2;
        volatile uint32_t IC3F : 4;
        volatile uint32_t CC4S : 2;
        volatile uint32_t IC4PSC : 2;
        volatile uint32_t IC4F : 4;
        const uint32_t res : 16;
      } IC;
    };
  } CCMR2;

  struct
  {
    volatile uint32_t CC1E : 1;
    volatile uint32_t CC1P : 1;
    const uint32_t res : 1;
    volatile uint32_t CC1NP : 1;
    volatile uint32_t CC2E : 1;
    volatile uint32_t CC2P : 1;
    const uint32_t res2 : 1;
    volatile uint32_t CC2NP : 1;
    volatile uint32_t CC3E : 1;
    volatile uint32_t CC3P : 1;
    const uint32_t res3 : 1;
    volatile uint32_t CC3NP : 1;
    volatile uint32_t CC4E : 1;
    volatile uint32_t CC4P : 1;
    const uint32_t res4 : 1;
    volatile uint32_t CC4NP : 1;
  } CCER;

  volatile uint32_t CNT;

  struct
  {
    volatile uint32_t PSC : 16;
    const uint32_t res : 16;
  } PSC;

  volatile uint32_t ARR;

  const uint32_t res1;

  volatile uint32_t CCR1;
  volatile uint32_t CCR2;
  volatile uint32_t CCR3;
  volatile uint32_t CCR4;

  const uint32_t res2;

  struct
  {
    volatile uint32_t DBA : 5;
    const uint32_t res : 3;
    volatile uint32_t DBL : 5;
    const uint32_t res2 : 19;
  } DCR;

  struct
  {
    volatile uint32_t DMAB : 16;
    const uint32_t res : 16;
  } DMAR;
} TIMgp_t; // General-purpose timers

typedef struct RCC_t
{
  struct
  {
    volatile uint32_t HSION : 1;
    volatile uint32_t HSIRDY : 1;
    const uint32_t res : 1;
    volatile uint32_t HSITRIM : 5;
    volatile uint32_t HSICAL : 8;
    volatile uint32_t HSEON : 1;
    volatile uint32_t HSERDY : 1;
    volatile uint32_t HSEBYP : 1;
    volatile uint32_t CSSON : 1;
    const uint32_t res2 : 4;
    volatile uint32_t PLLON : 1;
    volatile uint32_t PLLRDY : 1;
    const uint32_t res3 : 6;
  } CR;

  struct
  {
    volatile uint32_t SW : 2;
    volatile uint32_t SWS : 2;
    volatile uint32_t HPRE : 4;
    volatile uint32_t PPRE1 : 3;
    volatile uint32_t PPRE2 : 3;
    const uint32_t res : 1;
    volatile uint32_t PLLSRC : 2;
    volatile uint32_t PLLXTPRE : 1;
    volatile uint32_t PLLMUL : 4;
    volatile uint32_t USBPRE : 1;
    volatile uint32_t I2SSRC : 1;
    volatile uint32_t MCO : 3;
    const uint32_t res2 : 1;
    volatile uint32_t MCOF_MCOPRE0 : 1;
    volatile uint32_t MCOPRE : 2;
    volatile uint32_t PLLNODIV : 1;
  } CFGR;

  struct
  {
    volatile uint32_t LSIRDYF : 1;
    volatile uint32_t LSERDYF : 1;
    volatile uint32_t HSIRDYF : 1;
    volatile uint32_t HSERDYF : 1;
    volatile uint32_t PLLRDYF : 1;
    const uint32_t res : 2;
    volatile uint32_t CSSF : 1;
    volatile uint32_t LSIRDYIE : 1;
    volatile uint32_t LSERDYIE : 1;
    volatile uint32_t HSIRDYIE : 1;
    volatile uint32_t HSERDYIE : 1;
    volatile uint32_t PLLRDYIE : 1;
    const uint32_t res2 : 3;
    volatile uint32_t LSIRDYC : 1;
    volatile uint32_t LSERDYC : 1;
    volatile uint32_t HSIRDYC : 1;
    volatile uint32_t HSERDYC : 1;
    volatile uint32_t PLLRDYC : 1;
    const uint32_t res3 : 2;
    volatile uint32_t CSSC : 1;
    const uint32_t res4 : 8;
  } CIR;

  struct
  {
    volatile uint32_t SYSCFGRST : 1;
    const uint32_t res : 10;
    volatile uint32_t TIM1RST : 1;
    volatile uint32_t SPI1RST : 1;
    volatile uint32_t TIM8RST : 1;
    volatile uint32_t USART1RST : 1;
    volatile uint32_t SPI4RST : 1;
    volatile uint32_t TIM15RST : 1;
    volatile uint32_t TIM16RST : 1;
    volatile uint32_t TIM17RST : 1;
    const uint32_t res2 : 1;
    volatile uint32_t TIM20RST : 1;
    const uint32_t res3 : 11;
  } APB2RSTR;

  struct
  {
    volatile uint32_t TIM2RST : 1;
    volatile uint32_t TIM3RST : 1;
    volatile uint32_t TIM4RST : 1;
    const uint32_t res : 1;
    volatile uint32_t TIM6RST : 1;
    volatile uint32_t TIM7RST : 1;
    const uint32_t res2 : 5;
    volatile uint32_t WWDGRST : 1;
    const uint32_t res3 : 2;
    volatile uint32_t SPI2RST : 1;
    volatile uint32_t SPI3RST : 1;
    const uint32_t res4 : 1;
    volatile uint32_t USART2RST : 1;
    volatile uint32_t USART3RST : 1;
    volatile uint32_t USART4RST : 1;
    volatile uint32_t USART5RST : 1;
    volatile uint32_t I2C1RST : 1;
    volatile uint32_t I2C2RST : 1;
    volatile uint32_t USBRST : 1;
    const uint32_t res5 : 1;
    volatile uint32_t CANRST : 1;
    volatile uint32_t DAC2RST : 1;
    const uint32_t res6 : 1;
    volatile uint32_t PWRRST : 1;
    volatile uint32_t DAC1RST : 1;
    volatile uint32_t I2C3RST : 1;
    const uint32_t res7 : 1;
  } APB1RSTR;

  struct
  {
    volatile uint32_t DMA1EN : 1;
    volatile uint32_t DMA2EN : 1;
    volatile uint32_t SRAMEN : 1;
    const uint32_t res : 1;
    volatile uint32_t FLITFEN : 1;
    volatile uint32_t FMCEN : 1;
    volatile uint32_t CRCEN : 1;
    const uint32_t res2 : 9;
    volatile uint32_t IOPHEN : 1;
    volatile uint32_t IOPAEN : 1;
    volatile uint32_t IOPBEN : 1;
    volatile uint32_t IOPCEN : 1;
    volatile uint32_t IOPDEN : 1;
    volatile uint32_t IOPEEN : 1;
    volatile uint32_t IOPFEN : 1;
    volatile uint32_t IOPGEN : 1;
    volatile uint32_t TSCEN : 1;
    const uint32_t res3 : 3;
    volatile uint32_t ADC12EN : 1;
    volatile uint32_t ADC34EN : 1;
    const uint32_t res4 : 2;
  } AHBENR;

  struct
  {
    volatile uint32_t SYSCFGEN : 1;
    const uint32_t res : 10;
    volatile uint32_t TIM1EN : 1;
    volatile uint32_t SPI1EN : 1;
    volatile uint32_t TIM8EN : 1;
    volatile uint32_t USART1EN : 1;
    volatile uint32_t SPI4EN : 1;
    volatile uint32_t TIM15EN : 1;
    volatile uint32_t TIM16EN : 1;
    volatile uint32_t TIM17EN : 1;
    const uint32_t res2 : 1;
    volatile uint32_t TIM20EN : 1;
    const uint32_t res3 : 11;
  } APB2ENR;

  struct
  {
    volatile uint32_t TIM2EN : 1;
    volatile uint32_t TIM3EN : 1;
    volatile uint32_t TIM4EN : 1;
    const uint32_t res : 1;
    volatile uint32_t TIM6EN : 1;
    volatile uint32_t TIM7EN : 1;
    const uint32_t res2 : 5;
    volatile uint32_t WWDGEN : 1;
    const uint32_t res3 : 2;
    volatile uint32_t SPI2EN : 1;
    volatile uint32_t SPI3EN : 1;
    const uint32_t res4 : 1;
    volatile uint32_t USART2EN : 1;
    volatile uint32_t USART3EN : 1;
    volatile uint32_t UART4EN : 1;
    volatile uint32_t UART5EN : 1;
    volatile uint32_t I2C1EN : 1;
    volatile uint32_t I2C2EN : 1;
    volatile uint32_t USBEN : 1;
    const uint32_t res5 : 1;
    volatile uint32_t CANEN : 1;
    volatile uint32_t DAC2EN : 1;
    const uint32_t res6 : 1;
    volatile uint32_t PWREN : 1;
    volatile uint32_t DAC1EN : 1;
    volatile uint32_t I2C3EN : 1;
    const uint32_t res7 : 1;
  } APB1ENR;

  struct
  {
    volatile uint32_t LSEON : 1;
    volatile uint32_t LSERDY : 1;
    volatile uint32_t LSEBYP : 1;
    volatile uint32_t LSEDRV : 2;
    const uint32_t res : 3;
    volatile uint32_t RTCSEL : 2;
    const uint32_t res2 : 5;
    volatile uint32_t RTCEN : 1;
    volatile uint32_t BDRST : 1;
    const uint32_t res3 : 15;
  } BDCR;
  struct
  {
    volatile uint32_t LSION : 1;
    volatile uint32_t LSIRDY : 1;
    const uint32_t res : 21;
    volatile uint32_t V18PWRRSTF : 1;
    volatile uint32_t RMVF : 1;
    volatile uint32_t OBLRSTF : 1;
    volatile uint32_t PINRSTF : 1;
    volatile uint32_t PORRSTF : 1;
    volatile uint32_t SFTRSTF : 1;
    volatile uint32_t IWWDGRSTF : 1;
    volatile uint32_t WWDGRSTF : 1;
    volatile uint32_t LPWRRSTF : 1;
  } CSR;

  struct
  {
    const uint32_t res : 5;
    volatile uint32_t FMCRST : 1;
    const uint32_t res2 : 10;
    volatile uint32_t IOPHRST : 1;
    volatile uint32_t IOPARST : 1;
    volatile uint32_t IOPBRST : 1;
    volatile uint32_t IOPCRST : 1;
    volatile uint32_t IOPDRST : 1;
    volatile uint32_t IOPERST : 1;
    volatile uint32_t IOPFRST : 1;
    volatile uint32_t IOPGRST : 1;
    volatile uint32_t TSCRST : 1;
    const uint32_t res3 : 3;
    volatile uint32_t ADC12RST : 1;
    volatile uint32_t ADC34RST : 1;
    const uint32_t res4 : 2;
  } AHBRSTR;

  struct
  {
    volatile uint32_t PREDIV : 4;
    volatile uint32_t ADC12PRES : 5;
    volatile uint32_t ADC34PRES : 5;
    const uint32_t res : 18;
  } CFGR2;
  struct
  {
    volatile uint32_t USART1SW : 2;
    const uint32_t res : 2;
    volatile uint32_t I2C1 : 1;
    volatile uint32_t I2C2 : 1;
    volatile uint32_t I2C3 : 1;
    const uint32_t res2 : 1;
    volatile uint32_t TIM1SW : 1;
    volatile uint32_t TIM8SW : 1;
    volatile uint32_t TIM15SW : 1;
    volatile uint32_t TIM16SW : 1;
    const uint32_t res3 : 1;
    volatile uint32_t TIM17SW : 1;
    const uint32_t res4 : 1;
    volatile uint32_t TIM20SW : 1;
    volatile uint32_t USART2SW : 2;
    volatile uint32_t USART3SW : 2;
    volatile uint32_t UART4SW : 2;
    volatile uint32_t UART5SW : 2;
    volatile uint32_t TIM2SW : 1;
    volatile uint32_t TIM34SW : 1;
    const uint32_t res5 : 6;
  } CFGR3;
} RCC_t;

typedef struct FLASH_t
{
  struct
  {
    volatile uint32_t LATENCY : 3;
    volatile uint32_t HLFCYA : 1;
    volatile uint32_t PRFTBE : 1;
    volatile uint32_t PRFTBS : 1;
    volatile uint32_t res : 27;
  } ACR;
  struct
  {
    volatile uint32_t FKEYR;
  } KEYR;

  struct
  {
    volatile uint32_t OPTKEYR;
  } OPTKEYR;
  struct
  {
    volatile uint32_t BSY : 1;
    volatile uint32_t res : 1;
    volatile uint32_t PGERR : 1;
    volatile uint32_t res2 : 1;
    volatile uint32_t WRPRTERR : 1;
    volatile uint32_t EOP : 1;
    volatile uint32_t res3 : 26;
  } SR;
  struct
  {
    volatile uint32_t PG : 1;
    volatile uint32_t PER : 1;
    volatile uint32_t MER : 1;
    volatile uint32_t res : 1;
    volatile uint32_t OPTPG : 1;
    volatile uint32_t OPTER : 1;
    volatile uint32_t STRT : 1;
    volatile uint32_t LOCK : 1;
    volatile uint32_t res2 : 1;
    volatile uint32_t OPTWRE : 1;
    volatile uint32_t ERRIE : 1;
    volatile uint32_t res3 : 1;
    volatile uint32_t EOPIE : 1;
    volatile uint32_t OBL_LAUNCH : 1;
    volatile uint32_t res4 : 18;
  } CR;
  volatile uint32_t AR;
  struct
  {
    volatile uint32_t OPTERR : 1;
    volatile uint32_t RDPRT : 2;
    volatile uint32_t res : 5;
    volatile uint32_t WDG_SW : 1;
    volatile uint32_t nRST_STOP : 1;
    volatile uint32_t nRST_STDBY : 1;
    volatile uint32_t res2 : 1;
    volatile uint32_t nBOOT1 : 1;
    volatile uint32_t VDDA_MONITOR : 1;
    volatile uint32_t SRAM_PE : 1;
    volatile uint32_t res3 : 1;
    volatile uint32_t Data0 : 8;
    volatile uint32_t Data1 : 8;
  } OBR;
  volatile uint32_t WRPR;
} FLASH_t;

typedef struct ADC_t
{
  struct
  {
    volatile uint32_t ADRDY : 1;
    volatile uint32_t EOSMP : 1;
    volatile uint32_t EOC : 1;
    volatile uint32_t EOS : 1;
    volatile uint32_t OVR : 1;
    volatile uint32_t JEOC : 1;
    volatile uint32_t JEOS : 1;
    volatile uint32_t AWD1 : 1;
    volatile uint32_t AWD2 : 1;
    volatile uint32_t AWD3 : 1;
    volatile uint32_t JQOVF : 1;
    volatile uint32_t res : 21;
  } ISR;
  struct
  {
    volatile uint32_t ADRDYIE : 1;
    volatile uint32_t EOSMPIE : 1;
    volatile uint32_t EOCIE : 1;
    volatile uint32_t EOSIE : 1;
    volatile uint32_t OVRIE : 1;
    volatile uint32_t JEOCIE : 1;
    volatile uint32_t JEOSIE : 1;
    volatile uint32_t AWD1IE : 1;
    volatile uint32_t AWD2IE : 1;
    volatile uint32_t AWD3IE : 1;
    volatile uint32_t JQOVFIE : 1;
    volatile uint32_t res : 21;
  } IER;
  struct
  {
    volatile uint32_t ADEN : 1;
    volatile uint32_t ADDIS : 1;
    volatile uint32_t ADSTART : 1;
    volatile uint32_t JADSTART : 1;
    volatile uint32_t ADSTP : 1;
    volatile uint32_t JADSTP : 1;
    volatile uint32_t res : 22;
    volatile uint32_t ADVREGEN : 2;
    volatile uint32_t ADCALDIF : 1;
    volatile uint32_t ADCAL : 1;
  } CR;
  struct
  {
    volatile uint32_t DMAEN : 1;
    volatile uint32_t DMACFG : 1;
    volatile uint32_t res : 1;
    volatile uint32_t RES : 2;
    volatile uint32_t ALIGN : 1;
    volatile uint32_t EXTSEL : 4;
    volatile uint32_t EXTEN : 2;
    volatile uint32_t OVRMOD : 1;
    volatile uint32_t CONT : 1;
    volatile uint32_t AUTDLY : 1;
    volatile uint32_t res2 : 1;
    volatile uint32_t DISCEN : 1;
    volatile uint32_t DISCNUM : 3;
    volatile uint32_t JDISCEN : 1;
    volatile uint32_t JQM : 1;
    volatile uint32_t AWD1SGL : 1;
    volatile uint32_t AWD1EN : 1;
    volatile uint32_t JAWD1EN : 1;
    volatile uint32_t JAUTO : 1;
    volatile uint32_t AWD1CH : 5;
    volatile uint32_t res3 : 1;
  } CFGR;
  const uint32_t res;
  struct
  {
    volatile uint32_t res : 3;
    volatile uint32_t SMP1 : 3;
    volatile uint32_t SMP2 : 3;
    volatile uint32_t SMP3 : 3;
    volatile uint32_t SMP4 : 3;
    volatile uint32_t SMP5 : 3;
    volatile uint32_t SMP6 : 3;
    volatile uint32_t SMP7 : 3;
    volatile uint32_t SMP8 : 3;
    volatile uint32_t SMP9 : 3;
    volatile uint32_t res2 : 2;
  } SMPR1;
  struct
  {
    volatile uint32_t SMP10 : 3;
    volatile uint32_t SMP11 : 3;
    volatile uint32_t SMP12 : 3;
    volatile uint32_t SMP13 : 3;
    volatile uint32_t SMP14 : 3;
    volatile uint32_t SMP15 : 3;
    volatile uint32_t SMP16 : 3;
    volatile uint32_t SMP17 : 3;
    volatile uint32_t SMP18 : 3;
    volatile uint32_t res : 5;
  } SMPR2;
  const uint32_t res2;
  struct
  {
    volatile uint32_t LT1 : 12;
    volatile uint32_t res : 4;
    volatile uint32_t HT1 : 12;
    volatile uint32_t res2 : 4;
  } TR1;
  struct
  {
    volatile uint32_t LT2 : 8;
    volatile uint32_t res : 8;
    volatile uint32_t HT2 : 8;
    volatile uint32_t res2 : 8;
  } TR2;
  struct
  {
    volatile uint32_t LT3 : 8;
    volatile uint32_t res : 8;
    volatile uint32_t HT3 : 8;
    volatile uint32_t res2 : 8;
  } TR3;
  const uint32_t res3;
  struct
  {
    volatile uint32_t L : 4;
    volatile uint32_t res : 2;
    volatile uint32_t SQ1 : 5;
    volatile uint32_t res2 : 1;
    volatile uint32_t SQ2 : 5;
    volatile uint32_t res3 : 1;
    volatile uint32_t SQ3 : 5;
    volatile uint32_t res4 : 1;
    volatile uint32_t SQ4 : 5;
    volatile uint32_t res5 : 3;
  } SQR1;
  struct
  {
    volatile uint32_t SQ5 : 5;
    volatile uint32_t res : 1;
    volatile uint32_t SQ6 : 5;
    volatile uint32_t res2 : 1;
    volatile uint32_t SQ7 : 5;
    volatile uint32_t res3 : 1;
    volatile uint32_t SQ8 : 5;
    volatile uint32_t res4 : 1;
    volatile uint32_t SQ9 : 5;
    volatile uint32_t res5 : 3;
  } SQR2;
  struct
  {
    volatile uint32_t SQ10 : 5;
    volatile uint32_t res : 1;
    volatile uint32_t SQ11 : 5;
    volatile uint32_t res2 : 1;
    volatile uint32_t SQ12 : 5;
    volatile uint32_t res3 : 1;
    volatile uint32_t SQ13 : 5;
    volatile uint32_t res4 : 1;
    volatile uint32_t SQ14 : 5;
    volatile uint32_t res5 : 3;
  } SQR3;
  struct
  {
    volatile uint32_t SQ15 : 5;
    volatile uint32_t res : 1;
    volatile uint32_t SQ16 : 5;
    volatile uint32_t res2 : 21;
  } SQR4;
  struct
  {
    volatile uint16_t RDATA;
    volatile uint16_t res;
  } DR;
  const uint32_t res4[2];
  struct
  {
    volatile uint32_t JL : 2;
    volatile uint32_t JEXTSEL : 4;
    volatile uint32_t JEXTEN : 2;
    volatile uint32_t JSQ1 : 5;
    volatile uint32_t res : 1;
    volatile uint32_t JSQ2 : 5;
    volatile uint32_t res2 : 1;
    volatile uint32_t JSQ3 : 5;
    volatile uint32_t res3 : 1;
    volatile uint32_t JSQ4 : 5;
    volatile uint32_t res4 : 1;
  } JSQR;
  const uint32_t res5[4];
  struct
  {
    volatile uint32_t OFFSET1 : 12;
    volatile uint32_t res : 14;
    volatile uint32_t OFFSET1_CH : 5;
    volatile uint32_t OFFSET1_EN : 1;
  } OFR1;
  struct
  {
    volatile uint32_t OFFSET2 : 12;
    volatile uint32_t res : 14;
    volatile uint32_t OFFSET2_CH : 5;
    volatile uint32_t OFFSET2_EN : 1;
  } OFR2;
  struct
  {
    volatile uint32_t OFFSET3 : 12;
    volatile uint32_t res : 14;
    volatile uint32_t OFFSET3_CH : 5;
    volatile uint32_t OFFSET3_EN : 1;
  } OFR3;
  struct
  {
    volatile uint32_t OFFSET4 : 12;
    volatile uint32_t res : 14;
    volatile uint32_t OFFSET4_CH : 5;
    volatile uint32_t OFFSET4_EN : 1;
  } OFR4;
  const uint32_t res6[4];
  struct
  {
    volatile uint32_t JDATA1 : 16;
    volatile uint32_t res : 16;
  } JDR1;
  struct
  {
    volatile uint32_t JDATA2 : 16;
    volatile uint32_t res : 16;
  } JDR2;
  struct
  {
    volatile uint32_t JDATA3 : 16;
    volatile uint32_t res : 16;
  } JDR3;
  struct
  {
    volatile uint32_t JDATA4 : 16;
    volatile uint32_t res : 16;
  } JDR4;
  const uint32_t res7[4];
  struct
  {
    volatile uint32_t res : 1;
    volatile uint32_t AWD2CH : 18;
    volatile uint32_t res2 : 13;
  } AWD2CR;
  struct
  {
    volatile uint32_t res : 1;
    volatile uint32_t AWD3CH : 18;
    volatile uint32_t res2 : 13;
  } AWD3CR;
  const uint32_t res8[2];
  struct
  {
    volatile uint32_t res : 1;
    volatile uint32_t DIFSEL : 18;
    volatile uint32_t res2 : 13;
  } DIFSEL;
  struct
  {
    volatile uint32_t CALFACT_S : 7;
    volatile uint32_t res : 9;
    volatile uint32_t CALFACT_D : 7;
    volatile uint32_t res2 : 9;
  } CALFACT;
} ADC_t;

typedef struct ADCCommon_t
{
  struct
  {
    volatile uint32_t ADRDY_MST : 1;
    volatile uint32_t EOSMP_MST : 1;
    volatile uint32_t EOC_MST : 1;
    volatile uint32_t EOS_MST : 1;
    volatile uint32_t OVR_MST : 1;
    volatile uint32_t JEOC_MST : 1;
    volatile uint32_t JEOS_MST : 1;
    volatile uint32_t AWD1_MST : 1;
    volatile uint32_t AWD2_MST : 1;
    volatile uint32_t AWD3_MST : 1;
    volatile uint32_t JQOVF_MST : 1;
    volatile uint32_t res : 5;
    volatile uint32_t ADRDY_SLV : 1;
    volatile uint32_t EOSMP_SLV : 1;
    volatile uint32_t EOC_SLV : 1;
    volatile uint32_t EOS_SLV : 1;
    volatile uint32_t OVR_SLV : 1;
    volatile uint32_t JEOC_SLV : 1;
    volatile uint32_t JEOS_SLV : 1;
    volatile uint32_t AWD1_SLV : 1;
    volatile uint32_t AWD2_SLV : 1;
    volatile uint32_t AWD3_SLV : 1;
    volatile uint32_t JQOVF_SLV : 1;
    volatile uint32_t res2 : 5;
  } CSR;
  const uint32_t res[1];
  struct
  {
    volatile uint32_t DUAL : 5;
    volatile uint32_t res : 3;
    volatile uint32_t DELAY : 4;
    volatile uint32_t res2 : 1;
    volatile uint32_t DMACFG : 1;
    volatile uint32_t MDMA : 2;
    volatile uint32_t CKMODE : 2;
    volatile uint32_t res3 : 4;
    volatile uint32_t VREFEN : 1;
    volatile uint32_t TSEN : 1;
    volatile uint32_t VBATEN : 1;
    volatile uint32_t res4 : 7;
  } CCR;
  struct
  {
    volatile uint32_t RDATA_MST : 16;
    volatile uint32_t RDATA_SLV : 16;
  } CDR;
} ADCCommon_t;

typedef struct USART_t
{
  struct
  {
    volatile uint32_t UE : 1;
    volatile uint32_t UESM : 1;
    volatile uint32_t RE : 1;
    volatile uint32_t TE : 1;
    volatile uint32_t IDLEIE : 1;
    volatile uint32_t RXNEIE : 1;
    volatile uint32_t TCIE : 1;
    volatile uint32_t TXEIE : 1;
    volatile uint32_t PEIE : 1;
    volatile uint32_t PS : 1;
    volatile uint32_t PCE : 1;
    volatile uint32_t WAKE : 1;
    volatile uint32_t M0 : 1;
    volatile uint32_t MME : 1;
    volatile uint32_t CMIE : 1;
    volatile uint32_t OVER8 : 1;
    volatile uint32_t DEDT : 5;
    volatile uint32_t DEAT : 5;
    volatile uint32_t RTOIE : 1;
    volatile uint32_t EOBIE : 1;
    volatile uint32_t M1 : 1;
    volatile uint32_t res : 3;
  } CR1;
  struct
  {
    volatile uint32_t res : 4;
    volatile uint32_t ADDM7 : 1;
    volatile uint32_t LBDL : 1;
    volatile uint32_t LBDIE : 1;
    volatile uint32_t res2 : 1;
    volatile uint32_t LBCL : 1;
    volatile uint32_t CPHA : 1;
    volatile uint32_t CPOL : 1;
    volatile uint32_t CLKEN : 1;
    volatile uint32_t STOP : 2;
    volatile uint32_t LINEN : 1;
    volatile uint32_t SWAP : 1;
    volatile uint32_t RXINV : 1;
    volatile uint32_t TXINV : 1;
    volatile uint32_t DATAINV : 1;
    volatile uint32_t MSBFIRST : 1;
    volatile uint32_t ABREN : 1;
    volatile uint32_t ABRMOD : 2;
    volatile uint32_t RTOEN : 1;
    volatile uint32_t ADD : 8;
  } CR2;
  struct
  {
    volatile uint32_t EIE : 1;
    volatile uint32_t IREN : 1;
    volatile uint32_t IRLP : 1;
    volatile uint32_t HDSEL : 1;
    volatile uint32_t NACK : 1;
    volatile uint32_t SCEN : 1;
    volatile uint32_t DMAR : 1;
    volatile uint32_t DMAT : 1;
    volatile uint32_t RTSE : 1;
    volatile uint32_t CTSE : 1;
    volatile uint32_t CTSIE : 1;
    volatile uint32_t ONEBIT : 1;
    volatile uint32_t OVRDIS : 1;
    volatile uint32_t DDRE : 1;
    volatile uint32_t DEM : 1;
    volatile uint32_t DEP : 1;
    volatile uint32_t res : 1;
    volatile uint32_t SCARCNT : 3;
    volatile uint32_t WUS : 2;
    volatile uint32_t WUFIE : 1;
    volatile uint32_t res2 : 9;
  } CR3;
  struct
  {
    volatile uint16_t BRR;
    volatile uint16_t res;
  } BRR;
  struct
  {
    volatile uint32_t PSC : 8;
    volatile uint32_t GT : 8;
    volatile uint32_t res : 16;
  } GTPR;
  struct
  {
    volatile uint32_t RTO : 24;
    volatile uint32_t BLEN : 8;
  } RTOR;
  struct
  {
    volatile uint32_t ABRRQ : 1;
    volatile uint32_t SBKRQ : 1;
    volatile uint32_t MMRQ : 1;
    volatile uint32_t RXFRQ : 1;
    volatile uint32_t TXFRQ : 1;
    volatile uint32_t res : 27;
  } RQR;
  struct
  {
    volatile uint32_t PE : 1;
    volatile uint32_t FE : 1;
    volatile uint32_t NF : 1;
    volatile uint32_t ORE : 1;
    volatile uint32_t IDLE : 1;
    volatile uint32_t RXNE : 1;
    volatile uint32_t TC : 1;
    volatile uint32_t TXE : 1;
    volatile uint32_t LBDF : 1;
    volatile uint32_t CTSIF : 1;
    volatile uint32_t CTS : 1;
    volatile uint32_t RTOF : 1;
    volatile uint32_t EOBF : 1;
    volatile uint32_t res : 1;
    volatile uint32_t ABRE : 1;
    volatile uint32_t ABRF : 1;
    volatile uint32_t BUSY : 1;
    volatile uint32_t CMF : 1;
    volatile uint32_t SBKF : 1;
    volatile uint32_t RWU : 1;
    volatile uint32_t WUF : 1;
    volatile uint32_t TEACK : 1;
    volatile uint32_t REACK : 1;
    volatile uint32_t res2 : 9;
  } ISR;
  struct
  {
    volatile uint32_t PECF : 1;
    volatile uint32_t FECF : 1;
    volatile uint32_t NCF : 1;
    volatile uint32_t ORECF : 1;
    volatile uint32_t IDLECF : 1;
    volatile uint32_t res : 1;
    volatile uint32_t TCCF : 1;
    volatile uint32_t res2 : 1;
    volatile uint32_t LBDCF : 1;
    volatile uint32_t CTSCF : 1;
    volatile uint32_t res3 : 1;
    volatile uint32_t RTOCF : 1;
    volatile uint32_t EOBCF : 1;
    volatile uint32_t res4 : 4;
    volatile uint32_t CMCF : 1;
    volatile uint32_t res5 : 2;
    volatile uint32_t WUCF : 1;
    volatile uint32_t res6 : 11;
  } ICR;
  struct
  {
    volatile uint8_t RDR;
    const uint32_t res : 24;
  } RDR;
  struct
  {
    volatile uint8_t TDR;
    const uint32_t res : 24;
  } TDR;
} __attribute__ ((packed, aligned (1))) USART_t;

typedef struct DMA_t
{
  struct
  {
    volatile uint32_t GIF1 : 1;
    volatile uint32_t TCIF1 : 1;
    volatile uint32_t HTIF1 : 1;
    volatile uint32_t TEIF1 : 1;
    volatile uint32_t GIF2 : 1;
    volatile uint32_t TCIF2 : 1;
    volatile uint32_t HTIF2 : 1;
    volatile uint32_t TEIF2 : 1;
    volatile uint32_t GIF3 : 1;
    volatile uint32_t TCIF3 : 1;
    volatile uint32_t HTIF3 : 1;
    volatile uint32_t TEIF3 : 1;
    volatile uint32_t GIF4 : 1;
    volatile uint32_t TCIF4 : 1;
    volatile uint32_t HTIF4 : 1;
    volatile uint32_t TEIF4 : 1;
    volatile uint32_t GIF5 : 1;
    volatile uint32_t TCIF5 : 1;
    volatile uint32_t HTIF5 : 1;
    volatile uint32_t TEIF5 : 1;
    volatile uint32_t GIF6 : 1;
    volatile uint32_t TCIF6 : 1;
    volatile uint32_t HTIF6 : 1;
    volatile uint32_t TEIF6 : 1;
    volatile uint32_t GIF7 : 1;
    volatile uint32_t TCIF7 : 1;
    volatile uint32_t HTIF7 : 1;
    volatile uint32_t TEIF7 : 1;
    const uint32_t res : 4;
  } ISR;
  struct
  {
    volatile uint32_t CGIF1 : 1;
    volatile uint32_t CTCIF1 : 1;
    volatile uint32_t CHTIF1 : 1;
    volatile uint32_t CTEIF1 : 1;
    volatile uint32_t CGIF2 : 1;
    volatile uint32_t CTCIF2 : 1;
    volatile uint32_t CHTIF2 : 1;
    volatile uint32_t CTEIF2 : 1;
    volatile uint32_t CGIF3 : 1;
    volatile uint32_t CTCIF3 : 1;
    volatile uint32_t CHTIF3 : 1;
    volatile uint32_t CTEIF3 : 1;
    volatile uint32_t CGIF4 : 1;
    volatile uint32_t CTCIF4 : 1;
    volatile uint32_t CHTIF4 : 1;
    volatile uint32_t CTEIF4 : 1;
    volatile uint32_t CGIF5 : 1;
    volatile uint32_t CTCIF5 : 1;
    volatile uint32_t CHTIF5 : 1;
    volatile uint32_t CTEIF5 : 1;
    volatile uint32_t CGIF6 : 1;
    volatile uint32_t CTCIF6 : 1;
    volatile uint32_t CHTIF6 : 1;
    volatile uint32_t CTEIF6 : 1;
    volatile uint32_t CGIF7 : 1;
    volatile uint32_t CTCIF7 : 1;
    volatile uint32_t CHTIF7 : 1;
    volatile uint32_t CTEIF7 : 1;
    const uint32_t res : 4;
  } IFCR;
  struct
  {
    volatile uint32_t EN : 1;
    volatile uint32_t TCIE : 1;
    volatile uint32_t HTIE : 1;
    volatile uint32_t TEIE : 1;
    volatile uint32_t DIR : 1;
    volatile uint32_t CIRC : 1;
    volatile uint32_t PINC : 1;
    volatile uint32_t MINC : 1;
    volatile uint32_t PSIZE : 2;
    volatile uint32_t MSIZE : 2;
    volatile uint32_t PL : 2;
    volatile uint32_t MEM2MEM : 1;
    const uint32_t res : 17;
  } CCR1;
  struct
  {
    volatile uint32_t NDT : 16;
    const uint32_t res : 16;
  } CNDTR1;
  struct
  {
    volatile uint32_t PA;
  } CPAR1;
  struct
  {
    volatile uint32_t MA;
  } CMAR1;

  const uint32_t res[1];
  struct
  {
    volatile uint32_t EN : 1;
    volatile uint32_t TCIE : 1;
    volatile uint32_t HTIE : 1;
    volatile uint32_t TEIE : 1;
    volatile uint32_t DIR : 1;
    volatile uint32_t CIRC : 1;
    volatile uint32_t PINC : 1;
    volatile uint32_t MINC : 1;
    volatile uint32_t PSIZE : 2;
    volatile uint32_t MSIZE : 2;
    volatile uint32_t PL : 2;
    volatile uint32_t MEM2MEM : 1;
    const uint32_t res : 17;
  } CCR2;
  struct
  {
    volatile uint32_t NDT : 16;
    const uint32_t res : 16;
  } CNDTR2;
  struct
  {
    volatile uint32_t PA;
  } CPAR2;
  struct
  {
    volatile uint32_t MA;
  } CMAR2;

  const uint32_t res2[1];

  struct
  {
    volatile uint32_t EN : 1;
    volatile uint32_t TCIE : 1;
    volatile uint32_t HTIE : 1;
    volatile uint32_t TEIE : 1;
    volatile uint32_t DIR : 1;
    volatile uint32_t CIRC : 1;
    volatile uint32_t PINC : 1;
    volatile uint32_t MINC : 1;
    volatile uint32_t PSIZE : 2;
    volatile uint32_t MSIZE : 2;
    volatile uint32_t PL : 2;
    volatile uint32_t MEM2MEM : 1;
    const uint32_t res : 17;
  } CCR3;
  struct
  {
    volatile uint32_t NDT : 16;
    const uint32_t res : 16;
  } CNDTR3;
  struct
  {
    volatile uint32_t PA;
  } CPAR3;
  struct
  {
    volatile uint32_t MA;
  } CMAR3;

  const uint32_t res3[1];
  struct
  {
    volatile uint32_t EN : 1;
    volatile uint32_t TCIE : 1;
    volatile uint32_t HTIE : 1;
    volatile uint32_t TEIE : 1;
    volatile uint32_t DIR : 1;
    volatile uint32_t CIRC : 1;
    volatile uint32_t PINC : 1;
    volatile uint32_t MINC : 1;
    volatile uint32_t PSIZE : 2;
    volatile uint32_t MSIZE : 2;
    volatile uint32_t PL : 2;
    volatile uint32_t MEM2MEM : 1;
    const uint32_t res : 17;
  } CCR4;
  struct
  {
    volatile uint32_t NDT : 16;
    const uint32_t res : 16;
  } CNDTR4;
  struct
  {
    volatile uint32_t PA;
  } CPAR4;
  struct
  {
    volatile uint32_t MA;
  } CMAR4;

  const uint32_t res4[1];

  struct
  {
    volatile uint32_t EN : 1;
    volatile uint32_t TCIE : 1;
    volatile uint32_t HTIE : 1;
    volatile uint32_t TEIE : 1;
    volatile uint32_t DIR : 1;
    volatile uint32_t CIRC : 1;
    volatile uint32_t PINC : 1;
    volatile uint32_t MINC : 1;
    volatile uint32_t PSIZE : 2;
    volatile uint32_t MSIZE : 2;
    volatile uint32_t PL : 2;
    volatile uint32_t MEM2MEM : 1;
    const uint32_t res : 17;
  } CCR5;
  struct
  {
    volatile uint32_t NDT : 16;
    const uint32_t res : 16;
  } CNDTR5;
  struct
  {
    volatile uint32_t PA;
  } CPAR5;
  struct
  {
    volatile uint32_t MA;
  } CMAR5;

  const uint32_t res5[1];
  struct
  {
    volatile uint32_t EN : 1;
    volatile uint32_t TCIE : 1;
    volatile uint32_t HTIE : 1;
    volatile uint32_t TEIE : 1;
    volatile uint32_t DIR : 1;
    volatile uint32_t CIRC : 1;
    volatile uint32_t PINC : 1;
    volatile uint32_t MINC : 1;
    volatile uint32_t PSIZE : 2;
    volatile uint32_t MSIZE : 2;
    volatile uint32_t PL : 2;
    volatile uint32_t MEM2MEM : 1;
    const uint32_t res : 17;
  } CCR6;
  struct
  {
    volatile uint32_t NDT : 16;
    const uint32_t res : 16;
  } CNDTR6;
  struct
  {
    volatile uint32_t PA;
  } CPAR6;
  struct
  {
    volatile uint32_t MA;
  } CMAR6;

  const uint32_t res6[1];
  struct
  {
    volatile uint32_t EN : 1;
    volatile uint32_t TCIE : 1;
    volatile uint32_t HTIE : 1;
    volatile uint32_t TEIE : 1;
    volatile uint32_t DIR : 1;
    volatile uint32_t CIRC : 1;
    volatile uint32_t PINC : 1;
    volatile uint32_t MINC : 1;
    volatile uint32_t PSIZE : 2;
    volatile uint32_t MSIZE : 2;
    volatile uint32_t PL : 2;
    volatile uint32_t MEM2MEM : 1;
    const uint32_t res : 17;
  } CCR7;
  struct
  {
    volatile uint32_t NDT : 16;
    const uint32_t res : 16;
  } CNDTR7;
  struct
  {
    volatile uint32_t PA;
  } CPAR7;
  struct
  {
    volatile uint32_t MA;
  } CMAR7;

  const uint32_t res7[6];
} DMA_t;

typedef struct SYSCFG_t
{
  struct
  {
    volatile uint32_t MEM_MODE : 3;
    volatile uint32_t res : 2;
    volatile uint32_t USB_IT_RMP : 1;
    volatile uint32_t TIM1_ITR3_RMP : 1;
    volatile uint32_t DAC_TRIG_RMP : 1;
    volatile uint32_t ADC2_DMA_RMP : 1;
    volatile uint32_t res2 : 2;
    volatile uint32_t TIM16_DMA_RMP : 1;
    volatile uint32_t TIM17_DMA_RMP : 1;
    volatile uint32_t TIM6_DAC1_DMA_RMP : 1;
    volatile uint32_t TIM7_DAC2_DMA_RMP : 1;
    volatile uint32_t DAC2_CH1_DMA_RMP : 1;
    volatile uint32_t I2C_PB6_FMP : 1;
    volatile uint32_t I2C_PB7_FMP : 1;
    volatile uint32_t I2C_PB8_FMP : 1;
    volatile uint32_t I2C_PB9_FMP : 1;
    volatile uint32_t I2C1_FMP : 1;
    volatile uint32_t I2C2_FMP : 1;
    volatile uint32_t ENCODER_MODE : 2;
    volatile uint32_t I2C3_FMP : 1;
    volatile uint32_t res3 : 1;
    volatile uint32_t FPU_IE : 6;
  } CFGR1;

  struct
  {
    volatile uint32_t PAGE0_WP : 1;
    volatile uint32_t PAGE1_WP : 1;
    volatile uint32_t PAGE2_WP : 1;
    volatile uint32_t PAGE3_WP : 1;
    volatile uint32_t PAGE4_WP : 1;
    volatile uint32_t PAGE5_WP : 1;
    volatile uint32_t PAGE6_WP : 1;
    volatile uint32_t PAGE7_WP : 1;
    volatile uint32_t PAGE8_WP : 1;
    volatile uint32_t PAGE9_WP : 1;
    volatile uint32_t PAGE10_WP : 1;
    volatile uint32_t PAGE11_WP : 1;
    volatile uint32_t PAGE12_WP : 1;
    volatile uint32_t PAGE13_WP : 1;
    volatile uint32_t PAGE14_WP : 1;
    volatile uint32_t PAGE15_WP : 1;
    volatile uint32_t res : 16;
  } RCR;

  struct
  {
    volatile uint32_t EXTI0 : 4;
    volatile uint32_t EXTI1 : 4;
    volatile uint32_t EXTI2 : 4;
    volatile uint32_t EXTI3 : 4;
    volatile uint32_t res : 16;
  } EXTICR1;

  struct
  {
    volatile uint32_t EXTI4 : 4;
    volatile uint32_t EXTI5 : 4;
    volatile uint32_t EXTI6 : 4;
    volatile uint32_t EXTI7 : 4;
    volatile uint32_t res : 16;
  } EXTICR2;
  struct
  {
    volatile uint32_t EXTI8 : 4;
    volatile uint32_t EXTI9 : 4;
    volatile uint32_t EXTI10 : 4;
    volatile uint32_t EXTI11 : 4;
    volatile uint32_t res : 16;
  } EXTICR3;
  struct
  {
    volatile uint32_t EXTI12 : 4;
    volatile uint32_t EXTI13 : 4;
    volatile uint32_t EXTI14 : 4;
    volatile uint32_t EXTI15 : 4;
    volatile uint32_t res : 16;
  } EXTICR4;
  struct
  {
    volatile uint32_t LOCKUP_LOCK : 1;
    volatile uint32_t SRAM_PARITY_LOCK : 1;
    volatile uint32_t PVD_LOCK : 1;
    volatile uint32_t res : 1;
    volatile uint32_t BYP_ADDR_PAR : 1;
    volatile uint32_t res2 : 3;
    volatile uint32_t SRAM_PEF : 1;
    const uint32_t res3 : 23;
  } CFGR2;
  struct
  {
    volatile uint32_t SPI1_RX_DMA_RMP : 2;
    volatile uint32_t SPI1_TX_DMA_RMP : 2;
    volatile uint32_t I2C1_RX_DMA_RMP : 2;
    volatile uint32_t I2C1_TX_DMA_RMP : 2;
    volatile uint32_t ADC2_DMA_RMP : 2;
    const uint32_t res3 : 22;
  } CFGR3;
  struct
  {
    volatile uint32_t ADC12_EXT2_RMP : 1;
    volatile uint32_t ADC12_EXT3_RMP : 1;
    volatile uint32_t ADC12_EXT5_RMP : 1;
    volatile uint32_t ADC12_EXT13_RMP : 1;
    volatile uint32_t ADC12_EXT15_RMP : 1;
    volatile uint32_t ADC12_JEXT3_RMP : 1;
    volatile uint32_t ADC12_JEXT6_RMP : 1;
    volatile uint32_t ADC12_JEXT13_RMP : 1;
    volatile uint32_t ADC34_EXT5_RMP : 1;
    volatile uint32_t ADC34_EXT6_RMP : 1;
    volatile uint32_t ADC34_EXT15_RMP : 1;
    volatile uint32_t ADC34_JEXT5_RMP : 1;
    volatile uint32_t ADC34_JEXT11_RMP : 1;
    volatile uint32_t ADC34_JEXT14_RMP : 1;
    const uint32_t res3 : 18;
  } CFGR4;

} SYSCFG_t;

typedef struct IWDG_t
{
  struct
  {
    volatile uint32_t KEY : 16;
    const uint32_t res : 16;
  } KR;
  struct
  {
    volatile uint32_t PR : 3;
    const uint32_t res : 29;
  } PR;
  struct
  {
    volatile uint32_t RL : 12;
    const uint32_t res : 20;
  } RLR;
  struct
  {
    volatile uint32_t PVU : 1;
    volatile uint32_t RVU : 1;
    volatile uint32_t WVU : 1;
    const uint32_t res : 29;
  } SR;
  struct
  {
    volatile uint32_t WIN : 12;
    const uint32_t res : 20;
  } WINR;
} IWDG_t;

typedef struct WWDG_t
{
  struct
  {
    volatile uint32_t T : 7;
    volatile uint32_t WDGA : 1;
    const uint32_t res : 24;
  } CR;
  struct
  {
    volatile uint32_t W : 7;
    volatile uint32_t WDGTB : 2;
    volatile uint32_t EWI : 1;
    const uint32_t res : 22;
  } CFR;
  struct
  {
    volatile uint32_t EWIF : 1;
    const uint32_t res : 31;
  } SR;
} WWDG_t;

#endif