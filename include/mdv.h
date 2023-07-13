#pragma once
#ifndef MDV_LIBRARY_H
#define MDV_LIBRARY_H

#ifdef __CLANG_ATOMICS
#define __CLANG_ATOMICS __GNUC_ATOMICS
#endif

#define __FPU_PRESENT 1U

// typedef struct fsm_t fsm_t;

#include "stdio.h"
#include "stdlib.h"

#include "stddef.h"
#include "stdint.h"
#include "string.h"

#include "stdbool.h"

#include "stdatomic.h"

#include "cmsis_compiler.h"
#include "cmsis_gcc.h"

#include "ledctrl.h"

#include "cmsis_version.h"

#include "core_cm4.h"

#include "periph.h"

#include "core_periph.h"

#include "fsm.h"

#include "command.h"

typedef struct mdv_ops
{
  void (*init_gpioa) (GPIO_t *);
  void (*init_usart2) (USART_t *);
  void (*init_dma1) (DMA_t *, USART_t *, uint8_t *const);
  void (*init_tim2) (TIMgp_t *, uint16_t, uint32_t);
  void (*init_adc1) (ADC_t *, ADCCommon_t *);
  void (*init_flash) (FLASH_t *);
  void (*init_rcc) (RCC_t *);
} mdv_ops;

__STATIC_FORCEINLINE void
exclusive_incw (uint32_t *val)
{
  __DMB ();
  uint32_t tmp;
  do
    {
      tmp = __LDREXW (val);
      tmp++;
    }
  while (__STREXW (tmp, val));
  __DMB ();
}

__STATIC_FORCEINLINE void
exclusive_decw (uint32_t *val)
{
  __DMB ();
  uint32_t tmp;
  do
    {
      tmp = __LDREXW (val);
      tmp--;
    }
  while (__STREXW (tmp, val));
  __DMB ();
}

__STATIC_FORCEINLINE void
exclusive_inch (uint16_t *val)
{
  __DMB ();
  uint16_t tmp;
  do
    {
      tmp = __LDREXH (val);
      tmp++;
    }
  while (__STREXH (tmp, val));
  __DMB ();
}

__STATIC_FORCEINLINE void
exclusive_dech (uint16_t *val)
{
  __DMB ();
  uint16_t tmp;
  do
    {
      tmp = __LDREXH (val);
      tmp--;
    }
  while (__STREXH (tmp, val));
  __DMB ();
}

__STATIC_FORCEINLINE void
exclusive_incb (uint8_t *val)
{
  __DMB ();
  uint8_t tmp;
  do
    {
      tmp = __LDREXB (val);
      tmp++;
    }
  while (__STREXB (tmp, val));
  __DMB ();
}

__STATIC_FORCEINLINE void
exclusive_decb (uint8_t *val)
{
  __DMB ();
  uint8_t tmp;
  do
    {
      tmp = __LDREXB (val);
      tmp--;
    }
  while (__STREXB (tmp, val));
  __DMB ();
}

__STATIC_FORCEINLINE void
exclusive_clr (void)
{
  __CLREX ();
}

#endif