#pragma once
#ifndef LEDCTRL_H
#define LEDCTRL_H

#define LED_RED GPIOA->ODR.ODR10
#define LED_YELLOW GPIOA->ODR.ODR9
#define LED_GREEN GPIOA->ODR.ODR8

#define LED_TGL_RED GPIOA->ODR.ODR10 = !GPIOA->ODR.ODR10
#define LED_TGL_YELLOW GPIOA->ODR.ODR9 = !GPIOA->ODR.ODR9
#define LED_TGL_GREEN GPIOA->ODR.ODR8 = !GPIOA->ODR.ODR8

#define LED_YGR(yellow, green, red)                       \
  do                                                      \
    {                                                     \
      LED_RED = red;                                      \
      LED_GREEN = green;                                  \
      LED_YELLOW = yellow;                                \
    }                                                     \
  while (0)

#define LED_ADC(adc)                                      \
  do                                                      \
    {                                                     \
      if (adc <= 255)                                     \
        {                                                 \
          LED_YELLOW = 0;                                 \
          LED_GREEN = 0;                                  \
          LED_RED = 0;                                    \
        }                                                 \
      else if (adc <= 511)                                \
        {                                                 \
          LED_YELLOW = 1;                                 \
          LED_GREEN = 0;                                  \
          LED_RED = 0;                                    \
        }                                                 \
      else if (adc <= 767)                                \
        {                                                 \
          LED_YELLOW = 0;                                 \
          LED_GREEN = 1;                                  \
          LED_RED = 0;                                    \
        }                                                 \
      else if (adc <= 1023)                               \
        {                                                 \
          LED_YELLOW = 0;                                 \
          LED_GREEN = 0;                                  \
          LED_RED = 1;                                    \
        }                                                 \
    }                                                     \
  while (0)

#endif
