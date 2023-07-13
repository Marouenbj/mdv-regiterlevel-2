#pragma once
#ifndef FSM_H
#define FSM_H

#include "stdbool.h"
#include "stdint.h"
#include "stdlib.h"

#include "stdatomic.h"

typedef union adc_data
{
  volatile uint16_t hw;
  volatile uint8_t b[2];
} adc_data;

typedef struct fsm_t
{
  struct
  {
    volatile uint16_t psc;
    volatile uint32_t arr;
    volatile uint32_t count;
    volatile bool filter;
    volatile bool decim;
  } cmd;
  struct
  {
    volatile atomic_bool rto_detected;
    volatile atomic_bool cmd_valid;
    volatile atomic_bool coeff_acquired;
    volatile atomic_bool sampling_instant;
    volatile atomic_bool processing;
    volatile atomic_bool buffer_switch;
    volatile atomic_bool reset;
  } state;
  struct
  {
    uint8_t *const dmabuf_ptr;
    uint8_t *const mainbuf_ptr;
  } buf;
  struct
  {
    volatile uint16_t adc_k;
    volatile size_t counter;
  } data;
} fsm_t;

#endif
