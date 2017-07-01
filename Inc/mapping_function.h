#ifndef __MAPPING_FUNCTION_H
#define __MAPPING_FUNCTION_H

#include "stdint.h"

typedef struct {
  /* pulse width values for all the motors */
  uint16_t M1;
  uint16_t M2;
  uint16_t M3;
  uint16_t M4;
  
  uint16_t _base_thrust;
  const uint16_t _pulse_width_max;
  const uint16_t _pulse_width_min;
} motor_vars_t;

/* TODO: dummy values here; change these after testing */
#define CLOCKWISE_THRESHOLD (-0.1)
#define ANTI_CLOCKWISE_THRESHOLD (+0.1)
#define ERROR_AMPLIFICATION_FACTOR (3.0)

/* clockwise = -ve; anti-clockwise = +ve; */
#define CLOCKWISE(x,threshold) (x<threshold)?(1):(0)
#define ANTI_CLOCKWISE(x,threshold) (x>threshold)?(1):(0)

/**
 * Use MOTOR_VARS_INIT to initialize motor PWM values.
 * The ESC can take only a limited range of pulse width.
 */
#define MOTOR_VARS_INIT(x) motor_vars_t x = {.M1=0,.M2=0,.M3=0,.M4=0,._base_thrust=0,._pulse_width_min=1100,._pulse_width_max=1900}

void map(motor_vars_t*, float*, float*, float*);
#endif