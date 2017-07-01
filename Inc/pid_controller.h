/*
 * pid_controller.h
 *
 *  Created on: Apr 24, 2016
 *      Author: subham roy
 */

#ifndef __PID_CONTROLLER_H
#define __PID_CONTROLLER_H

typedef struct {
  /* PID controller parameters */
  float Kp;
  float Ki;
  float Kd;
  
  /* max output limits for the PID controller */
  float output_max;
  float output_min;
  
  /* below are session variables for the PID controller */
  float _integral_sum;
  float _prev_err;
  float _dt;
} PID_vars;

/** 
 * Use PID_VARS_INIT to correctly initialize PID variables.
 * Some fields like _dt are supposed to be non-zero.
 */
#define PID_VARS_INIT(x) PID_vars x = {.Kp=1.0,.Ki=0.0,.Kd=0.0,.output_max=100.0, \
                       .output_min=0.0,._integral_sum=0.0,._prev_err=0.0,._dt=1.0}

/* Function Prototypes */
float pid(PID_vars *vars, float current_err);

#endif