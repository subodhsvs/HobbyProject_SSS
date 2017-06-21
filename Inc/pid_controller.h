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
  double Kp;
  double Ki;
  double Kd;
  
  /* max output limits for the PID controller */
  double output_max;
  double output_min;
  
  /* below are session variables for the PID controller */
  double _integral_sum;
  double _prev_err;
  double _dt;
} PID_vars;


#define PID_VARS_INIT(x) PID_vars x = {.Kp=0.0,.Ki=0.0,.Kd=0.0,.output_max=100.0, \
                       .output_min=0.0,._integral_sum=0.0,._prev_err=0.0,._dt=1.0}

/* Function Prototypes */
double pid(PID_vars *vars, double current_err);

#endif