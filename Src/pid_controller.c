/*
 * pid_controller.c
 *
 *  Created on: Apr 24, 2016
 *      Author: subham roy
 */

#include "pid_controller.h"

double pid(PID_vars *vars, double current_err) {
  
  /* current_error = setpoint - current_process_variable */
  
  vars->_integral_sum += current_err*(vars->_dt);
  
  double output = (vars->Kp)*current_err                     \
                + (vars->Ki)*(vars->_integral_sum)           \
                + (vars->Kd)*((current_err-(vars->_prev_err))\
                              /(vars->_dt)); 
  
  vars->_prev_err = current_err;
  
  /* limit output within output_min and output_max */
  if (output>(vars->output_max))
    output = vars->output_max;
  else if (output<(vars->output_min))
    output = vars->output_min;
  
  return output;
}