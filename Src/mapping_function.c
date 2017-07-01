#include "mapping_function.h"
#include "stdint.h"
#include "float.h"


void map(motor_vars_t *mvars, float *yaw_err, float *pitch_err, float *roll_err) {
  
/**
 * @brief Assumed motor orientation and layout.
 *
 * Top View
 *
 *         M1 _         _ M2
 *           (_) front (_)
 * y-axis      \ .___. /
 * ^            \|   |/         M1,M4 = clockwise 
 * |   x-axis    |   |          M2,M3 = anti-clockwise
 * |---->        |   |
 * z-axis       /|___|\
 * (upward)   _/       \_
 *           (_)       (_)
 *         M3             M4
 * 
 * All angles are viewed from the origin as the frame of reference.
 * Anti-clockwise angle is assumed to be positive and clockwise negative.
 * Note, that this assumption is currently a placeholder as Sensor Fusion
 * output data may be different.
 * 
 * 
 * |error| M1 | M2 | M3 | M4 |
 * |:----|:--:|:--:|:--:|:--:|
 * | Y+  | +  | -  | -  | +  |
 * | Y-  | -  | +  | +  | -  |
 * | P+  | +  | +  |-/x |-/x |
 * | P-  |-/x |-/x | +  | +  |
 * | R+  | +  |-/x | +  |-/x |
 * | R-  |-/x | +  |-/x | +  |
 *     
 * x = don't care / no correction
 * + = positive correction = m*(error)
 * - = negative correction = m*(error)
 * m = slope of error vs correction line 
 *   = ERROR_AMPLIFICATION_FACTOR
 * 
 * The error amplification factor:
 * The error obtained from sensor readings is mapped to the final
 * pulse width values by using a straight line function, y=mx.
 * Here, y = correction in pulsewidth, which is a signed number
 *       m = ERROR_AMPLIFICATION_FACTOR
 *       x = error 
 * 
 * Higher values of ERROR_AMPLIFICATION_FACTOR will result in more
 * aggressive error correction.
 * 
 * @example: If roll error obtained is 0.0, then correction to be done
 *           in roll is y = m*0.0 = 0.0
 *           If roll error obtained is -1.2, then correction to be done
 *           in roll is y = m*(-1.2)
 */
	 
  double m1_correction=0.0, m2_correction=0.0, m3_correction=0.0, m4_correction=0.0;
  
  /* correction in yaw */
  if (ANTI_CLOCKWISE(*yaw_err, ANTI_CLOCKWISE_THRESHOLD) || CLOCKWISE(*yaw_err, CLOCKWISE_THRESHOLD)) {
    m1_correction += ERROR_AMPLIFICATION_FACTOR * (*yaw_err);
    m2_correction -= ERROR_AMPLIFICATION_FACTOR * (*yaw_err);
    m3_correction -= ERROR_AMPLIFICATION_FACTOR * (*yaw_err);
    m4_correction += ERROR_AMPLIFICATION_FACTOR * (*yaw_err);
  }
  
  /* correction in pitch */
  if (ANTI_CLOCKWISE(*pitch_err, ANTI_CLOCKWISE_THRESHOLD) || CLOCKWISE(*pitch_err, CLOCKWISE_THRESHOLD)) {
    m1_correction += ERROR_AMPLIFICATION_FACTOR * (*pitch_err);
    m2_correction += ERROR_AMPLIFICATION_FACTOR * (*pitch_err);
    m3_correction -= ERROR_AMPLIFICATION_FACTOR * (*pitch_err);
    m4_correction -= ERROR_AMPLIFICATION_FACTOR * (*pitch_err);
  }
  
  /* correction in roll */
  if (ANTI_CLOCKWISE(*roll_err, ANTI_CLOCKWISE_THRESHOLD) || CLOCKWISE(*roll_err, CLOCKWISE_THRESHOLD)) {
    m1_correction += ERROR_AMPLIFICATION_FACTOR * (*roll_err);
    m2_correction -= ERROR_AMPLIFICATION_FACTOR * (*roll_err);
    m3_correction += ERROR_AMPLIFICATION_FACTOR * (*roll_err);
    m4_correction -= ERROR_AMPLIFICATION_FACTOR * (*roll_err);
  }
  
  /* applying corrections to motors */
  /* corrections are typecast to signed int 
   * as final CCR values are in unsigned int 
   */
  mvars->M1 += (int16_t)m1_correction;
  mvars->M2 += (int16_t)m2_correction;
  mvars->M3 += (int16_t)m3_correction;
  mvars->M4 += (int16_t)m4_correction;
  
  /* checking boundaries */
  if (mvars->M1 > mvars->_pulse_width_max) {
    mvars->M1 = mvars->_pulse_width_max;
  } else if (mvars->M1 < mvars->_pulse_width_min) {
    mvars->M1 = mvars->_pulse_width_min;
  }
  if (mvars->M2 > mvars->_pulse_width_max) {
    mvars->M2 = mvars->_pulse_width_max;
  } else if (mvars->M2 < mvars->_pulse_width_min) {
    mvars->M2 = mvars->_pulse_width_min;
  }
  if (mvars->M3 > mvars->_pulse_width_max) {
    mvars->M3 = mvars->_pulse_width_max;
  } else if (mvars->M3 < mvars->_pulse_width_min) {
    mvars->M3 = mvars->_pulse_width_min;
  }
  if (mvars->M4 > mvars->_pulse_width_max) {
    mvars->M4 = mvars->_pulse_width_max;
  } else if (mvars->M4 < mvars->_pulse_width_min) {
    mvars->M4 = mvars->_pulse_width_min;
  }
  
  /*
   * TODO
   * - verify logical consistency
   * - verify system response at boundary conditions
   */
}