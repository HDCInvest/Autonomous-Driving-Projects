#include "PID.h"

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
  PID::Kp = Kp_;
  PID::Ki = Ki_;
  PID::Kd = Kd_;
  //Initiate the Proportional CTE,Differential CTE and Integrated CTE to 0
  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;

}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
    double prev_cte = p_error ;
    p_error = cte ;    // Set Proportional CTE to the cross track error
    d_error = cte - prev_cte ;  // Differentail CTE is difference between error at current timestep and previous timestep
    i_error += cte ;   //Integrated crosstrack error (int_CTE) is the sum of all the previous crosstrack errors

}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  //Total error is related to Proportional CTE,Differential CTE and Integrated CTE
  //There influence is controlled PID coefficients Kp,Kd & Ki respectively
  return -Kp * p_error - Kd * d_error - Ki * i_error;  // TODO: Add your total error calc here!
}





