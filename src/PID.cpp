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
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;
  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;
  prev_cte = 0.0;
  
}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
  this->p_error = cte;  
  this->i_error += cte;
  this->d_error = cte - this->prev_cte;
  this->prev_cte = cte;
    
}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  double error = -1.0 * this->Kp * this->p_error - this->Ki * this->i_error - this->Kd * this->d_error;
  return error;  // TODO: Add your total error calc here!
}
