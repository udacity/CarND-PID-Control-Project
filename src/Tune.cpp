#include "Tune.h"

Tune::Tune() {}

Tune::~Tune() {}

void Tune::Init(double orig_p, double orig_i, double orig_d) {
  new_pid = {orig_p, orig_i, orig_d};
  delta_pid = {1.0, 1.0, 1.0};
  tolerance = 0.2;
  double best_err = 1000000.0;

  //Kp = orig_p;
  //Ki = orig_i;
  //Kd = orig_d;
  p = {orig_p,orig_i,orig_d};
  dp = {orig_p/10.0, orig_i/10.0, orig_d/10.0};
  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;
  prev_cte = 0.0;
  threshold = 0.1;
  best_err = 999999.0;

}

void Tune::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
  this->p_error = cte;  
  this->i_error += cte;
  this->d_error = cte - this->prev_cte;
  this->prev_cte = cte;
    
}

double Tune::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  double error = -1.0 * this->p[0] * this->p_error - this->p[1] * this->i_error - this->p[2] * this->d_error;
  return error;  
}

void Tune::ResetError() {
  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;
  prev_cte = 0.0;
  
}
