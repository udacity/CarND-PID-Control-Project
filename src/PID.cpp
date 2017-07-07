#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double kp, double ki, double kd) {
  kp_m = kp;
  ki_m = ki;
  kd_m = kd;
  p_error_m = 0;
  i_error_m = 0;
  d_error_m = 0;
}

void PID::SetPID(double kp, double ki, double kd) {
  kp_m = kp;
  ki_m = ki;
  kd_m = kd;
}

double PID::UpdateError(double cte) {
  d_error_m = (cte - p_error_m) / dt_m;
  i_error_m += cte * dt_m;
  p_error_m = cte;

  double output;
  output = -(kp_m * p_error_m + ki_m * i_error_m + kd_m * d_error_m);
  output = output > max_output_m ? max_output_m : output;
  output = output < min_output_m ? min_output_m : output;
  return output;
}

double PID::TotalError() {}
