#include "PID.h"
#include <cmath>
#include <iostream>
using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double kp, double ki, double kd) {
  SetPID(kp, ki, kd);
  ClearError();
}

void PID::SetPID(double kp, double ki, double kd) {
  kp_m = kp;
  ki_m = ki;
  kd_m = kd;
}

void PID::SetDt(double dt) { dt_m = dt; }

void PID::ClearError() {
  p_error_m = 0;
  i_error_m = 0;
  d_error_m = 0;
}

void PID::SetDeadBand(double deadband) { deadband_m = deadband; }

void PID::PrintPID() {
  printf("kp ki kd :( %.3f, %.3f, %.3f )\n", kp_m, ki_m, kd_m);
}

double PID::UpdateError(double cte) {
  d_error_m = (cte - p_error_m) / dt_m;
  i_error_m += cte * dt_m;
  p_error_m = cte;

  if (std::abs(cte) < deadband_m)
    return 0;

  double output;
  output = -(kp_m * p_error_m + ki_m * i_error_m + kd_m * d_error_m);
  output = output > max_output_m ? max_output_m : output;
  output = output < min_output_m ? min_output_m : output;

  return output;
}

double PID::TotalError() { return 0; }
