#include "PID.h"
#include <iostream>
#include <chrono>
#include <ctime>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;

  p_error = 0.0;
  d_error = 0.0;
  i_error = 0.0;

  old_t = std::chrono::system_clock::now();

  has_old_d_error = false;
}

void PID::UpdateError(double cte) {
  if (!has_old_d_error)
  {
    old_d_error = cte;

    has_old_d_error = true;
  }
  t = std::chrono::system_clock::now();
  
  // calculate time elapsed
  std::chrono::duration<double> dt = t - old_t;
  
  // update errors
  p_error = cte;
  d_error = (cte - old_d_error)/dt.count();
  i_error += cte;

  old_t = t;
  old_d_error = cte;
}

double PID::TotalError() {
  return -Kp*p_error - Kd*d_error - Ki*i_error;
}

