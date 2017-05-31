#include "PID.h"
#include <iostream>
#include <time.h>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  this->Kp = Kp;
  this->Kd = Kd;
  this->Ki = Ki;

  p_error = 0.0;
  d_error = 0.0;
  i_error = 0.0;

  old_t = clock();

  has_old_d_error = false;
}

void PID::UpdateError(double cte) {
  if (!has_old_d_error)
  {
    old_d_error = cte;

    has_old_d_error = true;
  }
  t = clock();
  double dt = t - old_t;
  // update errors
  p_error = cte;
  d_error = (cte - old_d_error)/dt;
  // d_error = (cte - old_d_error);
  i_error += cte;

  old_t = t;
}

double PID::TotalError() {

  std::cout << "p_error: " << p_error << " d_error: " << d_error << " i_error: " << i_error << std::endl;
  std::cout << "\n" << std::endl;
  
  return -Kp*p_error - Kd*d_error - Ki*i_error;
}

