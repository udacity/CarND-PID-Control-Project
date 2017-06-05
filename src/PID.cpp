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
  // old_t = clock();

  has_old_d_error = false;
}

void PID::UpdateError(double cte) {
  if (!has_old_d_error)
  {
    old_d_error = cte;

    has_old_d_error = true;
  }
  t = std::chrono::system_clock::now();
  // t = clock();
  std::chrono::duration<double> dt = t - old_t;
  // double dt = t - old_t;
  // std::cout << "\n" << std::endl;
  // std::cout << "dt: " << dt.count() << std::endl;
  // update errors
  p_error = cte;
  // d_error = (cte - old_d_error); // for uncalculated dt
  d_error = (cte - old_d_error)/dt.count();
  // d_error = (cte - old_d_error);
  i_error += cte;

  old_t = t;
  old_d_error = cte;
}

double PID::TotalError() {

  std::cout << "\n" << std::endl;
  // std::cout << "p_error: " << p_error << " d_error: " << d_error << " i_error: " << i_error << std::endl;
  
  return -Kp*p_error - Kd*d_error - Ki*i_error;
}

