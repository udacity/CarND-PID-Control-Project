#include "PID.h"

#include <iostream>

using namespace std;


PID::PID(double tau_p, double tau_i, double tau_d)
  : p_cte_(0.0), i_cte_(0.0), d_cte_(0.0) {

  tau_[kP] = tau_p;
  tau_[kI] = tau_i;
  tau_[kD] = tau_d;
}


void PID::UpdateError(double cte) {
  d_cte_ = cte - p_cte_;
  p_cte_ = cte;
  i_cte_ =+ cte;
}


double PID::TotalError() {
  std::cout << tau_[kP] << " " << tau_[kI] << " " << tau_[kD] << std::endl;
  return (tau_[kP] * p_cte_ +
	  tau_[kI] * i_cte_ +
	  tau_[kD] * d_cte_);
}
