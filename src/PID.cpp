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


void PID::Twiddle(const double tolerance) {
  double dp[3] = {1.0, 1.0, 1.0};

  double best_err = TotalError();

  int it = 0;
  while ((dp[0] + dp[1] + dp[2]) > tolerance) {
    std::cout << "Iteration " << it << " best error " << best_err << std::endl;

    for (unsigned i = 0; i < 3; i++) {
      tau_[i] += dp[i];

      double err = TotalError();

      if (err < best_err) {
	best_err = err;
	dp[i] *= 1.1;
      } else {
	tau_[i] -= 2 * dp[i];

	err = TotalError();

	if (err < best_err) {
	  best_err = err;
	  dp[i] *= 1.05;
	} else {
	  tau_[i] += dp[i];
	  dp[i] *= 0.95;
	}
      }
    }

    it++;
  }
}
