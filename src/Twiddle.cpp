#include "Twiddle.h"
#include <iostream>
#include <limits>

using namespace std;

Twiddle::Twiddle()
{
  best_err_ = std::numeric_limits<double>::max();
}

Twiddle::~Twiddle() {}

void Twiddle::Init(double Kp, double Ki, double Kd)
{
  p_[0] = Kp;
  p_[1] = Ki;
  p_[2] = Kd;
  iteration = 0;
  err_ = 0.0;
  p_idx = 0;
  state = 0;
}


bool Twiddle::IterationAndErrorCount(double cte)
{
  iteration++;
  err_ += cte;
  return iteration < 500;
}

bool Twiddle::UpdateParams()
{
  bool restart = false;

  switch(state) {
    case 0:
      p_[p_idx] += dp_[p_idx];
      state = 1;
      restart = true;
      break;
    case 1:
      if (err_ < best_err_) {
        best_err_ = err_;
        dp_[p_idx] *= 1.1;
        state = 0;
        p_idx++;
      } else {
        p_[p_idx] -= 2 * dp_[p_idx];
        state = 2;
        restart = true;
      }
      break;
    case 2:
      if (err_ < best_err_) {
        best_err_ = err_;
        dp_[p_idx] *= 1.1;
      } else {
        p_[p_idx] += dp_[p_idx];
        dp_[p_idx] *= 0.9;
      }
      state = 0;
      p_idx++;
      break;
    default:
      break;
  }

  p_idx = p_idx % 3;
  if (restart) {
    iteration = 0;
    err_ = 0;
  }
  return restart;
}

double* Twiddle::GetP()
{
  return p_;
}

double Twiddle::GetBestError()
{
  return best_err_;
}

double Twiddle::SumOfDp()
{
  return dp_[0] + dp_[1] + dp_[2];
}
