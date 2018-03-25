#ifndef PID_H
#define PID_H

enum Tau {
  kP = 0,
  kI = 1,
  kD = 2
};


class PID {
public:
  /*
  * Errors
  */
  double p_cte_;
  double i_cte_;
  double d_cte_;

  /*
  * Coefficients
  */
  double tau_[3];

  /*
  * Constructor
  */
  PID(double tau_p, double tau_i, double tau_d);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

private:
  void Twiddle(const double tolerance);
};

#endif /* PID_H */
