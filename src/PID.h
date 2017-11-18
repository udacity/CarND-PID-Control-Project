#ifndef PID_H
#define PID_H

class PID {
public:
  /*
  * Errors
  */
  /* Proportional Error */
  double p_error;

  /* Integral Error */
  double i_error, i_error_pos, i_error_neg;

  /* Derivative Error */
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  double i_deadband;
  double antiwindup_lim;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd, double i_deadband, double antiwindup_lim);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

};

#endif /* PID_H */
