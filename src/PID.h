#ifndef PID_H
#define PID_H

#include <chrono>
#include <ctime>

// #include <time.h>

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;
  double old_d_error;
  bool has_old_d_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  std::chrono::time_point<std::chrono::system_clock> old_t, t;

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
  void Init(double Kp, double Ki, double Kd);

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
