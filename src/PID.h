#ifndef PID_H
#define PID_H
#include <uWS/uWS.h>
#include "OnlineStats.h"
#include <functional>

class PID {
public:
  PID();
  virtual ~PID();

  // Errors
  double p_error;
  double i_error;
  double d_error;

  double K[3];    // Coefficients. NB order: PDI, i.e: K[0]=P, K[2] = I, K[1] = D
  
  void Init(double Kp, double Ki, double Kd); // Initialize PID.
  void UpdateError(double cte);               // Update the PID error variables given cross track error
  double TotalError() const;                  // Calculate the total PID error.

  std::function<double(double total_error)> ControlFunction; // Maps total error to control value
  double ControlValue() const { return ControlFunction(TotalError()); } // Control value to be used

  std::function<double(double tolerance,double num_steps,double stdevp)> TwiddleErrorFunction; // Error to optimise Twiddle
  void InitTwiddle(bool twiddle, double dKp, double dKi, double dKd);
  void Twiddle(long long tolerance, long long num_steps);
  double ReplayAndEvalError();
  bool twiddlep;
  double dK[3];	        // tune params using twiddle
  long long best_steps; // tune params using twiddle
  double best_stdevp;
  double best_perf;
  int twiddle_iter;
  int twiddle_ki;
  enum TwiddleStage { Start, Inc, Dec, Done } stage;
  OnlineStats err_stats;

  void Reset();
  void Restart(uWS::WebSocket<uWS::SERVER>& ws);

private:
  static double DefaultControlFunction(double error) { return error; } // return unmapped
};

#endif /* PID_H */
