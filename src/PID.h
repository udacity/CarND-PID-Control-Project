#ifndef PID_H
#define PID_H

class PID {
public:
  PID();

  virtual ~PID();

  void Init(double kp, double ki, double kd);

  void SetPID(double kp, double ki, double kd);

  void SetDt(double dt);

  void ClearError();

  void SetDeadBand(double deadband);

  double UpdateError(double cte);

  double TotalError();

  double p_error_m;
  double i_error_m;
  double d_error_m;

  double kp_m;
  double ki_m;
  double kd_m;

  double dt_m = 0.1;
  double deadband_m = 0;

  double max_output_m = 1.0, min_output_m = -1.0;
};

#endif /* PID_H */
