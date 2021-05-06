#ifndef Tune_H
#define Tune_H


class Tune {
 public:
  /**
   * Constructor
   */
  Tune();

  /**
   * Destructor.
   */
  virtual ~Tune();

  /**
   * PID Coefficients
   */ 
  //double Kp;
  //double Ki;
  //double Kd;
  std::vector<double> p;
  std::vector<double> dp;
  // For use in coordinate ascent
  double threshold;
  double best_err;
  double err;
  /**
   * Initialize Tune.
   * @param (Kp_, Ki_, Kd_) The initial PID coefficients
   */
  void Init(double orig_p, double orig_i, double orig_d);

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  void UpdateError(double cte);

  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  double TotalError();

  /**
   * Reset the error values before each iteration of progressive tuning
   */
  void ResetError();

 private:
  /**
   * PID Errors
   */
  double p_error;
  double i_error;
  double d_error;

    // For use in calculations
  double prev_cte;

  std::vector<double> new_pid{};
  std::vector<double> delta_pid{};
  double tolerance;
  double best_errr;
};

#endif  // Tune_H
