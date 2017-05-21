#ifndef TWIDDLE_H
#define TWIDDLE_H

class Twiddle {
public:
  Twiddle();
  virtual ~Twiddle();

  void Init(double Kp, double Ki, double Kd);
  bool IterationAndErrorCount(double cte);
  bool UpdateParams();
  double* GetP();
  double GetBestError();
  double SumOfDp();

private:
  int state;
  int iteration;  // the number of checks
  int p_idx;  // parameters index
  double err_;
  double best_err_;
  double p_[3] = {0.0, 0.0, 0.0};
  double dp_[3] = {1.0, 0.0, 1.0};
};

#endif /* TWIDDLE_H */
