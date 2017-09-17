#ifndef PID_H
#define PID_H
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  double previous_cte;
  std::fstream parameter_file;
  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

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


  /*
  * Compute Steering angle.
  */
  double ComputeSteer();

  /*
    * Get paramters from a file
  */
  std::vector<float>  GetParamters(std::string file_name);

};

#endif /* PID_H */
