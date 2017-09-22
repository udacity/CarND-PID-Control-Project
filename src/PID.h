#ifndef PID_H
#define PID_H
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <string>
#include <algorithm>
#include <iterator>
#include <iostream>
#include <cassert>
#include <sstream>
#include <ctime>
#include <math.h>
#include <chrono>
#include <ctime>


//The Timer Class
//References:
//https://stackoverflow.com/questions/728068/how-to-calculate-a-time-difference-in-c
//https://gist.github.com/gongzhitaao/7062087
class Timer //(C++11)
{
public:
    Timer() : beg_(clock_::now()) {}
    void reset() { beg_ = clock_::now(); }
    double elapsed() const {
        return std::chrono::duration_cast<second_>
            (clock_::now() - beg_).count(); }

private:
    typedef std::chrono::high_resolution_clock clock_;
    typedef std::chrono::duration<double, std::ratio<1> > second_;
    std::chrono::time_point<clock_> beg_;
};
//class Timer (C++03)
//{
//public:
//    Timer() { clock_gettime(CLOCK_REALTIME, &beg_); }
//
//    double elapsed() {
//        clock_gettime(CLOCK_REALTIME, &end_);
//        return end_.tv_sec - beg_.tv_sec +
//            (end_.tv_nsec - beg_.tv_nsec) / 1000000000.;
//    }
//
//    void reset() { clock_gettime(CLOCK_REALTIME, &beg_); }
//
//private:
//    timespec beg_, end_;
//};


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
  clock_t begin_time;
  clock_t begin_time_duration;
  clock_t end_time_duration;

  Timer timer_for_command;
  Timer timer_for_episode;
  double start_time;

  double total_error;
  double total_distance;

  bool initial_command;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  // These parameters for twiddle
  std::vector<double> p  = std::vector<double>(3);
  std::vector<double> dp = std::vector<double>(3);
  int index_for_twiddle;
  double best_err;
  int which_scope_in_twiddle;


  enum FILE_COLUMNS {NUM, KP, KD, KI,DKP,DKD,DKI,TWIDDLE_INDEX, WHICH_SCOPE, BEST_ERROR};

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

  /*
   * Compute the time diff
   */
  double ComputeDeltaTime();

  /*
   * Compute the total distance the vehicle drove
   */
  double ComputeTotalDistance(double delta_distance);

  /*
   * Parameter tuning using twiddle
   */
  double Twiddle(double tol, double err);

  /*
   * Save parameters and errors after each episode
   */
  void LogData(std::string file_name, std::vector<float> parameter_vector);

};

#endif /* PID_H */
