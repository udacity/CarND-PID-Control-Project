/*****************************************************************************/
/**
\author	  Malcolm Ma <malichaooo@gmail.com>
\date	    2017-07-07
\version	v1.0.0

\brief
A PID auto tuner to tune the PID coefficients using Ziegler–Nichols method
\details


*/
/*****************************************************************************/
#ifndef PID_AUTO_TUNER_H
#define PID_AUTO_TUNER_H
#include <vector>
#include <array>
class PIDAutoTuner {
 public:
  enum ControlType {
    P = 0,
    PI,
    PD,
    PID,          // classic PID
    PIR,          // Pessen Integral Rule
    Overshoot,    // some overshoot
    NoOvershoot,  // no overshoot
    Size
  };

  enum AutoTuneType { Relay, ConstantP };

  PIDAutoTuner();

  bool update(const double &input, double &output);

  void setNoiseBand(double);

  double getNoiseBand();

  void setTrialCount(const int val);

  void setAutoTuneType(const AutoTuneType &type);

  void setControlType(const ControlType &type);

  void setP(const double p);

  void cancel();

  void setOscillate(const double val);

  double getKp();

  double getKi();

  double getKd();

  double currentTime();

 private:
  enum class InputState { HighBand, MidBand, LowBand };

  typedef std::array<double, 3> PID_Coeff_t;

  struct Peak {
    double val;
    double time;
  };

  void calcParameters();

  double getKp(const int type);

  double getKi(const int type);

  double getKd(const int type);

  static const char control_type_string_m[ControlType::Size][12];

  double noise_band_m;
  bool running;
  double output_osc_m;
  double output_origin_m;
  double output_prev_m;
  double ku_m, pu_m;

  ControlType control_type_m;
  AutoTuneType tune_type_m;
  double const_p_m;
  InputState state_m;
  InputState state_prev_m;
  std::vector<Peak> tops_m;
  std::vector<Peak> bottoms_m;
  Peak cur_peak_m;
  size_t trial_count_m = 10;
  std::array<PID_Coeff_t, ControlType::Size> pid_coeff_m;
};

#endif  // PID_AUTO_TUNER_H
