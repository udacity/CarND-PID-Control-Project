#include <pid_auto_tuner.h>
#include <chrono>

PIDAutoTuner::PIDAutoTuner() {
  control_type_m = ControlType::PID;  // default to PI
  tune_type_m = AutoTuneType::ConstantP;
  noise_band_m = 0.2;
  running = false;
  output_osc_m = 0.02;
  const_p_m = 0.01;
  trial_count_m = 3;

  pid_coeff_m[ControlType::P] = PID_Coeff_t{0.5, 0, 0};
  pid_coeff_m[ControlType::PI] = PID_Coeff_t{0.45, 0.542, 0};
  pid_coeff_m[ControlType::PD] = PID_Coeff_t{0.8, 0, 0.1};
  pid_coeff_m[ControlType::PID] = PID_Coeff_t{0.6, 1.2, 0.075};
  pid_coeff_m[ControlType::PIR] = PID_Coeff_t{0.7, 1.75, 0.105};
  pid_coeff_m[ControlType::Overshoot] = PID_Coeff_t{0.33, 0.66, 0.1089};
  pid_coeff_m[ControlType::NoOvershoot] = PID_Coeff_t{0.2, 0.4, 0.066};
}

const char PIDAutoTuner::control_type_string_m[ControlType::Size][12]={
  "P",
  "PI",
  "PD",
  "PID",
  "PIR",
  "Overshoot",
  "NoOvershoot"
};

bool PIDAutoTuner::update(const double& input, double& output) {
  if (!running) {  // initialize working variables the first time around
    running = true;
    // Give it an initial disturbance
    output_origin_m = output;
    output = output_origin_m + output_osc_m;
    output_prev_m = output;

    state_m = InputState::MidBand;
    state_prev_m = InputState::MidBand;
    tops_m.clear();
    bottoms_m.clear();
    cur_peak_m.time = -1;
    cur_peak_m.val = input;
  }

  // Don't record the MidBand state
  if (state_m != InputState::MidBand) state_prev_m = state_m;
  if (input > noise_band_m)
    state_m = InputState::HighBand;
  else if (input < - noise_band_m)
    state_m = InputState::LowBand;
  else
    state_m = InputState::MidBand;

  // Update the peak value and record if neccessary
  size_t peak_size = bottoms_m.size() + tops_m.size();
  switch (state_m) {
    case InputState::HighBand:
      if (state_prev_m != state_m && cur_peak_m.time > 0) {
        bottoms_m.push_back(cur_peak_m);
        double percent =
            (bottoms_m.size() + tops_m.size()) * 1.0 / (trial_count_m * 2.0);
        printf("[%.1f%%], Capture trough %.1f at %.1f\n", percent * 100,
               cur_peak_m.val, cur_peak_m.time);
      }

      if (input > cur_peak_m.val) {
        cur_peak_m.val = input;
        cur_peak_m.time = currentTime();
      }

      if (tune_type_m == AutoTuneType::Relay)
        output = output_origin_m - output_osc_m;
      else if (tune_type_m == AutoTuneType::ConstantP)
        output = (- input) * const_p_m;

      output_prev_m = output;
      break;
    case InputState::LowBand:
      if (state_prev_m != state_m && cur_peak_m.time > 0) {
        tops_m.push_back(cur_peak_m);
        double percent =
            (bottoms_m.size() + tops_m.size()) * 1.0 / (trial_count_m * 2.0);
        printf("[%.1f%%], Capture peak %.1f at %.1f\n", percent * 100,
               cur_peak_m.val, cur_peak_m.time);
      }

      if (input < cur_peak_m.val) {
        cur_peak_m.val = input;
        cur_peak_m.time = currentTime();
      }

      if (tune_type_m == AutoTuneType::Relay)
        output =  output_origin_m + output_osc_m;
      else if (tune_type_m == AutoTuneType::ConstantP)
        output = (- input) * const_p_m;

      output_prev_m = output;
      break;
    case InputState::MidBand:
      output = output_prev_m;
      break;
  }

  // There are new peaks captured, calculate the parameters!
  if (peak_size != tops_m.size() + bottoms_m.size()) calcParameters();
  if (tops_m.size() >= trial_count_m && bottoms_m.size() >= trial_count_m) {
    running = false;
    output = output_origin_m;
    return false;
  }

  return true;
}

double PIDAutoTuner::getKp() { 
  return pid_coeff_m[control_type_m][0] * ku_m; 
}

double PIDAutoTuner::getKi() {
  return pid_coeff_m[control_type_m][1] * ku_m / pu_m;
}

double PIDAutoTuner::getKd() {
  return pid_coeff_m[control_type_m][2] * ku_m * pu_m;
}

double PIDAutoTuner::getKp(const int type) { 
  return pid_coeff_m[type][0] * ku_m; 
}

double PIDAutoTuner::getKi(const int type) {
  return pid_coeff_m[type][1] * ku_m / pu_m;
}

double PIDAutoTuner::getKd(const int type) {
  return pid_coeff_m[type][2] * ku_m * pu_m;
}

void PIDAutoTuner::setControlType(const PIDAutoTuner::ControlType& type) {
  control_type_m = type;
}

double PIDAutoTuner::currentTime() { 
  auto now = std::chrono::system_clock::now().time_since_epoch();
  return  now.count()/1e9;}

void PIDAutoTuner::calcParameters() {
  if (tops_m.size() < 2 || bottoms_m.size() < 2) return;

  double max = 0, min = 0;
  int top_size = tops_m.size();
  int bottom_size = bottoms_m.size();
  for (auto& top : tops_m) {
    max += top.val;
  }
  max /= top_size;

  for (auto& bottom : bottoms_m) {
    min += bottom.val;
  }
  min /= bottom_size;

  double time_t = (tops_m.back().time - tops_m.front().time) / (top_size - 1);
  double time_b =
      (bottoms_m.back().time - bottoms_m.front().time) / (bottom_size - 1);

  ku_m = 4 * (2 * output_osc_m) / ((max - min) * 3.14159);
  pu_m = (time_t + time_b) / 2.0;

  printf("A(%.1f, %.1f) Pu(%.1f,%.1f) Ku %.5f\n", max, min, time_t, time_b,
         ku_m);
  for(int i=0;i<ControlType::Size;i++){
    printf("%s\t Kp %.5f\t Ki %.5f\t Kd %.5f\n",control_type_string_m[i],
                                                  getKp(i), getKi(i), getKd(i));
  }
}

void PIDAutoTuner::setNoiseBand(double Band) { noise_band_m = Band; }

double PIDAutoTuner::getNoiseBand() { return noise_band_m; }

void PIDAutoTuner::setTrialCount(const int val) { trial_count_m = val; }

void PIDAutoTuner::setAutoTuneType(const PIDAutoTuner::AutoTuneType& type) {
  tune_type_m = type;
}

void PIDAutoTuner::setP(const double p) { const_p_m = p; }

void PIDAutoTuner::cancel() { running = false; }

void PIDAutoTuner::setOscillate(const double val) { output_osc_m = val; }
