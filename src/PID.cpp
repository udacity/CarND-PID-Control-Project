#include "PID.h"
#include <iostream>
#include <cmath>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() : ControlFunction(&DefaultControlFunction) {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  //this->Kp = Kp; this->Ki = Ki; this->Kd = Kd;
  K[0] = Kp; K[2] = Ki; K[1] = Kd;  // K[1] is D (not I)
  p_error = i_error = d_error = 0.0;
}
void PID::InitTwiddle(bool twiddle, double dKp, double dKi, double dKd) {
  twiddlep = twiddle;
  dK[0] = dKp; dK[2] = dKi; dK[1] = dKd;   // K[1] is D (not I)
  best_steps = twiddle_iter = twiddle_ki = 0;
  best_stdevp = 1.0E127; best_perf = 0;
  stage = TwiddleStage::Start;
}

double PID::TotalError() const {
  //return -Kp*p_error - Kd*d_error - Ki*i_error;
  return -K[0] * p_error - K[1] * d_error - K[2] * i_error;
}

void PID::UpdateError(double cte) {
  d_error = cte - p_error;
  p_error = cte;
  if (fabs(cte)>0.0001) // tweak, dont integrate small errors.
    i_error += cte;
  err_stats.addData(cte);
}

double PID::ReplayAndEvalError() {
  return 0.0;
  p_error = i_error = d_error = 0.0;
  double t_err = 0.0;
  //for (const auto e : cte_hist) {
  //  UpdateError(e, true);
  //  auto t = TotalError();
  //  t_err += t*t;
  //}
  //return t_err/cte_hist.size();
}

void PID::Twiddle(long long tolerance, long long num_steps) {
  if (!twiddlep) return;
  if (num_steps < 50) return;
  //cerr << "********************************************************************************" << endl;
  cerr << endl << "*********************************** TWIDDLING **********************************" << endl;
  //cerr << "********************************************************************************" << endl;
  if (best_perf > tolerance) return;
  auto stdevp = err_stats.stdevp();
  auto perf = TwiddleErrorFunction(tolerance, num_steps, stdevp); // num_steps / (stdevp*stdevp);
  cerr << "Iteration#" << twiddle_iter++ << ", best (steps,stdevp,perf)=(" << best_steps << "," << best_stdevp << "," << best_perf << ")";
  cerr << endl << "; this run (num_steps,stdevp,perf)=(" << num_steps << "," << stdevp << "," << perf << ")" << endl;
  cerr << "; with:   p = " <<  K[0] << ", i = " <<  K[2] << ", d = " <<  K[1] << endl;
  cerr << "tolerance=" << tolerance << ", ki=" << twiddle_ki << ", stage=" << stage << endl;
  p_error = i_error = d_error = 0.0;
  switch (stage) {
  case TwiddleStage::Start:
    K[twiddle_ki] += dK[twiddle_ki];
    stage = TwiddleStage::Inc;
    cerr << "  p=" << K[0] << ",   i=" << K[2] << ",   d=" << K[1] << endl;
    cerr << "dKp=" << dK[0] << ", dKi=" << dK[2] << ", dKd=" << dK[1] << endl;
    cerr << "new stage=" << stage << endl;
    err_stats.clear();
    return;
  case TwiddleStage::Inc:
    if (perf > best_perf) {
      best_perf = perf;
      if (num_steps > best_steps) best_steps = num_steps; 
      if (err_stats.stdevp()<best_stdevp) best_stdevp = err_stats.stdevp();
      dK[twiddle_ki] *= 1.1;
      break;
    } else {
      K[twiddle_ki] -= 2 * dK[twiddle_ki];
      stage = TwiddleStage::Dec;
      cerr << "  p=" << K[0] << ",   i=" << K[2] << ",   d=" << K[1] << endl;
      cerr << "dKp=" << dK[0] << ", dKi=" << dK[2] << ", dKd=" << dK[1] << endl;
      cerr << "new stage=" << stage << endl;
      err_stats.clear();
      return;
    }
  case TwiddleStage::Dec:
    if (perf > best_perf) {
      best_perf = perf;
      if (num_steps > best_steps) best_steps = num_steps;
      if (err_stats.stdevp()<best_stdevp) best_stdevp = err_stats.stdevp();
      dK[twiddle_ki] *= 1.1;
      break;
    } else {
      K[twiddle_ki] += dK[twiddle_ki];
      dK[twiddle_ki] *= 0.9;
    }
  }
  twiddle_ki++;
  if (twiddle_ki == 3) {
    twiddle_iter++;
    twiddle_ki = 0;
  }
  err_stats.clear();
  stage = TwiddleStage::Start;
  //cerr << "********************************************************************************" << endl;
  cerr << "*********************************** TWIDDLED ***********************************" << endl;
  //cerr << "********************************************************************************" << endl;
  cerr << "  p=" <<  K[0] << ",   i=" <<  K[2] << ",   d=" <<  K[1] << endl;
  cerr << "dKp=" << dK[0] << ", dKi=" << dK[2] << ", dKd=" << dK[1] << endl;
  cerr << "new stage=" << stage << ", ki=" << twiddle_ki << endl;
}

void PID::Reset() {
  p_error = i_error = d_error = 0.0;
  err_stats.clear(); //stage = TwiddleStage::Start; twiddle_ki = 0;
}
void PID::Restart(uWS::WebSocket<uWS::SERVER>& ws) {
  cerr << "*********************************** RESTARTING *********************************" << endl;
  Reset();
  std::string reset_msg = "42[\"reset\",{}]";
  ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
}
