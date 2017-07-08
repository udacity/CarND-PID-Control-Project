#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
#include <pid_auto_tuner.h>

#include <atomic>
#include <thread>

#include <chrono>
#include <strstream>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  } else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

// kp ki kd :( 0.100, 0.000, 0.050 ) for 50mph
// kp ki kd :( 0.150, 0.000, 0.060 ) for 30mph
//     Kp 0.04537	 Ki 0.02211	 Kd 0.02327

// A(7.8, -1.6) Pu(10.5,4.5) Ku 0.02696
// P	 Kp 0.01348	 Ki 0.00000	 Kd 0.00000
// PI	 Kp 0.01213	 Ki 0.00194	 Kd 0.00000
// PD	 Kp 0.02157	 Ki 0.00000	 Kd 0.02035
// PID	 Kp 0.01618	 Ki 0.00429	 Kd 0.01526
// PIR	 Kp 0.01887	 Ki 0.00625	 Kd 0.02137
// Overshoot	 Kp 0.00890	 Ki 0.00236	 Kd 0.02216
// NoOvershoot	 Kp 0.00539	 Ki 0.00143	 Kd 0.01343

double kp_g = 0.1, ki_g = 0.001, kd_g = 0.06;
bool clear_error_g = false;
double throttle_g = 0;
double deadband_g = 0.0;
std::string tune_command_g;
void readCin(std::atomic<bool> &run) {
  std::string input;
  while (run.load()) {
    std::cout << "> ";
    getline(std::cin, input);
    if (input == "quit")
      run.store(false);
    else if (input[0] == 'c') {
      clear_error_g = true;
    } else if (input.find("tune") != std::string::npos) {
      tune_command_g = input.substr(4);
    } else {
      double val = std::stod(input.substr(1));
      switch (input[0]) {
      case 'p':
        kp_g = val;
        break;
      case 'i':
        ki_g = val;
        break;
      case 'd':
        kd_g = val;
        break;
      case 't':
        deadband_g = val;
        break;
      case 'v':
        throttle_g = val;
        break;
      }
    }
    if (input[0] == 'p' || input[0] == 'i' || input[0] == 'd')
      printf("kp ki kd :( %.3f, %.3f, %.3f )\n", kp_g, ki_g, kd_g);
  }
}

void CommandTune(std::stringstream &opts, bool &auto_tune,
                 PIDAutoTuner &pid_tuner) {
  std::string opt;
  opts >> opt;

  std::string printout;
  if (opt == "start") {
    auto_tune = true;
    printout = ("start auto tunning");
  } else if (opt == "stop") {
    auto_tune = false;
    pid_tuner.cancel();
    printout = ("stop auto tunning");
  } else if (opt == "noise") {
    double noise;
    opts >> noise;
    pid_tuner.setNoiseBand(noise);
    printout = ("noise band set to " + std::to_string(noise));
  } else if (opt == "trial") {
    int trial;
    opts >> trial;
    pid_tuner.setTrialCount(trial);
    printout = ("trial set to " + std::to_string(trial));
  } else if (opt == "osc") {
    double osc;
    opts >> osc;
    pid_tuner.setOscillate(osc);
    printout = ("oscillation set to " + std::to_string(osc));
  } else if (opt == "type") {
    std::string type;
    opts >> type;
    if (type == "relay") {
      pid_tuner.setAutoTuneType(PIDAutoTuner::AutoTuneType::Relay);
      printout = ("PID tuner type set to " + type);
    } else if (type == "p") {
      pid_tuner.setAutoTuneType(PIDAutoTuner::AutoTuneType::ConstantP);
      printout = ("PID tuner type set to " + type);
    } else {
      printout = ("No such type, use relay, p");
    }
  } else if (opt == "p") {
    double p;
    opts >> p;
    pid_tuner.setP(p);
    printout = ("P value set to" + std::to_string(p));
  } else if (opt == "ctrl") {
    std::string type;
    opts >> type;
    if (type == "p") {
      pid_tuner.setControlType(PIDAutoTuner::ControlType::P);
      printout = ("PID type set to " + type);
    } else if (type == "pi") {
      pid_tuner.setControlType(PIDAutoTuner::ControlType::PI);
      printout = ("PID type set to " + type);
    } else if (type == "pd") {
      pid_tuner.setControlType(PIDAutoTuner::ControlType::PD);
      printout = ("PID type set to " + type);
    } else if (type == "pid") {
      pid_tuner.setControlType(PIDAutoTuner::ControlType::PID);
      printout = ("PID type set to " + type);
    } else if (type == "pir") {
      pid_tuner.setControlType(PIDAutoTuner::ControlType::PIR);
      printout = ("PID type set to " + type);
    } else if (type == "os") {
      pid_tuner.setControlType(PIDAutoTuner::ControlType::Overshoot);
      printout = ("PID type set to " + type);
    } else if (type == "nos") {
      pid_tuner.setControlType(PIDAutoTuner::ControlType::NoOvershoot);
      printout = ("PID type set to " + type);
    } else {
      printout = ("No such type! Use p pi pd pid pir os nos");
    }
  }

  std::cout << printout << "\n";
}

int main() {
  uWS::Hub h;

  PID pid;
  pid.Init(kp_g, ki_g, kd_g);

  std::atomic<bool> run(true);
  std::thread cin_thread(readCin, std::ref(run));

  auto start = std::chrono::high_resolution_clock::now();

  bool auto_tune = false;
  PIDAutoTuner pid_tuner;
  pid_tuner.setAutoTuneType(PIDAutoTuner::AutoTuneType::Relay);
  pid_tuner.setControlType(PIDAutoTuner::ControlType::PID);
  pid_tuner.setP(0.08);
  pid_tuner.setOscillate(.1);
  pid_tuner.setTrialCount(3);
  pid_tuner.setNoiseBand(0);

  h.onMessage([&pid, &start, &auto_tune, &pid_tuner](
      uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
      uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {

          auto end = std::chrono::high_resolution_clock::now();
          std::chrono::duration<double> diff = end - start;
          //          std::cout << "Dt = " << diff.count() << "\n";
          start = end;

          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          // double speed = std::stod(j[1]["speed"].get<std::string>());
          // double angle =
          // std::stod(j[1]["steering_angle"].get<std::string>());
          //          double heading =
          // std::stod(j[1]["psi"].get<std::string>());
          double steer_value;

          if (!tune_command_g.empty()) {
            std::stringstream opt(tune_command_g);
            CommandTune(opt, auto_tune, pid_tuner);
            tune_command_g = "";
          }

          // Ingnore the initial inaccurate duration measure
          if (diff.count() > 1.0)
            pid.SetDt(0.05);
          else
            pid.SetDt(diff.count());

          if (clear_error_g) {
            std::cout << "Clear PID errors\n";
            clear_error_g = false;
            pid.ClearError();
          }

          pid.SetPID(kp_g, ki_g, kd_g);
          pid.SetDeadBand(deadband_g);

          steer_value = pid.UpdateError(cte);

          // If auto tunning, it overwrites the steering output
          if (auto_tune) {
            auto_tune = pid_tuner.update(cte, steer_value);
            steer_value -= 0.44 / 24.56;
            if (auto_tune == false) {
              printf("Auto tune completed\n kp %.4f ki %.4f kd %.4f\n",
                     pid_tuner.getKp(), pid_tuner.getKi(), pid_tuner.getKd());

              pid.SetPID(pid_tuner.getKp(), pid_tuner.getKi(),
                         pid_tuner.getKd());
              pid.PrintPID();
              // Enable feedback control immediately to see the result
              pid.ClearError();
            }
          }

          // DEBUG
          // std::cout << "CTE: " << cte << " heading: " << heading <<
          // std::endl;
          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_g;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h, &pid](uWS::WebSocket<uWS::SERVER> ws,
                            uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
    pid.ClearError();
    pid.PrintPID();
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();

  run.store(false);
  cin_thread.join();
}
