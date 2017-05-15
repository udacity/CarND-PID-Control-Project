#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <iomanip>
#include <chrono>

// for convenience
using json = nlohmann::json;
using namespace std;
using namespace std::chrono;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

void read_args(int argc, char* argv[], double& Kp, double& Ki, double& Kd, double& throttle,
               bool& twiddle, double& dKp, double& dKi, double& dKd, long long& tolerance, 
               bool& adaptiveThrottle, int& cteWindowSize,
               bool& breakOnHighCte, double& highCte) {
  //Kp = 0.125; Kd = 3.0; Ki = 0.0000; throttle = 0.5; // quite safe
  Kp = 0.088; Kd = 2.875; Ki = 0.0000; throttle = 0.95; // speedy
  twiddle =  false; dKp = 0.5; dKd = 1.2; dKi = 0.000001; tolerance = 250000ll;
  adaptiveThrottle = false;  cteWindowSize = 40; // on be default for speedy
  breakOnHighCte = false; highCte = 3.0;
  auto i = 1;

  while (argc > i) {
    switch (tolower(argv[i++][0])) {
    case 'y':
    case 't':
      twiddle = true;
      if (argc > i) dKp = stod(argv[i++]);
      if (argc > i) dKi = stod(argv[i++]);
      if (argc > i) dKd = stod(argv[i++]);
      if (argc > i) tolerance = stoi(argv[i++]);
      break;
    case 'a':
      adaptiveThrottle = true;
      if (argc > i) cteWindowSize = stoi(argv[i++]);
      break;
    case 'b':
      breakOnHighCte = true;
      if (argc > i) highCte = stod(argv[i++]);
      break;
    default:
      Kp = stod(argv[i-1]);
      if (argc > i) Ki = stod(argv[i++]);
      if (argc > i) Kd = stod(argv[i++]);
      if (argc > i) throttle = stod(argv[i++]);
      break;
    }
  }
}

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

int main(int argc, char* argv[])
{
  uWS::Hub h;
  double Kp, Ki, Kd, throttle, dKp, dKi, dKd, highCte; long long tolerance; 
  bool twiddle, adaptiveThrottle, breakOnHighCte;
  int cteWindowSize;
  read_args(argc, argv, Kp, Ki, Kd, throttle, twiddle, dKp, dKi, dKd, tolerance, adaptiveThrottle, cteWindowSize, breakOnHighCte, highCte);
  cout << "Kp = " << Kp << ", Ki = " << Ki << ", Kd = " << Kd << ", throttle = " << throttle << endl;
  cout << "dKp= " << dKp << ", dKi= " << dKi << ", dKd= " << dKd << ", tolerance=" << tolerance << endl;
  cout << "adaptiveThrottle=" << adaptiveThrottle << ", cteWindowSize=" << cteWindowSize << endl;
  cout << "breakOnHighCte=" << breakOnHighCte << ", highCte=" << highCte << endl;

  PID pid; MovingOnlineStats cteStats{ cteWindowSize }; auto t1 = steady_clock::now();
  // Initialize the pid variable.
  long long iter = 0ll;
  pid.Init(Kp, Ki, Kd);
  pid.ControlFunction = [](double e) { return e > 1.0 ? 1.0 : e < -1.0 ? -1.0 : e; };
  pid.InitTwiddle(twiddle, dKp, dKi, dKd);
  pid.TwiddleErrorFunction = [](double tolerance, double num_steps, double stdevp) { return num_steps; };

  std::cout << std::setiosflags(std::ios::fixed) << std::setprecision(6);
  h.onMessage([&](uWS::WebSocket<uWS::SERVER>* ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(std::string(data));
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = stod(j[1]["cte"].get<string>());
          double speed = stod(j[1]["speed"].get<string>());
          double angle = stod(j[1]["steering_angle"].get<string>());
          if (iter < 2 && fabs(cte)>5.0) pid.Restart(*ws);
          cout << "Input: Iter#" << iter <<",CTE=" << cte << ",Steering=" << angle << ",Speed:=" << speed;

          // Calcuate steering value here, remember the steering value is [-1, 1].
          auto steer_value = pid.ControlValue();  // TODO: think of scaling sensitivity down with speed ...  *(1.0 - (speed - 60.0) / 120.0);
          auto throttle_value = throttle;
          cteStats.addData(cte);
          pid.UpdateError(cte);
          iter++;

          if (twiddle) {
            if ((iter > 50 && (speed < 0.015)) || fabs(cte)>4) { //|| pid.err_stats.stdevs()>1.0)) || fabs(cte)>3.2) {
              pid.Twiddle(tolerance, iter);
              if (pid.twiddle_iter > 500) exit(1);
              iter = 0;
              cteStats.clear();
              pid.Restart(*ws);
            }
          }
          if (adaptiveThrottle) {
            throttle_value += (iter > cteWindowSize ? 0.01 - cteStats.varp() : 0); // Throttle adjusted by moving variance of last few CTE
          }
          if (breakOnHighCte && fabs(cte) > highCte && fabs(angle)>7.5) { // apply break if error beyond this
            // NB: doing nothing seems to give good result as well
            throttle_value = -throttle;                     // tried scaling the negative throttle, e.g -throttle/2.0;
            //steer_value = steer_value < 0.0 ? -1.0 : 1.0; // tried forcing to max steer in mediam direction
            //throttle_value = throttle*1.25;               // tried increasing the throttle, with max steer
          }

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value; // 0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          cout << "\tOutput: Steer=" << steer_value << ",Throttle=" << throttle_value;
          cout << ",ctevarp=" << cteStats.varp() << ",n=" << cteStats.n();
          auto t2 = steady_clock::now();
          cout << ",ms=" << (duration_cast<duration<double>>(t2 - t1)).count()*1000 << endl;
          t1 = t2;
          ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        cout << msg << endl;
        ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h,&iter,&pid](uWS::WebSocket<uWS::SERVER>* ws, uWS::HttpRequest req) {
    iter = 0;
    cout << "Connected!!!" << endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER>* ws, int code, char *message, size_t length) {
    ws->close();
    cout << "Disconnected" << endl;
  });

  int port = 4567;
  if (h.listen("0.0.0.0", port))
  {
    cout << "Listening to port " << port << endl;
  }
  else
  {
    cerr << "Failed to listen to port" << endl;
    return -1;
  }
  h.run();
}
