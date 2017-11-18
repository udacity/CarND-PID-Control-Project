#include <uWS/uWS.h>
#include <iostream>
#include <fstream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

#define DEBUG 0
#define TARGET_SPEED 32.

using namespace std;

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
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main(int argc, char *argv[])
{
  uWS::Hub h;

  PID ctrl_steer;  // steering controller
  PID ctrl_speed;  // speed controller

  ctrl_steer.Init(0.12, 0.001, 1.1, .05, 100.);
  ctrl_speed.Init(0.075, 0.0001, 0., 0., 0.);

  h.onMessage([&ctrl_steer, &ctrl_speed](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;
		  double throttle_cmd;

		  // DO STEER CONTROL
		  ctrl_steer.UpdateError(cte);
		  steer_value = ctrl_steer.TotalError();
		  steer_value=std::max(std::min(steer_value,1.), -1.);  // limit to [-1,1]
#if (DEBUG)
		  cout << cte << ", " << ctrl_steer.p_error << ", " << ctrl_steer.i_error << ", " << ctrl_steer.d_error << ", " << steer_value << ", "; // there is more below

          // DEBUG
          //std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;
#endif
		  // DO SPEED CONTROL
		  ctrl_speed.UpdateError(speed-TARGET_SPEED);
		  throttle_cmd = ctrl_speed.TotalError();

#if (DEBUG)
		  cout << (speed - TARGET_SPEED) << ", " << ctrl_speed.p_error << ", " << ctrl_speed.i_error << ", " << ctrl_speed.d_error << ", " << throttle_cmd << endl;

		  // DEBUG
		  //std::cout << "speed_error: " << (speed - TARGET_SPEED) << " Throttle Command: " << throttle_cmd << std::endl;
#endif

          json msgJson;
          msgJson["steering_angle"] = steer_value;
		  msgJson["throttle"] = throttle_cmd; // 0.6; // 0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
//          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
