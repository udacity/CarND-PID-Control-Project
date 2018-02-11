#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

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

int main()
{
  uWS::Hub h;

  PID pid;
  // TODO: Initialize the pid variable.
  // Optimize hyper parameters
  // Climping to the left, back and climbing to the right - need reduce the P
  //pid.Init(0.5, 0.5, 0.5);
  
  // The same error - climb to left - back - right
  //pid.Init(0.3, 0.5, 0.5);
  // It's not better
  //pid.Init(0.3, 0.25, 0.5);
  // does not help
  //pid.Init(0.2, 0.25, 0.5);
  // Better but stopped before the bridge and go zig zac
  //pid.Init(0.2, 0.0, 0.5);
  // better a little - can go almost near the bridge with less zig zac
  //pid.Init(0.2, 0.0, 1.0);
  // Much better - can finish a round but move like a snake - zig zac
  //pid.Init(0.2, 0.0, 1.5);
  // Completed round with some zig zac
  //pid.Init(0.2, 0.0, 2.0);
  // improve - completed round with a little zig zac on the curve
  //pid.Init(0.2, 0.0, 2.5);
  // more zig zac than 0.2, 0.0, 2.5
  // pid.Init(0.2, 0.0, 3.0);
  // Would like the car in the center
  // more center but still zig zac
  //pid.Init(0.3, 0.0, 2.75);
  // better with a little zig zac on the curve
  //pid.Init(0.25, 0.0, 2.5);
  // Cannot complete
  //pid.Init(0.25, 0.1, 2.5);
  // OH NO
  //pid.Init(0.25, 0.05, 2.5);
  // OH NO
  //pid.Init(0.25, 0.01, 2.5);
  // Can complete round with zig zac
  //pid.Init(0.25, 0.0004, 2.5);
  // more zig zac
  //pid.Init(0.25, 0.001, 2.5);
  // a little improve on zig zac
  //pid.Init(0.25, 0.0004, 2.2);
  // continue improve
  //pid.Init(0.23, 0.0004, 2.2);
  // more zig zac
  //pid.Init(0.23, 0.0002, 2.2);
  // more zig zac
  //pid.Init(0.23, 0.0004, 2.0);
  // a little less zig zac
  //pid.Init(0.23, 0.0004, 2.4);
  // a little improvement
  //pid.Init(0.22, 0.0004, 2.4);
  //pid.Init(0.224, 0.0004, 2.4);
  // ZIG ZAC
 // pid.Init(0.224, 0.0004, 2.2);
  //pid.Init(0.224, 0.0004, 2.3);
  pid.Init(0.224, 0.0004, 2.4);
  //pid.Init(0.25, 0.0, 2.5);

  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          pid.UpdateError(cte);
          steer_value = pid.TotalError();
          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;
          // Play around with the throttle and speed
          double throttle = 0.2;
          if(fabs(steer_value) < 0.1) {
            if(speed < 35) {
              throttle = 0.5;
            } else {
              throttle = 0.3;
            }
          } else if(fabs(steer_value) > 0.5) {
            if(speed > 30)
              throttle = -0.3;
            else
              throttle = 0.1;
          }

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
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
