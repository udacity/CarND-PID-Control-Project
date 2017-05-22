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

double SPEED_GOAL = 50;

int main(int argc, const char *argv[])
{
  uWS::Hub h;

  PID pid;
  PID pid_throttle;

  double kp = 0.1;
  double ki = 0.001;
  double kd = 4.0;
  int max_iterations = 0;
  int iteration = 0;
  double error = 0;

  if (argc == 4 ) {
    kp = strtod(argv[1], NULL);
    ki = strtod(argv[2], NULL);
    kd = strtod(argv[3], NULL);
  } else if (argc == 5) {
    kp = strtod(argv[1], NULL);
    ki = strtod(argv[2], NULL);
    kd = strtod(argv[3], NULL);
    max_iterations = strtod(argv[4], NULL);
  } else {
    std::cout << "Usage without twiddle ./pid kp ki kd" << std::endl;
    std::cout << "Usage with twiddle ./pid kp ki kd iterations" << std::endl;
    return -1 ;
  }

  pid.Init(kp, ki, kd);
  pid_throttle.Init(kp, ki, kd);

  h.onMessage([&max_iterations, &error, &iteration, &pid, &pid_throttle](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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

          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */

          pid.UpdateError(cte);
          double steer_value = pid.TotalError();
          steer_value = std::max(steer_value, -1.0);
          steer_value = std::min(steer_value, 1.0);

          double speed_goal = 60;
          pid_throttle.UpdateError(fabs(speed_goal - speed));

          double throttle = fabs(pid.TotalError());
          throttle = std::max(0.3, throttle);

          if (speed > speed_goal) {
            throttle = 0;
          }
          throttle = throttle / 10 * speed_goal;

          // it twiddle active
          if (max_iterations > 0) {
              iteration++;
              error += fabs(cte);
              // if the number of iterations is done
              // or the car goes out of the road, the program exits
              if ((iteration > max_iterations) || (fabs(cte) > 5)) {
                // if the car goes out of the road before the last iteration,
                // the error value is fulfilled with a proportional value, to avoid
                // falses "best_errors"
                if (fabs(cte) > 5) {
                  error += (max_iterations - iteration) * 5;
                }
                std::cout.setf(std::ios::fixed);  // to avoid output scientific notation
                std::cout << "Error: " << error << std::endl ;
                std::string reset_msg = "42[\"reset\", {}]";
                ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
                exit(0);
              }
          } else {
            std::cout << "CTE: " << cte
                    << " Speed: " << speed
                    << " Steering: " << steer_value
                    << " Throttle: " << throttle
                    << std::endl;
          }

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle;
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
