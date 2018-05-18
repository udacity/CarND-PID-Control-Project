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

// Checks if the SocketWeb event has JSON data.
std::string hasData(std::string s, size_t length) {
    return s.substr(0, length);
}

int main()
{
  uWS::Hub h;

  PID pid;
  //TODO: Initialize the pid variable.

 

  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> *ws, char *data, size_t length, uWS::OpCode opCode) {

    if (length && length > 2)
    {
      auto s = hasData(std::string(data),length);
      if (s != "") {
        auto j = json::parse(s);
          int process = std::stod(j.at("telemetry")["process"].get<std::string>());
          if(process == 1)
          {
            //There are data
            // j[1] is the data JSON object
            double cte = std::stod(j.at("telemetry")["cte"].get<std::string>());
            double speed = std::stod(j.at("telemetry")["speed"].get<std::string>());
            double angle = std::stod(j.at("telemetry")["steering_angle"].get<std::string>());
            double steer_value;


            /*
            * TODO: Calcuate steering value here, remember the steering value is
            * [-1, 1].
            * NOTE: Feel free to play around with the throttle and speed. Maybe use
            * another PID controller to control the speed!
            */

            // DEBUG
            std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

            json msgJson;
            msgJson["steering_angle"] = steer_value;
            msgJson["throttle"] = 0.3;
            auto msg = "{\"steer\"," + msgJson.dump() + "}";
            std::cout << msg << std::endl;
            ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);

        }
      else {
        // Manual driving
        std::string msg = "{\"manual\",{}}";        
        ws->send(msg.data(), msg.length(), uWS::OpCode::TEXT);

      }
    }
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> *ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  
  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> *ws, int code, char *message, size_t length) {
    ws->close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 3001;
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
