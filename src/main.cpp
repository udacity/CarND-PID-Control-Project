#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
#include <ctime>
#include <math.h>
#include <chrono>

#define DO_TWIDDLE 0 //1---> Training, 0--->Testing
#define REWARD_SIGNAL_TYPE 1

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

  // read parameters from a file
  static const std::vector<float> params = pid.GetParamters("parameters.csv");


  if(DO_TWIDDLE==1){
    pid.Init(params[pid.KP], params[pid.KI], params[pid.KD]);
  }else{
    pid.Init(0.21, 0.00399995, 3.16046);
  }
  static const auto start_time = std::chrono::system_clock::now();
  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {

    if (pid.initial_command == true){
      std::cout << "initial" << std::endl;
      pid.initial_command = false;
      pid.timer_for_episode.reset();
      pid.timer_for_command.reset();
    }
    //---- Episode Termination Criteria ----
    //clock_t current_time = clock(); //clock() does not provide a proper time (because this is cpu time?)
    //double time_duration = double(current_time - pid.begin_time_duration)/(double) CLOCKS_PER_SEC;

    std::cout << "total time=" << pid.timer_for_episode.elapsed() << std::endl;

    /*
     * In training, each episode is 70 seconds. After that the program automatically shuts off.
     *
     */
    auto end_time = std::chrono::system_clock::now(); //You can use chrono in c++11
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    if ((DO_TWIDDLE==1)&&(elapsed.count()/1000.0 > 70.0)){ //70
      std::cout << "time duration is " << elapsed.count()/1000 << ". This episode is done. Please reset the simulator and restart pid." << std::endl;
      if(REWARD_SIGNAL_TYPE==0){
        pid.Twiddle(0.2, pid.total_error);
      }else if(REWARD_SIGNAL_TYPE==1){
        double reward_signal = pid.total_error - 1000.0*pid.total_distance;
        pid.Twiddle(0.2, reward_signal);
      }
      pid.LogData("parameters.csv", params);
      exit (EXIT_FAILURE);
      return 1;
    }

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
          steer_value = pid.ComputeSteer();
          double throttle;
          double max_throttle = 0.3; //0.5 was used for training

          /*
           * throttle is adjuated based on the current steering angle.
           */
          throttle = (max_throttle-0.1) * (1-fabs(steer_value))+0.1; //used for training

          //compute elapsed time
          double elapsed_time_seconds = pid.ComputeDeltaTime();

          //Compute the travel distance
          double angle_radian = angle/360.0 * M_PI;
          double speed_in_mile_per_seconds = speed / 3600.0;
          double delta_distance = speed_in_mile_per_seconds * elapsed_time_seconds;

          //compte total error
          double distance = pid.ComputeTotalDistance(delta_distance);
          pid.TotalError();

          // DEBUG
          std::cout << "Delta time: " << elapsed_time_seconds << " speed: " << speed << " angle_radian: " << angle_radian << " delta_dist: " << delta_distance << std::endl;
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value << " Distance: " << distance << " Error: " << pid.total_error << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle; //0.3; //0.3;
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
