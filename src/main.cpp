#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"
#include "Tune.cpp"
//#include "Tune.h"
#include <vector>

// for convenience
using nlohmann::json;
using std::string;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main() {
  uWS::Hub h;

  PID pid;
  Tune tune;
  /**
   * TODO: Initialize the pid variable.
   */
  bool do_tune = false; //true;
  double init_p_coeff = 0.06643; // 0.0653159; // 0.05;
  double init_i_coeff = 0.00119783; // 0.00118822; //0.001;
  double init_d_coeff = 1.36291; // 1.47583; //1.4;
  double steer_value;
  double steering_change;
  pid.Init(init_p_coeff, init_i_coeff, init_d_coeff);
  tune.Init(init_p_coeff, init_i_coeff, init_d_coeff);
  int param_index = 0;
  int step_count = 0;
  int tune_size = 1100;
  bool dig_deeper = false;
    
  h.onMessage([&pid, &tune, &do_tune, & param_index, &step_count, &steer_value, &steering_change, &dig_deeper, &tune_size](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
  
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));
      
      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
	  
          double cte = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          double angle = std::stod(j[1]["steering_angle"].get<string>());  // in degrees
          
          /**
           * TODO: Calculate steering value here, remember the steering value is
           *   [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed.
           *   Maybe use another PID controller to control the speed!
           */
	  if (do_tune == false) {
	    pid.UpdateError(cte);
	    steering_change = pid.TotalError();
	    steer_value = steering_change;
	  }

	  if (do_tune == true) {
	    double sum_dp = tune.dp[0]+tune.dp[1]+tune.dp[2];
	    if (sum_dp > tune.threshold) { // do some more fine-tuning
	      if (step_count > tune_size) { // sample size of next `tune_size` points completed, now evaluate the results
	        if ((tune.err < tune.best_err) && (!dig_deeper)) { // tune iteration improved results
	          tune.best_err = tune.err;
	  	  tune.best_P = tune.p[0];
		  tune.best_I = tune.p[1];
		  tune.best_D = tune.p[2];
		  tune.dp[param_index] *= 1.1;  // make a greater increment of change in parameter
	          tune.p[param_index] += tune.dp[param_index];
	        }
	        else { // tune iteration did not improve results
	          tune.p[param_index] -= tune.dp[param_index]; // adjust the parameter value in the negative direction 
	          if (!dig_deeper) {dig_deeper = true;}
	          else if (dig_deeper) {dig_deeper = false;}
	          if (tune.err < tune.best_err) { // tune iteration changed from having not improved results to having improve results
	            tune.best_err = tune.err;
		    tune.dp[param_index] *= 1.05;
	      	    tune.p[param_index] += tune.dp[param_index];
		  }
	          else { // tune iteration still not improving results
		    tune.dp[param_index] *= 0.95;
		    tune.p[param_index] += tune.dp[param_index];
	          }
	        }
	        	        
		if (!dig_deeper) {
		  param_index += 1;
		}
	        if (param_index > 2) { 
		  param_index = 0;
	        }
	        step_count = 0;
	        tune.ResetError();
	      }
	      // most of the time, just do this:
	      tune.UpdateError(cte);
	      
	      steer_value = tune.TotalError();
	      
	      if (step_count > (tune_size/1000)) {
	        tune.err += std::abs(cte);
	      }
	      //std::cout << "tune.err = " << tune.err << "step_count = " << step_count << std::endl;
              step_count += 1;
	    }

	    else { // it is fine-tuned enough
	      do_tune = false;
	      pid.Kp = tune.p[0];
	      pid.Ki = tune.p[1];
	      pid.Kd = tune.p[2];
	      std::cout << "Ideal tune found: P = " << tune.best_P << ", I = " << tune.best_I << ", D = " << tune.best_D << std::endl;
	    }
	    
	  }
          
          // DEBUG
	  //std::cout << std::endl;
          //std::cout << "angle: " << angle << " CTE: " << cte << " Steering Value: " << steer_value << std::endl;
	  if (do_tune == true) { 
	    std::cout << "Best results: best err = " << tune.best_err << ", P = " << tune.best_P << ", I = " << tune.best_I << ", D = " << tune.best_D << std::endl;
	    std::cout << "Current settings: P = " << tune.p[0] << ", I = " << tune.p[1] << ", D = " << tune.p[2] << std::endl;
	  }
          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.2;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
  }); // end h.onMessage
  
  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
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
}
