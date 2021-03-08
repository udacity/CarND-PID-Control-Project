#include <cmath>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"

// for convenience
using nlohmann::json;
using std::string;

const double MAX_SPEED = 100.0;
const double MAX_ANGLE = 20.0;
const double MAX_THROTTLE = 0.3;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(const string &s)
{
    auto found_null = s.find("null");
    auto b1 = s.find_first_of('[');
    auto b2 = s.find_last_of(']');
    if (found_null != string::npos)
    {
        return "";
    }
    else if (b1 != string::npos && b2 != string::npos)
    {
        return s.substr(b1, b2 - b1 + 1);
    }
    return "";
}

int main()
{
    uWS::Hub h;

    // PID controller for strering and throttle
    // Manual tuning
    PID pid_steer(0.139999, 0.009, 8.3);
    PID pid_throttle(0.8999, 0.00001, 0.99999);

    h.onMessage([&pid_steer, &pid_throttle](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                                            uWS::OpCode opCode)
                {
                    // "42" at the start of the message means there's a websocket message event.
                    // The 4 signifies a websocket message
                    // The 2 signifies a websocket event
                    if (length > 2 && data[0] == '4' && data[1] == '2')
                    {
                        auto s = hasData(string(data).substr(0, length));
                        if (!s.empty())
                        {
                            auto j = json::parse(s);
                            string event = j[0].get<string>();
                            if (event == "telemetry")
                            {
                                // j[1] is the data JSON object
                                double cte = std::stod(j[1]["cte"].get<string>());
                                double speed = std::stod(j[1]["speed"].get<string>());
                                double angle = std::stod(j[1]["steering_angle"].get<string>());
                                double steer_value;
                                double throttle_value;

                                pid_steer.UpdateError(cte);
                                steer_value = pid_steer.GetController();

                                double target_speed =
                                        std::max(0.0, MAX_SPEED * (1.0 - fabs(angle / MAX_ANGLE * cte) / 4));

                                target_speed = std::min(MAX_SPEED, target_speed);
                                pid_throttle.UpdateError(speed - target_speed);

                                throttle_value = std::min(MAX_THROTTLE, 0.7 + pid_throttle.GetController());

                                // DEBUG
                                std::cout << "CTE: " << cte << " Steering Value: " << steer_value
                                          << std::endl;

                                json msgJson;
                                msgJson["steering_angle"] = steer_value;
                                msgJson["throttle"] = throttle_value;
                                auto msg = "42[\"steer\"," + msgJson.dump() + "]";
                                std::cout << msg << std::endl;
                                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                            }  // end "telemetry" if
                        }
                        else
                        {
                            // Manual driving
                            string msg = "42[\"manual\",{}]";
                            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                        }
                    }  // end websocket message if
                }); // end h.onMessage

    h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req)
                   {
                       std::cout << "Connected!!!" << std::endl;
                   });

    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                           char *message, size_t length)
                      {
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