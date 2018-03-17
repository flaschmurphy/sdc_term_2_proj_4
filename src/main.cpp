#include <uWS/uWS.h>
#include <iostream>
#include <math.h>
#include <cstdlib>
#include "json.hpp"
#include "PID.h"

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

  //DEBUG
  std::cout << "cte,steering,p_err,i_err,d_err" << std::endl;

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
//          double speed = std::stod(j[1]["speed"].get<std::string>());
//          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;
          /*
          * TODO: Calcuate steering value here, remember the steering 
          * value is [-1, 1]. NOTE: Feel free to play around with the 
          * throttle and speed. Maybe use another PID controller to 
          * control the speed!
          */

          /* 
          * Sample data:
          *   {"steering_angle":"0.0000","throttle":"0.0000",
          *   "speed":"0.4380", "cte":"0.7598","image":"/9j/4AAQ..."
          */

          // Error for the current step
          double current_err;

          // Values to include while Twidling 
          std::vector<double> twidle_params;
          twidle_params.push_back(pid.Kp);
          twidle_params.push_back(pid.Ki);
          twidle_params.push_back(pid.Kd);

          // Potential increments to Kp, Ki and Kd coefficients for Twidle algorithm
          double increment_p = 0.1;
          double increment_i = 0.0005;
          double increment_d = 0.05;

          // Increments to perform while Twidling 
          std::vector<double> twidle_increments;
          twidle_increments.push_back(increment_p);
          twidle_increments.push_back(increment_i);
          twidle_increments.push_back(increment_d);

          if (!pid.is_initialized) 
          {
              //TODO: Optimize these params
              double init_p = 0.05;
              double init_i = 0.0001;
              double init_d = 0.05;

              pid.Init(init_p, init_i, init_d);
          } 

          pid.UpdateError(cte);

          current_err = std::abs(pid.TotalError());
          if (current_err >= pid.best_err)
          {
              for (unsigned int i=0; i<twidle_params.size(); i++)
              {
                  double param = twidle_params[i];
                  double increment = twidle_increments[i];

                  param += increment;
                  current_err = std::abs(pid.TotalError());
                  if (current_err < pid.best_err)
                  {
                      // If we get here, there was some improvement
                      pid.best_err = current_err;
                      increment *= 1.1;
                  }
                  else
                  {
                      // If we get here, there was no improvement
                      param -= 2*increment;
                      current_err = std::abs(pid.TotalError());

                      if (current_err < pid.best_err)
                      {
                          // There was an improvment
                          pid.best_err = current_err;
                          increment *= 1.05;
                      }
                      else
                      {
                          // There was no improvement
                          param += increment;
                          increment *= 0.95;
                      }
                  }
              }
          }
          else
          {
              pid.best_err = current_err;
          }

          // Ensure that the steering angle is within the allowed range
          //if (current_err > 1) current_err = 1;

          // Once we reach here we should have the best params possible (local
          // perspective). The steering angle should always be in the opposite
          // direction to the CTE, meaning we may or may not need to change
          // it's sign
          if (cte >=0) {steer_value = -1 * current_err;}
          else {steer_value = current_err;}

          // DEBUG
          std::cout 
              //<< "CTE: " 
              << cte 
              //<< " Steering Value: " << steer_value 
              << "," << steer_value 
              << "," << -(pid.Kp * pid.p_error) 
              << "," << -(pid.Kd * pid.d_error) 
              << "," << -(pid.Ki * pid.i_error)
              //<< "," << pid.p_error 
              //<< "," << pid.i_error 
              //<< "," << pid.d_error 
              << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
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
