#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"
#include "twiddle.hpp"

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
string hasData(string s)
{
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
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

void reset_sim(uWS::WebSocket<uWS::SERVER> &ws)
{
  std::string msg = "42[\"reset\",{}]";
  ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
}

int main()
{
  uWS::Hub h;

  // Initialize the pid variable with coefficients
  PID pid;
  double Kp = 0.25;
  double Ki = 0.0003;
  double Kd = 9;
  pid.Init(Kp, Ki, Kd);

  bool use_twiddle = false; // set to true to enable twiddle

  Twiddle twiddle({0.00065, 0.000032, 0.00071}, 0.001, 3500, use_twiddle);

  h.onMessage([&pid, &twiddle](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 25 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(string(data).substr(0, length));

      if (s != "")
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
          /**
           * TODO: Calculate steering value here, remember the steering value is
           *   [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed.
           *   Maybe use another PID controller to control the speed!
           */
          if (twiddle.use_twiddle_ && twiddle.SumPD() < twiddle.threshold_)
          {
            std::cout << "Twiddle stopped due to threshold reached.\n";
            twiddle.use_twiddle_ = false;
          }

          if (twiddle.use_twiddle_)
          {
            // increment iteration
            ++twiddle.distance_count_;
            // Square error to ignore sign
            twiddle.average_error_ += cte * cte / twiddle.distance_count_;
            // std::cout << twiddle.distance_count_ << " " << twiddle.average_error_  << " " << fabs(cte) << '\n';

            // Abort conditions
            // Don't stop in the first 50 steps of simulation
            if (twiddle.distance_count_ > 50 &&
                // Stop if target distance reached, car stopped or off-road
                (twiddle.DistanceReached() || speed < 0.5 || fabs(cte) >= 4.0))
            {
              twiddle.LogStep(pid);

              // Check error is lower and distance reached greater
              if (twiddle.average_error_ < twiddle.best_error_ &&
                  twiddle.distance_count_ >= twiddle.best_distance_)
              {
                twiddle.UpdateBestParams();
              }
              // Set flag to tune PID parameter backwards
              else if (twiddle.tuning_direction_ == Twiddle::Direction::forward)
              {
                twiddle.tuning_direction_ = Twiddle::Direction::backward;
              }
              // Failed to reduce error, reset state and move to next PID parameter
              else
              {
                twiddle.ResetParams(pid);
              }

              // Log state every cycle of P I D tuning
              if (twiddle.delta_index_ == 0)
              {
                twiddle.LogState(pid);
              }

              // Tune PID- initially forward increment
              twiddle.Tune(pid);

              std::cout << "Resetting sim\n";
              // Reset current run
              twiddle.Reset();

              // Reset the simulator
              reset_sim(ws);
            }
          }

          pid.UpdateError(cte);
          steer_value = pid.TotalError();

          // Clamp steer value between -1 and 1
          if (steer_value > 1)
          {
            steer_value = 1;
          }
          else if (steer_value < -1)
          {
            steer_value = -1;
          }

          // DEBUG
          // std::cout << "CTE: " << cte << " Steering Value: " << steer_value
          //           << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        } // end "telemetry" if
      }
      else
      {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    } // end websocket message if
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