#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <iterator> // std::size
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"
#include "config.hpp"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road. 
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // *************************** MY CODE ***************************
          /**
           *  Future work:
           *  Cost function
           *  Predict best lane
           *  
           */
          int prev_path_size = previous_path_x.size();

          // Collision Detection
          // i. Set car_s to be previous path's last point. Frenet coordinates make logic easier
          if (prev_path_size > 0) car_s = end_path_s;

          bool too_close = false;
          bool init_lane_switch = false;
          bool left_lane_available = true;
          bool right_lane_available = true;
          int car_lane; // store lane for each detected car

          // 1. Loop through sensor data for other cars
          for (auto i=0; i<sensor_fusion.size(); ++i) { 
            auto other_car = sensor_fusion[i];
            // 2. Check and record car lane- assumes 3 lane highway of fixed width
            if (other_car[6] <= PathPlanner::kLaneWidth) {
              // Left lane
              car_lane = PathPlanner::kLeft;
            } else if (other_car[6] > PathPlanner::kLaneWidth && other_car[6] <= PathPlanner::kLaneWidth*2) {
              // Middle lane
              car_lane = PathPlanner::kMiddle;
            } else if (other_car[6] > PathPlanner::kLaneWidth*2 && other_car[6] <= PathPlanner::kLaneWidth*3) {
              // Right lane
              car_lane = PathPlanner::kRight;
            }

            // 3. Get speed of car and distance
            double x_vel = other_car[3];
            double y_vel = other_car[4];
            double other_s = other_car[5];
            double other_vel = sqrt(x_vel*x_vel + y_vel*y_vel);

            // 4. Project car's s into future proportionally to previous path length
            other_s += ((double) prev_path_size * other_vel * PathPlanner::kTimeStep);

            // 5. Check lane of the car and within a certain range e.g. 30m
            bool within_safe_gap =  abs(other_s - car_s) < PathPlanner::kSafetyGap;
            // Check car ahead in same lane within hotzone
            if (car_lane == PathPlanner::lane && other_s > car_s && within_safe_gap) {
              // i. Raise flag to reduce speed to avoid collision
              too_close = true;
              // ii. Raise flag to prepare to switch lanes
              init_lane_switch = true;

            } else if (car_lane == PathPlanner::lane - 1 && within_safe_gap) {
              // Car is in the left lane
              left_lane_available = false;
            } else if (car_lane == PathPlanner::lane + 1 && within_safe_gap) {
              // Car is in the right lane
              right_lane_available = false;
            } 
          }


          // 6. Slow down if car in front else speed up until limit
          if (too_close) {
            PathPlanner::curr_speed -= PathPlanner::max_accel;
          } else if (PathPlanner::curr_speed < PathPlanner::max_speed) {
            PathPlanner::curr_speed += PathPlanner::max_accel;
          }

          // 7. Switch lanes if available and sufficient gap
          if (init_lane_switch) {
            // If not in left lane and space available
            if (PathPlanner::lane > PathPlanner::kLeft && left_lane_available) {
              // std::cout << "Lane switched from " << lane << " to ";
              --PathPlanner::lane;
              // std::cout << lane << std::endl;
            } else if (PathPlanner::lane < PathPlanner::kRight && right_lane_available) {
              // If not in right lane and space available
              // std::cout << "Lane switched from " << lane << " to ";
              ++PathPlanner::lane;
              // std::cout << lane << std::endl;
            }
          }

          // Trajectory Generation
          double dist_inc = 0.5;

          // Fill up the next path points with leftover previous path points
          // auto prev_path_size = previous_path_si;
          for (auto i=0; i<prev_path_size; ++i) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          // 2. Initialise sparsely spaced (x, y) waypoints which will be interpolated with a spline
          vector<double> xpts, ypts;
          
          // 3. Track car's current state (x, y, yaw)- either current state or previous path endpoint
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          if (prev_path_size < 2) {
            // a. If no previous path, use current state to create path tangent to the car and store the waypoints
            double prev_x = ref_x - cos(ref_yaw);
            double prev_y = ref_y - sin(ref_yaw);

            xpts.push_back(prev_x);
            xpts.push_back(ref_x);
            ypts.push_back(prev_y);
            ypts.push_back(ref_y);

          } else {
            // b. Else, redefine current state as previous endpoint and store along with 2nd-last previous point
            ref_x = previous_path_x[prev_path_size-1];
            ref_y = previous_path_y[prev_path_size-1];

            double prev_x = previous_path_x[prev_path_size-2];
            double prev_y = previous_path_y[prev_path_size-2];

            ref_yaw = atan2(ref_y - prev_y, ref_x - prev_x);

            xpts.push_back(prev_x);
            xpts.push_back(ref_x);
            ypts.push_back(prev_y);
            ypts.push_back(ref_y);
          }

          // 4. In Frenet, add 30m-spaced waypoints ahead of current state. Store as waypoints in cartesian
          for (auto i=0; i<2; ++i) {
            vector<double> w = getXY(car_s+(i+1)*PathPlanner::kHorizonDistance, 2 + PathPlanner::kLaneWidth*PathPlanner::lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            xpts.push_back(w[0]);
            ypts.push_back(w[1]);
          }

          // 5. Transform the waypoints into the car's reference frame so that the previous last point of the car is the origin
          for (auto i=0; i<xpts.size(); ++i) {
            double shiftx = xpts[i] - ref_x;
            double shifty = ypts[i] - ref_y;

            xpts[i] = shiftx * cos(-ref_yaw) - shifty * sin(-ref_yaw);
            ypts[i] = shiftx * sin(-ref_yaw) + shifty * cos(-ref_yaw);
          }

          // 6. Create a spline and set the waypoints as its points
          tk::spline s(xpts,ypts);

          // 7. Calculate number of break points in spline to travel at desired speed
          double tgt_x = PathPlanner::kHorizonDistance;
          double tgt_y = s(tgt_x);
          double distance = sqrt(tgt_x*tgt_x + tgt_y*tgt_y);

          // b. Break points, N, is N = distance / 0.02 * speed
          double N = distance / (PathPlanner::kTimeStep * PathPlanner::curr_speed);
          double x_inc = tgt_x / N;

          // 8. Fill up the rest of the next path points using the spline
          double next_x, next_y;
          for (auto i=1; i <= 50-prev_path_size; ++i) {
            // a. Generate x and y spline points
            next_x = i * x_inc;
            next_y = s(next_x);

            // b. Transform back to global frame
            double next_x_ref = next_x;
            double next_y_ref = next_y;

            next_x = next_x_ref * cos(ref_yaw) - next_y_ref * sin(ref_yaw);
            next_y = next_x_ref * sin(ref_yaw) + next_y_ref * cos(ref_yaw);

            next_x += ref_x;
            next_y += ref_y;

            // c. Add to next path points
            next_x_vals.push_back(next_x);
            next_y_vals.push_back(next_y);
          }

          // ** Blind lane following ** //
          // for (int i=0; i < 50; ++i) {
          //   double next_s = car_s + (dist_inc * (i+1));
          //   double next_d = 6;

          //   vector<double> xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          //   next_x_vals.push_back(xy[0]);
          //   next_y_vals.push_back(xy[1]);
          // }
          // ** END ** //

          // ** Circle ** //
          // for (auto i=0; i < path_size; ++i) {
          //   next_x_vals.push_back(previous_path_x[i]);
          //   next_y_vals.push_back(previous_path_y[i]);
          // }

          // double pos_x, pos_y, angle;
          // // if previous path = 0, use current state
          // if (path_size == 0) {
          //   pos_x = car_x;
          //   pos_y = car_y;
          //   angle = car_yaw;
          // } else {
          //   // else use the last waypoint from previous state along with angle
          //   pos_x = previous_path_x[path_size - 1];
          //   pos_y = previous_path_y[path_size - 1];

          //   double pos_x2, pos_y2;
          //   pos_x2 = previous_path_x[path_size -2];
          //   pos_y2 = previous_path_y[path_size -2];
          //   angle = atan2(pos_y - pos_y2, pos_x - pos_x2);
          // }

          // for (int i=0; i < 50; ++i) {
          //   // Using equation: x(i) = A · cos(ωi + φ) 
          //   // where A = amplitude, ω = Frequency, φ = Phase
          //   next_x_vals.push_back(pos_x + dist_inc * cos(angle + (i+1) * pi()/100));
          //   next_y_vals.push_back(pos_y + dist_inc * sin(angle + (i+1) * pi()/100));

          //   pos_x += dist_inc * cos(angle + (i+1) * pi()/100);
          //   pos_y += dist_inc * sin(angle + (i+1) * pi()/100);
          // }
          // ** END ** //
          
          // ** Straight Line ** //
          // for (int i=0; i < 50; ++i) {
          //   next_x_vals.push_back(car_x + (dist_inc*i) * cos(deg2rad(car_yaw)));
          //   next_y_vals.push_back(car_y + (dist_inc*i) * sin(deg2rad(car_yaw)));
          // }
          //** END **//

          // ************************ END OF MY CODE ***********************

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
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
