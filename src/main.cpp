#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include "helper.hpp"

using namespace std;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s)
{
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos)
  {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos)
  {
    return s.substr(b1, b2 - b1 + 2);
  }
  {
  }
  return "";
}

int main()
{
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

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line))
  {
    istringstream iss(line);
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

  // initialize hyperparameters
  int lane = 1;
  bool keep_lane = true;
  double ref_vel = 0.0;
  const double max_vel = 49.5;
  const int ticks = 50;

  h.onMessage([&ref_vel, &lane, &max_vel, &map_waypoints_x, &map_waypoints_y, &map_waypoints_s, &map_waypoints_dx, &map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {

      auto s = hasData(data);

      if (s != "")
      {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry")
        {
          // j[1] is the data JSON object

          /*******************
          Sim Output
          *******************/
          // Main car's localization Data
          double car_x = j[1]["x"];         // Vehicle position Map x
          double car_y = j[1]["y"];         // Vehicle position Map y
          double car_s = j[1]["s"];         // Vehicle position Frenet s
          double car_d = j[1]["d"];         // Vehicle position Frenet d
          double car_yaw = j[1]["yaw"];     // Vehicle heading
          double car_speed = j[1]["speed"]; // Vehicle speed

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"]; // previous path waypoints x
          auto previous_path_y = j[1]["previous_path_y"]; // previous path waypoints y
          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          auto sensor_fusion = j[1]["sensor_fusion"]; // other vehicles
          double behavior_plan_speed = max_vel;
          vector<bool> lane_free = {false, false, false};
          vector<double> vehicle_in_front = findVehicleInLane(lane, car_s, sensor_fusion);

          /**********************************************************
           *  Behavior Planning
          **********************************************************/

          // check if there is a vehicle in front that is slower
          if ((vehicle_in_front[1] < max_vel) && vehicle_in_front[0] < 40)
          {

            cout << "There is a slower car in front!"
                 << " Speed: " << vehicle_in_front[1] << endl;

            // go through all lanes
            for (int i = 0; i < 3; i++)
            {
              // check other lanes while omitting own lane
              if (i != lane)
              {
                vector<double> other_lane_vehicle = findVehicleInLane(i, car_s, sensor_fusion);
                // check if there is a vehicle in the other lane that is far enough in front
                // and faster than the vehicle in the own lane
                // and check if there is a vehicle right beside or behind the car
                cout << "Vehicle in lane " << i << ": " << other_lane_vehicle[0] << "; " << other_lane_vehicle[1] << endl;
                if ((other_lane_vehicle[0] > 40) && laneClear(i, car_s, sensor_fusion))
                {
                  lane_free[i] = true; // save which lane would be possible
                  cout << "It would be possible to change to lane " << i << "." << endl;
                }
              }
            }

            // cout << "Debug out: lane_free[0] : " << lane_free[0] << ", " << lane_free[1] << ", " << lane_free[2] << endl;

            if (lane_free[1])
            {
              lane = 1; // if lane 1 is free, change to it. This is independent of the current lane. Check README for explainer.
            }
            else if (lane_free[0] && lane_free[2]) // both left and right lane is free. Decide on overall lane speed.
            {
              // decide based on slowest speed in lane in front of car
              if (slowLaneSpeed(0, car_s, sensor_fusion) > slowLaneSpeed(2, car_s, sensor_fusion))
              {
                lane = 0;
              }
              else
              {
                lane = 2;
              }
            }
            else if (lane == 1 && lane_free[0] && !lane_free[2])
            {
              lane = 0;
            }
            else if (lane == 1 && !lane_free[0] && lane_free[2])
            {
              lane = 2;
            }
            // else if (lane == 0 && lane_free[2] && !lane_free[1] && laneClear(1, car_s, sensor_fusion)) {lane = 1;}
            // else if (lane == 2 && lane_free[0] && !lane_free[1] && laneClear(1, car_s, sensor_fusion)) {lane = 1;}
            else // in all other cases, there is no free lane
            {
              // adjust speed based on current lane speed
              if (vehicle_in_front[0] < 30.0)
              {
                // if the closest vehicle in front is at a proper distance, match speed
                behavior_plan_speed = vehicle_in_front[1];
                cout << "updated planned speed to: " << behavior_plan_speed << endl;
                cout << "Distance to vehicle in front: " << vehicle_in_front[0] << endl;
              }
              if (vehicle_in_front[0] < 25.0)
              {
                // if the vehicle in front is to close, reduce speed a bit more until the distance is safe
                behavior_plan_speed -= 5;
                cout << "vehicle in front to close, breaking!" << endl;
              }
            }
          }
          /**********************************************************
           *  Controller adopted from walkthrough video
          **********************************************************/

          json msgJson;

          vector<double> behavior_plan_s;    // Behavior planning in direction of street
          vector<double> behavior_plan_lane; // Behavior planning: be in lane at point s

          // Test output of behavior planner!

          behavior_plan_s.push_back(30);
          behavior_plan_s.push_back(60);
          behavior_plan_s.push_back(90); //needed?!

          behavior_plan_lane.push_back(lane);
          behavior_plan_lane.push_back(lane);
          behavior_plan_lane.push_back(lane);

          int prev_size = previous_path_x.size();

          // super simple controller to change the speed slowly
          if ((ref_vel > behavior_plan_speed))
          {
            // if the current speed is above the planned speed, slow down
            // also if the vehicle in front is to close, slow down a bit until the distance is right
            ref_vel -= 0.3 * (ticks - prev_size);
          }
          else if (ref_vel < behavior_plan_speed)
          {
            // if the current speed is below the planned speed, speed up
            ref_vel += 0.1 * (ticks - prev_size);
            ref_vel = min(ref_vel, max_vel);
          }

          // when there is no old trajectory, use vehicle position as start
          if (prev_size < 1)
          {
            end_path_s = car_s;
          }
          vector<double> ptsx;
          vector<double> ptsy;

          // preserving the direction of the car by using the current position of the vehicle and one previous point
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          double pref_ref_x, prev_ref_y;
          if (prev_size < 2)
          {
            // use two points that are tangent to the direction of the vehicle
            pref_ref_x = ref_x - cos(ref_yaw);
            prev_ref_y = ref_y - sin(ref_yaw);
          }
          else
          {
            // use two points from the previous
            ref_x = previous_path_x[prev_size - 1];
            ref_y = previous_path_y[prev_size - 1];

            pref_ref_x = previous_path_x[prev_size - 2];
            prev_ref_y = previous_path_y[prev_size - 2];
            ref_yaw = atan2(ref_y - prev_ref_y, ref_x - pref_ref_x);
          }

          ptsx.push_back(pref_ref_x);
          ptsy.push_back(prev_ref_y);
          ptsx.push_back(ref_x);
          ptsy.push_back(ref_y);

          // transform behavior planning to map coordinates
          for (int i = 0; i < behavior_plan_s.size(); i++)
          {
            vector<double> map_waypoint_temp = getXY(end_path_s + behavior_plan_s[i], (2 + 4 * behavior_plan_lane[i]), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            ptsx.push_back(map_waypoint_temp[0]);
            ptsy.push_back(map_waypoint_temp[1]);
          }

          // transform waypoints from map coordinates to vehicle coordinates
          for (int i = 0; i < ptsx.size(); i++)
          {
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;
            ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
            ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
          }

          // fit spline to points in frenet cordinate system
          tk::spline spline;
          spline.set_points(ptsx, ptsy);

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // recover unused driving points and add again
          for (int i = 0; i < prev_size; i++)
          {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          double target_x = 30;
          double target_y = spline(target_x);
          double target_dist = sqrt((target_x * target_x) + (target_y * target_y));
          double x_add_on = 0.0;

          // create driving points
          for (int i = 0; i < ticks - prev_size; i++)
          {
            double N = target_dist / (0.02 * ref_vel / 2.24);
            double x_point = x_add_on + target_x / N;
            double y_point = spline(x_point); // matching y corrdinate
            x_add_on = x_point;               // recursive

            double x_ref = x_point;
            double y_ref = y_point;

            x_point = ref_x + ((x_ref * cos(ref_yaw)) - (y_ref * sin(ref_yaw)));
            y_point = ref_y + ((x_ref * sin(ref_yaw)) + (y_ref * cos(ref_yaw)));
            ;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }

          // define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\"," + msgJson.dump() + "]";

          //this_thread::sleep_for(chrono::milliseconds(1000));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      }
      else
      {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
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
