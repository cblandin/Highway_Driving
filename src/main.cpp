#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

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
  
  int lane = 1;
  double ref_vel = 0;
  double last_change_s = 0.0;

  h.onMessage([&ref_vel, &max_s, &last_change_s, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &lane]
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


          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          
          //Go through sensor fusion checking for a vehicle in front to track
          int track_remote = -1;

          double actual_lane = floor(car_d / 4);
          for(int i = 0; i < sensor_fusion.size(); i++){
            double remote_d = sensor_fusion[i][6];
          	if(remote_d < (2+4.0*actual_lane+2) and remote_d > (2+4.0*actual_lane - 2)){ //control speed based on the car of the lane you are currently in. Not necesarrily the desired lane
              double remote_s = sensor_fusion[i][5];
              if(remote_s > car_s && (remote_s - car_s) < 30){ //could check for looping around the track but it is crossed quickly and doesn't seem to have a big effect
                track_remote = i;
              }
            }
          }
          
          //check for looping around
          if(last_change_s > car_s){
            last_change_s = -(max_s - last_change_s);
          }
          
          //Behavior Planning
          if ((car_s - last_change_s) > 100){ // only allow lane changes every 100m
            vector<double> costs = lane_cost(car_s, car_d, car_speed/2.24, lane, sensor_fusion, map_waypoints_x, map_waypoints_y);
            vector<double>::iterator min_it = min_element(costs.begin(),costs.end());
            int desired_lane = (int) (min_it - costs.begin());
            if (desired_lane > lane){
              lane++;
              //track_remote = -1;
              std::cout << "RIGHT" << std::endl;
              last_change_s = car_s;
              std::cout << costs[0] << " " <<costs[1] << " " <<costs[2] << std::endl;
            }
            else if(desired_lane < lane){
              lane--;
              //track_remote = -1;
              std::cout << "Left" << std::endl;
              std::cout << costs[0] << " " <<costs[1] << " " <<costs[2] << std::endl;
              last_change_s = car_s;
            }
          }
          
          // Speed Control
          
          double max_speed = 49.0;
          
          if(track_remote != -1){
            double delta_s = (double)sensor_fusion[track_remote][5] - car_s;
            if(previous_path_x.size() > 2){
              double dy = (double)previous_path_y[previous_path_y.size()-1];
              dy -= (double)previous_path_y[previous_path_y.size()-2];
              double dx = (double)previous_path_x[previous_path_x.size()-1] - (double)previous_path_x[previous_path_x.size()-2];
              double ref_yaw = atan2(dy,dx );
              vector<double> ref_f = getFrenet(previous_path_x[previous_path_x.size()-1], previous_path_y[previous_path_y.size()-1],ref_yaw, map_waypoints_x, map_waypoints_y);
              delta_s = (double)sensor_fusion[track_remote][5] - ref_f[0];
            }
            
            if(delta_s < 27){ //could make these speed dependent
              if(delta_s < 10){
                ref_vel -=.5;
              }
              else{
              	ref_vel -= .35;
              }
            }
            else if(delta_s > 29 && ref_vel < max_speed){ 
              ref_vel += .5;
            }
           
          }          
          else if (ref_vel < max_speed){
            ref_vel += 0.5;
          }
          
          
          
          // Trajectory Planning
          int prev_size = previous_path_x.size();
          
          vector<double> ptsx;
          vector<double> ptsy;
          
          
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);
          
          
          if(prev_size < 2){
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);
            
            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);
            
            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          }
          else{
            ref_x = previous_path_x[prev_size - 1];
            ref_y = previous_path_y[prev_size-1];
            double ref_x_prev = previous_path_x[prev_size - 2];
            double ref_y_prev = previous_path_y[prev_size-2];
            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
            
            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);
            
            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }
          vector<double> ref_fer = getFrenet(ref_x, ref_y, ref_yaw, map_waypoints_x, map_waypoints_y);
          double ref_d = ref_fer[1];
          double delta_d = 2+4*lane - ref_d;
          vector<double> next_wp0 = getXY(car_s + 30, ref_d + .45*delta_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s + 60,  ref_d + .8*delta_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s + 90,  ref_d + delta_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          
          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);
          
          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);
          
          for(int i = 0; i < ptsx.size(); i++){
            
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;
            ptsx[i] = shift_x*cos(0-ref_yaw) - shift_y*sin(0-ref_yaw);
            ptsy[i] = shift_x*sin(0-ref_yaw) +shift_y*cos(0-ref_yaw);
            
          }
          
          
          tk::spline s;
          
          s.set_points(ptsx,ptsy);
          
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          
          for(int i = 0; i < prev_size; i++){
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt(target_x*target_x + target_y*target_y);
          
          double x_add_on = 0.0;
          
          for(int i = 0; i < 35-prev_size; i++){ //using 35 points so that it can react quicker (gives up some smoothness and lane centering)
            double N = (target_dist/(.02*ref_vel/2.24));
            double x_point = x_add_on + target_x/N;
            double y_point = s(x_point);
            
            x_add_on = x_point;
            
            double x_ref = x_point;
            double y_ref = y_point;
            
            x_point = x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw) + ref_x;
            y_point = x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw) + ref_y;
            
            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }
                                  
          


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