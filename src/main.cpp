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
#include "vehicle.h"
#include "road.h"

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

  //start in lane 1
  int lane = 1;
  //initial reference velocity
  double ref_vel = 1; //mph
  double max_vel = 49.5;
  //## check if best approach:
  //## flag for car too slow down
  bool too_close = false;
  //Vehicle(int lane, float s, float v, float a, string state="CS", int id);
  Vehicle my_car(-1, 0, 0, 0, "KL");
  Road road;

  my_car.goal_lane = 1; // center lane
  my_car.lanes_available = 3;
  my_car.goal_s = max_s;
  my_car.target_speed = 22.00; // m/s (22.35 is 50 miles/h)
  my_car.max_acceleration = 9; //  m/s²
  my_car.v=0;
  
  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
    &map_waypoints_dx,&map_waypoints_dy, &ref_vel, &lane, &too_close, &my_car, &road]
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
              //open file for debug
              std::ofstream debug_file;
              debug_file.open ("debug.csv", std::ios::out | std::ios::app);
              // j[1] is the data JSON object
              // Main car's localization Data
              double car_x = j[1]["x"];
              double car_y = j[1]["y"];
             // double car_s = j[1]["s"];
             // double car_d = j[1]["d"];
              double car_yaw = j[1]["yaw"];
             // double car_speed = j[1]["speed"];
              // Previous path data given to the Planner
              auto previous_path_x = j[1]["previous_path_x"];
              auto previous_path_y = j[1]["previous_path_y"];
              // Previous path's  end s and d values
              double end_path_s = j[1]["end_path_s"];
              double end_path_d = j[1]["end_path_d"];
              // Sensor Fusion Data, a list of all other cars on the same side
              //   of the road.
              auto sensor_fusion = j[1]["sensor_fusion"];
              my_car.d= j[1]["d"];
              my_car.set_lane(j[1]["d"]);   
              
              int prev_size = previous_path_x.size();
              double car_s = my_car.s;
              if(prev_size > 0) {
                car_s = end_path_s;
                my_car.s = end_path_s;
              }
              else{
                  my_car.s=j[1]["s"];
                  car_s = my_car.s;
              }
              my_car.yaw = j[1]["yaw"];
              my_car.v = j[1]["speed"];
              my_car.v = my_car.v/2.24; //convert miles/h to m/s
              
              
              debug_file << "car data-> s:" << j[1]["s"] << "  d: " << j[1]["d"] << "  speed: " << j[1]["speed"] << " my_car.a " << my_car.a << std::endl;

              for(int i = 0; i < sensor_fusion.size(); i++) {
                //first check where cars are and update them
                road.add_car(sensor_fusion[i]);
                  
                //for debug
                
                int id = sensor_fusion[i][0];
                //int x_pos = sensor_fusion[1];
                //int y_pos = sensor_fusion[2];
                double vx = sensor_fusion[i][3];
                double vy = sensor_fusion[i][4];
                double d = sensor_fusion[i][6];
                double s = sensor_fusion[i][5];
                double velocity = sqrt(vx*vx+vy*vy);
                
                
               // std::cout << "Sensor Fusion-> id:" << id << " vx:" << vx << " vy:" << vy << " d:" << d << " s:"<< s << " vel:" << velocity << std::endl;
               // std::cout << "Road id " << road.other_cars[i].id << " s:" << road.other_cars[i].s << " d:" << road.other_cars[i].d << std::endl;
              }
                        
              //update predictions of the cars on the road for the defined time steps ahead
              int horizon = 5;
              road.update_predictions(horizon);
              vector <Vehicle> next_trajectory = my_car.choose_next_state(road.predictions);
              
              //debug_file << "done choosing next state" << std::endl;
              
              //debug_file << "my car" << std::endl;
              //debug_file << -1 << "," << my_car.s <<"," << my_car.d <<"," << my_car.v << "," << my_car.speed << std::endl;
              
              for(int i = 0; i < next_trajectory.size(); i++){
              debug_file << next_trajectory[i].id << "," << next_trajectory.size()<<"," <<next_trajectory[i].s <<"," << next_trajectory[i].d <<"," << next_trajectory[i].lane << "," << next_trajectory[i].v << "," << next_trajectory[i].state << std::endl;
              }

              // List of (x,y) waypoints to be interpolated with a spline
              vector<double> ptsx;
              vector<double> ptsy;
              

              // reference states
              //use car as starting reference
              double ref_x = car_x;
              double ref_y = car_y;
              double ref_yaw = deg2rad(car_yaw);

              //if prev size is almost empty, use car as starting reference
              if ( prev_size < 2 ) {
                //Use two points thats makes path tangent to the car
                double prev_car_x = car_x - cos(car_yaw);
                double prev_car_y = car_y - sin(car_yaw);

                ptsx.push_back(prev_car_x);
                ptsx.push_back(car_x);

                ptsy.push_back(prev_car_y);
                ptsy.push_back(car_y);
              } else {
                //update the reference values to last endpoint
                ref_x = previous_path_x[prev_size - 1];
                ref_y = previous_path_y[prev_size - 1];

                double ref_x_prev = previous_path_x[prev_size-2];
                double ref_y_prev = previous_path_y[prev_size-2];
                ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

                // using the last two points we make sure we will get a new tangent path
                ptsx.push_back(ref_x_prev);
                ptsx.push_back(ref_x);

                ptsy.push_back(ref_y_prev);
                ptsy.push_back(ref_y);
              }
              for(int i=0; i<ptsx.size(); i++){
                  debug_file<< "ptsx[" << i << "]= "<< ptsx[i] << " ptsy[" << i << "]= "<<ptsy[i]<<std::endl;
              }
              
              //debug_file << "calculate xy points for new trajectory" << " traj-size:" <<next_trajectory.size() << std::endl;
              //for(int i = 0; i< next_trajectory.size(); i++){
                //debug_file << "next_trajectory[i].s: " << next_trajectory[i].s << std::endl;
               // debug_file << "next_trajectory[i].lane: " << next_trajectory[i].lane << std::endl;
              
              std::cout << "next S" << next_trajectory[1].s  << std::endl;
              std::cout << "my car  S" << my_car.s  << std::endl;
              if(my_car.s - next_trajectory[1].s < 10 || my_car.s + next_trajectory[1].s < 10){
                  next_trajectory[1].s = my_car.s + 65;
              }
              
              if(next_trajectory[1].s-my_car.s > 60){
                vector<double> next_wp0 = getXY(next_trajectory[1].s-60, 2 + 4*next_trajectory[1].lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
                vector<double> next_wp1 = getXY(next_trajectory[1].s-30, 2 + 4*next_trajectory[1].lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
                vector<double> next_wp2 = getXY(next_trajectory[1].s, 2 + 4*next_trajectory[1].lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
                ptsx.push_back(next_wp0[0]);
                ptsx.push_back(next_wp1[0]);
                ptsx.push_back(next_wp2[0]);
                ptsy.push_back(next_wp0[1]);
                ptsy.push_back(next_wp1[1]);
                ptsy.push_back(next_wp2[1]);
              }else if(next_trajectory[1].s-my_car.s > 30){
                vector<double> next_wp0 = getXY(next_trajectory[1].s-30, 2 + 4*next_trajectory[1].lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
                vector<double> next_wp1 = getXY(next_trajectory[1].s, 2 + 4*next_trajectory[1].lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
                ptsx.push_back(next_wp0[0]);
                ptsx.push_back(next_wp1[0]);
                ptsy.push_back(next_wp0[1]);
                ptsy.push_back(next_wp1[1]);
              }else{
                vector<double> next_wp0 = getXY(next_trajectory[1].s, 2 + 4*next_trajectory[1].lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
                ptsx.push_back(next_wp0[0]);
                ptsy.push_back(next_wp0[1]);
;
              }
              
              
              
              
              //vector<double> next_wp1 = getXY(car_s + 60, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
              //vector<double> next_wp2 = getXY(car_s + 90, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
              

              //} //for
              
              for(int i = 0; i < ptsx.size(); i++) {
                //shift the car reference angle to 0 degress
                double shift_x = ptsx[i]-ref_x;
                double shift_y = ptsy[i]-ref_y;
                ptsx[i] = (shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw));
                ptsy[i] = (shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw));
              }
              debug_file << "Use the spline" << std::endl;
              //create a spline
              tk::spline spline;
              // set the points to the spline
             //for(int i=0; i<ptsx.size(); i++){
             //     debug_file<< "ptsx[" << i << "]= "<< ptsx[i] << " ptsy[" << i << "]= "<<ptsy[i]<<std::endl;
            //  }
              spline.set_points(ptsx, ptsy);
              //debug_file << "done setting points" << std::endl;

              // define vectors for next points
              vector<double> next_x_vals;
              vector<double> next_y_vals;

              for (int i = 0; i < prev_size; i++) {
                //debug_file << previous_path_x[i] << "," << previous_path_y[i] << std::endl;
                next_x_vals.push_back(previous_path_x[i]);
                next_y_vals.push_back(previous_path_y[i]);
              }

              // calculate how to break up spline points
              //double target_x = 30.0;
              double target_x_idx = ptsx.size()-1;
              double target_x = ptsx[target_x_idx];
              double target_y = spline(target_x);
              double target_v_idx = next_trajectory.size()-1;
              double target_v = next_trajectory[target_v_idx].v;
              double target_dist = sqrt(target_x*target_x + target_y*target_y);
              double x_add_on = 0;
             // debug_file << "target_y: " << target_y << "  target_dist:" << target_dist << " target_v:" << target_v << std::endl;
              //fill up the rest of the path Planner
              for(int i = 1; i <= 50-prev_size;i++) {
                  
                // N * t(0.02s) * vel = dist
                //double N = target_dist/(0.02*target_v/2.24);
                double N = target_dist/(0.02*target_v);
                double x_point = x_add_on+target_x/N;
                double y_point = spline(x_point);
                x_add_on = x_point;
             //   debug_file << "N: " << N << "  x_point:" << x_point << "  y_point:" << y_point << std::endl;
                double x_ref = x_point;
                double y_ref = y_point;

                //rotate back to normal coordinates
                x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
                y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

                x_point += ref_x;
                y_point += ref_y;

                //debug_file << x_point << "," << y_point << std::endl;

                next_x_vals.push_back(x_point);
                next_y_vals.push_back(y_point);
              }
              
              debug_file.close();
              //fim
              
              json msgJson;
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
