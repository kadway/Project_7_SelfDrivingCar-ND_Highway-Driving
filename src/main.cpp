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

struct cars_on_road{
  int my_lane_closest;
  double my_lane_closest_s;
  int left_closest;
  double left_closest_s;
  int right_closest;
  double right_closest_s;
}

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
  double ref_vel = 0.0; //mph
  double max_vel = 49.5;
  //## check if best approach:
  //## flag for car too slow down
  bool too_close = false;

  cars_on_road other_car;
  //need to init other_car ! ! !

  enum car_states { KEEP, CL, CR, PCL, PCR} car_state = KEEP;


  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
    &map_waypoints_dx,&map_waypoints_dy, &ref_vel, &lane, &too_close, &car_state]
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
              // std::ofstream debug_file;
              //debug_file.open ("debug.csv", std::ios::out | std::ios::app);
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
              // Previous path's  end s and d values
              double end_path_s = j[1]["end_path_s"];
              double end_path_d = j[1]["end_path_d"];
              // Sensor Fusion Data, a list of all other cars on the same side
              //   of the road.
              auto sensor_fusion = j[1]["sensor_fusion"];
              json msgJson;

              //debug_file << "x, y, s, d, yaw, speed, prev_path_x.size(), prev_path_y.size(), end_path_s, end_path_d - - - - " << std::endl;
              // debug_file << car_x << "," <<car_y << "," << car_s << "," << car_d << "," << car_yaw << "," << car_speed << "," << previous_path_x.size() << "," << previous_path_y.size() << "," << end_path_s << "," << end_path_d << std::endl;

              /**
              * TODO: define a path made up of (x,y) points that the car will visit
              *   sequentially every .02 seconds
              */

              int prev_size = previous_path_x.size();

              if(prev_size > 0) {
                car_s = end_path_s;
              }
        /*  switch(car_state) {
     			case KEEP   : keep(); break;
     			//case CL : .....  ; break;
     			//case CR  : ..... ; break;
			 }*/

              for(int i = 0; i < sensor_fusion.size(); i++) {
                //first check where cars are and save the closest to the struct other_car
                float d = sensor_fusion[i][6];
                if(d < (4+4*lane) && d > (4*lane)){
                    double check_car_s = sensor_fusion[i][5];
                    //car is whithin 30 meters
                    if((check_car_s > car_s) && (check_car_s-car_s < 30)) {
                      if
                    }
                } // if(d < (2+4*lane) && d > (2+4*lane)){

                  double vx = sensor_fusion[i][3];
                  double vy = sensor_fusion[i][4];
                  double check_speed = sqrt(vx*vx+vy*vy);


                  //Prediction step
                  //predict where the car will be after we visit all points assigned so far
                  //I'm assuming the car will not switch lanes in this period
                  check_car_s += ((double) prev_size*0.02*check_speed);

                  //check if the cars s value is greater than mine if there is a gap of less than 30m

                    //## To do: some logic so we dont crash

                    //theres a car ahed so check if its possible to switch lanes
                    if (lane>0){
                      //try to switch to lane on the left

                    else{ //slow down
                    too_close = true;
                  }
                } //if((check_car_s > car_s) && (check_car_s-car_s < 30))


              } // for(int i = 0; i < sensor_fusion.size(); i++)

              if(too_close) {
                //std::cout << "too close!" << std::endl;
                ref_vel -= 0.224;
                //std::cout << "ref_vel -= 0.224" << std::endl;
              }
              else if(ref_vel < 49.5) {
                //std::cout << "ref_vel = " << ref_vel << std::endl;
                ref_vel += 0.224;
                //std::cout << "ref_vel += 0.224" << ref_vel << std::endl;
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



            // add evenly 30m spaced points ahead of the starting reference
            vector<double> next_wp0 = getXY(car_s + 30, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp1 = getXY(car_s + 60, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp2 = getXY(car_s + 90, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);

            ptsx.push_back(next_wp0[0]);
            ptsx.push_back(next_wp1[0]);
            ptsx.push_back(next_wp2[0]);

            ptsy.push_back(next_wp0[1]);
            ptsy.push_back(next_wp1[1]);
            ptsy.push_back(next_wp2[1]);

          for(int i = 0; i < ptsx.size(); i++) {
            //shift the car reference angle to 0 degress
            double shift_x = ptsx[i]-ref_x;
            double shift_y = ptsy[i]-ref_y;
            ptsx[i] = (shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw));
            ptsy[i] = (shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw));
          }

          //create a spline
          tk::spline spline;
          // set the points to the spline
          spline.set_points(ptsx, ptsy);

          // define vectors for next points
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          for (int i = 0; i < prev_size; i++) {
            //debug_file << previous_path_x[i] << "," << previous_path_y[i] << std::endl;
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          // calculate how to break up spline points
          double target_x = 30.0;
          double target_y = spline(target_x);
          double target_dist = sqrt(target_x*target_x + target_y*target_y);
          double x_add_on = 0;

          //fill up the rest of the path Planner
          for(int i = 1; i <= 50-prev_size;i++) {
            // N * t(0.02s) * vel = dist
            double N = target_dist/(0.02*ref_vel/2.24);
            double x_point = x_add_on+target_x/N;
            double y_point = spline(x_point);
            x_add_on = x_point;

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
          //debug_file.close();
          //fim

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
