#include <iostream>
#include <iterator>
#include <map>
#include <sstream>
#include <string>
#include <vector>
#include "road.h"
#include "vehicle.h"
#include <math.h>

void Road::add_car(vector <double> sensor_fusion) {
    
  std::map<int, Vehicle>::iterator it = this->other_cars.begin();
  
  int id = sensor_fusion[0];
//int x_pos = sensor_fusion[1];
//int y_pos = sensor_fusion[2];
  double vx = sensor_fusion[3];
  double vy = sensor_fusion[4];
  double d = sensor_fusion[6];
  double s = sensor_fusion[5];
  double velocity = sqrt(vx*vx+vy*vy);
  
  Vehicle other_car(-1, s, velocity, 0, "CS", d, id);
  
    while (it != this->other_cars.end()) {
    int v_id = it->first;
    Vehicle v = it->second;
    if (v.lane == other_car.lane && v.s == s) {
      this->other_cars.erase(v_id);
    }
    ++it;
    }
  
  this->other_cars.insert(std::make_pair(id,other_car));
  
  //for debug
  int idx = other_cars.size();
  std::cout << "car added with id: " << other_cars[idx-1].id << " s: "<< other_cars[idx-1].s << " d: "<< other_cars[idx-1].d<< " v: " << other_cars[idx-1].v << std::endl;
  std::cout << "other cars size: " << other_cars.size() << std::endl;
}

void Road::update_predictions(int horizon){
    for(int i=0; i< other_cars.size(); i++){
    this->predictions.insert(std::make_pair(this->other_cars[i].id, this->other_cars[i].generate_predictions(horizon)));
    }
    //for debug:
   int idx = predictions.size();
   std::cout << "predictions " << idx << " size: " << predictions[idx-1].size() << std::endl;
  
}