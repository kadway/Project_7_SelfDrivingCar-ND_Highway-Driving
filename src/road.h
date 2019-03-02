//#ifndef ROAD_H
//#define ROAD_H

#include <map>
#include <string>
#include <vector>
#include "vehicle.h"

class Road {
 public:   
  //vector of Vehicles for all other cars on the road
  map <int, Vehicle> other_cars;
  map <int, vector<Vehicle> > predictions;
  
  //add cars to the road
  void add_car(vector <double> sensor_fusion);
  void update_predictions(int horizon);
};
