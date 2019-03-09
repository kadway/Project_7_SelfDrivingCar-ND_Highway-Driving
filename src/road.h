//#ifndef ROAD_H
//#define ROAD_H

#include <map>
#include <string>
#include <vector>
#include "vehicle.h"

class Road {
public:   
    //map of Vehicles for all other cars on the road
    map <int, Vehicle> other_cars;
    //map of vectors of Vehicle predictions
    map <int, vector<Vehicle> > predictions;
    
    //add cars to the road
    void add_car(vector <double> sensor_fusion);
    //update cars predictions
    void update_predictions(int horizon);
};
