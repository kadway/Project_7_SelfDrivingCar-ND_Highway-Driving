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
    double vx = sensor_fusion[3]; //in m/s
    double vy = sensor_fusion[4]; // in m/s
    double d = sensor_fusion[6]; //frenet coordinates
    double s = sensor_fusion[5]; // frenet coordinates
    double velocity = sqrt(vx*vx+vy*vy); //  m/s
    //velocity = velocity*2.24; //convert to miles per hour
    
    Vehicle other_car(-1, s, velocity, 0, "KL", d, id);
    if (other_cars.size()>id){
        //remove old car data
        other_cars.erase(id);
    }
    //add recent data
    other_cars.emplace(id,other_car);
}

void Road::update_predictions(int horizon){
    for(int i=0; i< other_cars.size(); i++){
        if (predictions.size()>i){
            //remove old predictions
            predictions.erase(i);
        }
        //generate new predictions
        predictions.emplace(other_cars[i].id, other_cars[i].generate_predictions(horizon));
    }
}