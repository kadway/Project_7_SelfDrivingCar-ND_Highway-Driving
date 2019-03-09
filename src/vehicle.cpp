#include "vehicle.h"
#include <algorithm>
#include <iterator>
#include <map>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>

using std::string;
using std::vector;

// Initializes Vehicle
Vehicle::Vehicle(){}

Vehicle::Vehicle(int lane, float s, float v, float a, string state, float d, int id) {
    
    this->s = s;
    this->v = v;
    this->a = a;
    this->state = state;
    this->id = id;
    
    if(lane==-1){
        this->set_lane(d);
        this->d = d;
    }
    else if(d==-1){
        this->lane = lane;
    }
}

Vehicle::~Vehicle() {}

vector<Vehicle> Vehicle::choose_next_state(map<int, vector<Vehicle>> &predictions) {
    
    vector<string> states = successor_states();
    map<string, vector<Vehicle>> final_trajectories;
    float best_speed=0;
    string best_state = "KL"; // defaults to Keep Lane
    for (vector<string>::iterator it = states.begin(); it != states.end(); ++it) {
        vector<Vehicle> trajectory = generate_trajectory(*it, predictions);
        
        if (trajectory.size() != 0) {
            final_trajectories.emplace(trajectory[0].state, trajectory);
            // if trajectory change results in higher velocity do it 
            // but only if actual velocity is higher than 13.4 m/s (close to 30 mph)
            if(trajectory[0].v>best_speed+0.1 && this->ref_v > 13.4){
                best_speed = trajectory[0].v;
                best_state = trajectory[0].state;
            }
        }
    }
    return final_trajectories[best_state];
}

vector<string> Vehicle::successor_states() {
    // Provides the possible next states given the current state for the FSM
    //   discussed in the course, with the exception that lane changes happen
    //   instantaneously, so LCL and LCR can only transition back to KL.
    vector<string> states;
    states.push_back("KL");
    string state = this->state;
    
    if (this->state == "KL" && lane != 0) {
        states.push_back("LCL");
    }
    else if (this->state == "KL" && lane != lanes_available - 1) {
        states.push_back("LCR");
    }
    
    return states;
}

vector<Vehicle> Vehicle::generate_trajectory(string state,
        map<int, vector<Vehicle>> &predictions) {
    // Given a possible next state, generate the appropriate trajectory to realize
    //   the next state.
    vector<Vehicle> trajectory;
    
    if (state.compare("KL") == 0) {
        trajectory = keep_lane_trajectory(predictions);
    } else if (state.compare("LCL") == 0 || state.compare("LCR") == 0) {
        trajectory = lane_change_trajectory(state, predictions);
    }
    
    return trajectory;
}

float Vehicle::get_kinematics(map<int, vector<Vehicle>> &predictions,
        int lane) {
    // Gets next timestep kinematics
    //   for a given lane. Tries to choose the maximum velocity and acceleration,
    //   given other vehicle positions and accel/velocity constraints.
    
    float max_velocity_accel_limit = this->ref_v + this->max_acceleration*timestep;
    float new_velocity;
    Vehicle vehicle_ahead;
    float max_velocity_in_front = max_velocity_accel_limit; //default to max
    
    if (get_vehicle_ahead(predictions, lane, vehicle_ahead)) {
        max_velocity_in_front = this->ref_v - this->max_acceleration*timestep;
    }
    new_velocity = std::min(std::min(max_velocity_in_front, max_velocity_accel_limit), this->target_speed);
    
    return new_velocity;
}


vector<Vehicle> Vehicle::keep_lane_trajectory(map<int, vector<Vehicle>> &predictions) {
    
    // Generate a keep lane trajectory.
    vector<Vehicle> trajectory;
    
    float new_v = get_kinematics(predictions, this->lane);
    
    trajectory.push_back(Vehicle(this->lane, 0, new_v, 0, "KL"));
    
    return trajectory;
}

vector<Vehicle> Vehicle::lane_change_trajectory(string state,
        map<int, vector<Vehicle>> &predictions) {
    
    // Generate a lane change trajectory.
    int new_lane = this->lane + lane_direction[state];
    
    vector<Vehicle> trajectory;
    Vehicle next_lane_vehicle;
    // Check if a lane change is possible (check if another vehicle occupies
    //   that spot).
    for (map<int, vector<Vehicle>>::iterator it = predictions.begin();
            it != predictions.end(); ++it) {
        for(int i=0; i< it->second.size(); i++){
            next_lane_vehicle = it->second[i];
            if (abs(next_lane_vehicle.s -this->s)<30 && next_lane_vehicle.lane == new_lane) {
                // If lane change is not possible, return empty trajectory.
                return trajectory;
            }
        }
    }
    float new_v = get_kinematics(predictions, new_lane);
    trajectory.push_back(Vehicle(new_lane, 0, new_v, 0, state));
    
    return trajectory;
}

void Vehicle::increment(int dt, int prev_size) {
    this->s = position_at(dt, prev_size);
}

float Vehicle::position_at(int t, int prev_size) {
    //check_car_s += ((double)prev_size*0.02*check_speed);
    return this->s + this->v*t*prev_size;
}

bool Vehicle::get_vehicle_behind(map<int, vector<Vehicle>> &predictions,
        int lane, Vehicle &rVehicle) {
    // Returns a true if a vehicle is found behind the current vehicle, false
    //   otherwise. The passed reference rVehicle is updated if a vehicle is found.
    int max_s = -1;
    bool found_vehicle = false;
    Vehicle temp_vehicle;
    for (map<int, vector<Vehicle>>::iterator it = predictions.begin();
            it != predictions.end(); ++it) {
        temp_vehicle = it->second[0];
        if (temp_vehicle.lane == lane && temp_vehicle.s < this->s
                && temp_vehicle.s > max_s) {
            max_s = temp_vehicle.s;
            rVehicle = temp_vehicle;
            if((this->s - max_s) < 30){
                found_vehicle = true;
            }
        }
    }
    
    return found_vehicle;
}

bool Vehicle::get_vehicle_ahead(map<int, vector<Vehicle>> &predictions,
        int lane, Vehicle &rVehicle) {
    // Returns a true if a vehicle is found ahead of the current vehicle, false
    //   otherwise. The passed reference rVehicle is updated if a vehicle is found.
    //std::ofstream debug_file;
    //debug_file.open ("debug_vehicle.log", std::ios::out | std::ios::app);
    
    int min_s = this->goal_s;
    bool found_vehicle = false;
    Vehicle temp_vehicle;
    //debug_file << "get vehicle ahead: this->s, this-> d "<< this->s << ","<< this->d<<std::endl;
    for (map<int, vector<Vehicle>>::iterator it = predictions.begin();
            it != predictions.end(); ++it) {
        for(int i=0; i< it->second.size(); i++){
            temp_vehicle = it->second[i];
            //debug_file << "other vehicle ->s, ->d: ->"<<temp_vehicle.s << ", ->"<<temp_vehicle.d << std::endl;
            if (temp_vehicle.lane == lane && temp_vehicle.s > this->s
                    && temp_vehicle.s < min_s) {
                min_s = temp_vehicle.s;
                rVehicle = temp_vehicle;
                if((min_s-this->s) < 30){
                    found_vehicle = true;
                }
            }
        }
    }
    //debug_file.close();
    return found_vehicle;
}

vector<Vehicle> Vehicle::generate_predictions(int horizon, int prev_size) {
    // Generates predictions for non-ego vehicles to be used in trajectory
    //   generation for the ego vehicle.
    vector<Vehicle> predictions;
    for(int i = 0; i < horizon; i++) {
        float next_s = position_at(i*timestep, prev_size);
        float next_v = 0;
        if (i < horizon-1) {
            next_v = position_at(i*timestep, prev_size) - s;
        }
        predictions.push_back(Vehicle(this->lane, next_s, next_v, 0));
    }
    
    return predictions;
}

void Vehicle::set_lane(double d){
    this->d = d;
    if((4 < d) && (d < 8)){
        this->lane  = 1;
    }
    else if(d < 4){
        this->lane = 0;
    }
    else if(d> 8){
        this->lane = 2;
    }
}

