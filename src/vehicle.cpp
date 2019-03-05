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

Vehicle::Vehicle(int lane, float s, float v, float a, string state, double d, int id) {
  
  this->s = s;
  this->v = v;
  this->a = a;
  this->state = state;
  this->id = id;
  max_acceleration = 100;
  
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
  //std::ofstream debug_file;
  //debug_file.open ("debug_vehicle.log", std::ios::out | std::ios::app);
  vector<string> states = successor_states();
  //for(int i =0; i< states.size(); i++){
  //    debug_file << "sucessor states are: "<< states[i]<<std::endl;
  //}
  //debug_file.close();
  vector<vector<Vehicle>> final_trajectories;

  for (vector<string>::iterator it = states.begin(); it != states.end(); ++it) {
    vector<Vehicle> trajectory = generate_trajectory(*it, predictions);
    //debug_file.open ("debug_vehicle.log", std::ios::out | std::ios::app);
    if (trajectory.size() != 0) {
      final_trajectories.push_back(trajectory);
    }
    //debug_file.close();
     
  }
   //debug_file.open ("debug_vehicle.log", std::ios::out | std::ios::app);
   //debug_file << " final_traj. size" << final_trajectories.size() << std::endl;
   //debug_file.close();

  return final_trajectories[0];
}

vector<string> Vehicle::successor_states() {
  // Provides the possible next states given the current state for the FSM
  //   discussed in the course, with the exception that lane changes happen
  //   instantaneously, so LCL and LCR can only transition back to KL.
  vector<string> states;
  states.push_back("KL");
  string state = this->state;
  if(state.compare("KL") == 0) {
    states.push_back("PLCL");
    states.push_back("PLCR");
  } else if (state.compare("PLCR") == 0) {
    if (lane != lanes_available - 1) {
      states.push_back("PLCR");
      states.push_back("LCR");
    }
  } else if (state.compare("PLCL") == 0) {
    if (lane != 0) {
      states.push_back("PLCL");
      states.push_back("LCL");
    }
  }

  // If state is "LCL" or "LCR", then just return "KL"
  return states;
}

vector<Vehicle> Vehicle::generate_trajectory(string state,
                                             map<int, vector<Vehicle>> &predictions) {
  // Given a possible next state, generate the appropriate trajectory to realize
  //   the next state.
  vector<Vehicle> trajectory;
  std::ofstream debug_file;
  debug_file.open ("debug_vehicle.log", std::ios::out | std::ios::app);
  //debug
  debug_file << "state is: " << state << std::endl;
  
  if (state.compare("CS") == 0) {
    trajectory = constant_speed_trajectory();
  } else if (state.compare("KL") == 0) {
    trajectory = keep_lane_trajectory(predictions);
  } else if (state.compare("LCL") == 0 || state.compare("LCR") == 0) {
    trajectory = lane_change_trajectory(state, predictions);
  } else if (state.compare("PLCL") == 0 || state.compare("PLCR") == 0) {
    trajectory = prep_lane_change_trajectory(state, predictions);
  }
 debug_file.close();
  return trajectory;
}

vector<float> Vehicle::get_kinematics(map<int, vector<Vehicle>> &predictions,
                                      int lane) {
  // Gets next timestep kinematics (position, velocity, acceleration)
  //   for a given lane. Tries to choose the maximum velocity and acceleration,
  //   given other vehicle positions and accel/velocity constraints.
  std::ofstream debug_file;
  debug_file.open ("debug_vehicle.log", std::ios::out | std::ios::app);
  
  float max_velocity_accel_limit = this->max_acceleration*timestep + this->v;
  debug_file << "Kinematics- max_velocity_accel_limit: " << max_velocity_accel_limit << std::endl;
  debug_file << "Kinematics- this-> target speed: " << this->target_speed << std::endl;
  debug_file.close();
  float new_position;
  float new_velocity;
  float new_accel;
  Vehicle vehicle_ahead;
  Vehicle vehicle_behind;
  float max_velocity_in_front = 99999;
  
  if (get_vehicle_ahead(predictions, lane, vehicle_ahead)) {
      debug_file.open ("debug_vehicle.log", std::ios::out | std::ios::app);
    //  debug_file << "Kinematics- Theres a Vehicle ahead!" << std::endl;
    /*if (get_vehicle_behind(predictions, lane, vehicle_behind)) {      
      // must travel at the speed of traffic, regardless of preferred buffer
        if(vehicle_ahead.v>0){
            new_velocity = vehicle_ahead.v;
            debug_file << "Kinematics- Theres a Vehicle behind! new velocity: " << new_velocity << std::endl;
        }
      debug_file << "Kinematics- Theres a Vehicle behind! new velocity was ZERO " << std::endl;
    } else {
      */
      //max_velocity_in_front = (vehicle_ahead.s - this->s - this->preferred_buffer)/timestep; //+ vehicle_ahead.v - 0.5 * (this->a)*timestep;
      max_velocity_in_front = this->v - 0.3; 
      
      // float max_velocity_in_front = (vehicle_ahead.position_at(timestep) - this->position_at(timestep)-this->preferred_buffer);
      debug_file << "Kinematics- Vehicle ahead with S:" << vehicle_ahead.s << " vehicle d:"<<vehicle_ahead.d <<" vehicle V:" << vehicle_ahead.v << " || My S:" << this->s <<" my d:" << this->d<< " max_velocity_in_front: " << max_velocity_in_front << std::endl;
     // debug_file << "Kinematics- Vehicle ahead - new velocity: " << new_velocity << std::endl;
   /* }
  } else {
    debug_file << "Kinematics- no Vehicle ahead" << std::endl;
    new_velocity = std::min(max_velocity_accel_limit, this->target_speed);
    debug_file << "Kinematics- no Vehicle ahead - new_velocity: " << new_velocity << std::endl;
  }*/
  }
  
  new_velocity = std::min(std::min(max_velocity_in_front, max_velocity_accel_limit), this->target_speed);
  new_accel = (new_velocity - this->v)/timestep; // Equation: (v_1 - v_0)/t = acceleration
  new_position = this->s + new_velocity*timestep + new_accel*timestep*timestep / 2.0;
  debug_file << "Kinematics end: new_acel: " << new_accel << " new S pos: " << new_position << std::endl;
  debug_file.close();
  
  return{new_position, new_velocity, new_accel};
}


vector<Vehicle> Vehicle::constant_speed_trajectory() {
  // Generate a constant speed trajectory.
  float next_pos = position_at(timestep);
  vector<Vehicle> trajectory = {Vehicle(this->lane,this->s,this->v,this->a,this->state),
                                Vehicle(this->lane,next_pos,this->v,0,this->state)};
  //vector<Vehicle> trajectory = {Vehicle(this->lane,next_pos,this->v,0,this->state)};
  return trajectory;
}

vector<Vehicle> Vehicle::keep_lane_trajectory(map<int, vector<Vehicle>> &predictions) {
  std::ofstream debug_file;
  debug_file.open ("debug_vehicle.log", std::ios::out | std::ios::app);
  
          // Generate a keep lane trajectory.
  vector<Vehicle> trajectory = {Vehicle(lane, this->s, this->v, this->a, state)};
  //vector<Vehicle> trajectory;
  //debug
  debug_file<< "Keep lane trajectory: lane:" << lane << " this->s:" << this->s << "  this->v:"<<this->v <<" this->a: "<<this->a <<" state:"<<this->state<<std::endl;
  debug_file << "Keep lane trajectory will get kinematics" << std::endl;
  debug_file.close();
  vector<float> kinematics = get_kinematics(predictions, this->lane);
  float new_s = kinematics[0];
  float new_v = kinematics[1];
  float new_a = kinematics[2];
  
  trajectory.push_back(Vehicle(this->lane, new_s, new_v, new_a, "KL"));
  //debug
  debug_file.open ("debug_vehicle.log", std::ios::out | std::ios::app);
  for (int i=0; i<trajectory.size(); i++){
  debug_file<< " New trajectory -> lane:" << trajectory[i].lane << " s:" <<trajectory[i].s << " v:"<<trajectory[i].v <<" a:"<<trajectory[i].a << " state:"<<trajectory[i].state<< std::endl;
  }
  debug_file.close();
  return trajectory;
}

vector<Vehicle> Vehicle::prep_lane_change_trajectory(string state,
                                                     map<int, vector<Vehicle>> &predictions) {
  // Generate a trajectory preparing for a lane change.
  float new_s;
  float new_v;
  float new_a;
  Vehicle vehicle_behind;
  std::ofstream debug_file;
  debug_file.open ("debug_vehicle.log", std::ios::out | std::ios::app);
          
  int new_lane = this->lane + lane_direction[state];
  debug_file << "Prepare lane change traj. new lane:" << new_lane << std::endl;
  vector<Vehicle> trajectory = {Vehicle(this->lane, this->s, this->v, this->a,
                                        this->state)};
  //vector<Vehicle> trajectory;
  debug_file << "Prepare lane change traj. - will get kinematics" << std::endl;
  debug_file.close(); 
  vector<float> curr_lane_new_kinematics = get_kinematics(predictions, this->lane);
 debug_file.open ("debug_vehicle.log", std::ios::out | std::ios::app);
  if (get_vehicle_behind(predictions, this->lane, vehicle_behind)) {
    // Keep speed of current lane so as not to collide with car behind.
    new_s = curr_lane_new_kinematics[0];
    new_v = curr_lane_new_kinematics[1];
    new_a = curr_lane_new_kinematics[2];
    debug_file << "Prepare lane change traj. - theres a vehicle behind!" << std::endl;
    debug_file << "Prepare lane change traj. - new_s:" << new_s << " new_v:" << new_v << " new_a:" << new_a <<std::endl;
  } else {
    debug_file << "Prepare lane change traj. - no vehicle behind" << std::endl;
    vector<float> best_kinematics;
    debug_file.close();
    vector<float> next_lane_new_kinematics = get_kinematics(predictions, new_lane);
     debug_file.open ("debug_vehicle.log", std::ios::out | std::ios::app);
    // Choose kinematics with lowest velocity.
    if (next_lane_new_kinematics[1] < curr_lane_new_kinematics[1]) {
      best_kinematics = next_lane_new_kinematics;
      debug_file << "Prepare lane change traj. - best kinematics is in next lane" << std::endl;
    } else {
      best_kinematics = curr_lane_new_kinematics;
      debug_file << "Prepare lane change traj. - best kinematics is in current lane" << std::endl;
    }
    new_s = best_kinematics[0];
    new_v = best_kinematics[1];
    new_a = best_kinematics[2];
  }

  trajectory.push_back(Vehicle(this->lane, new_s, new_v, new_a, state));
  for (int i=0; i<trajectory.size(); i++){
  debug_file<< " New trajectory -> lane:" << trajectory[i].lane << " s:" <<trajectory[i].s << " v:"<<trajectory[i].v <<" a:"<<trajectory[i].a << " state:"<<trajectory[i].state<< std::endl;
  }
  debug_file.close(); 
  return trajectory;
}

vector<Vehicle> Vehicle::lane_change_trajectory(string state,
                                                map<int, vector<Vehicle>> &predictions) {
  std::ofstream debug_file;
  debug_file.open ("debug_vehicle.log", std::ios::out | std::ios::app);
    // Generate a lane change trajectory.
  int new_lane = this->lane + lane_direction[state];
  debug_file << "Lane change traj. new lane:" <<  new_lane << std::endl;
  vector<Vehicle> trajectory;
  Vehicle next_lane_vehicle;
  // Check if a lane change is possible (check if another vehicle occupies
  //   that spot).
  for (map<int, vector<Vehicle>>::iterator it = predictions.begin();
       it != predictions.end(); ++it) {
    next_lane_vehicle = it->second[0];
    if (next_lane_vehicle.s == this->s && next_lane_vehicle.lane == new_lane) {
        debug_file << "Lane change traj. - change to lane " << new_lane << " is not possible" << std::endl;
      // If lane change is not possible, return empty trajectory.
      return trajectory;
    }
  }
  trajectory.push_back(Vehicle(this->lane, this->s, this->v, this->a,
                               this->state));
  vector<float> kinematics = get_kinematics(predictions, new_lane);
  trajectory.push_back(Vehicle(new_lane, kinematics[0], kinematics[1],
                               kinematics[2], state));
   
  for (int i=0; i<trajectory.size(); i++){
  debug_file<< " New trajectory -> lane:" << trajectory[i].lane << " s:" <<trajectory[i].s << " v:"<<trajectory[i].v <<" a:"<<trajectory[i].a << " state:"<<trajectory[i].state<< std::endl;
  }
    debug_file.close(); 
  return trajectory;
}

void Vehicle::increment(int dt = timestep) {
  this->s = position_at(dt);
}

float Vehicle::position_at(int t) {
  return this->s + this->v*t + this->a*t*t/2.0;
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
      if((this->s - max_s) < 50){
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
  std::ofstream debug_file;
  debug_file.open ("debug_vehicle.log", std::ios::out | std::ios::app);
  
  int min_s = this->goal_s;
  bool found_vehicle = false;
  Vehicle temp_vehicle;
  debug_file << "get vehicle ahead: this->s, this-> d "<< this->s << ","<< this->d<<std::endl;
  for (map<int, vector<Vehicle>>::iterator it = predictions.begin();
       it != predictions.end(); ++it) {
    temp_vehicle = it->second[0];
    debug_file << "other vehicle ->s, ->d: ->"<<temp_vehicle.s << ", ->"<<temp_vehicle.d << std::endl;
    if (temp_vehicle.lane == lane && temp_vehicle.s > this->s
        && temp_vehicle.s < min_s) {
      min_s = temp_vehicle.s;
      rVehicle = temp_vehicle;
      if((min_s-this->s) < 50){
        found_vehicle = true;
      }
    }
  }
 debug_file.close();
  return found_vehicle;
}

vector<Vehicle> Vehicle::generate_predictions(int horizon) {
  // Generates predictions for non-ego vehicles to be used in trajectory
  //   generation for the ego vehicle.
  vector<Vehicle> predictions;
  for(int i = 0; i < horizon; i++) {
    float next_s = position_at(i*timestep);
    float next_v = 0;
    if (i < horizon-1) {
      next_v = position_at(i*timestep) - s;
    }
    predictions.push_back(Vehicle(this->lane, next_s, next_v, 0));
  }

  return predictions;
}

void Vehicle::realize_next_state(vector<Vehicle> &trajectory) {
  // Sets state and kinematics for ego vehicle using the last state of the trajectory.
  Vehicle next_state = trajectory[1];
  this->state = next_state.state;
  this->lane = next_state.lane;
  this->s = next_state.s;
  this->v = next_state.v;
  this->a = next_state.a;
}

void Vehicle::configure(vector<int> &road_data) {
  // Called by simulator before simulation begins. Sets various parameters which
  //   will impact the ego vehicle.
  target_speed = road_data[0];
  lanes_available = road_data[1];
  goal_s = road_data[2];
  goal_lane = road_data[3];
  max_acceleration = road_data[4];
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

