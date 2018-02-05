//
//  Lane_Change_FSM.cpp
//  path_planning
//
//  Created by Scott Clegg on 1/29/18.
//
#include <algorithm>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <map>
#include <string>
#include <iterator>
#include "lane_change_FSM.h"
#include "vehicle.h"

/*
 * Initializes Finite State Machine
 */

Lane_Change_FSM::Lane_Change_FSM() {
    this->time_since_last_lane_change = time(0);
}

Lane_Change_FSM::Lane_Change_FSM(Vehicle & ego, map<int, Vehicle> & traffic) {
    this->ego = ego;
    this->traffic = traffic;
    this->time_since_last_lane_change = time(0);
}

Lane_Change_FSM::Lane_Change_FSM(Vehicle & ego, map<int, Vehicle> & traffic, vector<double> Horizon, double lane_width) {
    this->ego = ego;
    this->traffic = traffic;
    this->time_step = Horizon[0];
    this->time_horizon = Horizon[1];
    this->lane_width = lane_width;
    this->time_since_last_lane_change = time(0);
}

Lane_Change_FSM::~Lane_Change_FSM() {}

void Lane_Change_FSM::update_ego(double car_s, double car_d, double car_speed) {
    /*
     * update the ego car with it's current position and velocity
     */
    this->ego.s = car_s;
    this->ego.d = car_d;
    this->ego.v = car_speed;
}

void Lane_Change_FSM::update_traffic(const vector<vector<double>> & sensor_fusion) {
    /*
     * Update the traffic map with new sensor data
     */
    
    // Define some local varialbes
    double vehicle_s;
    double vehicle_d;
    int vehicle_lane;
    double vehicle_vx;
    double vehicle_vy;
    double vehicle_v;
    vector<int> vehicle_ids;
    
    // Loop through the sensor fusion vector
    for(int i=0; i<sensor_fusion.size(); i++) {
        
        vehicle_d = sensor_fusion[i][6];
        if(vehicle_d > 0) {
            // only consider vehicles with a positive d (simulator artifact)
            // parsing the ith row of the sensor fusion data
            vehicle_lane = int(vehicle_d/4.);
            vehicle_s = sensor_fusion[i][5];
            vehicle_vx = sensor_fusion[i][3];
            vehicle_vy = sensor_fusion[i][4];
            vehicle_v = sqrt((vehicle_vx*vehicle_vx) + (vehicle_vy*vehicle_vy));
            
            // Make a list of the sensor fusion vehicle id's
            // This will be to check if the id does or does not already
            // exist in the traffic map
            int vehicle_id = sensor_fusion[i][0];
            vehicle_ids.push_back(vehicle_id);
            
            
            map<int, Vehicle>::iterator it = this->traffic.find(vehicle_id);
            if(it != this->traffic.end()) {
                // id in traffic map so update vehicle data
                it->second.lane = vehicle_lane;
                it->second.s = vehicle_s;
                it->second.d = vehicle_d;
                it->second.v = vehicle_v;
            } else {
                // id not in traffic map so add new vehicle
                Vehicle vehicle = Vehicle(vehicle_lane, vehicle_s, vehicle_d, vehicle_v, 0., "CS");
                this->traffic.insert(std::pair<int, Vehicle>(vehicle_id, vehicle));
            }
        }
    }
    
    // now check vehicle id's that are nolonger in the fusion sensor list
    // and remove any vehicles that are not in the sensor fusion lis.
    map<int, Vehicle>::iterator it = this->traffic.begin();
    vector<int>::iterator idx;
    while(it != this->traffic.end()) {
        int vehicle_id = it->first;
        idx = find(vehicle_ids.begin(), vehicle_ids.end(), vehicle_id);
        if(idx == vehicle_ids.end()) {
            // this vehicle is dropped off the list
            // so remove it from the vehicle list
            this->traffic.erase(it);
        }
        it++;
    }
}

Vehicle Lane_Change_FSM::choose_next_state() {
    /*
     * Choose the next ego state given the current ego state
     * and surrounding traffic.
     *
     * Select the lowest cost of the possible successor lane choices.
     *
     */

    // Get successor states for the ego car given it's current state
    successor_states();
    vector<string> successors = this->possible_states;
    
    // Compute cost assocated with each potential successor state
    vector<double> costs;
    vector<Vehicle> possible_ego_states;

    cout << "costs: ";
    for(int i=0; i<successors.size(); i++){
        string possible_state = successors[i];
        
        // Generate the next succsor position for the ego for the given state
        // The trajectory is estimated for a specified time horizon
        Vehicle next_ego_state = gen_next_state(possible_state, this->ego);
        next_ego_state.state = possible_state;
        double cost = cost_function(next_ego_state);
        cout << setw(5) << possible_state << ":" << setprecision(4) << cost << ", ";
        costs.push_back(cost);
        possible_ego_states.push_back(next_ego_state);
    }

    // Now get state for minimum cost
    vector<double>::iterator minimum_cost = min_element(begin(costs), end(costs));
    int minimum_cost_idx = distance(begin(costs), minimum_cost);
    // Finally return trajectory of minimum cost
    return possible_ego_states[minimum_cost_idx];
}

void Lane_Change_FSM::successor_states() {
    /*
     * Make a list of the possible next states given the current state for the ego vehicle.
     * Note the list checks if ego car is in left or rightmost lane to prevent moving the
     * vehicle off the road.
     */
    vector<string> states;
    
    string current_state = this->ego.state;
    int current_lane = this->ego.lane;
    int lanes_available = this->ego.lanes_available;
    
    // All states all for keep lane is an option for all states
    states.push_back("KL");

    
    // If time since last change is less than the allowable time then keep lane
    time_t current_time = time(0);
    double delta_time = difftime(current_time, this->time_since_last_lane_change);
    if(delta_time < time_between_lane_chages) {
        this->possible_states = states;
        return;
    }
    
    // Determine successor states given the current state
    if(current_state.compare("KL") == 0) {
        // If Keep lane is current state then next possible states
        // are prep for lane change left or right
        if (current_lane != 0)
            states.push_back("PLCL");
        if (current_lane != lanes_available - 1)
            states.push_back("PLCR");
        
    } else if (current_state.compare("PLCL") == 0) {
        // If prep for lane change left is current state AND
        // the ego is not alreay in the left lane then
        // two additional states are possible left lane change or
        // prep for lane change left
        if (current_lane != 0) {
            states.push_back("PLCL");
            states.push_back("LCL");
        }
    } else if (current_state.compare("PLCR") == 0) {
        // If prep for lane change right is current state AND
        // the ego is not alreay in the right lane then
        // two additional states are possible right lane change or
        // prep for lane change change right
        if (current_lane != lanes_available - 1) {
            states.push_back("PLCR");
            states.push_back("LCR");
        }
    }
    
    //If state is "LCL" or "LCR", then just return "KL"
    this->possible_states = states;
}

Vehicle Lane_Change_FSM::gen_next_state(string state, Vehicle & vehicle_current_state) {
    /*
     * Given a possible next lane state, generate the vehicle's next state.
     * to realize the next state for the car.
     */
    if (state.compare("CS") == 0) {
        return constant_speed_trajectory(vehicle_current_state);
    } else if (state.compare("KL") == 0) {
        return keep_lane_trajectory(vehicle_current_state);
    } else if (state.compare("LCL") == 0 || state.compare("LCR") == 0) {
        return lane_change_trajectory(state, vehicle_current_state);
    } else if (state.compare("PLCL") == 0 || state.compare("PLCR") == 0) {
        return prep_lane_change_trajectory(state, vehicle_current_state);
    } else {
        cout << "State " << state << " not accessable" << endl;
        throw exception();
    }
}

Vehicle Lane_Change_FSM::constant_speed_trajectory(Vehicle & vehicle) {
    /*
     * Generate the next vehicle state for a constant speed vehicle.
     */
    Vehicle next_vehicle_state = Vehicle(vehicle.lane, vehicle.s, vehicle.d, vehicle.v, 0., "CS");
    next_vehicle_state.s = next_vehicle_state.position_at(time_horizon);
    return next_vehicle_state;
}

Vehicle Lane_Change_FSM::keep_lane_trajectory(Vehicle & vehicle) {
    /*
     * Generate the next vehicle state if the vehicle remains in the lane
     * over the next time horizon.
     */
    int lane = vehicle.lane;
    double target_speed = this->ego.max_speed;
    double delta_t = this->time_horizon;
    double max_a = this->ego.max_acceleration;
    double safe_following_distance = vehicle.length + this->gap_ahead_factor;
    
    // Get traffic ahead and behind of vehicle
    Vehicle vehicle_ahead = get_vehicle_ahead(vehicle, lane);
    Vehicle vehicle_behind = get_vehicle_behind(vehicle, lane);
    
    double gap_a = vehicle_ahead.gap_ahead;     // gap to vehicle ahead
    double gap_b = vehicle_behind.gap_behind;   // gap to vehicle behind
    
    // Set new speed which cannot exceed the target speed
    // Note the new speed is weighted by the proximity of the vehicle ahead such that
    // as the gap closes the vehicle ahead speed gets more weight.
    double new_speed;
    double new_accel;
    if(vehicle_ahead.v == 0) {
        // If there is not vehicle ahead then go as fast as possible
        new_speed = min(vehicle.v + max_a*delta_t, target_speed);
        new_accel = min((new_speed-vehicle.v)/delta_t, max_a);
    } else {
        // Set the new speed based on an average of the current vehicle speed and
        // the vehicle ahead speed.  The average is weighted by the proximity of the
        // vehicle ahead.
        double exp_gap = exp(-gap_a/this->gap_ahead_factor);
        if(gap_a < safe_following_distance) {
            // too close to car ahead...match the speed of the car ahead
            new_speed = vehicle_ahead.v;
            new_accel = (new_speed - vehicle.v)/delta_t;
        } else {
            // Accelerate as much as possible
            new_accel = max_a;
            new_speed = min((vehicle.v*(1.-exp_gap) + vehicle_ahead.v*exp_gap + new_accel*delta_t), target_speed);
            new_accel = max(min((new_speed-vehicle.v)/delta_t, max_a), -0.5*max_a);
        }
    }
    // Finally calculate the new position (s, d) for the vehicle
    double new_s = vehicle.position_at(delta_t);
    double new_d = (lane+0.5)*this->lane_width;
    // double new_d = vehicle.d;   // Move to the center of the lane
    
    // Return the next vehicle state
    return Vehicle(lane, new_s, new_d, new_speed, new_accel, "KL", gap_a, gap_b);
}

Vehicle Lane_Change_FSM::prep_lane_change_trajectory(string state, Vehicle & vehicle) {
    /*
     * Generates next vehicle state for a vehicle that is preparing for a
     * lane change.
     */
    int lane = vehicle.lane;
    int new_lane = lane + lane_direction[state];
    double target_speed = this->ego.max_speed;
    double delta_t = this->time_horizon;
    double max_a = this->ego.max_acceleration;
    double safe_following_distance = vehicle.length + this->gap_ahead_factor;
    
    // Get traffic ahead and behind of vehicle
    Vehicle vehicle_ahead = get_vehicle_ahead(vehicle, lane);
    Vehicle vehicle_behind = get_vehicle_behind(vehicle, lane);
    
    double gap_a = vehicle_ahead.gap_ahead;     // gap to vehicle ahead
    double gap_b = vehicle_behind.gap_behind;   // gap to vehicle behind
    
    // Get traffic ahead and behind of vehicle for new lane
    Vehicle vehicle_ahead_new = get_vehicle_ahead(vehicle, new_lane);
    Vehicle vehicle_behind_new = get_vehicle_behind(vehicle, new_lane);
    
    double gap_a_new = vehicle_ahead_new.gap_ahead;     // gap to vehicle ahead
    double gap_b_new = vehicle_behind_new.gap_behind;   // gap to vehicle behind
    
    // Set new speed which cannot exceed the target speed
    // Note the new speed is weighted by the proximity of the vehicle ahead such that
    // as the gap closes the vehicle ahead speed gets more weight.
    double new_speed;
    double new_accel;
    if(vehicle_ahead.v == 0) {
        // No vehicle ahead...go as fast as possible
        new_speed = min(vehicle.v + max_a*delta_t, target_speed);
        new_accel = min((new_speed-vehicle.v)/delta_t, max_a);
    } else {
        // Set the new speed on an average of the current vehicle speed and the
        // speeds of the vehicles ahead and behind.
        double exp_gap_a = exp(-gap_a/this->gap_ahead_factor);
        double exp_gap_b = exp(-gap_b/this->gap_behind_factor);
        if(gap_a < safe_following_distance) {
            // too close to car ahead...slow down
            new_accel = -0.5*max_a;
        } else {
            new_accel = max_a;
        }
        new_speed = min(vehicle.v*(1.-exp_gap_a-exp_gap_b) + vehicle_ahead.v*exp_gap_a + vehicle_behind.v*exp_gap_b + new_accel*delta_t, target_speed);
        new_accel = max(min((new_speed-vehicle.v)/delta_t, max_a), -0.5*max_a);
    }
    
    // Finally calculate the new position (s, d) for the vehicle
    double new_s = vehicle.position_at(delta_t);
    double new_d = (lane+0.5)*this->lane_width;
    // double new_d = vehicle.d;
    
    // Return the next vehicle state
    return Vehicle(lane, new_s, new_d, new_speed, new_accel, state, gap_a_new, gap_b_new);
}

Vehicle Lane_Change_FSM::lane_change_trajectory(string state, Vehicle & vehicle) {
    /*
     * Generates next vehicle state for a vehicle that is chaning lanes.
     */
    int lane = vehicle.lane;
    int new_lane = lane + lane_direction[state];
    double target_speed = this->ego.max_speed;
    double delta_t = this->time_horizon;
    double max_a = this->ego.max_acceleration;
    /*
    // Get traffic ahead and behind of vehicle
    Vehicle vehicle_ahead = get_vehicle_ahead(vehicle, lane);
    Vehicle vehicle_behind = get_vehicle_behind(vehicle, lane);
    
    double gap_a = vehicle_ahead.gap_ahead;     // gap to vehicle ahead
    double gap_b = vehicle_behind.gap_behind;   // gap to vehicle behind
    */
    // Get traffic ahead and behind of vehicle for new lane
    Vehicle vehicle_ahead = get_vehicle_ahead(vehicle, new_lane);
    Vehicle vehicle_behind = get_vehicle_behind(vehicle, new_lane);
    
    double gap_a = vehicle_ahead.gap_ahead;     // gap to vehicle ahead
    double gap_b = vehicle_behind.gap_behind;   // gap to vehicle behind
    
    // Set new speed which cannot exceed the target speed
    if(vehicle_ahead.v == 0)
        vehicle_ahead.v = target_speed;     // No vehicle ahead so go as fast as possible
    
    double new_speed = (vehicle_ahead.v + vehicle.a*delta_t);
    new_speed = min(new_speed, target_speed);
    
    // Set new acceleration which cannot exceed max acceleration
    double new_accel = min((new_speed - vehicle.v)/delta_t, max_a);
    
    // Finally calculate the new position (s, d) for the vehicle
    double new_s = vehicle.position_at(delta_t);
    double new_d = (double(new_lane)+0.5)*this->lane_width;
    
    // Return the next vehicle state
    return Vehicle(new_lane, new_s, new_d, new_speed, new_accel, state, gap_a, gap_b);
}

Vehicle Lane_Change_FSM::get_vehicle_ahead(Vehicle & vehicle, int lane) {
    /*
     * Returns rVehicle found in the traffic that is ahead of the current vehicle.
     */
    
    Vehicle rVehicle;
    double min_gap = 1.e99;
    double car_length = vehicle.length;
    
    // Loop through traffic
    for (map<int, Vehicle>::iterator it = this->traffic.begin(); it != this->traffic.end(); ++it) {
        Vehicle test_vehicle = it->second;
        float gap = (test_vehicle.s - vehicle.s) - car_length;
        
        if ((test_vehicle.lane == lane) && (test_vehicle.s > vehicle.s) && (gap < min_gap)) {
            // Vehicle is ahead and in lane
            min_gap = gap;
            rVehicle = test_vehicle;
            rVehicle.gap_ahead = gap;
        }
    }

    return rVehicle;
}

Vehicle Lane_Change_FSM::get_vehicle_behind(Vehicle & vehicle, int lane) {
    /*
     * Returns rVehicle found in the traffic that is behind of the current vehicle
     */
    
    Vehicle rVehicle;
    double min_gap = 1.e99;
    double car_length = vehicle.length;
    
    // Loop through traffic
    for (map<int, Vehicle>::iterator it = this->traffic.begin(); it != this->traffic.end(); ++it) {
        Vehicle test_vehicle = it->second;
        float gap = (vehicle.s - test_vehicle.s) - car_length;
        
        if ((test_vehicle.lane == lane) && (test_vehicle.s < vehicle.s) && (gap < min_gap)) {
            // Vehicle is behind and in lane
            min_gap = gap;
            rVehicle = test_vehicle;
            rVehicle.gap_behind = gap;
        }
    }
    
    return rVehicle;
}

vector<double> Lane_Change_FSM::get_new_kinematics(Vehicle & v1, Vehicle & v2) {
    /*
     * Determine the speed and acceration for matching v1 and v2
     */
    float new_s;
    float new_d;
    float new_speed;
    float new_accel;
    
    if(v2.v > v1.v) {
        // Vehicle 2 is driving faster than vechicle 1.
        // Accelerate to match speed of vehicle ahead without over accelerating
        // or exceeding the target speed.
        new_speed = min(min(v2.v, v1.v + v1.a*time_horizon),v1.max_speed);
        new_accel = (new_speed-v1.v)/time_horizon;
    } else {
        // Vehicle 2 is driving slower
        // Decelerate to match speed to vehicle ahead
        new_speed = v2.v;
        new_accel = (new_speed-v1.v)/time_horizon;
    }
    
    // Get new position in fernet coordinates give new speed and acceleration and time horizon.
    new_s = Vehicle(v1.lane, v1.s, v1.d, new_speed, new_accel).position_at(time_horizon);
    new_d = v1.d;
    return {new_s, new_d, new_speed, new_accel};
}

double Lane_Change_FSM::cost_function(Vehicle & vehicle) {
    /*
     * Lane change cost funciton.
     *
     * 1. At desired target speed
     * 2. Gap to traffic immediately ahead
     * 3. Gap to traffic immediately behind
     * 4. Penalty for repeated change of lanes
     *
     */
    vector<double> weight = {1000., 100., 100., 1.};
    double cost = 0;
    
    // Cost component related to cost of not going the target speed.
    double new_speed = vehicle.v;
    double speed_cost = exp(-new_speed);
    cost += weight[0]*speed_cost;
    
    // Cost componet due to gap ahead
    double gap_ahead = vehicle.gap_ahead;
    double gap_ahead_cost = exp(-gap_ahead/this->gap_ahead_factor);
    cost += weight[1]*gap_ahead_cost;
    
    // Cost componet due to gap behind
    double gap_behind = vehicle.gap_behind;
    double gap_behind_cost = exp(-gap_behind/this->gap_behind_factor);
    cost += weight[2]*gap_behind_cost;
    
    return cost;
}

void Lane_Change_FSM::next_ego_state(Vehicle & vehicle_next_state) {
    /*
     * update the ego state to the vehicle's next state
     */
    
    // Reset lane change timer if a lane change has occured
    if(this->ego.lane != vehicle_next_state.lane)
        this->time_since_last_lane_change = time(0);
    
    // Now update the ego states
    this->ego.state = vehicle_next_state.state;
    this->ego.s = vehicle_next_state.s;
    this->ego.d = vehicle_next_state.d;
    this->ego.v = vehicle_next_state.v;
    this->ego.a = vehicle_next_state.a;
    this->ego.lane = vehicle_next_state.lane;
}
