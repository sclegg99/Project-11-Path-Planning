//
//  Lane_Change_FSM.hpp
//  path_planning
//
//  Created by Scott Clegg on 1/29/18.
//

#ifndef lane_change_FSM_h
#define lane_change_FSM_h

#include <iostream>
#include <vector>
#include <map>
#include <string>
#include <ctime>
#include "vehicle.h"

using namespace std;

class Lane_Change_FSM {
private:
    vector<string> possible_states;
    
    double gap_ahead_factor = 20.;
    
    double gap_behind_factor = 10.;
    
    time_t time_since_last_lane_change;
    
public:
    
    map<string, int> lane_direction = {{"PLCL", -1}, {"LCL", -1}, {"LCR", 1}, {"PLCR", 1}};
    
    double lane_width = 4.;
    
    double time_horizon = 1;
    
    double time_step = .02;
    
    int ahead_gap = 2;  // Factor for minimum allowable gap ahead.
    
    int behind_gap = 1; // Factor for minimum allowable gap behind.
    
    double time_between_lane_chages = 5;    // Minimum time (sec) allowed between lane changes.
    
    Vehicle ego;
    
    map<int, Vehicle> traffic;
    
    /*
     * Constructors
     */
    Lane_Change_FSM();
    Lane_Change_FSM(Vehicle & ego, map<int, Vehicle> & traffic);
    Lane_Change_FSM(Vehicle & ego, map<int, Vehicle> & traffic, vector<double> Horizon, double lane_width);
    
    /*
     * Destructor
     */
    virtual ~Lane_Change_FSM();
    
    void successor_states();
    
    void update_ego(double car_s, double car_d, double car_speed);
    
    void update_traffic(const vector<vector<double>> & sensor_fusion);
    
    Vehicle choose_next_state();
    
    Vehicle gen_next_state(string state, Vehicle & vehicle_current_state);
    
    Vehicle constant_speed_trajectory(Vehicle & vehicle);
    
    Vehicle keep_lane_trajectory(Vehicle & vehicle);
    
    Vehicle prep_lane_change_trajectory(string state, Vehicle & vehicle);
    
    Vehicle lane_change_trajectory(string state, Vehicle & vehicle);
    
    Vehicle get_vehicle_ahead(Vehicle & car, int lane);
    
    Vehicle get_vehicle_behind(Vehicle & car, int lane);
    
    double cost_function(Vehicle & vehicle_next_state);
    
    void next_ego_state(Vehicle & vehicle_next_state);
    
    vector<double> get_new_kinematics(Vehicle & v1, Vehicle & v2);
};

#endif /* lane_change_FSM_h */
