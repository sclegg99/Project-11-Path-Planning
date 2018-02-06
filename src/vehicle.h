#ifndef VEHICLE_H
#define VEHICLE_H
#include <iostream>
#include <vector>
#include <map>
#include <string>

using namespace std;

class Vehicle {
public:
    
    int lane;
    
    double s = 0;
    
    double d = 0;
    
    double v = 0;
    
    double a = 5;
    
    double length = 2;  // Length of car
    
    double max_speed = 0;
    
    double min_speed = 0;
    
    double max_acceleration = 5;
    
    double lanes_available = 1;
    
    double gap_ahead = 1.e99;
    
    double gap_behind = 1.e99;
    
    string state;
    
    /*
     * Constructors
     */
    Vehicle();
    Vehicle(int lane, double s, double d, double v, double a);
    Vehicle(int lane, double s, double d, double v, double a, string state);
    Vehicle(int lane, double s, double d, double v, double a, string state, double gap_a, double gap_b);
    
    /*
     * Destructor
     */
    virtual ~Vehicle();
    
    void configure(int lanes_available, double max_speed, double min_speed, double max_acceleration, string state);
    
    double position_at(double dt);
    
};

#endif
