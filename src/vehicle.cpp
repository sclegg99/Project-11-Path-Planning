#include <algorithm>
#include <iostream>
#include <cmath>
#include <map>
#include <string>
#include <iterator>
#include "vehicle.h"

/*
 * Initializes Vehicle
 */

Vehicle::Vehicle(){}

Vehicle::Vehicle(int lane, double s, double d, double v, double a) {
    /*
     * Initialize vehicle position, velocity and acceleration
     */
    this->lane = lane;
    this->s = s;
    this->d = d;
    this->v = v;
    this->a = a;
}

Vehicle::Vehicle(int lane, double s, double d, double v, double a, string state) {
    /*
     * Initialize vehicle position, velocity, acceleration and state
     */
    this->lane = lane;
    this->s = s;
    this->d = d;
    this->v = v;
    this->a = a;
    this->state = state;
}

Vehicle::Vehicle(int lane, double s, double d, double v, double a, string state, double gap_a, double gap_b) {
    /*
     * Initialize vehicle position, velocity, acceleration and state
     */
    this->lane = lane;
    this->s = s;
    this->d = d;
    this->v = v;
    this->a = a;
    this->state = state;
    this->gap_ahead = gap_a;
    this->gap_behind = gap_b;
}

Vehicle::~Vehicle() {}

void Vehicle::configure(int lanes_available, double max_speed, double min_speed, double max_acceleration, string state) {
    /*
     * Called by simulator before simulation begins. Sets various
     * parameters which will impact the ego vehicle.
     */
    this->max_speed = max_speed;
    this->min_speed = min_speed;
    this->lanes_available = lanes_available;
    this->max_acceleration = max_acceleration;
    this->state = state;
}

double Vehicle::position_at(double dt) {
    /*
     * Calculate the position of the vehicle dt in the future
     * given it's current posiiton, velocity, and acceleration.
     * The calculation is in fernet coordinates so only s is updated.
     */
    
    return this->s + this->v*dt + .5*this->a*dt*dt;
}
