vector<float> Lane_Change_FSM::get_kinematics(Vehicle car, int lane) {
/*
* Gets next timestep kinematics (position, velocity, acceleration)
* for a given lane. Tries to choose the maximum velocity and acceleration,
* given other vehicle positions and accel/velocity constraints.
*/

float new_velocity;
Vehicle vehicle_ahead;
Vehicle vehicle_behind;

// Set maximum velocity for this time step to be equal to
// the current velocity plus the maximum acceleration times the time horizon
float max_velocity_accel_limit = car.max_acceleration*time_horizon + car.v;
float min_velocity_accel_limit =-car.max_acceleration*time_horizon + car.v;

if (get_vehicle_ahead(car, car.lane, vehicle_ahead)) {
// Look for traffic ahead in car's lane.

if (get_vehicle_behind(car, car.lane, vehicle_behind)) {
// Vehicle is behind
// Car must travel at the speed of traffic, regardless of preferred buffer
new_velocity = vehicle_behind.v;

} else {
// Vehicle ahead but not behind
// Match speed of car ahead without over acceleration
new_velocity = max(min(vehicle_ahead.v, max_velocity_accel_limit), min_velocity_accel_limit);
}
} else {
// otherwise drive at target speed.
new_velocity = max(min(max_velocity_accel_limit, car.target_speed), min_velocity_accel_limit);
}

// Create a tempoary vehicle with update acceleration
Vehicle temp = car;

// Acceleration required to get to this new velocity over the time_horizon
float new_acceleration = (new_velocity - car.v)/time_horizon;
temp.a = new_acceleration;

// Now calculate next position of car
float next_s = temp.position_at(time_horizon);
float next_d = (4*lane)+2;
float next_v = new_velocity;
float next_a = new_acceleration;

return{next_s, next_d, next_v, next_a};
}
