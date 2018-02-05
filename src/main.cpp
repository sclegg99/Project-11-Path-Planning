#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <set>
#include <map>
#include <thread>         // std::this_thread::sleep_for
#include <chrono>         // std::chrono::seconds
#include "json.hpp"
#include "spline.h"
#include "vehicle.h"
#include "lane_change_FSM.h"

#define LANE_WIDTH 4.
#define CRASH_ZONE 100.

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.find_first_of("}");
    if (found_null != string::npos) {
        return "";
    } else if (b1 != string::npos && b2 != string::npos) {
        return s.substr(b1, b2 - b1 + 2);
    }
    return "";
}

double distance(double x1, double y1, double x2, double y2)
{
    return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{
    
    double closestLen = 100000; //large number
    int closestWaypoint = 0;
    
    for(int i = 0; i < maps_x.size(); i++)
    {
        double map_x = maps_x[i];
        double map_y = maps_y[i];
        double dist = distance(x,y,map_x,map_y);
        if(dist < closestLen)
        {
            closestLen = dist;
            closestWaypoint = i;
        }
        
    }
    
    return closestWaypoint;
    
}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
    
    int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);
    
    double map_x = maps_x[closestWaypoint];
    double map_y = maps_y[closestWaypoint];
    
    double heading = atan2((map_y-y),(map_x-x));
    
    double angle = fabs(theta-heading);
    angle = min(2*pi() - angle, angle);
    
    if(angle > pi()/4)
    {
        closestWaypoint++;
        if (closestWaypoint == maps_x.size())
        {
            closestWaypoint = 0;
        }
    }
    
    return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
    int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);
    
    int prev_wp;
    prev_wp = next_wp-1;
    if(next_wp == 0)
    {
        prev_wp  = maps_x.size()-1;
    }
    
    double n_x = maps_x[next_wp]-maps_x[prev_wp];
    double n_y = maps_y[next_wp]-maps_y[prev_wp];
    double x_x = x - maps_x[prev_wp];
    double x_y = y - maps_y[prev_wp];
    
    // find the projection of x onto n
    double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
    double proj_x = proj_norm*n_x;
    double proj_y = proj_norm*n_y;
    
    double frenet_d = distance(x_x,x_y,proj_x,proj_y);
    
    //see if d value is positive or negative by comparing it to a center point
    
    double center_x = 1000-maps_x[prev_wp];
    double center_y = 2000-maps_y[prev_wp];
    double centerToPos = distance(center_x,center_y,x_x,x_y);
    double centerToRef = distance(center_x,center_y,proj_x,proj_y);
    
    if(centerToPos <= centerToRef)
    {
        frenet_d *= -1;
    }
    
    // calculate s value
    double frenet_s = 0;
    for(int i = 0; i < prev_wp; i++)
    {
        frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
    }
    
    frenet_s += distance(0,0,proj_x,proj_y);
    
    return {frenet_s,frenet_d};
    
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
    int prev_wp = -1;
    
    while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
    {
        prev_wp++;
    }
    
    int wp2 = (prev_wp+1)%maps_x.size();
    
    double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
    // the x,y,s along the segment
    double seg_s = (s-maps_s[prev_wp]);
    
    double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
    double seg_y = maps_y[prev_wp]+seg_s*sin(heading);
    
    double perp_heading = heading-pi()/2;
    
    double x = seg_x + d*cos(perp_heading);
    double y = seg_y + d*sin(perp_heading);
    
    return {x,y};
    
}

int main() {
    uWS::Hub h;
    
    // Load up map values for waypoint's x,y,s and d normalized normal vectors
    vector<double> map_waypoints_x;
    vector<double> map_waypoints_y;
    vector<double> map_waypoints_s;
    vector<double> map_waypoints_dx;
    vector<double> map_waypoints_dy;
    
    // Waypoint map to read from
    string map_file_ = "../data/highway_map.csv";
    // The max s value before wrapping around the track back to 0
    // double max_s = 6945.554;
    
    ifstream in_map_(map_file_.c_str(), ifstream::in);
    
    string line;
    while (getline(in_map_, line)) {
        istringstream iss(line);
        double x;
        double y;
        double s;
        double d_x;
        double d_y;
        iss >> x;
        iss >> y;
        iss >> s;
        iss >> d_x;
        iss >> d_y;
        map_waypoints_x.push_back(x);
        map_waypoints_y.push_back(y);
        map_waypoints_s.push_back(s);
        map_waypoints_dx.push_back(d_x);
        map_waypoints_dy.push_back(d_y);
    }
    
    map<int, Vehicle> traffic;
    
    // Define ego speed limit, maximum acceleration and number of lanes it can travel in
    int NUM_LANES = 3;
    int START_LANE = 1;
    double MAX_SPEED_LIMIT = float(49.5/2.24); // m/s
    double MIN_SPEED_LIMIT = 1.;
    double MAX_ACCEL = 5;   // m/s/s
    string STATE = "KL";

    Vehicle ego;
    ego.configure(NUM_LANES, MAX_SPEED_LIMIT, MIN_SPEED_LIMIT, MAX_ACCEL, STATE);
    ego.lane = START_LANE;
    
    vector<double> Horizon;
    double dt = 0.02;    //Prediction time steps
    double T = 1.0;      //Prediciton time duration
    Horizon.push_back(dt);
    Horizon.push_back(T);
    
    // Initialize the lane change finite state machine;
    Lane_Change_FSM fsm = Lane_Change_FSM(ego, traffic, Horizon, LANE_WIDTH);
    
    // Define reference velocity
    double ref_vel = 0.0;  // MPH
    
    h.onMessage([&map_waypoints_x, &map_waypoints_y, &map_waypoints_s, &map_waypoints_dx, &map_waypoints_dy,
                 &ref_vel, &fsm](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                                            uWS::OpCode opCode) {
        
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        //auto sdata = string(data).substr(0, length);
        //cout << sdata << endl;
        
        if (length && length > 2 && data[0] == '4' && data[1] == '2') {
            
            auto s = hasData(data);
            
            if (s != "") {
                auto j = json::parse(s);
                
                string event = j[0].get<string>();
                
                if (event == "telemetry") {
                    // j[1] is the data JSON object
                    
                    // Main car's localization Data
                    double car_x = j[1]["x"];
                    double car_y = j[1]["y"];
                    double car_s = j[1]["s"];
                    double car_d = j[1]["d"];
                    double car_yaw = j[1]["yaw"];
                    double car_speed = j[1]["speed"];
                    car_speed = car_speed/2.24;     // convert car speed to m/s
                    
                    // Previous path data given to the Planner
                    auto previous_path_x = j[1]["previous_path_x"];
                    auto previous_path_y = j[1]["previous_path_y"];
                    // Previous path's end s and d values
                    double end_path_s = j[1]["end_path_s"];
                    double end_path_d = j[1]["end_path_d"];
                    
                    // Sensor Fusion Data, a list of all other cars on the same side of the road.
                    auto sensor_fusion = j[1]["sensor_fusion"];
                    
                    json msgJson;
                    
                    vector<double> next_x_vals;
                    vector<double> next_y_vals;
                    
                    // Get prediction time step and time horizon
                    double dt = fsm.time_step;
                    double Time = fsm.time_horizon;
                    int time_steps = Time/dt;

                    double MAX_SPEED_LIMIT = fsm.ego.max_speed;
                    double MIN_SPEED_LIMIT = fsm.ego.min_speed;
                    
                    // Update ego car
                    fsm.update_ego(car_s, car_d, car_speed);
                    
                    // Update the traffic
                    fsm.update_traffic(sensor_fusion);
                    
                    // Determine the best next state for the ego vehicle
                    Vehicle next_ego_state = fsm.choose_next_state();
                    
                    cout << "next state:" << setw(4) << next_ego_state.state << " lane:" << setw(2)  << next_ego_state.lane << endl;
                    
                    // Lane to drive in
                    int old_lane = fsm.ego.lane;
                    int new_lane = next_ego_state.lane;
                    
                    int prev_size = previous_path_x.size();

                    if(prev_size > 0) {
                        car_s = end_path_s;
                    }
                    
                    // Create a list of widely space (x,y) waypoints, evenly space at 30m
                    // Later we will interpolate these points with a spline and fill it in with more
                    // points that control speed.
                    vector<double> ptsx;
                    vector<double> ptsy;
                    
                    // Reference x, y, and yaw states
                    // either we will reference the starting point as where the car is or
                    // at the previous paths and point.
                    double ref_x = car_x;
                    double ref_y = car_y;
                    double ref_yaw = deg2rad(car_yaw);

                    // If previous path is almost empty
                    if(prev_size < 2) {
                        // Use the points that make the path tangent to the car
                        double prev_car_x = car_x - cos(ref_yaw);
                        double prev_car_y = car_y - sin(ref_yaw);
                        
                        ptsx.push_back(prev_car_x);
                        ptsx.push_back(car_x);
                        
                        ptsy.push_back(prev_car_y);
                        ptsy.push_back(car_y);
                    }
                    else {
                        // Use the previous path endpoint as the starting reference
                        // Redefine reference state as previous path endpoint
                        ref_x = previous_path_x[prev_size-1];
                        ref_y = previous_path_y[prev_size-1];
                        
                        double ref_x_prev = previous_path_x[prev_size-2];
                        double ref_y_prev = previous_path_y[prev_size-2];
                        
                        ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);
                        
                        // Use these points to make the path tangent to the previous path endpoint
                        ptsx.push_back(ref_x_prev);
                        ptsx.push_back(ref_x);
                        
                        ptsy.push_back(ref_y_prev);
                        ptsy.push_back(ref_y);
                    }
                    
                    // In Frenet add evenly space (30m) points ahead of the starting reference point
                    // Note 30m yields a lateral jerk when changing lanes of ~1g/s at the max velocity which
                    // which is an acceptable jerk.  If max speed were to increase then the spacing would
                    // also need to increase.
                    vector<double> next_wp0 = getXY(car_s+30, (float(old_lane)+.5)*LANE_WIDTH, map_waypoints_s, map_waypoints_x, map_waypoints_y);
                    vector<double> next_wp1 = getXY(car_s+60, (float(new_lane)+.5)*LANE_WIDTH, map_waypoints_s, map_waypoints_x, map_waypoints_y);
                    vector<double> next_wp2 = getXY(car_s+90, (float(new_lane)+.5)*LANE_WIDTH, map_waypoints_s, map_waypoints_x, map_waypoints_y);
                    
                    // Push these waypoits to the ptsx and ptsy vector
                    ptsx.push_back(next_wp0[0]);
                    ptsx.push_back(next_wp1[0]);
                    ptsx.push_back(next_wp2[0]);
                    
                    ptsy.push_back(next_wp0[1]);
                    ptsy.push_back(next_wp1[1]);
                    ptsy.push_back(next_wp2[1]);
                    
                    // Shift ptsx and ptxy into car reference frame
                    for(int i=0; i<ptsx.size(); i++) {
                        double shift_x = ptsx[i] - ref_x;
                        double shift_y = ptsy[i] - ref_y;
                        
                        ptsx[i] = (shift_x*cos(0-ref_yaw) - shift_y*sin(0-ref_yaw));
                        ptsy[i] = (shift_x*sin(0-ref_yaw) + shift_y*cos(0-ref_yaw));
                    }
                    
                    // create spline
                    tk::spline s;
                    
                    // set (x,y) points for spline
                    s.set_points(ptsx, ptsy);
                    
                    // start with all the path points from the previous time
                    for(int i=0; i<prev_size; i++) {
                        next_x_vals.push_back(previous_path_x[i]);
                        next_y_vals.push_back(previous_path_y[i]);
                    }
                    
                    // calculate how the break up the spline points so that the
                    // speed is set to the reference velocity
                    double target_x = 30.;
                    double target_y = s(target_x);
                    double target_dist = sqrt((target_x*target_x) + (target_y*target_y));
                    
                    double x_add_on = 0;
                    
                    float acceleration = next_ego_state.a/time_steps;
                    
                    // fill in the rest of the path planner. Note always fill path plannter to 50 points
                    for(int i=prev_size; i<time_steps; i++) {
                        
                        ref_vel = max(min(ref_vel, MAX_SPEED_LIMIT),MIN_SPEED_LIMIT);
                        
                        double N = (target_dist/(dt*ref_vel));
                        double x_point = x_add_on + target_x/N;
                        double y_point = s(x_point);
                        
                        x_add_on = x_point;
                        
                        // rotate back to global coordinates
                        double x_ref = x_point;
                        double y_ref = y_point;
                        x_point = (x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw));
                        y_point = (x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw));
                        
                        x_point += ref_x;
                        y_point += ref_y;
                        
                        next_x_vals.push_back(x_point);
                        next_y_vals.push_back(y_point);
                        
                        ref_vel += acceleration;    // Increase speed
                    }

                    msgJson["next_x"] = next_x_vals;
                    msgJson["next_y"] = next_y_vals;
                    
                    auto msg = "42[\"control\","+ msgJson.dump()+"]";
                    
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                    
                    // Update the ego state to the new state
                    fsm.next_ego_state(next_ego_state);
                    
                    // this_thread::sleep_for(chrono::milliseconds(100));
                    
                }
            } else {
                // Manual driving
                std::string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
        }
    });
    
    // We don't need this since we're not using HTTP but if it's removed the
    // program
    // doesn't compile :-(
    h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                       size_t, size_t) {
        const std::string s = "<h1>Hello world!</h1>";
        if (req.getUrl().valueLength == 1) {
            res->end(s.data(), s.length());
        } else {
            // i guess this should be done more gracefully?
            res->end(nullptr, 0);
        }
    });
    
    h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
        std::cout << "Connected!!!" << std::endl;
    });
    
    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                           char *message, size_t length) {
        ws.close();
        std::cout << "Disconnected" << std::endl;
    });
    
    int port = 4567;
    if (h.listen(port)) {
        std::cout << "Listening to port " << port << std::endl;
    } else {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    }
    h.run();
}
