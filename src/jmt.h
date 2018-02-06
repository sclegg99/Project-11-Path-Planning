//
//  jmt.hpp
//  path_planning
//
//  Created by Scott Clegg on 2/4/18.
//

#ifndef jmt_h
#define jmt_h

#include <iostream>
#include <vector>

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Dense"

using namespace std;

class JMT {
    
public:
    Eigen::VectorXd coeffs;
    
    /*
     * Constructors
     */
    JMT();
    
    /*
     * Destructor
     */
    virtual ~JMT();
    
    void solve(vector<double> start, vector<double> end, double t);
    
    vector<double> trajectory(double dt, double time_horizon);
};

#endif /* jmt_h */
