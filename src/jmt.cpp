//
//  jmt.cpp
//  path_planning
//
//  Created by Scott Clegg on 2/4/18.
//
#include <iostream>
#include <vector>
#include "jmt.h"

/*
 * Initializes Jerk Minimization Trajectory
 */

JMT::JMT(){}

JMT::~JMT() {}

void JMT::solve(vector<double> start, vector<double> end, double t) {
    /*
     * Solve for the coefficients of the quint polynominal
     */
    
    double t2 = t*t;
    double t3 = t2*t;
    double t4 = t3*t;
    double t5 = t4*t;
    
    double c_0 = start[0]+start[1]*t+.5*start[2]*t*t;
    double c_1 = start[1]+start[2]*t;
    double c_2 = start[2];
    

    Eigen::MatrixXd A(3,3);
    A <<  t3,    t4,    t5,
        3*t2,  4*t3,  5*t4,
         6*t, 12*t2, 20*t3;
    
    Eigen::VectorXd B(3);
    B << end[0] - c_0,
         end[1] - c_1,
         end[2] - c_2;
    
    Eigen::VectorXd X(3);
    
    X = A.inverse()*B;
    
    Eigen::VectorXd coeffs(6);
    coeffs << start[0], start[1], .5*start[2], X[0], X[1], X[2];
    
    this->coeffs = coeffs;
}

vector<double> JMT::trajectory(double dt, double time_horizon) {
    /*
     * Calculate the trajectory for the coefficients
     */
    vector<double> trajectory;
    Eigen::VectorXd coeffs = this->coeffs;
    
    double t = dt;
    while(t <= time_horizon) {
        double t2 = t*t;
        double t3 = t2*t;
        double t4 = t3*t;
        double t5 = t4*t;
        
        Eigen::VectorXd time(6);
        time << 1.0, t, t2, t3, t4, t5;
        
        double position = time.transpose()*coeffs;
        trajectory.push_back(position);
        t += dt;
    }
    return trajectory;
}
