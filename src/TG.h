#ifndef TG_H
#define TG_H

#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/LU"
#include "json.hpp"
#include "spline.h"

#include "Aux.h"


#include <cmath>
#include <algorithm>



using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

struct Route 
{
    vector<double> waypoints_x;
    vector<double> waypoints_y;
    vector<double> waypoints_s;
    vector<double> waypoints_dx;
    vector<double> waypoints_dy;

    Route();
    virtual ~Route();
    
    void loadMap();
};

class TG 
{
    public:
        double car_x;
        double car_y;
        double car_s;
        double car_d;
        double car_yaw;
        double car_speed;
        
        double end_path_s;
        double end_path_d;
        int prev_size;

        double ref_vel;
        //int lane;
        enum Sf {id, x, y, vx, vy, s, d};
        
        Route route;      

        
    
        /*
        Constructor
        */
        TG();

        /*    
        /Destructor
        */
        virtual ~TG();

        /*
        Calculate the Jerk Minimizing Trajectory that connects the initial state
        to the final state in time T.
        */
    string hasData(string s);
    vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y);
    
    pair <vector<double>, vector<double>> getTrajectory(string s);
    vector<double> JMT(vector< double> start, vector <double> end, double T);
    template<typename I>
    pair<double, double> minFinder(I carlist, int prev_size, double car_s, double range, int lane);
    void setTargetSpeed(double& actual_speed, double target_speed, double distance);
    template<typename I>
    int bestLane(I carlist, int prev_size, double car_s);


};


#endif /* TG_H */