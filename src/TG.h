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
#include "PID.h"

#include <cmath>
#include <algorithm>



using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

/*
/The map data is stored here
*/
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

/*
/UNUSED The localization of our own vehicle
/not implemented yet
*/

struct Localization
{
    vector<double> x;
    vector<double> y;
    vector<double> s;
    vector<double> d;
    vector<double> yaw;
    vector<double> speed;
    vector<double> end_path_s;
    vector<double> end_path_d;
    Localization();
    virtual ~Localization();
    
    void update();
};

/*
The trajectory generator 
*/
class TG 
{
    public:
        /*
        Simulator data
        */

        double car_x;
        double car_y;
        double car_s;
        double car_d;
        double car_yaw;
        double car_speed;        
        double end_path_s;
        double end_path_d;
        int prev_size;
        int lane = 1; 

        /*
        Reference velocity
        */
        double ref_vel;
        //int lane;
        enum Sf {id, x, y, vx, vy, s, d};
        
        /*
        the route to drive
        */
        Route route; 
        /*
        Localization, not yet used
        */
        Localization localization;   
        /*
        PD controller for the setting of the car's speed
        */
        PID pid;          
    
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
        
        /*
        Alternative path generator based on polynomial fit. Not used currently
        */
        pair <vector<double>, vector<double>> getTrajectory(string s);
        vector<double> JMT(vector< double> start, vector <double> end, double T);
        
        template<typename I>
        pair<double, double> getClosestCarAhead(I carlist, int prev_size, double car_s, double range, int lane);
        
        /*
        Sets the speed based on the distance and speed of the closest car ahead, utilizing a PD controller
        */
        void setTargetSpeed(double& actual_speed, double target_speed, double distance);
        
        /* 
        Returns the lane with the lowest cost value which is used in the path generator
        */
        template<typename I>
        int getBestLane(I carlist, int prev_size, double car_s, int currentLane, double ourSpeed);
        
        /*
        Check is the line is occupied in the specified lane section,
        based on the provided traffic information in carlist
        */
        template<typename I>
        bool laneIsFreeAt(int ln, double our_s, I carlist, double from_delta_s, double to_delta_s, int prev_size);

};

#endif /* TG_H */