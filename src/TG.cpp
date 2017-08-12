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

#include <fstream>
#include <cmath>
#include <vector>

#include "TG.h"
#include "Aux.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

Route::Route(){}
Route::~Route(){}

void Route::loadMap()
{
    // Waypoint map to read from
    string map_file_ = "../data/highway_map.csv";
    // The max s value before wrapping around the track back to 0
    double max_s = 6945.554;

    ifstream in_map_(map_file_.c_str(), ifstream::in);

    string line;
    while (getline(in_map_, line)) 
    {
        istringstream iss(line);
        double x;
        double y;
        float s;
        float d_x;
        float d_y;
        iss >> x;
        iss >> y;
        iss >> s;
        iss >> d_x;
        iss >> d_y;
        waypoints_x.push_back(x);
        waypoints_y.push_back(y);
        waypoints_s.push_back(s);
        waypoints_dx.push_back(d_x);
        waypoints_dy.push_back(d_y);
    }
    
}

/******************************************|
|***************Trajectory Generator*******|
|******************************************/

TG::TG() 
{           
    route.loadMap();
}
TG::~TG() {}

vector<double> TG::getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
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

    double perp_heading = heading-Aux::pi()/2;

    double x = seg_x + d*cos(perp_heading);
    double y = seg_y + d*sin(perp_heading);

    return {x,y};
}
pair <vector<double>, vector<double>> TG::getTrajectory(string s)
{
    auto j = nlohmann::json::parse(s);    
    string event = j[0].get<string>();
    if (event == "telemetry") 
    {
        // j[1] is the data JSON object
        // Main car's localization Data
        car_x = j[1]["x"];
        car_y = j[1]["y"];
        car_s = j[1]["s"];
        car_d = j[1]["d"];
        car_yaw = j[1]["yaw"];
        car_speed = j[1]["speed"];
        end_path_s = j[1]["end_path_s"];
        end_path_d = j[1]["end_path_d"];
        // Previous path data given to the Planner
        auto previous_path_x = j[1]["previous_path_x"];
        auto previous_path_y = j[1]["previous_path_y"];
        // Previous path's end s and d values 
        
        // Sensor Fusion Data, a list of all other cars on the same side of the road.
        auto sensor_fusion = j[1]["sensor_fusion"];            

        vector<double> next_x_vals;
        vector<double> next_y_vals;

        // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
        double dist_inc = 0.5;
        for(int i = 0; i < 50; i++)
        {
            double next_s = car_s + (i+1)*dist_inc; 
            double next_d = 6;
            vector<double> xy = getXY(next_s, next_d, route.waypoints_s, route.waypoints_x, route.waypoints_y);

            next_x_vals.push_back(xy[0]);
            next_y_vals.push_back(xy[1]);
        }
        pair <vector<double>, vector<double>> points;
        points = make_pair(next_x_vals, next_y_vals);
        return points;
    }    
}

vector<double> TG::JMT(vector< double> start, vector <double> end, double T)
{
    /*
    Calculate the Jerk Minimizing Trajectory that connects the initial state
    to the final state in time T.

    INPUTS

    start - the vehicles start location given as a length three array
        corresponding to initial values of [s, s_dot, s_double_dot]

    end   - the desired end state for vehicle. Like "start" this is a
        length three array.

    T     - The duration, in seconds, over which this maneuver should occur.

    OUTPUT 
    an array of length 6, each value corresponding to a coefficent in the polynomial 
    s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5

    EXAMPLE

    > JMT( [0, 10, 0], [10, 10, 0], 1)
    [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
    */
    
    //Example code CODE
    MatrixXd A = MatrixXd(3, 3);
	A << T*T*T, T*T*T*T, T*T*T*T*T,
			    3*T*T, 4*T*T*T,5*T*T*T*T,
			    6*T, 12*T*T, 20*T*T*T;
		
	MatrixXd B = MatrixXd(3,1);	    
	B << end[0]-(start[0]+start[1]*T+.5*start[2]*T*T),
			    end[1]-(start[1]+start[2]*T),
			    end[2]-start[2];
			    
	MatrixXd Ai = A.inverse();
	
	MatrixXd C = Ai*B;
	
	vector <double> result = {start[0], start[1], .5*start[2]};
	for(int i = 0; i < C.size(); i++)
	{
	    result.push_back(C.data()[i]);
	}    
	
    return result;
    
    
    /*
    vector <double> a;
    a.push_back(1.0);
    a.push_back(2.0);
  
    return a;
    */
    
}