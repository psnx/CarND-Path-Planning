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
#include <algorithm>

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

/*********************************************************|
|***************Car Localization &sensor fusion***********|
|*********************************************************/
Localization::Localization(){}
Localization::~Localization(){}


/******************************************|
|***************Trajectory Generator*******|
|******************************************/


TG::TG() 
{           
    route.loadMap();
    cout<<"\nMap loaded\n";

    //initialize a simple PID controller from Term2
    double init_Kp = -0.5;
    double init_Kd = -0.8;
    double init_Ki = -0;
    

    pid.Init(init_Kp, init_Ki, init_Kd);

    
}
TG::~TG() 
{
    //initial velocity
    ref_vel = 0;
    // initial lane position
    lane = 1;       

}


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

template<typename I>
pair<double, double> TG::getClosestCarAhead(I carlist, int prev_size, double car_s, double range, int inLane)
{
    double minDistance = 100;
    double speed_of_closest = 100; 
    double check_car_s = car_s + 100;
    for (auto& car: carlist)
    {
        // for each detected car
        float d = car[TG::Sf::d];
        if (Aux::identifyLane(d) == inLane)      
        {
            double vx = car[TG::Sf::vx];
            double vy = car[TG::Sf::vy];
            double check_speed = sqrt(vx*vx + vy*vy);
            check_car_s = car[TG::Sf::s];
            // prev_size*0.02 tells us how long time since path planning start. 
            check_car_s += ((double)prev_size*0.02*check_speed);
            
            // check vehicles only ahead veh_s > than our s
            if((check_car_s > car_s) && (check_car_s - car_s < range))
            {
                double distance = check_car_s - car_s;
                if(distance < minDistance)
                {
                    minDistance = distance;
                    speed_of_closest = check_speed;        
                }
            }
        }   
    }
    return make_pair(minDistance, speed_of_closest);
}



template<typename I>
bool TG::laneIsFreeAt(int ln, double our_s, I carlist, double from_delta_s, double to_delta_s, int prev_size)
{
    bool isFree = true;
    for (auto c: carlist)
    {
        // for each detected car
        float d = c[TG::Sf::d];
        if (d < 2+4*ln+2 && d > 2+4*ln-2)
        {
            double vx = c[TG::Sf::vx];
            double vy = c[TG::Sf::vy];
            double c_speed = sqrt(vx*vx + vy*vy);
            double c_s = c[TG::Sf::s];
            
            // prev_size*0.02 tells us how long time since path planning start. 
            c_s += ((double)prev_size*0.02*c_speed);



            // check vehicles only ahead veh_s > than our s
            if((c_s > our_s - from_delta_s) && (c_s < our_s + to_delta_s))
            {
                isFree = false;
            }   
        }
    }
    return isFree;
}


void TG::setTargetSpeed(double& actual_speed, double target_speed, double distance)
{    
    target_speed = Aux::clip(target_speed, 0.0, 49.5);
    double delta_speed = target_speed - actual_speed;
    double target_tailing = 30;
    pid.UpdateError(distance - target_tailing);
    
    double e = pid.TotalError();
    
    cout << "Dist: " << distance << "\t Total error: " << e << "\n";
    actual_speed -= Aux::clip(0.05*e, -0.5, 0.5);
    cout << "Dist: " << distance << "\t Total error: " << e << " 0.05*e: " << Aux::clip(0.05*e, -0.5, 0.5) <<"\n";
    if (abs(delta_speed) < 0.2) { actual_speed = target_speed; }        
    actual_speed = Aux::clip(actual_speed, 0.0, 49.5);    
        
}

template<typename I>
int TG::getBestLane(I carlist, int prev_size, double car_s, int currentLane, double ourSpeed) 
{    
    double minCost = 10000;
    int targetLane = currentLane;
    //evaluate adjacent lanes for line change
    //iterates through all the lanes       
    
    for (int i = 2; i>=0; i--)
    {        
        double speed;
        double distance;        
        int cost = 0;        

        if (currentLane != i) //evaluation of any other lane than our own
        {
            //critical moves
            if (!laneIsFreeAt(i, car_s, carlist, 10, 20, prev_size)) {cost+=10000; }            
            if (ourSpeed < 15.0) {cost += 1000;} //avoid almost standstill lane change due to untracked incoming traffic in target lane
            if (abs(i - currentLane) > 1) { cost += 10000; } // only one lane change at a time
            
            //comfort             
            cost += 100; // no change without reason;
        }
        std::tie(distance, speed) = getClosestCarAhead(carlist, prev_size, car_s, 80.0, i);
        if (distance < 80) 
        { 
            cost += 30 * (50 - Aux::clip(speed, 0.0, 50.0)); // if the lane is occupied by a slow vehicle
            cost += 5 * (80-distance); //penalize "tailing"
        }   
        
        if (i != 1) {cost += 101;}
        
        if (cost < minCost) {
            targetLane = i; 
            minCost = cost;            
        }
        cout << "lane: "<< i << "\t cost: " << cost << "\n";
    } 
    cout<<"-------------->"<< targetLane << "------------------------\n";
    //targetLane = 1;

    return targetLane;
}

pair <vector<double>, vector<double>> TG::getTrajectory(string sensor_data)
{   
    auto j = nlohmann::json::parse(sensor_data);
    //auto j = nlohmann::json::parse(s)
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

    //cout << "data read, car x" << car_x << "\n";
    
    //initial reference values
    

    lane = Aux::identifyLane(end_path_d);    
    

    prev_size = previous_path_x.size();
    if (prev_size > 0)
    {
        car_s = end_path_s;
    }
    
    double target_speed;
    double distance;
    
    std::tie(distance, target_speed) = TG::getClosestCarAhead(sensor_fusion, prev_size, car_s, 40, lane);
    setTargetSpeed(ref_vel, target_speed, distance);
    lane = getBestLane(sensor_fusion, prev_size, car_s, lane, ref_vel);
    

    vector<double> ptsx;
    vector<double> ptsy;

    double ref_x = car_x;
    double ref_y = car_y;
    double ref_yaw = Aux::deg2rad(car_yaw);

    if (prev_size < 2)
    {
        double prev_car_x = car_x - cos(car_yaw);
        double prev_car_y = car_y - sin(car_yaw);

        ptsx.push_back(prev_car_x);
        ptsx.push_back(car_x);

        ptsy.push_back(prev_car_y);
        ptsy.push_back(car_y);
    } 
    else 
    {
        ref_x = previous_path_x[prev_size-1];
        ref_y = previous_path_y[prev_size-1];

        double ref_x_prev = previous_path_x[prev_size-2];
        double ref_y_prev = previous_path_y[prev_size-2];
        
        ptsx.push_back(ref_x_prev);
        ptsx.push_back(ref_x);

        ptsy.push_back(ref_y_prev);
        ptsy.push_back(ref_y);            
    }    
    

    vector<double> next_wp0 = getXY(car_s+40, 2+4*lane, route.waypoints_s, route.waypoints_x, route.waypoints_y);
    vector<double> next_wp1 = getXY(car_s+80, 2+4*lane, route.waypoints_s, route.waypoints_x, route.waypoints_y);
    vector<double> next_wp2 = getXY(car_s+100, 2+4*lane, route.waypoints_s, route.waypoints_x, route.waypoints_y);

    ptsx.push_back(next_wp0[0]);
    ptsx.push_back(next_wp1[0]);
    ptsx.push_back(next_wp2[0]);

    ptsy.push_back(next_wp0[1]);
    ptsy.push_back(next_wp1[1]);
    ptsy.push_back(next_wp2[1]);
    
    //transform to cars frame of refernce

    for (int i = 0; i<ptsx.size(); i++)
    {
        double shift_x = ptsx[i]-ref_x;
        double shift_y = ptsy[i]-ref_y;

        ptsx[i] = (shift_x*cos(0-ref_yaw) - shift_y*sin(0-ref_yaw));
        ptsy[i] = (shift_x*sin(0-ref_yaw) + shift_y*cos(0-ref_yaw));
    }       
    
    //cout << "waypoint transform: ptsx size: " << ptsx.size() << "\n";
    
    tk::spline s;
    s.set_points(ptsx, ptsy);        

    vector<double> next_x_vals;
    vector<double> next_y_vals;    

    // start with prev path points;
    for (int i=0; i<previous_path_x.size(); i++)
    {
        next_x_vals.push_back(previous_path_x[i]);
        next_y_vals.push_back(previous_path_y[i]);
    }

    double target_x = 30.0;
    double target_y = s(target_x);
    double target_dist = sqrt(target_x*target_x + target_y*target_y);

    double x_add_on = 0;
    
    
    for (int i = 1; i<=50-previous_path_x.size(); i++)
    {
        double N = (target_dist/(0.02*ref_vel/2.24));
        double x_point = x_add_on + (target_x)/N;
        double y_point = s(x_point);

        x_add_on = x_point;

        double x_ref = x_point;
        double y_ref = y_point;

        //rotate back

        x_point = x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw);
        y_point = x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw);

        x_point += ref_x;
        y_point += ref_y;

        next_x_vals.push_back(x_point);
        next_y_vals.push_back(y_point);       

    }
        
    pair <vector<double>, vector<double>> points;
    points = make_pair(next_x_vals, next_y_vals);
    return points;       
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