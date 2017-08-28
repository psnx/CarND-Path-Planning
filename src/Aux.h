#ifndef Aux_H
#define Aux_H

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



#include <cmath>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

class Aux 
{
    public:

    /*
    Constructor
    */
    Aux();

    /*    
    /Destructor
    */
    virtual ~Aux();

    /*
    Parser
    */
    static string hasData(string s);
    
    /*
    Math
    */
    static constexpr double pi() { return M_PI; } 
    static double deg2rad(double x) { return x * pi() / 180; }
    static double rad2deg(double x) { return x * 180 / pi(); }       
    template <typename T>
    static T clip(const T& n, const T& lower, const T& upper) {
      return std::max(lower, std::min(n, upper));
    }
    static double distance(double x1, double y1, double x2, double y2)
    {
      return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
    }    
    
    /*
    Frenet coordinates (d,s) from XY
    */
    static vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y);
    /*
    Not used way-point functions
    */    
    static int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y);
    static int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y);
    
    /*
    Returns the 0 based lane number from d (ref. Frenet coordinates)
    */
    static int identifyLane(double d);
};
#endif /* Aux_H */