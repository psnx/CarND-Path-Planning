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
    static constexpr double pi() { return M_PI; } 
    static double deg2rad(double x) { return x * pi() / 180; }
    static double rad2deg(double x) { return x * 180 / pi(); }   
    template <typename T>
    static T clip(const T& n, const T& lower, const T& upper) {
      return std::max(lower, std::min(n, upper));
    }
    

};


#endif /* Aux_H */