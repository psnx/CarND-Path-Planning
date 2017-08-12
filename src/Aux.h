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

};


#endif /* Aux_H */