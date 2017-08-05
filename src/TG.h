#ifndef TG_H
#define TG_H

#include <fstream>
#include <math.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"

#include <fstream>
#include <cmath>
#include <vector>


using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

class TG 
{
    public:

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
    vector<double> JMT(vector< double> start, vector <double> end, double T);

};


#endif /* TG_H */