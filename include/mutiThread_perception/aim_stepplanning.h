#ifndef _AIM_STEPPLANNING_H_
#define _AIM_STEPPLANNING_H_
#include <iostream>
#include <math.h>
#include <Eigen/Core>
#include <vector>
#include <ros/ros.h>
#include "type.h"
using namespace std;


class aim_stepplanning
{
private:
    Eigen::Vector2d goal;
    Eigen::Vector2d direct;
    double fit_length;
    double theta;
    double go_length;
    vector<footstep> steps;
public:
    aim_stepplanning(Eigen::Vector2d goal_, Eigen::Vector2d direct_, double fit_length_);
    void go(/* std::ofstream & fs */);
    vector<footstep> inline getResult()
    {
        return steps;
    }
    ~aim_stepplanning();
};



#endif