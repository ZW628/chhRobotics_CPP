//
// Created by chh3213 on 2022/11/24.
//

#ifndef CHHROBOTICS_CPP_MYREFERENCEPATH_H
#define CHHROBOTICS_CPP_MYREFERENCEPATH_H

#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <Eigen/Dense>

using namespace std;

using namespace Eigen;

#define PI 3.1415926

struct refTraj
{
    // xref:参考轨迹的状态，x、y、yaw
    // dref:参考控制量, [v,delta]
    MatrixXd xref, dref;
    // 轨迹点索引,min_index
    int ind;
};

struct parameters
{
    int L;
    int NX, NU, T;
    double dt;
};

class MyReferencePath
{

public:
    MyReferencePath();

    vector<double> calcTrackError(vector<double> robot_state);

    double normalizeAngle(double angle);

    // 计算参考轨迹点，统一化变量数组，便于后面MPC优化使用.
    refTraj calc_ref_trajectory(vector<double> robot_state, parameters param, double dl = 1.0);

public:
    // refer_path包括4维：位置x, 位置y， 轨迹点的切线方向(yaw), 曲率k
    vector<vector<double>> refer_path;
    vector<double> refer_x, refer_y;
};

#endif // CHHROBOTICS_CPP_MYREFERENCEPATH_H
