#pragma once
#include <iostream>
#include <vector>
#include <random>
#include <chrono>
#include "Kalman.h"

using namespace std;
void atti_cal_cq_modi(vector<double> &posiN, vector<double> &attiN, vector<double> &veloN, vector<vector<double>> &Wibb, vector<vector<double>> &WnbbA_old, double T);
void velo_cal(double T, vector<double> &attiN, vector<double> &posiN, vector<double> &veloN, vector<vector<double>> &Fb);
void posi_cal(double T, vector<double> &veloN, vector<double> &posiN);
void imu_err_random(double t, double T, vector<double> &Gyro_b, vector<double> &Gyro_r, vector<double> &Gyro_wg, vector<double> &Acc_r, vector<vector<double>> &Fb, vector<double> &Wibb);

void posi_compensation(int64_t &time_now, int64_t &time_last, vector<double> &veloN, vector<double> &posiT, double T);