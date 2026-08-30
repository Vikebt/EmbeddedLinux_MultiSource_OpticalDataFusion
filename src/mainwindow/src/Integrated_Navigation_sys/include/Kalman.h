#pragma once
#include <iostream>
#include <vector>
#include <cmath>
#include "Mat_Math.h"
using namespace std;
#ifndef _GLV_H
#define _GLV_H
const double deg_rad = 0.01745329252e0; // pai/180  Transfer from angle degree to rad
const double g = 9.7903698;             // 9.7803698  重力加速度    （单位：米/秒/秒）
const double Wie = 7.292115147e-5;      // 地球自转角速度（弧度/秒）
const double C = 299792458;             // 光速常数
const double Re = 6378137.0;            //    地球半径（米）
const double Rp = 6356752;
const double f = 1 / 298.257; // 地球的椭圆率
const double pi = 3.1415926;
const double ug = g * 1e-6; // ug
#endif                      // !_GLV_H

void Kalman_init(vector<double> &posiN, vector<vector<double>> &Xc, vector<vector<double>> &PK, vector<double> &Xerr);
void Kalman_TERCOM(vector<double> &posiN, vector<double> &posiT, vector<double> &attiN, vector<vector<double>> &Fb, vector<double> &veloN, vector<vector<double>> &PK, vector<vector<double>> &Xc, vector<double> &Xerr, double T_D, int Kalman_flag);
void Kalman_TERCOM_modi(vector<double> &attiN, vector<double> &veloN, vector<double> &posiN, vector<vector<double>> &Xc);