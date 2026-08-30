#pragma once
#include <iostream>
#include <vector>
#include <cmath>
#include <string>
#include <fstream>
#include <numeric>
#include <chrono>
#include <random>
#include "Mat_Math.h"
#include "Kalman.h"
#pragma pack(1)
using namespace std;

#define M 10
#define N_ 3 //会和QT中模板参数名N重复 所以改成了N_

// #ifndef _GLV_H
// #define _GLV_H
// const double deg_rad = 0.01745329252e0; // pai/180  Transfer from angle degree to rad
// const double g = 9.7903698;             // 9.7803698  重力加速度    （单位：米/秒/秒）
// const double Wie = 7.292115147e-5;      // 地球自转角速度（弧度/秒）
// const double C = 299792458;             // 光速常数
// const double Re = 6378137.0;            //    地球半径（米）
// const double Rp = 6356752;
// const double f = 1 / 298.257; // 地球的椭圆率
// const double pi = 3.1415926;
// const double ug = g * 1e-6; // ug
// #endif                      // !_GLV_H

struct Heigh_data
{
    float BA;    // 气压高度
    float GPS_H; // 卫星高度
    float RA;    // 无线电高度
    float INS_lon;
    float INS_lat;
    float INS_heig;
    float INS_ve;
    float INS_vn;
    float INS_vu;
    float INS_head;
    float INS_roll;
    float INS_pitch;
    float realtime_pressure;
    float realtime_temp;
    float GPS_vu;
    float INS_TERCOM_lat;
    float GPS_TERCOM_lon;
    int Is_GPS_valid;
};

class Height_fusion
{
public:
    Height_fusion()
    {
        this->BA = 0;
        this->GPS_H = 0;
        this->RA = 0;
        this->INS_lon = 0;
        this->INS_lat = 0;
        this->INS_heig = 0;
        this->INS_head = 0;
        this->INS_roll = 0;
        this->INS_pitch = 0;
        this->INS_ve = 0;
        this->INS_vn = 0;
        this->GPS_vu = 0;
        this->realtime_pressure = 0;
        this->realtime_temp = 0;
        this->Is_GPS_valid = 0;
        this->GPS_TERCOM_lon = 0;
        this->GPS_TERCOM_lon = 0;
        this->GPS_vu = 0;
        this->ba_size = 2 * M + 1;
        this->gps_size = 2 * M + 1;
        this->ra_size = 2 * M + 1;
        this->ba_filter_flag = 0;
        this->gps_filter_flag = 0;
        this->ra_filter_flag = 0;
        this->abnormal_filename = "integrated_navigation/Global_abnormal_elevation_value_database.bin";
        this->abnormal_flag = 1;
        this->heig_filename = "integrated_navigation/a.bin";
        this->altitude_withgps = 0;
        this->abs_altitude_withgps = 0;
        this->altitude_withoutgps = 0;
        this->abs_altitude_withoutgps = 0;
    }
    void map_data_load(float lon, float lat);
    vector<float> GA_Filter(int M_ba, int M_gps, int M_ra, double T, double vu);
    vector<float> ToAltitude(vector<float> &heig, float lon, float lat, vector<float> &map_heig_info, vector<int16_t> &heig_map_data, vector<float> &abnrmal_heig_data);
    void Height_sequence();
    void heig_kalman_init(vector<vector<double>> &Xc, vector<vector<double>> &PK, vector<double> &Xerr);
    void heig_kalman(float T_D, vector<float> &multi_heig, vector<float> &posiN, vector<float> &veloN, float &veloGu, vector<vector<double>> &Xc, vector<vector<double>> &Xc_gps, vector<vector<double>> &PK, vector<double> &Xerr, int kflag);
    // void data_recv();
    void cal_altitude(vector<float> &posiN, vector<float> &veloN, vector<vector<double>> &Xc, vector<vector<double>> &Xc_gps, int kflag);
    void cal_abs_altitude(vector<float> &map_heig_info, vector<float> &heig_filter, vector<vector<double>> &PK);

    float read_topographic_heig(float lon, float lat, vector<int16_t> &heig_map_data);

    vector<float> heig_simu(vector<float> &posi, vector<int16_t> &heig_map_data, vector<float> &abnrmal_heig_data, float &veloBu);
    vector<int16_t> heig_map_data;
    vector<float> abnrmal_heig_data;

    // private:
    float BA;    // 气压高度
    float GPS_H; // 卫星高度
    float RA;    // 无线电高度
    float INS_lon;
    float INS_lat;
    float INS_heig;
    float INS_ve;
    float INS_vn;
    float INS_vu;
    float INS_head;
    float INS_roll;
    float INS_pitch;
    float realtime_pressure;
    float realtime_temp;
    float GPS_vu;
    float INS_TERCOM_lat;
    float GPS_TERCOM_lon;
    int Is_GPS_valid;

    float altitude_withgps;
    float abs_altitude_withgps;
    float altitude_withoutgps;
    float abs_altitude_withoutgps;

    // float time;
    // vector<int16_t> heig_map_data;
    // vector<float> abnrmal_heig_data;
    vector<float> BA_sequence;
    int ba_size;
    vector<float> GPS_sequence;
    int gps_size;
    vector<float> RA_sequence;
    int ra_size;
    int ba_filter_flag;
    int gps_filter_flag;
    int ra_filter_flag;
    string abnormal_filename;
    int abnormal_flag;
    string heig_filename;
};