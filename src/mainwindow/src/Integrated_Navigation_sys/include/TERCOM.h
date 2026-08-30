#pragma once
#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <regex>
#include <cmath>
#include <numeric>
#include <random>
#include <chrono>
using namespace std;

// 地图信息处理
// class Map_process
// {
// public:
//     // Map_process();
//     // static double Map_message[Map_size][Map_size];
//     static double Map_start_lat;
//     static double Map_start_lon;
//     static int Map_resolution;
//     static int Map_size;
//     // Map_process():Map_start_lat(0.0),Map_start_lon(0.0),Map_resolution(0),Map_size(0){}
//     vector<vector<int>> Map;
//     // void Read_map(string &filename);
//     // void extractFileInFo(string& filename);
//     // void Creat_map(int row, int column, string& filename);
//     //vector<double> readMapmessage(double lat, double lon);
//     // void printmessage();
// };

// 地形轮廓匹配
struct Match_InFo
{
    double ins_lat;
    double ins_lon;
    double terrain_height;
    Match_InFo(double lat, double lon, double height) : ins_lat(lat), ins_lon(lon), terrain_height(height) {}
};

struct Map_database
{
    vector<int16_t> Map;
    Map_database(int row, int col) : Map(row * col, 0) {}
};

class TERCOM
{
public:
    // double time_info; // 时间信息
    int64_t time_info;
    // 标志位
    int match_flag;
    int start_flag;
    int serch_map_update_flag;
    int fusion_flag;
    // 数字地图边界及搜索区域信息
    int serch_radius;
    int max_match_size;
    double map_start_lon;
    double map_start_lat;
    int map_resolution;
    // 惯导经纬高
    double Ins_lon;
    double Ins_lat;
    double Ins_altitude;
    // 匹配所得经纬高
    double match_lat;
    double match_lon;
    double match_altitude;
    // 用于比较前后位置信息是否大于一个网格
    double old_lat;
    double old_lon;
    // 用于判断匹配信息可靠性参数
    double analysis_time;
    double analysis_lat;
    double analysis_lon;
    // 匹配所得行列数
    int match_row;
    int match_col;
    // 存储9格数字地图信息
    vector<Map_database> mapdata;
    // 9个地图信息文件命名
    vector<string> filename;
    // 匹配序列存储格式
    vector<Match_InFo> Match_sequence;
    // vector<vector<vector<double>>> Map_match;
    // 由匹配序列生成的连续搜索区域地图信息
    vector<vector<vector<int16_t>>> Map_match;
    // vector<double>fitness;
    TERCOM() : match_flag(0), start_flag(0), serch_map_update_flag(0), fusion_flag(0), match_lat(0.0), match_lon(0.0), match_altitude(0.0), old_lat(0.0), old_lon(0.0) {}
    void Set_sequence(double lat, double lon, double height, int sequence_size);
    // void Set_serch_radius(Map_process &Map, double radius);
    // void Tercom_realization(Map_process &Map, int serch_mode);
    vector<int> APSO(int n, int D, int GEN);
    vector<int> Ergodic();
    vector<double> cal_fitness(int n, vector<vector<int>> posi);
    void Tercom_realization(int serch_mode);
    void Serch_map(const vector<int16_t> &Map, vector<Map_database> &Mapdata);
    void Serch_map_generator(int &count, const vector<int16_t> &Map, double &map_start_lat, double &map_start_lon, int &map_resolution); // 判断惯导指示位置是否在搜索区域内并生成相应的搜索地图
    void Serch_boundary_map_generator(int &count, vector<Map_database> &Map, double &lon_temp_left, double &lon_temp_right, double &lat_temp_up, double &lat_temp_down, double &map_start_lat, double &map_start_lon, int &map_resolution);
    void map_read();
    void var_init(int map_reso, int map_cell, double radius, double ins_lat_start, double ins_lon_start, double ins_altitude_start, int max_size);
    void Tercom_evaluation();
    double Match_analysis(int match_lon, int match_lat);
    void Serch_map_generator(const vector<int16_t> &Map, int &row_up, int &row_down, int &col_left, int &col_right);
    void Serch_boundary_map_generator(vector<Map_database> &Map, int &row_up, int &row_down, int &col_left, int &col_right);
};

void map_generator(vector<int16_t> &map, const string &filename);
void map_generator_9(vector<Map_database> &Map, int map_count, vector<string> &filename);

double height_simu(vector<double> &posi, vector<Map_database> &mapdata, int reso);