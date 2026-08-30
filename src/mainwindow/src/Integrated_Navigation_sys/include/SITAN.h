#pragma once

#include "data_struct.h"
#include <eigen3/Eigen/Dense>
#include "Kalman.h"

class SITAN
{
public:
    int mode;         // 0:搜索模式；1：跟踪模式
    int liner_method; // 0:泰勒展开法 1：九点拟合法
    int result_state; // 定位状态 0:搜索模式 1：跟踪模式
    // TERCOM t;
    // vector<string>map_file_name;//存储地图数据名称，9个
    // 地图信息共用TERCOM载入地图
    double ins_lon;
    double ins_lat;
    double ins_heig;
    double ins_ve;
    double ins_vn;
    // 惯导指示位置
    double BA; // 气压高度
    double RA; // 无线电高度
    // TAN求得位置
    double tan_lon;
    double tan_lat;
    double tan_heig;
    double tan_lon_p;  // 匹配定位经度标准差
    double tan_lat_p;  // 匹配定位纬度标准差
    double tan_heig_p; // 匹配定位高度标准差

    double x0_s; /* SITAN算法搜索滤波器初始化状态量 */
    double p0_s; /* SITAN算法搜索滤波器初始化方差 */
    double swrs; /* SITAN算法搜索滤波器初始化残差 */

    double q0_s;    /* SITAN算法搜索滤波器量测噪声 */
    double r0_s;    /* SITAN算法搜索滤波器系统噪声 */
    double a_s;     /* SITAN算法搜索滤波器加权系数 */
    double p0_t[5]; /* SITAN算法跟踪滤波器状态方差 */
    double q0_t[5];
    double r0_t; /*SITAN算法踪滤波观测噪声方差 */
    double a_t;
    double swrt;
    double H1, H2, N1, N2;      // 搜索模式切换检验阈值1（定位准则1,2阈值）
    double minswrs, extswrs;    // swrs_min和swrs_max-swrs_max的检验阈值
    int max_n1, max_n2, max_n3; // n1,n2,n3计数阈值
    int tac_n1, tac_n2;         // 跟踪模式的切换逻辑
    double minswrs_t;           // 跟踪模式的切换逻辑
    double sigmax, sigmay;      // x和y的误差协方差阈值

    // double TAN_lon;
    // double TAN_lat;
    // double TAN_heig;

    vector<vector<double>> x;  /* SITAN滤波器的状态矩阵 */
    vector<vector<double>> p;  /* SITAN滤波器的状态协方差矩阵 */
    vector<vector<double>> sw; /* 滤波残差存储矩阵 (搜索模式用)*/
    // double sw_track;           // 滤波残差存储矩阵（跟踪模式用）
    double swrs_min;
    double swrs_mins;
    double swrs_max;
    int n1, n2, n3;                                               /* SITNA模式逻辑切换评价指标计数器 */
    int search_result, track_result; /* 搜索状态和跟踪状态标识 */ // 初始化均置为0；

    vector<vector<int16_t>> serch_map;
    double serch_map_center_lon;
    double serch_map_center_lat;
    int map_reso; // 地图分辨率
    int serch_radiu;
    double tt; // 时间间隔（离散）

    void Filter_parameter_init();
    void Serch_map_creat(TERCOM &T);
    void Filter_serch();
    void Filter_track(TERCOM &T);
    void Mode_track_judge();
    void Mode_serch_judge();
    void Method_sitan(TERCOM &T);
    void Initfilter();
    vector<double> Terrain_liner();
    void proctan(TERCOM &T);
};