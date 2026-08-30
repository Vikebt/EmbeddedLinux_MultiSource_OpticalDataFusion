#pragma once
#include <iostream>
#include <stack>
#include <mutex>
#include <vector>
#include <semaphore.h>
#include "TERCOM.h"
// #include "muti_thread.h"
#pragma pack(1)
struct data_recv
{
    // 时标信息
    float time_sec;

    // 惯导相关信息
    double INS_lon;
    double INS_lat;
    double INS_heig;
    // 惯导经度、维度、高度
    double INS_head;
    double INS_pitch;
    double INS_roll;
    // 惯导航向、俯仰、横滚
    double INS_Fb_x;
    double INS_Fb_y;
    double INS_Fb_z;
    // 惯导x、y、z加速度
    double INS_Wibb_x;
    double INS_Wibb_y;
    double INS_WIbb_z;
    // 惯导x、y、z角速率
    double INS_ve;
    double INS_vn;
    double INS_vu;
    // 惯导东、北、天速度

    // 气压高度相关信息
    float BA;
    float real_time_temp;
    float real_time_press;
    // 气压高度、实时温度信息、实时气压信息

    // 卫星相关信息
    int Is_GPS_valid;
    // 卫星有效性；1：有效，0：无效
    float GPS_lon;
    float GPS_lat;
    float GPS_heig;
    float GPS_ve;
    float GPS_vn;
    float GPS_vu;
    // 卫星经度、纬度、高度、东、北、天向速度信息

    // 无线电高度相关信息
    float RA;
    // 无线电高度信息
    int64_t real_time;
    // 仿真真值
    double real_lon;
    double real_lat;
    double real_heig;
    // 仿真真实经、纬、高
};

struct data_send
{
    // 时标信息
    float time_sec;

    // 地形匹配相关信息
    float TER_lon;
    float TER_lat;
    float TER_heig;
    // 地形匹配经度、纬度、高度
    int Is_TER_valid;
    // 地形匹配有效性；1：有效，0：无效，-1：未进行匹配
    float INS_TER_lon;
    float INS_TER_lat;
    // 惯导/地形匹配融合经度、纬度

    // 高度融合相关信息
    int Is_heig_fusion;
    // 1:进行高度融合，相关数值有效；0：未进行高度融合，相关数值为0
    float altitude;
    float geometric_height;
    // 输出海拔高度信息、几何高度信息
    float fusion_gps_altitude;
    float fusion_gps_geometric_height;
    // GPS有效融合海拔高度、几何高度
    float fusion_altitude;
    float fusion_geometric_height;
    // GPS失效融合海拔高度、几何高度
    float adjust_BA;
    float adjust_GPS_heig;
    float adjust_RA;
    // 修正气压高度、卫星高度、无线电高度

    // 内部惯导信息(导航信息融合输出)
    float iner_INS_lon;
    float iner_INS_lat;
    float iner_INS_heig;
    // 内部惯导经度、纬度、高度
    float iner_INS_head;
    float iner_INS_pitch;
    float iner_INS_roll;
    // 内部惯导航向、俯仰、横滚
    float iner_INS_ve;
    float iner_INS_vn;
    float iner_INS_vu;
    // 内部惯导东、北、天向速度

    // 景象匹配相关
    int Is_scene_match_valid;
    // 景象匹配有效性；1：有效；0：无效；-1：未进行匹配
    float scene_match_lon;
    float scene_match_lat;
    // 景象匹配经、纬位置信息
    float INS_scene_match_lon;
    float INS_scene_match_lat;
    // 惯导/景象匹配融合经度、纬度
    int64_t real_time;

    //SITAN相关
    int SITAN_result_state;
    float sitan_lon;
    float sitan_lat;
    float sitan_heig;
};

struct TERCOM_recv
{
    // float time_sec;
    int64_t real_time;

    double iner_INS_lon;
    double iner_INS_lat;
    double iner_INS_heig;

    double iner_INS_ve;
    double iner_INS_vn;

    float BA;

    float RA;
};

struct TERCOM_send
{
    // float time_sec;
    int64_t time_sec;
    float match_lon;
    float match_lat;
    float match_heig;
    int Is_match_valid;
};

struct time_stamp
{
    int year;
    int month;
    int day;
    int hour;
    int min;
    int s;
    int millis;
    // 年/月/日/时/分/秒/毫秒
};

struct SITAN_send
{
    float time_sec;
    float match_lon;
    float match_lat;
    float match_heig;
    int result_state;
};

#ifndef global_variable
#define global_variable
extern std::stack<data_recv> data_recv_pool;
extern std::stack<data_recv> data_recv_pool_sitan;
extern std::stack<data_send> data_send_pool;
extern std::stack<TERCOM_recv> TERCOM_recv_pool;
extern std::stack<TERCOM_send> TERCOM_send_pool;
extern std::stack<SITAN_send> SITAN_send_pool;
extern std::vector<double> initial_posi;
extern std::vector<double> initial_atti;
extern std::vector<double> initial_velo;
extern std::mutex mymutex;
extern sem_t tercom_recv_data_flag;
extern sem_t tercom_flag;
extern TERCOM t1;
extern TERCOM t2;
// extern SITAN s1;
#endif // !global_variable
