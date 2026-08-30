#pragma once
#include "data_struct.h"
#include <iostream>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <cstdint>
#include <unistd.h>
// #include <thread>
#include <cstring>
#include <fstream>
#include <string>
#include <chrono>
#include <ctime>
// #include <mutex>
#include "Kalman.h"
#include "INS_update.h"
#include "Height_fusion.h"
#include "TERCOM.h"
#include "SITAN.h"
#include <atomic>


void recv_thread();
void fusion_thread();
void TERCOM_thread();
void send_thread();
int64_t get_time_stamp();
void SITAN_thread();
// vector<double> posi_init();

extern std::atomic<bool> stop_thread;

//地形匹配的经纬高
extern float Res_TER_lon;
extern float Res_TER_lat;
extern float Res_TER_heig;

//高度融合海拔结果
extern float Res_altitude;

