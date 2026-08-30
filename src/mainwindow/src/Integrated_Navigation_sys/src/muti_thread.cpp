#include <muti_thread.h>
#include <thread>      
#include <chrono> 

using namespace std;
stack<data_recv> data_recv_pool;
stack<data_recv> data_recv_pool_sitan;
stack<data_send> data_send_pool;
stack<TERCOM_recv> TERCOM_recv_pool;
stack<TERCOM_send> TERCOM_send_pool;
stack<SITAN_send> SITAN_send_pool;
vector<double> initial_posi(3, 1.0);
vector<double> initial_atti(3, 1.0);
vector<double> initial_velo(3, 1.0);
mutex mymutex;
sem_t tercom_recv_data_flag;
sem_t tercom_flag;
TERCOM t1;
TERCOM t2;

std::atomic<bool> stop_thread{false}; 

float Res_TER_lon = 0;
float Res_TER_lat = 0;
float Res_TER_heig = 0;
float Res_altitude = 0;

int64_t get_time_stamp()
{
    auto now = std::chrono::system_clock::now();
    std::chrono::milliseconds ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch());

    // std::time_t time = std::chrono::system_clock::to_time_t(now);
    // std::time_t time = static_cast<std::time_t>(ms.count() / 1000);
    // std::tm *local_time = std::localtime(&time);

    // t_s.year = static_cast<int>(1900 + local_time->tm_year);
    // t_s.month = static_cast<int>(1 + local_time->tm_mon);
    // t_s.day = static_cast<int>(local_time->tm_mday);
    // t_s.hour = static_cast<int>(local_time->tm_hour);
    // t_s.min = static_cast<int>(local_time->tm_min);
    // t_s.s = static_cast<int>(local_time->tm_sec);
    // t_s.millis = static_cast<int>(ms.count() % 1000);
    return ms.count();
}

void recv_thread()
{
    struct sockaddr_in Server_addr, Client_addr;
    socklen_t Client_addr_size = sizeof(Client_addr);
    int socketfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (socketfd < 0)
    {
        cout << "recv_thread socket error!" << endl;
    }
    else
    {
        cout << "recv_thread socket sucess!" << endl;
    }
    memset(&Server_addr, 0, sizeof(Server_addr));
    Server_addr.sin_family = AF_INET;
    // Server_addr.sin_addr.s_addr = inet_addr("192.168.1.102");
    Server_addr.sin_addr.s_addr = INADDR_ANY;
    Server_addr.sin_port = htons(9998);
    if (bind(socketfd, (struct sockaddr *)&Server_addr, sizeof(Server_addr)) < 0)
    {
        close(socketfd);
        cout << "recv_thread bind error!" << endl;
    }
    int recvnum = 0;
    // char recvbuf[1024] = "";
    double recvbuf[28]; ///
    data_recv recv;
    int stack_size = 1;
    while (!stop_thread)
    {
        //
        recvnum = recvfrom(socketfd, recvbuf, sizeof(recvbuf), 0, (struct sockaddr *)&Client_addr, &Client_addr_size);
        // recvnum = recvfrom(socketfd, recvbuf, sizeof(recvbuf), 0, nullptr, nullptr);
        if (recvnum > 0)
        {
            recv.time_sec = static_cast<float>(recvbuf[0]);
            recv.INS_lon = recvbuf[1];
            recv.INS_lat = recvbuf[2];
            recv.INS_heig = recvbuf[3];
            recv.INS_head = recvbuf[4];
            recv.INS_pitch = recvbuf[5];
            recv.INS_roll = recvbuf[6];
            recv.INS_Fb_x = recvbuf[7];
            recv.INS_Fb_y = recvbuf[8];
            recv.INS_Fb_z = recvbuf[9];
            recv.INS_Wibb_x = recvbuf[10];
            recv.INS_Wibb_y = recvbuf[11];
            recv.INS_WIbb_z = recvbuf[12];
            recv.INS_ve = recvbuf[13];
            recv.INS_vn = recvbuf[14];
            recv.INS_vu = recvbuf[15];
            recv.BA = static_cast<float>(recvbuf[16]);
            recv.real_time_temp = static_cast<float>(recvbuf[17]);
            recv.real_time_press = static_cast<float>(recvbuf[18]);
            recv.Is_GPS_valid = static_cast<int>(recvbuf[19]);
            recv.GPS_lon = static_cast<float>(recvbuf[20]);
            recv.GPS_lat = static_cast<float>(recvbuf[21]);
            recv.GPS_heig = static_cast<float>(recvbuf[22]);
            recv.GPS_ve = static_cast<float>(recvbuf[23]);
            recv.GPS_vn = static_cast<float>(recvbuf[24]);
            recv.GPS_vu = static_cast<float>(recvbuf[25]);
            recv.RA = static_cast<float>(recvbuf[26]);
            recv.real_time = static_cast<int64_t>(recvbuf[27]);
            // recv.real_lon = recvbuf[28];
            // recv.real_lat = recvbuf[29];
            // recv.real_heig = recvbuf[30];
            // cout<<recv.RA<<endl;
            // cout<<recvbuf[26]<<endl;
            // char *pt = recvbuf;
            // memcpy(&recv.time_sec, pt, sizeof(float));
            // pt += sizeof(float);
            // memcpy(&recv.INS_lon, pt, sizeof(double));
            // pt += sizeof(double);
            // memcpy(&recv.INS_lat, pt, sizeof(double));
            // pt += sizeof(double);
            // memcpy(&recv.INS_heig, pt, sizeof(double));
            // pt += sizeof(double);
            // memcpy(&recv.INS_head, pt, sizeof(double));
            // pt += sizeof(double);
            // memcpy(&recv.INS_pitch, pt, sizeof(double));
            // pt += sizeof(double);
            // memcpy(&recv.INS_roll, pt, sizeof(double));
            // pt += sizeof(double);
            // memcpy(&recv.INS_Fb_x, pt, sizeof(double));
            // pt += sizeof(double);
            // memcpy(&recv.INS_Fb_y, pt, sizeof(double));
            // pt += sizeof(double);
            // memcpy(&recv.INS_Fb_z, pt, sizeof(double));
            // pt += sizeof(double);
            // memcpy(&recv.INS_Wibb_x, pt, sizeof(double));
            // pt += sizeof(double);
            // memcpy(&recv.INS_Wibb_y, pt, sizeof(double));
            // pt += sizeof(double);
            // memcpy(&recv.INS_WIbb_z, pt, sizeof(double));
            // pt += sizeof(double);
            // memcpy(&recv.INS_ve, pt, sizeof(double));
            // pt += sizeof(double);
            // memcpy(&recv.INS_vn, pt, sizeof(double));
            // pt += sizeof(double);
            // memcpy(&recv.INS_vu, pt, sizeof(double));
            // pt += sizeof(double);
            // memcpy(&recv.BA, pt, sizeof(float));
            // pt += sizeof(float);
            // memcpy(&recv.real_time_temp, pt, sizeof(float));
            // pt += sizeof(float);
            // memcpy(&recv.real_time_press, pt, sizeof(float));
            // pt += sizeof(float);
            // memcpy(&recv.Is_GPS_valid, pt, sizeof(int));
            // pt += sizeof(int);
            // memcpy(&recv.GPS_lon, pt, sizeof(float));
            // pt += sizeof(float);
            // memcpy(&recv.GPS_lat, pt, sizeof(float));
            // pt += sizeof(float);
            // memcpy(&recv.GPS_heig, pt, sizeof(float));
            // pt += sizeof(float);
            // memcpy(&recv.GPS_vu, pt, sizeof(float));
            // pt += sizeof(float);
            // memcpy(&recv.RA, pt, sizeof(float));
            // pt += sizeof(float);

            mymutex.lock();
            if (data_recv_pool.size() >= stack_size)
            {
                data_recv_pool.pop();
            }
            data_recv_pool.push(recv);
            if (data_recv_pool_sitan.size() >= stack_size)
            {
                data_recv_pool_sitan.pop();
            }
            data_recv_pool_sitan.push(recv);
            mymutex.unlock();
        }
        else if (recvnum == 0)
        {
            cout << "link close!" << endl;
        }
        else
        {
            cout << "link error!" << endl;
        }
    }
    close(socketfd);
}

void send_thread()
{
    struct sockaddr_in Client_addr;
    socklen_t Client_addr_size = sizeof(Client_addr);
    int socketfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (socketfd < 0)
    {
        cout << "send_thread socket error!" << endl;
    }
    else
    {
        cout << "send_thread socket sucess!" << endl;
    }
    // memset(&Server_addr, 0, sizeof(Server_addr));
    Client_addr.sin_family = AF_INET;
    // Client_addr.sin_addr.s_addr = inet_addr("192.168.1.112");
    Client_addr.sin_addr.s_addr = INADDR_ANY;
    Client_addr.sin_port = htons(9999);
    // if (bind(socketfd, (struct sockaddr *)&Server_addr, sizeof(Server_addr)) < 0)
    // {
    //     close(socketfd);
    //     cout << "send_thread bind error!" << endl;
    // }
    int sendnum = 0;
    // char sendbuf[1024] = "";
    float sendbuf[37];

    data_send dsend;
    int send_flag = 0;
    while (!stop_thread)
    {
        mymutex.lock();
        if (!data_send_pool.empty())
        {
            dsend = data_send_pool.top();
            data_send_pool.pop();
            send_flag = 1;
        }
        else
        {
            send_flag = 0;
        }
        mymutex.unlock();
        if (send_flag == 1)
        {
            sendbuf[0] = dsend.time_sec;
            sendbuf[1] = dsend.TER_lon;
            sendbuf[2] = dsend.TER_lat;
            sendbuf[3] = dsend.TER_heig;
            sendbuf[4] = dsend.Is_TER_valid;
            sendbuf[5] = dsend.INS_TER_lon;
            sendbuf[6] = dsend.INS_TER_lat;
            sendbuf[7] = dsend.Is_heig_fusion;
            sendbuf[8] = dsend.altitude;
            sendbuf[9] = dsend.geometric_height;
            sendbuf[10] = dsend.fusion_gps_altitude;
            sendbuf[11] = dsend.fusion_gps_geometric_height;
            sendbuf[12] = dsend.fusion_altitude;
            sendbuf[13] = dsend.fusion_geometric_height;
            sendbuf[14] = dsend.adjust_BA;
            sendbuf[15] = dsend.adjust_GPS_heig;
            sendbuf[16] = dsend.adjust_RA;
            sendbuf[17] = dsend.iner_INS_lon;
            sendbuf[18] = dsend.iner_INS_lat;
            sendbuf[19] = dsend.iner_INS_heig;
            sendbuf[20] = dsend.iner_INS_head;
            sendbuf[21] = dsend.iner_INS_pitch;
            sendbuf[22] = dsend.iner_INS_roll;
            sendbuf[23] = dsend.iner_INS_ve;
            sendbuf[24] = dsend.iner_INS_vn;
            sendbuf[25] = dsend.iner_INS_vu;
            sendbuf[26] = dsend.Is_scene_match_valid;
            sendbuf[27] = dsend.scene_match_lon;
            sendbuf[28] = dsend.scene_match_lat;
            sendbuf[29] = dsend.INS_scene_match_lon;
            sendbuf[30] = dsend.INS_scene_match_lat;
            sendbuf[31] = static_cast<int32_t>(dsend.real_time >> 32);
            sendbuf[32] = static_cast<int32_t>(dsend.real_time);
            sendbuf[33] = dsend.SITAN_result_state;
            sendbuf[34] = dsend.sitan_lon;
            sendbuf[35] = dsend.sitan_lat;
            sendbuf[36] = dsend.sitan_heig;
            // mymutex.lock();
            // if(!SITAN_send_pool.empty())
            // {
            //     sendbuf[33]=SITAN_send_pool.top().result_state;
            //     sendbuf[34]=
            // }

            // memcpy(sendbuf, &dsend, sizeof(dsend));
            sendto(socketfd, sendbuf, sizeof(sendbuf), 0, (struct sockaddr *)&Client_addr, Client_addr_size);
           
            Res_TER_lon = dsend.TER_lon;
            Res_TER_lat = dsend.TER_lat;
            Res_TER_heig = dsend.TER_heig;
            Res_altitude = dsend.altitude;

            send_flag = 0;
        }
    }
    close(socketfd);
}

void fusion_thread()
{
    double T = 0.02;
    double T_D = 1;
    double T_M = 0;
    int cal_flag = 0;
    int ter_fusion_flag = 0;
    int heig_fusion_flag = 0;
    int Is_tercom = 0;
    int Is_heig_fusion = 0;
    int Is_scene_match = 0;
    int Is_sitan = 0;
    float topographic_heig = 0;
    // 地形匹配/景象匹配/高度融合（无效或未进行时）数据存储
    float TER_lon = 0;
    float TER_lat = 0;
    float TER_heig = 0;
    float INS_TER_lon = 0;
    float INS_TER_lat = 0;
    float altitude_with_gps = 0;
    float geo_heig_with_gps = 0;
    float altitude_without_gps = 0;
    float geo_heig_without_gps = 0;
    float BA_filter = 0;
    float gps_filter = 0;
    float RA_filter = 0;
    float scene_match_lon = 0;
    float scene_match_lat = 0;
    float INS_scene_match_lon = 0;
    float INS_scene_match_lat = 0;
    //
    Height_fusion hf1;
    // 初始姿态、速度、位置由外部文件设置
    // vector<double> initial_posi(3, 0.0);
    // vector<double> initial_velo(3, 0.0);
    // vector<double> initial_atti(3, 0.0);
    vector<double> attiN(3, 0.0);                                // 横滚、俯仰、航向
    vector<double> veloN(3, 0.0);                                // 东、北、天
    vector<double> posiN(3, 0.0);                                // 经度、纬度、高度
    vector<vector<double>> Fb(3, vector<double>(1, 0.0));        // 机体系x、y、z加速度
    vector<vector<double>> Wibb(3, vector<double>(1, 0.0));      // 机体系下陀螺仪输出x、y、z
    vector<vector<double>> WnbbA_old(3, vector<double>(1, 0.0)); // 角速度积分输出
    vector<double> posiT(3, 0.0);                                // 地形匹配位置
    // 简易对准
    mymutex.lock();
    posiN = initial_posi;
    attiN = initial_atti;
    veloN = initial_velo;
    mymutex.unlock();
    Fb[0][0] = 0.0;
    Fb[1][0] = 0.0;
    Fb[2][0] = 9.78;
    Wibb[0][0] = 0.0;
    Wibb[1][0] = 0.0;
    Wibb[2][0] = 0.0;
    //
    vector<vector<double>> Xc(18, vector<double>(1, 0.0));
    vector<double> Xerr(18, 0.0);
    vector<vector<double>> PK(18, vector<double>(18, 0.0));
    Kalman_init(posiN, Xc, PK, Xerr);
    // 高度融合相关
    hf1.map_data_load(static_cast<float>(posiN[0]), static_cast<float>(posiN[1]));
    vector<vector<double>> Xcc(3, vector<double>(1, 0.0));
    vector<vector<double>> Xcc_gps(3, vector<double>(1, 0.0));
    vector<double> Xerrr(3, 0.0);
    vector<vector<double>> PKK(3, vector<double>(3, 0.0));
    hf1.heig_kalman_init(Xcc, PKK, Xerrr);
    hf1.heig_kalman_init(Xcc_gps, PKK, Xerrr);
    vector<float> multi_heig_filter;
    vector<float> multi_heig_toaltitude;
    vector<float> posiN_temp(3, 0.0);
    vector<float> veloN_temp(3, 0.0);
    vector<float> map_heig_info(2, 0.0);

    data_recv recv;
    data_send send;
    TERCOM_recv TER_recv;
    TERCOM_send TER_send;
    SITAN_send SITAN_send;
    // time_stamp ts1;
    int64_t real_time;
    // int64_t real_end_time;
    while (!stop_thread)
    {
        mymutex.lock();
        if (!data_recv_pool.empty())
        {
            recv = data_recv_pool.top();
            data_recv_pool.pop();
            cal_flag = 1;
            // sem_post(&tercom_recv_data_flag);
        }
        else
        {
            cal_flag = 0;
        }
        mymutex.unlock();
        if (cal_flag == 1)
        {
            real_time = get_time_stamp();
            // cout<<"test!"<<endl;
            // cout << ts1.year << "/" << ts1.month << "/" << ts1.day << "-" << ts1.hour << ":" << ts1.min << ":" << ts1.s << ":" << ts1.millis << endl;
            T_M += T;
            Fb[0][0] = recv.INS_Fb_x;
            Fb[1][0] = recv.INS_Fb_y;
            Fb[2][0] = recv.INS_Fb_z;
            Wibb[0][0] = recv.INS_Wibb_x;
            Wibb[1][0] = recv.INS_Wibb_y;
            Wibb[2][0] = recv.INS_WIbb_z;
            // 内部惯导计算
            atti_cal_cq_modi(posiN, attiN, veloN, Wibb, WnbbA_old, T);
            velo_cal(T, attiN, posiN, veloN, Fb);
            posi_cal(T, veloN, posiN);

            // TER_recv.time_sec = recv.time_sec;
            TER_recv.real_time = recv.real_time;
            TER_recv.iner_INS_lon = posiN[0];
            TER_recv.iner_INS_lat = posiN[1];
            TER_recv.iner_INS_heig = posiN[2];
            TER_recv.iner_INS_ve = veloN[0];
            TER_recv.iner_INS_vn = veloN[1];
            TER_recv.BA = recv.BA;
            TER_recv.RA = recv.RA;
            // cout<<recv.RA<<endl;
            mymutex.lock();
            if (TERCOM_recv_pool.size() >= 1)
            {
                TERCOM_recv_pool.pop();
            }
            TERCOM_recv_pool.push(TER_recv);

            mymutex.unlock();
            int nn;
            sem_getvalue(&tercom_recv_data_flag, &nn);
            // cout<<nn<<endl;
            if (nn == 0)
            {
                sem_post(&tercom_recv_data_flag);
            }

            if (T_M >= 1)
            {
                T_M = 0;
                int n;
                sem_getvalue(&tercom_flag, &n);
                if (n == 0)
                {
                    sem_post(&tercom_flag);
                }
                heig_fusion_flag = 1;
            }
            else
            {
                heig_fusion_flag = 0;
            }
            mymutex.lock();
            if (!TERCOM_send_pool.empty())
            {
                TER_send = TERCOM_send_pool.top();
                TERCOM_send_pool.pop();
                ter_fusion_flag = 1;
                // cout<<ter_fusion_flag<<endl;
            }
            else
            {
                ter_fusion_flag = 0;
            }
            mymutex.unlock();
            // cal_flag = 0;
            if (ter_fusion_flag == 1)
            {
                posiT[0] = static_cast<double>(TER_send.match_lon);
                posiT[1] = static_cast<double>(TER_send.match_lat);
                posiT[2] = static_cast<double>(TER_send.match_heig);
                posi_compensation(recv.real_time, TER_send.time_sec, veloN, posiT, T * 1000.0);
                // cout << "地形匹配时间差：" << recv.real_time - TER_send.time_sec << endl;
                for (int i = 0; i < Xc.size(); i++)
                {
                    Xc[i][0] = 0.0;
                }
                Kalman_TERCOM(posiN, posiT, attiN, Fb, veloN, PK, Xc, Xerr, T_D, 1);
                Kalman_TERCOM_modi(attiN, veloN, posiN, Xc);
                cout << "地形匹配修正惯导中。。。" << endl;
                ter_fusion_flag = 0;
                Is_tercom = 1;
            }
            else
            {
                Is_tercom = 0;
            }
            // 地形匹配结束
            // 开始高度融合
            hf1.BA = recv.BA;
            hf1.GPS_H = recv.GPS_heig;
            hf1.RA = recv.RA;
            // cout<<hf1.RA<<endl;
            hf1.Height_sequence();
            // cout << "step-2---out!" << endl;
            hf1.map_data_load(static_cast<float>(posiN[0]), static_cast<float>(posiN[1]));
            // cout << "step-1---out!" << endl;
            if (heig_fusion_flag == 1)
            {
                // cout<<posiN[2]<<endl;
                posiN_temp[0] = static_cast<float>(posiN[0]);
                posiN_temp[1] = static_cast<float>(posiN[1]);
                posiN_temp[2] = static_cast<float>(posiN[2]);
                // cout<<posiN_temp[2]<<endl;
                veloN_temp[0] = static_cast<float>(veloN[0]);
                veloN_temp[1] = static_cast<float>(veloN[1]);
                veloN_temp[2] = static_cast<float>(veloN[2]);
                // cout << "step0---out!" << endl;
                multi_heig_filter = hf1.GA_Filter(M, M, M, T, veloN[2]);
                // cout << "step1---out!" << endl;
                // cout<<multi_heig_filter[0]<<" "<<multi_heig_filter[1]<<" "<<multi_heig_filter[2]<<endl;
                multi_heig_toaltitude = hf1.ToAltitude(multi_heig_filter, posiN_temp[0], posiN_temp[1], map_heig_info, hf1.heig_map_data, hf1.abnrmal_heig_data);
                // cout << "step2---out!" << endl;
                // cout<<multi_heig_toaltitude[0]<<" "<<multi_heig_toaltitude[1]<<" "<<multi_heig_toaltitude[2]<<endl;
                Xcc[0][0] = 0;
                Xcc[1][0] = 0;
                Xcc[2][0] = 0;
                Xcc_gps[0][0] = 0;
                Xcc_gps[1][0] = 0;
                Xcc_gps[2][0] = 0;
                hf1.heig_kalman(static_cast<float>(T_D), multi_heig_toaltitude, posiN_temp, veloN_temp, recv.GPS_vu, Xcc, Xcc_gps, PKK, Xerrr, 1);
                // cout << "step3---out!" << endl;
                // cout<<posiN_temp[2]<<endl;
                hf1.cal_altitude(posiN_temp, veloN_temp, Xcc, Xcc_gps, 1);
                // cout << "step4---out!" << endl;
                // cout<<posiN_temp[2]<<endl;
                hf1.cal_abs_altitude(map_heig_info, multi_heig_filter, PKK);
                cout << "高度融合中。。。" << endl;
                posiN[2] = static_cast<double>(posiN_temp[2]);
                // cout<<posiN[2]<<endl;
                veloN[2] = static_cast<double>(veloN_temp[2]);
                heig_fusion_flag = 0;
                Is_heig_fusion = 1;
            }
            else
            {
                Is_heig_fusion = 0;
            }

            mymutex.lock();
            if (!SITAN_send_pool.empty())
            {
                SITAN_send = SITAN_send_pool.top();
                SITAN_send_pool.pop();
                Is_sitan = 1;
            }
            else
            {
                Is_sitan = 0;
            }
            mymutex.unlock();
            // 打包数据
            send.time_sec = recv.time_sec;
            if (Is_tercom == 1)
            {
                TER_lon = TER_send.match_lon;
                TER_lat = TER_send.match_lat;
                TER_heig = TER_send.match_heig;
                INS_TER_lon = static_cast<float>(posiN[0]);
                INS_TER_lat = static_cast<float>(posiN[1]);
                send.Is_TER_valid = TER_send.Is_match_valid;
                // send.TER_lon = TER_send.match_lon;
                // send.TER_lat = TER_send.match_lat;
                // send.TER_heig = TER_send.match_heig;
                // send.Is_TER_valid = TER_send.Is_match_valid;
                // send.INS_TER_lon = static_cast<float>(posiN[0]);
                // send.INS_TER_lat = static_cast<float>(posiN[1]);
            }
            else
            {
                send.Is_TER_valid = -1;
                // send.TER_lon = 0;
                // send.TER_lat = 0;
                // send.TER_heig = 0;
                // send.INS_TER_lon = 0;
                // send.INS_TER_lat = 0;
            }
            if (Is_heig_fusion == 1)
            {
                send.Is_heig_fusion = 1;
                altitude_with_gps = hf1.altitude_withgps;
                geo_heig_with_gps = hf1.abs_altitude_withgps;
                altitude_without_gps = hf1.altitude_withoutgps;
                geo_heig_without_gps = hf1.abs_altitude_withoutgps;
                BA_filter = multi_heig_toaltitude[0];
                gps_filter = multi_heig_toaltitude[1];
                RA_filter = multi_heig_toaltitude[2];
                // send.fusion_gps_altitude = hf1.altitude_withgps;
                // send.fusion_gps_geometric_height = hf1.abs_altitude_withgps;
                // send.fusion_altitude = hf1.altitude_withoutgps;
                // send.fusion_geometric_height = hf1.abs_altitude_withoutgps;
                // send.adjust_BA = multi_heig_toaltitude[0];
                // send.adjust_GPS_heig = multi_heig_toaltitude[1];
                // send.adjust_RA = multi_heig_toaltitude[2];
            }
            else
            {
                send.Is_heig_fusion = 0;
                // send.fusion_gps_altitude = 0;
                // send.fusion_gps_geometric_height = 0;
                // send.fusion_altitude = 0;
                // send.fusion_geometric_height = 0;
                // send.adjust_BA = 0;
                // send.adjust_GPS_heig = 0;
                // send.adjust_RA = 0;
            }
            if (Is_scene_match == 1)
            {
                send.Is_scene_match_valid = 1;
            }
            else
            {
                send.Is_scene_match_valid = 0;
            }
            send.TER_lon = TER_lon;
            send.TER_lat = TER_lat;
            send.TER_heig = TER_heig;
            send.INS_TER_lon = INS_TER_lon;
            send.INS_TER_lat = INS_TER_lat;

            send.altitude = static_cast<float>(posiN[2]);
            topographic_heig = hf1.read_topographic_heig(static_cast<float>(posiN[0]), static_cast<float>(posiN[1]), hf1.heig_map_data);
            send.geometric_height = static_cast<float>(posiN[2] - topographic_heig);

            send.fusion_gps_altitude = altitude_with_gps;
            send.fusion_gps_geometric_height = geo_heig_with_gps;
            send.fusion_altitude = altitude_without_gps;
            send.fusion_geometric_height = geo_heig_without_gps;
            send.adjust_BA = BA_filter;
            send.adjust_GPS_heig = gps_filter;
            send.adjust_RA = RA_filter;
            send.scene_match_lon = scene_match_lon;
            send.scene_match_lat = scene_match_lat;
            send.INS_scene_match_lon = INS_scene_match_lon;
            send.INS_scene_match_lat = INS_scene_match_lat;
            send.iner_INS_lon = static_cast<float>(posiN[0]);
            send.iner_INS_lat = static_cast<float>(posiN[1]);
            send.iner_INS_heig = static_cast<float>(posiN[2]);
            send.iner_INS_head = static_cast<float>(attiN[2]);
            send.iner_INS_roll = static_cast<float>(attiN[0]);
            send.iner_INS_pitch = static_cast<float>(attiN[1]);
            send.iner_INS_ve = static_cast<float>(veloN[0]);
            send.iner_INS_vn = static_cast<float>(veloN[1]);
            send.iner_INS_vu = static_cast<float>(veloN[2]);
            send.real_time = real_time;
            // mymutex.lock();
            // if (!SITAN_send_pool.empty())
            // {
            //     send.SITAN_result_state = SITAN_send_pool.top().result_state;
            //     send.sitan_lon = SITAN_send_pool.top().match_lon;
            //     send.sitan_lat = SITAN_send_pool.top().match_lat;
            //     send.sitan_heig = SITAN_send_pool.top().match_heig;
            //     SITAN_send_pool.pop();
            // }
            // else
            // {
            if (Is_sitan == 1)
            {
                send.SITAN_result_state = SITAN_send.result_state;
                send.sitan_lon = SITAN_send.match_lon;
                send.sitan_lat = SITAN_send.match_lat;
                send.sitan_heig = SITAN_send.match_heig;
                Is_sitan = 0;
            }
            else
            {
                send.SITAN_result_state = -1;
                send.sitan_lon = 0;
                send.sitan_lat = 0;
                send.sitan_heig = 0;
            }
            // real_end_time = get_time_stamp();
            // cout << "融合线程用时：" << real_end_time - real_time << endl;
            // }
            // mymutex.unlock();
            mymutex.lock();
            if (data_send_pool.size() >= 1)
            {
                data_send_pool.pop();
            }
            data_send_pool.push(send);
            mymutex.unlock();
        }
    }
}

void TERCOM_thread()
{
    // TERCOM t1;
    // int64_t time1;
    // int64_t time2;
    TERCOM_recv trecv;
    TERCOM_send tsend;
    int max_size = 128;
    int tercom_count = 0;
    int Is_tercom_valid = 0;
    double analysis_value = 0;
    double dist_NtoT = 0;
    double dist_time = 0;
    vector<double> init_posi(3, 0.0);
    mymutex.lock();
    init_posi = initial_posi;
    mymutex.unlock();
    // vector<double> initial_posi(3, 0.0);
    // vector<double> initial_velo(3, 0.0);
    // vector<double> initial_atti(3, 0.0);
    double topographic_measure_heig = 0;
    t1.var_init(3600, 30, 2000.0, init_posi[1], init_posi[0], init_posi[2], max_size);
    // cout << "地形匹配地图载入成功！地图大小为：" << t1.mapdata[0].Map.size() << endl;
    cout << "地形匹配地图载入成功！" << endl;
    while (!stop_thread)
    {
        // int res1 = sem_trywait(&tercom_recv_data_flag);
       
        //sem_wait(&tercom_recv_data_flag);
        //改成带超时的信号量的等待 防止退出线程时阻塞
        if (sem_trywait(&tercom_recv_data_flag) != 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }
        
        // if (res1 == 0)
        // {
            mymutex.lock();
            if (!TERCOM_recv_pool.empty())
            {
                trecv = TERCOM_recv_pool.top();
                TERCOM_recv_pool.pop();
                // cout<<"TERCOM在接收数据!"<<endl;
            }
            mymutex.unlock();
            // t1.time_info = static_cast<double>(trecv.time_sec);
            // time1 = get_time_stamp();
            t1.time_info = trecv.real_time;
            topographic_measure_heig = trecv.BA - trecv.RA;
            t1.Set_sequence(trecv.iner_INS_lat, trecv.iner_INS_lon, topographic_measure_heig, max_size);
            // cout<<t1.time_info<<" ";
            t1.Serch_map(t1.mapdata[0].Map, t1.mapdata);
            // cout<<t1.Match_sequence.size()<<endl;
        // }
        if (t1.match_flag == 1)
        {
            int res = sem_trywait(&tercom_flag);
            if (res == 0)
            {
                tercom_count += 1;
                t1.Tercom_realization(1);
                // 地形匹配结果分析
                if (tercom_count == 1)
                {
                    analysis_value = t1.Match_analysis(t1.match_col, t1.match_row);
                    if (analysis_value > 0.2)
                    {
                        t1.match_lon = t1.Match_sequence[t1.Match_sequence.size() - 1].ins_lon;
                        t1.match_lat = t1.Match_sequence[t1.Match_sequence.size() - 1].ins_lat;
                        Is_tercom_valid = 0;
                    }
                    else
                    {
                        Is_tercom_valid = 1;
                    }
                    t1.analysis_lat = t1.match_lat;
                    t1.analysis_lon = t1.match_lon;
                    t1.analysis_time = t1.time_info;
                }
                else if (tercom_count >= 2)
                {
                    dist_NtoT = sqrt(((t1.match_lon - t1.analysis_lon) * 111000 * cos(t1.match_lat * pi / 180.0)) * ((t1.match_lon - t1.analysis_lon) * 111000 * cos(t1.match_lat * pi / 180.0)) + ((t1.match_lat - t1.analysis_lat) * 111000) * ((t1.match_lat - t1.analysis_lat) * 111000));
                    dist_time = sqrt((abs(t1.time_info - t1.analysis_time) * trecv.iner_INS_ve) * (abs(t1.time_info - t1.analysis_time) * trecv.iner_INS_ve) + (abs(t1.time_info - t1.analysis_time) * trecv.iner_INS_vn) * (abs(t1.time_info - t1.analysis_time) * trecv.iner_INS_vn));
                    if (dist_NtoT > (1.5 * dist_time))
                    {
                        analysis_value = t1.Match_analysis(t1.match_col, t1.match_row);
                        if (analysis_value > 0.2)
                        {
                            t1.match_lon = t1.Match_sequence[t1.Match_sequence.size() - 1].ins_lon;
                            t1.match_lat = t1.Match_sequence[t1.Match_sequence.size() - 1].ins_lat;
                            Is_tercom_valid = 0;
                        }
                        else
                        {
                            Is_tercom_valid = 1;
                        }
                    }
                    else
                    {
                        Is_tercom_valid = 1;
                    }
                    t1.analysis_lat = t1.match_lat;
                    t1.analysis_lon = t1.match_lon;
                    t1.analysis_time = t1.time_info;
                }
                tsend.Is_match_valid = Is_tercom_valid;
                tsend.match_lon = static_cast<float>(t1.match_lon);
                tsend.match_lat = static_cast<float>(t1.match_lat);
                // tsend.time_sec = static_cast<float>(t1.time_info);
                tsend.time_sec = t1.time_info;
                tsend.match_heig = static_cast<float>(t1.match_altitude + trecv.RA);
                // time2 = get_time_stamp();
                // cout << "单次地形匹配用时：" << time2 - time1 << endl;
                mymutex.lock();
                if (TERCOM_send_pool.size() >= 1)
                {
                    TERCOM_send_pool.pop();
                }
                TERCOM_send_pool.push(tsend);
                mymutex.unlock();
            }
        }
    }
}

// vector<double> posi_init()
// {
//     vector<double> data_temp;
//     //int i = 0;
//     ifstream inputfile("initial_posi.txt");
//     if (!inputfile.is_open())
//     {
//         cout << "file open error!" << endl;
//         return data_temp;
//     }
//     //string line;
//     double value;
//     while(inputfile>>value)
//     {
//         data_temp.push_back(value);
//     }
//     // while (getline(inputfile, line))
//     // {
//     //     // sscanf(line.c_str(), "%lf", &data_temp[i]);
//     //     // i += 1;
//     //     // cout<<i<<endl;
//     //     double value;
//     //     if (istringstream(line) >> value)
//     //     {
//     //         data_temp.push_back(value);
//     //     }
//     // }
//     inputfile.close();
//     return data_temp;
//     // initial_posi[0] = data_temp[0];
//     // initial_posi[1] = data_temp[1];
//     // initial_posi[2] = data_temp[2];

//     // initial_atti[0] = data_temp[3];
//     // initial_atti[1] = data_temp[4];
//     // initial_atti[2] = data_temp[5];

//     // initial_velo[0] = data_temp[6];
//     // initial_velo[1] = data_temp[7];
//     // initial_velo[2] = data_temp[8];
// }

void SITAN_thread()
{
    SITAN s1;
    s1.Filter_parameter_init();
    ofstream record_txt;
    record_txt.open("record_sitan.txt", std::ios::out);
    double T = 0.02;
    vector<double> attiN(3, 0.0);                                // 横滚、俯仰、航向
    vector<double> veloN(3, 0.0);                                // 东、北、天
    vector<double> posiN(3, 0.0);                                // 经度、纬度、高度
    vector<vector<double>> Fb(3, vector<double>(1, 0.0));        // 机体系x、y、z加速度
    vector<vector<double>> Wibb(3, vector<double>(1, 0.0));      // 机体系下陀螺仪输出x、y、z
    vector<vector<double>> WnbbA_old(3, vector<double>(1, 0.0)); // 角速度积分输出

    // 简易对准
    mymutex.lock();
    posiN = initial_posi;
    attiN = initial_atti;
    veloN = initial_velo;
    mymutex.unlock();
    Fb[0][0] = 0.0;
    Fb[1][0] = 0.0;
    Fb[2][0] = 9.78;
    Wibb[0][0] = 0.0;
    Wibb[1][0] = 0.0;
    Wibb[2][0] = 0.0;
    // 地图载入
    int max_size = 1;
    t2.var_init(3600, 30, 2000.0, posiN[1], posiN[0], posiN[2], max_size);
    cout << "SITAN地图载入成功!" << endl;
    data_recv recv;
    int cal_flag = 0;
    SITAN_send ssend;
    while (!stop_thread)
    {
        mymutex.lock();
        if (!data_recv_pool_sitan.empty())
        {
            recv = data_recv_pool_sitan.top();
            data_recv_pool_sitan.pop();
            cal_flag = 1;
            // sem_post(&tercom_recv_data_flag);
        }
        else
        {
            cal_flag = 0;
        }
        mymutex.unlock();
        if (cal_flag == 1)
        {
            // cout<<"YES1?"<<endl;
            // cout << Fb.size() << endl;
            // cout<<s1.result_state<<endl;
            // cout<<s1.sw.size()<<" "<<s1.sw[0].size()<<endl;
            Fb[0][0] = recv.INS_Fb_x;
            // Fb[0][0] = 0;
            Fb[1][0] = recv.INS_Fb_y;
            Fb[2][0] = recv.INS_Fb_z;
            Wibb[0][0] = recv.INS_Wibb_x;
            Wibb[1][0] = recv.INS_Wibb_y;
            Wibb[2][0] = recv.INS_WIbb_z;
            // 内部惯导计算
            atti_cal_cq_modi(posiN, attiN, veloN, Wibb, WnbbA_old, T);
            // cout<<"YES2?"<<endl;
            velo_cal(T, attiN, posiN, veloN, Fb);
            // cout<<"YES3?"<<endl;
            posi_cal(T, veloN, posiN);
            // cout<<"YES4?"<<endl;
            // 地图判断是否需要更新
            t2.Set_sequence(posiN[1], posiN[0], (posiN[2] - static_cast<double>(recv.RA)), max_size);
            // cout<<"YES5?"<<endl;
            t2.Serch_map(t2.mapdata[0].Map, t2.mapdata);
            // cout<<"YES6?"<<endl;
            s1.Serch_map_creat(t2); // SITAN的搜索地图也是会根据位置进行更新，即使是在跟踪模式下！
            // cout<<"YES7?"<<endl;
            //
            s1.ins_lon = posiN[0];
            s1.ins_lat = posiN[1];
            s1.ins_heig = posiN[2];
            s1.ins_ve = recv.INS_ve;
            s1.ins_vn = recv.INS_vn;
            s1.BA = static_cast<double>(recv.BA);
            s1.RA = static_cast<double>(recv.RA);
            // cout<<"YES8?"<<endl;
            s1.Method_sitan(t2); // SITAN中
            // cout<<"YES9?"<<endl;
            //
            if (s1.result_state == 1)
            {
                posiN[0] = s1.tan_lon;
                posiN[1] = s1.tan_lat;
                posiN[2] = s1.tan_heig;
                ssend.match_lon = s1.tan_lon;
                ssend.match_lat = s1.tan_lat;
                ssend.match_heig = s1.tan_heig;
                ssend.result_state = s1.result_state;
            }
            else
            {
                ssend.match_lon = s1.tan_lon;
                ssend.match_lat = s1.tan_lat;
                ssend.match_heig = s1.tan_heig;
                ssend.result_state = s1.result_state;
            }
            ssend.time_sec = recv.time_sec;
            record_txt << ssend.time_sec << " " << ssend.result_state << " " << ssend.match_lon << " " << ssend.match_lat << " " << ssend.match_heig << endl;

            mymutex.lock();
            if (SITAN_send_pool.size() >= 1)
            {
                SITAN_send_pool.pop();
            }
            SITAN_send_pool.push(ssend);
            mymutex.unlock();
        }
    }
    record_txt.close();
}