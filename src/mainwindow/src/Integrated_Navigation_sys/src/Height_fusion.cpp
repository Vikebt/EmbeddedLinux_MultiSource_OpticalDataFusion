#include "Height_fusion.h"

void Height_fusion::Height_sequence()
{
    // vector<float>height_ans(3,0);//数据依次为气压高度、卫星高度、无线电高度
    int n_ba = this->BA_sequence.size();
    int n_gps = this->GPS_sequence.size();
    int n_ra = this->RA_sequence.size();
    if (n_ba >= this->ba_size)
    {
        this->BA_sequence.erase(this->BA_sequence.begin());
        this->BA_sequence.push_back(this->BA);
        this->ba_filter_flag = 1;
    }
    else
    {
        this->BA_sequence.push_back(this->BA);
        if (this->BA_sequence.size() == this->ba_size)
        {
            this->ba_filter_flag = 1; // GA_FILTER
        }
        else
        {
            this->ba_filter_flag = 0;
        }
    }

    if (n_gps >= this->gps_size)
    {
        this->GPS_sequence.erase(this->GPS_sequence.begin());
        this->GPS_sequence.push_back(this->GPS_H);
        this->gps_filter_flag = 1; // GA_FILTER
    }
    else
    {
        this->GPS_sequence.push_back(this->GPS_H);
        if (this->GPS_sequence.size() == this->gps_size)
        {
            this->gps_filter_flag = 1; // GA_FILTER
        }
        else
        {
            this->gps_filter_flag = 0;
        }
    }

    if (n_ra >= this->ra_size)
    {
        this->RA_sequence.erase(this->RA_sequence.begin());
        this->RA_sequence.push_back(this->RA);
        this->ra_filter_flag = 1; // GA_FILTER
    }
    else
    {
        this->RA_sequence.push_back(this->RA);
        if (this->RA_sequence.size() == this->ra_size)
        {
            this->ra_filter_flag = 1; // GA_FILTER
        }
        else
        {
            this->ra_filter_flag = 0;
        }
    }
}

vector<float> Height_fusion::GA_Filter(int M_ba, int M_gps, int M_ra, double T, double vu)
{
    vector<float> heig_filter(3, 0); // 依次为气压高度、卫星高度、无线电高度

    if (this->ba_filter_flag == 1)
    {
        vector<vector<double>> AT_BA(N_ + 1, vector<double>(2 * M_ba + 1, 0));
        Eigen::MatrixXd ATT_BA = convertToeigenmatrix(AT_BA);
        for (int i = 0; i <= N_; i++)
        {
            for (int j = 0; j <= 2 * M_ba; j++)
            {
                ATT_BA(i, j) = pow((-M_ba + j), i);
            }
        }
        ATT_BA(0, M_ba) = 1;
        Eigen::MatrixXd H_temp1 = ATT_BA * ATT_BA.transpose();
        Eigen::MatrixXd H1 = H_temp1.inverse() * ATT_BA;
        float BA_filter = 0;
        for (int i = 0; i < 2 * M_ba + 1; i++)
        {
            BA_filter = BA_filter + static_cast<float>(H1(0, i)) * this->BA_sequence[i];
        }
        BA_filter = BA_filter + (M_ba * T * vu);
        heig_filter[0] = BA_filter;
    }
    else
    {
        int n_ba = this->BA_sequence.size();
        heig_filter[0] = this->BA_sequence[n_ba - 1];
    }

    if (this->gps_filter_flag == 1)
    {
        vector<vector<double>> AT_GPS(N_ + 1, vector<double>(2 * M_gps + 1, 0));
        Eigen::MatrixXd ATT_GPS = convertToeigenmatrix(AT_GPS);
        for (int i = 0; i <= N_; i++)
        {
            for (int j = 0; j <= 2 * M_gps; j++)
            {
                ATT_GPS(i, j) = pow((-M_gps + j), i);
            }
        }
        ATT_GPS(0, M_gps) = 1;
        Eigen::MatrixXd H_temp2 = ATT_GPS * ATT_GPS.transpose();
        Eigen::MatrixXd H2 = H_temp2.inverse() * ATT_GPS;
        float GPS_filter = 0;
        for (int i = 0; i < 2 * M_gps + 1; i++)
        {
            GPS_filter = GPS_filter + static_cast<float>(H2(0, i)) * this->GPS_sequence[i];
        }
        GPS_filter = GPS_filter + (M_gps * T * vu);
        heig_filter[1] = GPS_filter;
    }
    else
    {
        int n_gps = this->GPS_sequence.size();
        heig_filter[1] = this->GPS_sequence[n_gps - 1];
    }

    if (this->ra_filter_flag == 1)
    {
        vector<vector<double>> AT_RA(N_ + 1, vector<double>(2 * M_ra + 1, 0));
        Eigen::MatrixXd ATT_RA = convertToeigenmatrix(AT_RA);
        for (int i = 0; i <= N_; i++)
        {
            for (int j = 0; j <= 2 * M_ra; j++)
            {
                ATT_RA(i, j) = pow((-M_ra + j), i);
            }
        }
        ATT_RA(0, M_ra) = 1;
        Eigen::MatrixXd H_temp3 = ATT_RA * ATT_RA.transpose();
        Eigen::MatrixXd H3 = H_temp3.inverse() * ATT_RA;
        float RA_filter = 0;
        for (int i = 0; i < 2 * M_ra + 1; i++)
        {
            RA_filter = RA_filter + static_cast<float>(H3(0, i)) * this->RA_sequence[i];
        }
        RA_filter = RA_filter + (M_ra * T * vu);
        heig_filter[2] = RA_filter;
    }
    else
    {
        int n_ra = this->RA_sequence.size();
        heig_filter[2] = this->RA_sequence[n_ra - 1];
    }

    return heig_filter;
}

void Height_fusion::map_data_load(float lon, float lat)
{
    // this->abnormal_filename = 'Global_abnormal_elevation_value_database.bin';
    int map_lon = floor(lon);
    int map_lat = floor(lat) + 1;
    int map_resolution = 3600;
    ostringstream strs;
    strs << "integrated_navigation/"<<map_lat << "_" << map_lon << "_" << map_resolution;
    string map_name = strs.str() + ".bin";
    if (this->heig_filename != map_name)
    {
        // this->heig_filename=map_name;
        ifstream file(map_name, ios::binary | ios::ate);
        auto i = file.is_open();
        if (!file.is_open())
        {
            cout << "无法打开文件" << endl;
            // return;
        }
        streamsize size = file.tellg();
        file.seekg(0, ios::beg);
        this->heig_map_data.resize(size / sizeof(int16_t), 0);
        file.read(reinterpret_cast<char *>(this->heig_map_data.data()), size);
        file.close();
        cout << "地形数据载入成功！" << endl;
        this->heig_filename = map_name;
    }
    if (this->abnormal_flag == 1)
    {
        ifstream file(this->abnormal_filename, ios::binary | ios::ate);
        if (!file)
        {
            cout << "无法打开文件" << endl;
        }
        streamsize size = file.tellg();
        file.seekg(0, ios::beg);
        this->abnrmal_heig_data.resize(size / sizeof(float), 0);
        file.read(reinterpret_cast<char *>(this->abnrmal_heig_data.data()), size);
        file.close();
        cout << "高程异常值数据载入成功！" << endl;
        this->abnormal_flag = 0;
    }
}

vector<float> Height_fusion::ToAltitude(vector<float> &heig, float lon, float lat, vector<float> &map_heig_info, vector<int16_t> &heig_map_data, vector<float> &abnrmal_heig_data)
{
    vector<float> heig_changebase(3, 0);
    // ba to altitude
    // do nothing
    heig_changebase[0] = heig[0];
    // gps to altitude
    int row = 1200 - floor((lat + 60) * 10);
    int col = floor(lon * 10);
    // cout<<row*3601+col<<endl;
    float abh1 = abnrmal_heig_data[row * 3601 + col];
    float abh2 = abnrmal_heig_data[row * 3601 + col + 1];
    float abh3 = abnrmal_heig_data[(row - 1) * 3601 + col];
    float abh4 = abnrmal_heig_data[(row - 1) * 3601 + col + 1];
    float abha = abh1 + (abh2 - abh1) * ((lon * 10 - floor(lon * 10)) / 1);
    float abhb = abh3 + (abh4 - abh3) * ((lon * 10 - floor(lon * 10)) / 1);
    float abnromal_heig = abha + (abhb - abha) * ((lat * 10 - floor(lat * 10)) / 1);
    heig_changebase[1] = heig[1] - abnromal_heig;
    // ra to altitude
    int posi_row_boundary = floor(lat) + 1;
    int posi_col_boundary = floor(lon);
    int posi_row = floor((posi_row_boundary - lat) * 3600);
    int posi_col = floor((lon - posi_col_boundary) * 3600);
    if (posi_row < 0)
    {
        posi_row = 0;
    }
    if (posi_row > 3600)
    {
        posi_row = 3600;
    }
    if (posi_col < 0)
    {
        posi_col = 0;
    }
    if (posi_col > 3600)
    {
        posi_col = 3600;
    }
    int height = heig_map_data[(posi_row) * 3601 + posi_col];
    heig_changebase[2] = heig[2] + height;

    map_heig_info[0] = abnromal_heig;
    map_heig_info[1] = height;

    return heig_changebase;
}

void Height_fusion::heig_kalman_init(vector<vector<double>> &Xc, vector<vector<double>> &PK, vector<double> &Xerr)
{
    // float g = 9.7803698;
    vector<double> Xerr0(3, 0);
    Xerr0[0] = 0.6;
    Xerr0[1] = 50;
    Xerr0[2] = (1e-4) * g;
    for (int i = 0; i < 3; i++)
    {
        PK[i][i] = Xerr0[i] * Xerr0[i];
    }
    for (int i = 0; i < Xc.size(); i++)
    {
        Xc[i][0] = 0;
    }
    Xerr[0] = sqrt(PK[0][0]);
    Xerr[1] = sqrt(PK[1][1]);
    Xerr[2] = sqrt(PK[2][2]) / g;
}

void Height_fusion::heig_kalman(float T_D, vector<float> &multi_heig, vector<float> &posiN, vector<float> &veloN, float &veloGu, vector<vector<double>> &Xc, vector<vector<double>> &Xc_gps, vector<vector<double>> &PK, vector<double> &Xerr, int kflag)
{
    // vector<float> modi_heig(4, 0); // 依次为卫星有效下的融合海拔高度、卫星有效下融合几何高度、卫星失效下融合海拔高度、卫星失效下融合几何高度
    float lon = posiN[0];
    float lat = posiN[1];
    float heig = posiN[2];
    float Taz = 1800.0;
    float Rm = (1 - f) * (1 - f) * Re / (((1 - f) * (1 - f) * sin(lat) * sin(lat) + cos(lat) * cos(lat)) * sqrt((1 - f) * (1 - f) * sin(lat) * sin(lat) + cos(lat) * cos(lat)));
    float Rn = Re / sqrt((1 - f) * (1 - f) * sin(lat) * sin(lat) + cos(lat) * cos(lat));
    float Ve = veloN[0];
    float Vn = veloN[1];
    float Vu = veloN[2];
    // vector<vector<double>> FI(3, vector<double>(3, 0.0));
    // vector<vector<double>> GI(3, vector<double>(1, 0.0));
    // Eigen::MatrixXd FII = convertToeigenmatrix(FI);
    // Eigen::MatrixXd GII = convertToeigenmatrix(GI);
    Eigen::MatrixXd FII(3, 3);
    Eigen::MatrixXd GII(3, 1);
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(3, 3);

    FII(0, 0) = 0.0;
    FII(0, 1) = static_cast<double>(-((Ve * Ve) / ((Rn + heig) * (Rn + heig)) + (Vn * Vn) / ((Rm + heig) * (Rm + heig))));
    FII(0, 2) = 1.0;
    FII(1, 0) = 1.0;
    FII(1, 1) = 0.0;
    FII(1, 2) = 0.0;
    FII(2, 0) = 0.0;
    FII(2, 1) = 0.0;
    FII(2, 2) = static_cast<double>(-1 / Taz);

    GII(0, 0) = 0.0;
    GII(1, 0) = 0.0;
    GII(2, 0) = 1.0;

    Eigen::MatrixXd FL = I + FII * T_D + FII * FII / 2.0 * T_D * T_D;
    Eigen::MatrixXd GL = (I + FII / 2.0 * T_D + FII * FII / 6.0 * T_D * T_D) * GII * T_D;

    Eigen::MatrixXd PKK = convertToeigenmatrix(PK);
    Eigen::MatrixXd XCC = convertToeigenmatrix(Xc);

    Eigen::MatrixXd PK_K_gps(3, 3);
    if (kflag == 1)
    {
        Eigen::MatrixXd W_gps(1, 1);
        W_gps(0, 0) = static_cast<double>((sqrt(2 * T_D / Taz) * (1e-4) * g) * (sqrt(2 * T_D / Taz) * (1e-4) * g));
        Eigen::MatrixXd Q_gps = Eigen::MatrixXd::Identity(1, 1);
        Q_gps(0, 0) = W_gps(0, 0);

        Eigen::MatrixXd HG_gps(4, 3);
        HG_gps(0, 0) = 0.0;
        HG_gps(0, 1) = 1.0;
        HG_gps(0, 2) = 0.0;
        HG_gps(1, 0) = 0.0;
        HG_gps(1, 1) = 1.0;
        HG_gps(1, 2) = 0.0;
        HG_gps(2, 0) = 0.0;
        HG_gps(2, 1) = 1.0;
        HG_gps(2, 2) = 0.0;
        HG_gps(3, 0) = 1.0;
        HG_gps(3, 1) = 0.0;
        HG_gps(3, 2) = 0.0;

        Eigen::MatrixXd VG_gps(1, 4);
        VG_gps(0, 0) = 7.5 * 7.5;
        VG_gps(0, 1) = 7.5 * 7.5;
        VG_gps(0, 2) = 15.0 * 15.0;
        VG_gps(0, 3) = 0.2 * 0.2;
        Eigen::MatrixXd RG_gps = Eigen::MatrixXd::Identity(4, 4);
        RG_gps(0, 0) = VG_gps(0, 0);
        RG_gps(1, 1) = VG_gps(0, 1);
        RG_gps(2, 2) = VG_gps(0, 2);
        RG_gps(3, 3) = VG_gps(0, 3);

        // Eigen::MatrixXd Q_gps = W_gps.diagonal();
        // Eigen::MatrixXd RG_gps = VG_gps.diagonal();

        PK_K_gps = FL * PKK * FL.transpose() + GL * Q_gps * GL.transpose();
        Eigen::MatrixXd KK_gps = PK_K_gps * HG_gps.transpose() * (HG_gps * PK_K_gps * HG_gps.transpose() + RG_gps).inverse();
        PK_K_gps = (I - KK_gps * HG_gps) * PK_K_gps * (I - KK_gps * HG_gps).transpose() + KK_gps * RG_gps * KK_gps.transpose();
        Eigen::MatrixXd YC_gps(4, 1);
        YC_gps(0, 0) = static_cast<double>(posiN[2] - multi_heig[0]);
        YC_gps(1, 0) = static_cast<double>(posiN[2] - multi_heig[1]);
        YC_gps(2, 0) = static_cast<double>(posiN[2] - multi_heig[2]);
        YC_gps(3, 0) = static_cast<double>(veloN[2] - veloGu);
        Eigen::MatrixXd XCC_gps = FL * XCC;
        XCC_gps = XCC_gps + KK_gps * (YC_gps - HG_gps * XCC_gps);

        Xc_gps = convertTovector(XCC_gps);
    }
    else
    {
        Xc_gps[0][0] = 0.0;
        Xc_gps[1][0] = 0.0;
        Xc_gps[2][0] = 0.0;
    }

    Eigen::MatrixXd W(1, 1);
    W(0, 0) = 0.55 * 0.55;
    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(1, 1);
    Q(0, 0) = W(0, 0);
    Eigen::MatrixXd HG(2, 3);
    HG(0, 0) = 0.0;
    HG(0, 1) = 1.0;
    HG(0, 2) = 0.0;
    HG(1, 0) = 0.0;
    HG(1, 1) = 1.0;
    HG(1, 2) = 0.0;

    Eigen::MatrixXd VG(1, 2);
    VG(0, 0) = 7.5 * 7.5;
    VG(0, 1) = 15.0 * 15.0;
    Eigen::MatrixXd RG = Eigen::MatrixXd::Identity(2, 2);
    RG(0, 0) = VG(0, 0);
    RG(1, 1) = VG(0, 1);
    // Eigen::MatrixXd Q = W.diagonal();
    // Eigen::MatrixXd RG = VG.diagonal();

    // Eigen::MatrixXd PKK = convertToeigenmatrix(PK);
    // Eigen::MatrixXd XCC = convertToeigenmatrix(Xc);

    Eigen::MatrixXd PK_K = FL * PKK * FL.transpose() + GL * Q * GL.transpose();
    Eigen::MatrixXd KK = PK_K * HG.transpose() * (HG * PK_K * HG.transpose() + RG).inverse();
    PK_K = (I - KK * HG) * PK_K * (I - KK * HG).transpose() + KK * RG * KK.transpose();
    Eigen::MatrixXd YC(2, 1);
    YC(0, 0) = static_cast<double>(posiN[2] - multi_heig[0]);
    YC(1, 0) = static_cast<double>(posiN[2] - multi_heig[2]);
    XCC = FL * XCC;
    XCC = XCC + KK * (YC - HG * XCC);

    Xc = convertTovector(XCC);

    if (kflag == 1)
    {
        PK = convertTovector(PK_K_gps);
    }
    else
    {
        PK = convertTovector(PK_K);
    }
}

void Height_fusion::cal_altitude(vector<float> &posiN, vector<float> &veloN, vector<vector<double>> &Xc, vector<vector<double>> &Xc_gps, int kflag)
{
    if (kflag == 1)
    {
        this->altitude_withgps = posiN[2] - Xc_gps[1][0];
        posiN[2] = posiN[2] - Xc_gps[1][0];
        veloN[2] = veloN[2] - Xc_gps[0][0];
    }
    else
    {
        this->altitude_withgps = posiN[2] - Xc[1][0];
        posiN[2] = posiN[2] - Xc[1][0];
        veloN[2] = veloN[2] - Xc[0][0];
    }

    this->altitude_withoutgps = posiN[2] - Xc[1][0];
}

void Height_fusion::cal_abs_altitude(vector<float> &map_heig_info, vector<float> &heig_filter, vector<vector<double>> &PK)
{
    if (this->ra_filter_flag != 1)
    {
        this->abs_altitude_withgps = this->altitude_withgps - map_heig_info[1];
        this->abs_altitude_withoutgps = this->altitude_withoutgps - map_heig_info[1];
    }
    else
    {
        float sum = accumulate(this->RA_sequence.begin(), this->RA_sequence.end(), 0.0);
        float mean = sum / this->RA_sequence.size();
        float accum = 0.0;
        for (int i = 0; i < this->RA_sequence.size(); i++)
        {
            accum += (this->RA_sequence[i] - mean) * (this->RA_sequence[i] - mean);
        }
        float var = accum / (this->RA_sequence.size() - 1);
        this->abs_altitude_withgps = (var / (var + PK[1][1])) * (this->altitude_withgps - map_heig_info[1]) + (PK[1][1] / (var + PK[1][1])) * heig_filter[2];
        this->abs_altitude_withoutgps = (var / (var + PK[1][1])) * (this->altitude_withoutgps - map_heig_info[1]) + (PK[1][1] / (var + PK[1][1])) * heig_filter[2];
    }
}

vector<float> Height_fusion::heig_simu(vector<float> &posi, vector<int16_t> &heig_map_data, vector<float> &abnrmal_heig_data, float &veloBu)
{
    srand(time(nullptr));
    unsigned seed = chrono::system_clock::now().time_since_epoch().count();
    default_random_engine generator(seed);

    // random_device rd;
    // mt19937 gen(rd);
    normal_distribution<float> distribute(0.0, 1.0);
    // 轨迹高度
    float real_heig = posi[2];
    // 气压高度仿真
    float BA_heig = real_heig + real_heig * 0.002 * distribute(generator);
    // 卫星高度仿真（大地高）
    float GPS_heig = real_heig + 10 * distribute(generator);
    int row = 1200 - floor((posi[1] + 60) * 10);
    int col = floor(posi[0] * 10);
    float abh1 = abnrmal_heig_data[row * 3601 + col];
    float abh2 = abnrmal_heig_data[row * 3601 + col + 1];
    float abh3 = abnrmal_heig_data[(row - 1) * 3601 + col];
    float abh4 = abnrmal_heig_data[(row - 1) * 3601 + col + 1];
    float abha = abh1 + (abh2 - abh1) * ((posi[0] * 10 - floor(posi[0] * 10)) / 1);
    float abhb = abh3 + (abh4 - abh3) * ((posi[0] * 10 - floor(posi[0] * 10)) / 1);
    float abnromal_heig = abha + (abhb - abha) * ((posi[1] * 10 - floor(posi[1] * 10)) / 1);
    GPS_heig = GPS_heig + abnromal_heig;
    float veloGu = veloBu + 0.2 * distribute(generator);
    // 无线电高度仿真
    int posi_row_boundary = floor(posi[1]) + 1;
    int posi_col_boundary = floor(posi[0]);
    int posi_row = floor((posi_row_boundary - posi[1]) * 3600);
    int posi_col = floor((posi[0] - posi_col_boundary) * 3600);
    if (posi_row < 0)
    {
        posi_row = 0;
    }
    if (posi_row > 3600)
    {
        posi_row = 3600;
    }
    if (posi_col < 0)
    {
        posi_col = 0;
    }
    if (posi_col > 3600)
    {
        posi_col = 3600;
    }
    int height = heig_map_data[(posi_row) * 3601 + posi_col];
    float RA_heig = (real_heig - height) + 0.01 * distribute(generator) * (real_heig - height);

    vector<float> multi_simu_heig(4, 0.0); // 依次为气压高度仿真、卫星高度仿真、无线电高度仿真、GPS天向速度
    multi_simu_heig[0] = BA_heig;
    multi_simu_heig[1] = GPS_heig;
    multi_simu_heig[2] = RA_heig;
    multi_simu_heig[3] = veloGu;
    return multi_simu_heig;
}

// void Height_fusion::data_recv()
// {

// }

float Height_fusion::read_topographic_heig(float lon, float lat, vector<int16_t> &heig_map_data)
{
    float ans = 0;
    int posi_row_boundary = floor(lat) + 1;
    int posi_col_boundary = floor(lon);
    int posi_row = floor((posi_row_boundary - lat) * 3600);
    int posi_col = floor((lon - posi_col_boundary) * 3600);
    if (posi_row < 0)
    {
        posi_row = 0;
    }
    if (posi_row > 3600)
    {
        posi_row = 3600;
    }
    if (posi_col < 0)
    {
        posi_col = 0;
    }
    if (posi_col > 3600)
    {
        posi_col = 3600;
    }
    int height = heig_map_data[(posi_row) * 3601 + posi_col];
    ans = static_cast<float>(height);
    return ans;
}