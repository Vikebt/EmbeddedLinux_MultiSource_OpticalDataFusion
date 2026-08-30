#include "SITAN.h"

using namespace std;

void SITAN::Filter_parameter_init()
{
    this->mode = 0;         // SITAN初始化为搜索模式
    this->liner_method = 1; // 默认泰勒展开法
    this->tt = 0.02;
    // this->result_state=?;

    this->ins_lon = 0;
    this->ins_lat = 0;
    this->ins_heig = 0;
    this->ins_ve = 0;
    this->ins_vn = 0;

    this->BA = 0;
    this->RA = 0;

    this->tan_lon = 0;
    this->tan_lat = 0;
    this->tan_heig = 0;
    this->tan_lon_p = 0;
    this->tan_lat_p = 0;
    this->tan_heig_p = 0;

    this->swrs_max = 0;  // 残差最大值
    this->swrs_min = 0;  // 残差最小值
    this->swrs_mins = 0; // 残差次最小值
    this->n1 = 0;
    this->n2 = 0;
    this->n3 = 0; // 初始化计数器
    this->search_result = 0;
    this->track_result = 0;
    this->serch_radiu = 33; // 初始化搜索范围（半径）
    this->map_reso = 3600;
    this->x0_s = 0.0;
    // 我的参数
    this->p0_s = 225.0;
    //

    // QQ
    //  this->p0_s = 100.0;
    //
    this->swrs = 1.0;
    this->q0_s = 10;
    this->r0_s = 500;
    this->a_s = 0.01;
    // QQ
    //  this->p0_t[0] = 5.5;
    //  this->p0_t[1] = 5.5;
    //  this->p0_t[2] = 2.5;
    //  this->p0_t[3] = 1.0;
    //  this->p0_t[4] = 1.0;
    //

    // 我的
    this->p0_t[0] = 50;
    this->p0_t[1] = 50;
    this->p0_t[2] = 10;
    this->p0_t[3] = 1.0;
    this->p0_t[4] = 1.0;
    //
    // QQ的参数
    // this->q0_t[0] = 1.0;
    // this->q0_t[1] = 1.0;
    // this->q0_t[2] = 1.0;
    // this->q0_t[3] = 9.8e-4;
    // this->q0_t[4] = 9.8e-4;
    //

    // 我的参数
    this->q0_t[0] = 50.0;
    this->q0_t[1] = 50.0;
    this->q0_t[2] = 10.0;
    this->q0_t[3] = 1;
    this->q0_t[4] = 1;
    //

    // 我的参数
    this->r0_t = 15.0;
    //

    // QQ
    //  this->r0_t = 10.0;
    //

    this->a_t = 1.0;
    this->swrt = 0.001;
    this->H1 = 0.35;
    this->H2 = 0.08;
    this->N1 = 30;
    this->N2 = 50;
    this->minswrs = 2.5;
    this->extswrs = 0.3;
    this->max_n1 = 15;
    this->max_n2 = 10;
    this->max_n3 = 128;
    this->tac_n1 = 0;
    this->tac_n2 = 10;
    this->minswrs_t = 20.0;
    this->sigmax = 100;
    this->sigmay = 100;
    this->Initfilter();
}
// 搜索模式：搜索区域生成
void SITAN::Serch_map_creat(TERCOM &T)
{
    // int map_start_lon=floor(this->ins_lon);
    // int map_start_lat=floor(this->ins_lat)+1;
    // double cell=1.0/static_cast<double>(this->map_reso);
    // if (this->ins_lon >= map_start_lon && this->ins_lon <= (map_start_lon + 1) && this->ins_lat <= map_start_lat && this->ins_lat >= (map_start_lat - 1))
    // {
    //     double lon_temp_left = this->ins_lon - (this->serch_radiu * cell);
    //     double lon_temp_right = this->ins_lon + (this->serch_radiu * cell);
    //     double lat_temp_up = this->ins_lat + (this->serch_radiu * cell);
    //     double lat_temp_down = this->ins_lat - (this->serch_radiu * cell);
    // }
    int sq_size = T.Match_sequence.size();
    // cout<<"YES11?"<<sq_size<<endl;
    this->serch_map_center_lon = T.Match_sequence[sq_size - 1].ins_lon;
    // cout<<"YES12?"<<endl;
    this->serch_map_center_lat = T.Match_sequence[sq_size - 1].ins_lat;
    // cout<<"YES13?"<<endl;
    // 搜索区域中心坐标
    int serch_ter_radiu = static_cast<int>((T.Map_match[sq_size - 1].size() - 1) / 2);
    // cout<<"YES14?"<<endl;
    // 保证SITAN的搜索范围小于等于TERCOM范围
    if (this->serch_radiu <= serch_ter_radiu)
    {
        // cout<<this->serch_radiu<<endl;
        int len = 2 * this->serch_radiu + 1;
        // cout << len << endl;
        int dis = serch_ter_radiu - this->serch_radiu;
        // cout<<dis<<endl;
        this->serch_map.resize(len, vector<int16_t>(len, 0));
        // cout<<T.Map_match.size()<<" "<<T.Map_match[0].size()<<" "<<T.Map_match[0][0].size()<<endl;
        for (int i = 0; i < len; i++)
        {
            for (int j = 0; j < len; j++)
            {
                this->serch_map[i][j] = T.Map_match[sq_size - 1][dis + i][dis + j];
            }
        }
    }
}

// 搜索模式
void SITAN::Filter_serch()
{
    int rows = 2 * this->serch_radiu + 1;
    int cols = 2 * this->serch_radiu + 1;
    vector<vector<double>> z(rows, vector<double>(cols, 0.0));
    vector<vector<double>> p1(rows, vector<double>(cols, 0.0));
    vector<vector<double>> wrs(rows, vector<double>(cols, 0.0));
    vector<vector<double>> k(rows, vector<double>(cols, 0.0));
    double tt = this->tt; // 时间差，具体如何实现再议
    double x_0;
    int flag = 1;
    int min_row = 0;
    int min_col = 0;
    double swrs_lon = 0;
    double swrs_lat = 0;
    double cell = 1.0 / static_cast<double>(this->map_reso);

    for (int i = 0; i < rows; i++)
    {
        for (int j = 0; j < cols; j++)
        {
            z[i][j] = (this->ins_heig - this->RA) - static_cast<double>(this->serch_map[i][j]);
            p1[i][j] = this->p[i][j] + this->q0_s * tt;
            k[i][j] = p1[i][j] / (p1[i][j] + this->r0_s);
            x_0 = this->x[i][j];
            //
            this->x[i][j] = x_0 + k[i][j] * (z[i][j] - x_0); // 更新状态
            this->p[i][j] = (1 - k[i][j]) * p1[i][j];        // 更新协方差矩阵
            wrs[i][j] = ((z[i][j] - x_0) * (z[i][j] - x_0)) / (p1[i][j] + this->r0_s);
            this->sw[i][j] = this->a_s * wrs[i][j] + (1 - this->a_s) * this->sw[i][j];

            swrs_lat = this->serch_map_center_lat + (this->serch_radiu - i) * cell;
            swrs_lon = this->serch_map_center_lon + (j - this->serch_radiu) * cell;
            // 确定swrs的最大和最小值
            if (i == 0 && j == 0)
            {
                this->swrs_max = this->sw[i][j];
                this->swrs_min = this->sw[i][j];
                min_row = i;
                min_col = j;
                this->tan_lon = swrs_lon;
                this->tan_lat = swrs_lat;
                this->tan_heig = this->serch_map[i][j] + this->RA;
                this->result_state = 0;
            }
            if (this->sw[i][j] < this->swrs_min)
            {
                this->swrs_min = this->sw[i][j];
                min_row = i;
                min_col = j;
                this->tan_lon = swrs_lon;
                this->tan_lat = swrs_lat;
                this->tan_heig = this->serch_map[i][j] + this->RA;
                this->result_state = 0;
            }
            if (this->sw[i][j] > this->swrs_max)
            {
                this->swrs_max = this->sw[i][j];
            }
        }
    }
    // 确定swrs的次最小值
    for (int i = 0; i < rows; i++)
    {
        for (int j = 0; j < cols; j++)
        {
            if (i == min_row && j == min_col)
            {
                continue;
            }
            if (flag == 1)
            {
                flag = 0;
                this->swrs_mins = this->sw[i][j];
            }
            if (this->sw[i][j] < this->swrs_mins)
            {
                this->swrs_mins = this->sw[i][j];
            }
        }
    }
}

//
// 搜索模式是否切换为跟踪模式判断
void SITAN::Mode_serch_judge()
{
    if (this->swrs_min > this->minswrs)
    {
        this->n1++;
    }
    else
    {
        this->n1 = 0;
    }
    if ((this->swrs_max - this->swrs_min) < this->extswrs)
    {
        this->n2++;
    }
    else
    {
        this->n2 = 0;
    }
    this->n3++;
    if (this->n1 > this->max_n1) // n1超出阈值，滤波失败，重置滤波器
    {
        this->search_result = 2;
        this->track_result = -1;
        this->n1 = 0;
        this->result_state = 0;
        return;
    }
    if (this->n2 > this->max_n2) // n2超出阈值，继续执行搜索滤波
    {
        this->search_result = 1;
        this->track_result = -1;
        this->n2 = 0;
        // this->result_state=0;
        return;
    }
    if (this->n3 > this->max_n3) // n3超出阈值，继续执行搜索滤波
    {
        this->search_result = 1;
        this->track_result = -1;
        // this->result_state=0;
        this->n3 = 0;
        return;
    }
    if (((((this->swrs_mins - this->swrs_min) / this->swrs_min) > this->H1) && (this->n3 > this->N1)) || ((((this->swrs_mins - this->swrs_min) / this->swrs_min) > this->H2) && (this->n3 > this->N2)))
    {
        // 切换跟踪模式
        this->search_result = 0;
        this->track_result = -1;
        return;
    }
    // 依然搜索模式
    this->search_result = 1;
    this->track_result = -1;
}

void SITAN::Initfilter()
{
    int rows, cols;
    if (this->mode == 0)
    {
        rows = 2 * this->serch_radiu + 1;
        cols = 2 * this->serch_radiu + 1;
        vector<vector<double>> temp_x(rows, vector<double>(cols, 0.0));
        this->x = temp_x;
        vector<vector<double>> temp_p(rows, vector<double>(cols, 0.0));
        this->p = temp_p;
        vector<vector<double>> temp_sw(rows, vector<double>(cols, 0.0));
        this->sw = temp_sw;

        for (int i = 0; i < rows; i++)
        {
            for (int j = 0; j < cols; j++)
            {
                this->x[i][j] = this->x0_s;
                this->p[i][j] = this->p0_s;
                this->sw[i][j] = this->swrs;
            }
        }
    }
    if (this->mode == 1)
    {
        // this->x.resize(5, vector<double>(1, 0.0));
        // this->p.resize(5, vector<double>(5, 0.0));
        // this->sw.resize(1, vector<double>(1, 0.0));
        vector<vector<double>> temp_x(5, vector<double>(1, 0.0));
        this->x = temp_x;
        vector<vector<double>> temp_p(5, vector<double>(5, 0.0));
        this->p = temp_p;
        vector<vector<double>> temp_sw(1, vector<double>(1, 0.0));
        this->sw = temp_sw;
        // cout << "Initfilter初始化sw大小:" << this->sw.size() << "   " << this->sw[0].size() << endl;
        for (int i = 0; i < 5; i++)
        {
            this->p[i][i] = this->p0_t[i] * this->p0_t[i];
        }
    }
}

vector<double> SITAN::Terrain_liner()
{
    double h1, h2, h3, h4, h5, h6, cell, dis, M, N;
    vector<double> ans(2, 0.0); // ans[0]:hx,ans[1]=hy
    cell = 1.0 / static_cast<double>(this->map_reso);
    dis = Re * cell * pi / 180.0;
    int center_posi = this->serch_radiu;
    if (this->liner_method == 0)
    {
        h1 = this->serch_map[center_posi][center_posi + 1];
        h2 = this->serch_map[center_posi][center_posi - 1];
        ans[0] = (h1 - h2) / (2 * dis);
        h1 = this->serch_map[center_posi - 1][center_posi];
        h2 = this->serch_map[center_posi + 1][center_posi];
        ans[1] = (h1 - h2) / (2 * dis);
        // h1 = T.mapdata[0].Map[row * (this->map_reso + 1) + col + 1];
        // h2 = T.mapdata[0].Map[row * (this->map_reso + 1) + col - 1];
        // ans[0] = (h1 - h2) / (2 * dis);
        // h1 = T.mapdata[0].Map[(row + 1) * (this->map_reso + 1) + col]; // 低纬
        // h2 = T.mapdata[0].Map[(row - 1) * (this->map_reso + 1) + col]; // 高纬
        // ans[1] = (h1 - h2) / (2 * dis);
    }
    if (this->liner_method == 1) // 九点拟合
    {
        N = round(this->p[0][0] / cell);
        M = round(this->p[1][1] / cell);
        if (N < 1 || N > (1.5 * center_posi))
        {
            N = 1;
        }
        if (M < 1 || M > (1.5 * center_posi))
        {
            M = 1;
        }
        int M_temp = floor(1.5 * M);
        int N_temp = floor(1.5 * N);
        h1 = this->serch_map[center_posi - M_temp][center_posi - N_temp];
        h2 = this->serch_map[center_posi][center_posi - N_temp];
        h3 = this->serch_map[center_posi + M_temp][center_posi - N_temp];
        h4 = this->serch_map[center_posi - M_temp][center_posi + N_temp];
        h5 = this->serch_map[center_posi][center_posi + N_temp];
        h6 = this->serch_map[center_posi + M_temp][center_posi + N_temp];
        ans[0] = (h4 + h5 + h6 - h1 - h2 - h3) / (6 * N_temp * dis);
        h1 = this->serch_map[center_posi - M_temp][center_posi - N_temp];
        h2 = this->serch_map[center_posi - M_temp][center_posi];
        h3 = this->serch_map[center_posi - M_temp][center_posi + N_temp];
        h4 = this->serch_map[center_posi + M_temp][center_posi - N_temp];
        h5 = this->serch_map[center_posi + M_temp][center_posi];
        h6 = this->serch_map[center_posi + M_temp][center_posi + N_temp];
        ans[1] = (h4 + h5 + h6 - h1 - h2 - h3) / (6 * M_temp * dis);
        // h1 = T.mapdata[0].Map[row * (this->map_reso + 1) + col + 1];
        // h2 = T.mapdata[0].Map[row * (this->map_reso + 1) + col - 1];
        // ans[0] = (h1 - h2) / (2 * dis);
        // h1 = T.mapdata[0].Map[(row + 1) * (this->map_reso + 1) + col]; // 低纬
        // h2 = T.mapdata[0].Map[(row - 1) * (this->map_reso + 1) + col]; // 高纬
        // ans[1] = (h1 - h2) / (2 * dis);
    }
    return ans;
}

void SITAN::Filter_track(TERCOM &T)
{
    Eigen::MatrixXd phi = Eigen::MatrixXd::Zero(5, 5);
    Eigen::MatrixXd gama = Eigen::MatrixXd::Zero(5, 5);
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(1, 5);
    Eigen::MatrixXd x1 = Eigen::MatrixXd::Zero(5, 1);
    Eigen::MatrixXd p0 = Eigen::MatrixXd::Zero(5, 5);
    Eigen::MatrixXd q0 = Eigen::MatrixXd::Zero(5, 5);
    Eigen::MatrixXd x0 = Eigen::MatrixXd::Zero(5, 1);
    // Eigen::MatrixXd F = Eigen::MatrixXd::Zero(5, 5);
    Eigen::MatrixXd p1 = Eigen::MatrixXd::Zero(5, 5);
    Eigen::MatrixXd K = Eigen::MatrixXd::Zero(5, 1);
    Eigen::MatrixXd R = Eigen::MatrixXd::Zero(1, 1); // ？
    Eigen::MatrixXd M = Eigen::MatrixXd::Identity(5, 5);
    Eigen::MatrixXd z = Eigen::MatrixXd::Identity(1, 1);
    // cout << "YES-TRACK1" << endl;
    double tt = this->tt;
    R(0, 0) = this->r0_t * this->r0_t;
    // 初始化P，Q
    // for (int i = 0; i < 5; i++)
    // {
    //     p0(i, i) = this->p0_t[i] * this->p0_t[i];
    // }
    p0 = convertToeigenmatrix(this->p);
    for (int i = 0; i < 5; i++)
    {
        q0(i, i) = this->q0_t[i] * this->q0_t[i];
    }
    // cout << "YES-TRACK2" << endl;
    // 初始化PHI和GAMA
    for (int i = 0; i < 5; i++)
    {
        phi(i, i) = 1.0;
    }
    phi(0, 3) = tt;
    phi(1, 4) = tt;
    for (int i = 0; i < 5; i++)
    {
        gama(i, i) = tt;
    }
    gama(0, 3) = tt * tt / 2.0;
    gama(1, 4) = tt * tt / 2.0;
    // 初始化状态参数x0
    for (int i = 0; i < 5; i++)
    {
        x0(i, 0) = 0.0;
    }
    // cout << "YES-TRACK3" << endl;
    int row = floor((T.map_start_lat - this->tan_lat) * this->map_reso);
    int col = floor((this->tan_lon - T.map_start_lon) * this->map_reso);
    // 观测量z
    // z(0, 0) = (this->BA - this->RA - T.mapdata[0].Map[row * (this->map_reso + 1) + col]);
    z(0, 0) = (this->ins_heig - T.mapdata[0].Map[row * (this->map_reso + 1) + col] - this->RA);
    // 判断线性跟踪的地图是否需要更新

    // 观测矩阵H
    // 线性化
    vector<double> hxhy = this->Terrain_liner();
    H(0, 0) = -hxhy[0];
    H(0, 1) = -hxhy[1];
    H(0, 2) = 1.0;
    // cout << "YES-TRACK4" << endl;
    /*---------------Kalman---------------*/
    // 预测状态
    x1 = phi * x0;
    // cout << "YES-TRACK5" << endl;
    // 更新协方差矩阵p1
    //  F=phi*p0;
    //  p0=F*phi;
    //  F=gama*q0;
    //  p1=F*gama;
    p1 = phi * p0 * phi.transpose() + gama * q0 * gama.transpose();
    // cout << "YES-TRACK6" << endl;
    // 计算增益矩阵K
    K = p1 * H.transpose() * (H * p1 * H.transpose() + R).inverse();
    // cout << "YES-TRACK7" << endl;
    // R=H*p1;
    // //r=R*H;
    // K=p1*H;
    // 估计状态向量x
    Eigen::MatrixXd X = x1 - K * (z - H * x1);
    // cout << "YES-TRACK8" << endl;
    // 更新预测状态协方差矩阵
    // cout << M.size() << " " << K.size() << " " << H.size() << " " << p1.size() << " " << R.size() << endl;
    Eigen::MatrixXd temp1 = M - K * H;
    Eigen::MatrixXd P = temp1 * p1 * (temp1.transpose()) + K * R * (K.transpose());
    // Eigen::MatrixXd P = (M - K * H) * p1 * ((M - K * H).transpose()) + K * R * (K.transpose());
    // Eigen::MatrixXd pt1 = (M - K * H) * p1 * (M - K * H).transpose();
    // cout << "YES-TRACK12" << endl;
    // Eigen::MatrixXd pt2 = K * R * K.transpose();
    // cout << "YES-TRACK13" << endl;
    // Eigen::MatrixXd P = pt1 + pt2;
    // cout << "YES-TRACK9" << endl;
    // 上述计算中的R有很大的问题还没有解决。//貌似已经解决
    this->x = convertTovector(X);
    this->p = convertTovector(P);
    /*-------------------------------------------------*/
    Eigen::MatrixXd temp = z - H * x1;
    double swr = (temp(0, 0) * temp(0, 0)) / (P(2, 2) + this->r0_t);
    this->sw[0][0] = this->a_t * swr + (1 - this->a_t) * this->sw[0][0];
    // cout << "YES-TRACK10" << endl;
    this->swrs_min = swr;
    this->swrs_max = this->sw[0][0];
    // cout << "YES-TRACK11" << endl;

    // 更新解
    double scale = Re * pi / 180.0;
    this->tan_lon = this->ins_lon - (this->x[0][0] / scale);
    this->tan_lat = this->ins_lat + (this->x[1][0] / scale);

    // 修正高度？不需要
    this->tan_lon_p = sqrt(this->p[0][0]);
    this->tan_lat_p = sqrt(this->p[1][1]);
    this->tan_heig_p = sqrt(this->p[2][2]);
    this->result_state = 1;
    // cout << "YES-TRACK14" << endl;
    // cout << this->tan_lon << " " << this->tan_lat << endl;
}

void SITAN::Mode_track_judge()
{
    double sigmax, sigmay;
    this->n1++;
    if (this->swrs_max > this->minswrs_t)
    {
        this->n2++;
    }
    else
    {
        this->n2 = 0;
    }
    sigmax = this->tan_lon_p;
    sigmay = this->tan_lat_p;
    if (sigmax > this->sigmax || sigmay > this->sigmay)
    {
        this->search_result = -1;
        this->track_result = 1;
        this->result_state = 0;
        return;
    }

    if (this->n2 > this->tac_n2 || this->swrs_max > 1e4)
    {
        this->search_result = -1;
        this->track_result = 1;
        this->result_state = 0;
        return;
    }
    this->search_result = -1; // 继续跟踪滤波
    this->track_result = 0;
}

void SITAN::Method_sitan(TERCOM &T)
{
    switch (this->mode)
    {
    case 0:
    { // 搜索模式
        this->Filter_serch();
        this->Mode_serch_judge();
    }
    break;

    case 1:
    { // 跟踪模式
        this->Filter_track(T);
        this->Mode_track_judge();
    }
    break;
    }

    switch (this->search_result)
    {
    case 0:
    {
        this->mode = 1; // 切换跟踪模式
        this->search_result = -1;
        this->Initfilter();
        this->result_state = 1;
    }
    break;

    case 1:
    {
        this->mode = 0; // 搜索模式
        this->search_result = -1;
    }
    break;

    case 2:
    {
        this->mode = 0; // 搜索失败，重新初始化
        this->search_result = 0;
        this->n1 = 0;
        this->n2 = 0;
        this->n3 = 0;
        this->Initfilter();
    }
    break;
    }

    switch (this->track_result)
    {
    case 0:
    {
        this->mode = 1; // 进入跟踪模式
        this->track_result = -1;
    }
    break;

    case 1:
    {
        this->mode = 0; // 进入搜索模式
        this->track_result = -1;
        this->n1 = 0;
        this->n2 = 0;
        this->n3 = 0;
        this->Initfilter();
    }
    break;
    }
}

void SITAN::proctan(TERCOM &T)
{
    // int stat,flag;
    // 初始化操作//初始化操作应该在外部实现
    // this->Filter_parameter_init();
    // this->Serch_map_creat(T);
    //
    //
    // 感觉不需要这个函数。
    this->Method_sitan(T);
}
