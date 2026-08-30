#include "Kalman.h"

void Kalman_init(vector<double> &posiN, vector<vector<double>> &Xc, vector<vector<double>> &PK, vector<double> &Xerr)
{
    // double lon=0;
    // double lat=0;
    // double heig=0;
    // double Rm = 0;
    // double Rn = 0;
    vector<double> Xerr0(18, 0.0);
    // Xerr0.resize(18, 0);

    double lon = posiN[0] * pi / 180.0;
    double lat = posiN[1] * pi / 180.0;
    double heig = posiN[2];
    // 飞行器位置

    // 地球曲率半径
    double Rm = (1 - f) * (1 - f) * Re / (((1 - f) * (1 - f) * sin(lat) * sin(lat) + cos(lat) * cos(lat)) * sqrt((1 - f) * (1 - f) * sin(lat) * sin(lat) + cos(lat) * cos(lat)));
    double Rn = Re / sqrt((1 - f) * (1 - f) * sin(lat) * sin(lat) + cos(lat) * cos(lat));

    // 估计误差初值
    Xerr0[0] = (150.0 / 3600.0) * pi / 180.0;
    Xerr0[1] = (150.0 / 3600.0) * pi / 180.0;
    Xerr0[2] = (300.0 / 3600.0) * pi / 180.0;
    // 姿态误差
    Xerr0[3] = 0.6;
    Xerr0[4] = 0.6;
    Xerr0[5] = 0.6;
    // 速度误差
    Xerr0[6] = 50.0 / (Rm + heig);
    Xerr0[7] = 50.0 / (Rn + heig) / cos(lat);
    Xerr0[8] = 50.0;
    // 位置误差

    Xerr0[9] = 0.1 * pi / (3600.0 * 180.0);
    Xerr0[10] = 0.1 * pi / (3600.0 * 180.0);
    Xerr0[11] = 0.1 * pi / (3600.0 * 180.0);
    // 陀螺随即常数
    Xerr0[12] = 0.1 * pi / (3600.0 * 180.0);
    Xerr0[13] = 0.1 * pi / (3600.0 * 180.0);
    Xerr0[14] = 0.1 * pi / (3600.0 * 180.0);
    // 陀螺一阶马尔可夫
    Xerr0[15] = (1e-4) * g;
    Xerr0[16] = (1e-4) * g;
    Xerr0[17] = (1e-4) * g;
    // 加速度一阶马尔可夫

    // int numPK = PK.size();
    // PK.resize(numPK, vector<double>(numPK, 0.0));
    for (int i = 0; i < PK.size(); i++)
    {
        PK[i][i] = Xerr0[i] * Xerr0[i];
    }
    // for (int i = 0; i < numPK; i++)
    //{
    //	for (int j = 0; j < numPK; j++)
    //	{
    //		if (i == j)
    //		{
    //			PK[i][j] = Xerr0[i] * Xerr0[j];
    //		}
    //		else
    //		{
    //			PK[i][j] = 0;
    //		}
    //	}
    // }

    for (int i = 0; i < Xc.size(); i++)
    {
        Xc[i][0] = 0;
    }

    Xerr[0] = sqrt(PK[0][0]) * 180.0 * 3600.0 / pi; // deg
    Xerr[1] = sqrt(PK[1][1]) * 180.0 * 3600.0 / pi;
    Xerr[2] = sqrt(PK[2][2]) * 180.0 * 3600.0 / pi;
    Xerr[3] = sqrt(PK[3][3]); // m/s
    Xerr[4] = sqrt(PK[4][4]);
    Xerr[5] = sqrt(PK[5][5]);
    Xerr[6] = sqrt(PK[6][6]) * (Rm + heig); // m
    Xerr[7] = sqrt(PK[7][7]) * (Rn + heig) * cos(lat);
    Xerr[8] = sqrt(PK[8][8]);
    // INS的9个导航误差量

    Xerr[9] = sqrt(PK[9][9]) * 180.0 * 3600.0 / pi; // deg/h
    Xerr[10] = sqrt(PK[10][10]) * 180.0 * 3600.0 / pi;
    Xerr[11] = sqrt(PK[11][11]) * 180.0 * 3600.0 / pi;
    Xerr[12] = sqrt(PK[12][12]) * 180.0 * 3600.0 / pi;
    Xerr[13] = sqrt(PK[13][13]) * 180.0 * 3600.0 / pi;
    Xerr[14] = sqrt(PK[14][14]) * 180.0 * 3600.0 / pi;
    Xerr[15] = sqrt(PK[15][15]) / g; // g
    Xerr[16] = sqrt(PK[16][16]) / g;
    Xerr[17] = sqrt(PK[17][17]) / g;
    // IMU的9个误差量
}

void Kalman_TERCOM(vector<double> &posiN, vector<double> &posiT, vector<double> &attiN, vector<vector<double>> &Fb, vector<double> &veloN, vector<vector<double>> &PK, vector<vector<double>> &Xc, vector<double> &Xerr, double T_D, int Kalman_flag)
{
    double lon = posiN[0] * pi / 180.0;
    double lat = posiN[1] * pi / 180.0;
    double heig = posiN[2];

    double Rm = (1 - f) * (1 - f) * Re / (((1 - f) * (1 - f) * sin(lat) * sin(lat) + cos(lat) * cos(lat)) * sqrt((1 - f) * (1 - f) * sin(lat) * sin(lat) + cos(lat) * cos(lat)));
    double Rn = Re / sqrt((1 - f) * (1 - f) * sin(lat) * sin(lat) + cos(lat) * cos(lat));

    double roll = attiN[0] * pi / 180.0;
    double pitch = attiN[1] * pi / 180.0;
    double head = attiN[2] * pi / 180.0;

    vector<vector<double>> Cbn(3, vector<double>(3, 0.0));
    vector<vector<double>> CbnTurn;
    // Cbn.resize(3, vector<double>(3, 0.0));

    Cbn[0][0] = cos(roll) * cos(head) + sin(roll) * sin(pitch) * sin(head);
    Cbn[0][1] = -cos(roll) * sin(head) + sin(roll) * sin(pitch) * cos(head);
    Cbn[0][2] = -sin(roll) * cos(pitch);
    Cbn[1][0] = cos(pitch) * sin(head);
    Cbn[1][1] = cos(pitch) * cos(head);
    Cbn[1][2] = sin(pitch);
    Cbn[2][0] = sin(roll) * cos(head) - cos(roll) * sin(pitch) * sin(head);
    Cbn[2][1] = -sin(roll) * sin(head) - cos(roll) * sin(pitch) * cos(head);
    Cbn[2][2] = cos(roll) * cos(pitch);
    // 坐标系N-->B

    vector<vector<double>> Fn; // 3行1列
    // Fn.resize(3, 0);
    // MatTurn(Cbn,3,3,CbnTurn);
    CbnTurn = cal_mat_turn(Cbn);
    // MatMul(CbnTurn,Fb,3,3,1,Fn);
    Fn = cal_mat_mul(CbnTurn, Fb);
    // datatemp = Fn;
    double Ve = veloN[0];
    double Vn = veloN[1];
    double Vu = veloN[2];

    vector<vector<double>> FN(9, vector<double>(9, 0.0));
    // FN.resize(9);
    // for (int i = 0; i < 9; i++)
    //{
    //	FN[i].resize(9, 0);
    // }
    FN[0][1] = Wie * sin(lat) + Ve / (Rn + heig) * tan(lat);
    FN[0][2] = -(Wie * cos(lat) + Ve / (Rn + heig));
    FN[0][4] = -1.0 / (Rm + heig);

    FN[1][0] = -(Wie * sin(lat) + Ve / (Rn + heig) * tan(lat));
    FN[1][2] = -Vn / (Rm + heig);
    FN[1][3] = 1.0 / (Rn + heig);
    FN[1][6] = -Wie * sin(lat);

    FN[2][0] = Wie * cos(lat) + Ve / (Rn + heig);
    FN[2][1] = Vn / (Rm + heig);
    FN[2][3] = 1.0 / (Rn + heig) * tan(lat);
    FN[2][6] = Wie * cos(lat) + Ve / (Rn + heig) * (1.0 / cos(lat)) * (1.0 / cos(lat)); // sec(lati)*sec(lati)

    FN[3][1] = -Fn[2][0];
    FN[3][2] = Fn[1][0];
    FN[3][3] = Vn / (Rm + heig) * tan(lat) - Vu / (Rm + heig);
    FN[3][4] = 2.0 * Wie * sin(lat) + Ve / (Rn + heig) * tan(lat);
    FN[3][5] = -(2.0 * Wie * cos(lat) + Ve / (Rn + heig));
    FN[3][6] = 2.0 * Wie * cos(lat) * Vn + Ve * Vn / (Rn + heig) * (1.0 / cos(lat)) * (1.0 / cos(lat)) + 2 * Wie * sin(lat) * Vu; // sec(lati)*sec(lati)

    FN[4][0] = Fn[2][0];
    FN[4][2] = -Fn[0][0];
    FN[4][3] = -2.0 * (Wie * sin(lat) + Ve / (Rn + heig) * tan(lat));
    FN[4][4] = -Vu / (Rm + heig);
    FN[4][5] = -Vn / (Rm + heig);
    FN[4][6] = -(2.0 * Wie * cos(lat) + Ve / (Rn + heig) * (1.0 / cos(lat)) * (1.0 / cos(lat))) * Ve; // sec(lati)*sec(lati)

    FN[5][0] = -Fn[1][0];                                 // 61
    FN[5][1] = Fn[0][0];                                  // 62
    FN[5][3] = 2.0 * (Wie * cos(lat) + Ve / (Rn + heig)); // 64
    FN[5][4] = 2.0 * Vn / (Rm + heig);                    // 65
    FN[5][6] = -2.0 * Ve * Wie * sin(lat);                // 67
    FN[5][8] = 2.0 * g / Rn;                              // 增加与重力模型有关的此项，69

    FN[6][4] = 1.0 / (Rm + heig);

    FN[7][3] = (1.0 / cos(lat)) / (Rn + heig);
    FN[7][6] = Ve / (Rn + heig) * (1.0 / cos(lat)) * tan(lat);
    FN[8][5] = 1.0;

    vector<vector<double>> FS(9, vector<double>(9, 0.0));
    // FS.resize(9,vector<double>(9,0.0));

    // FS[0][0] = Cbn[0][0];
    // FS[0][1] = Cbn[1][0];
    // FS[0][2] = Cbn[2][0];
    // FS[1][0] = Cbn[0][1];
    // FS[1][1] = Cbn[1][1];
    // FS[1][2] = Cbn[2][1];
    // FS[2][0] = Cbn[0][2];
    // FS[2][1] = Cbn[1][2];
    // FS[2][2] = Cbn[2][2];

    // FS[0][3] = Cbn[0][0];
    // FS[0][4] = Cbn[1][0];
    // FS[0][5] = Cbn[2][0];
    // FS[1][3] = Cbn[0][1];
    // FS[1][4] = Cbn[1][1];
    // FS[1][5] = Cbn[2][1];
    // FS[2][3] = Cbn[0][2];
    // FS[2][4] = Cbn[1][2];
    // FS[2][5] = Cbn[2][2];

    // FS[3][6] = Cbn[0][0];
    // FS[3][7] = Cbn[1][0];
    // FS[3][8] = Cbn[2][0];
    // FS[4][6] = Cbn[0][1];
    // FS[4][7] = Cbn[1][1];
    // FS[4][8] = Cbn[2][1];
    // FS[5][6] = Cbn[0][2];
    // FS[5][7] = Cbn[1][2];
    // FS[5][8] = Cbn[2][2];

    FS[0][0] = CbnTurn[0][0];
    FS[0][1] = CbnTurn[0][1];
    FS[0][2] = CbnTurn[0][2];
    FS[1][0] = CbnTurn[1][0];
    FS[1][1] = CbnTurn[1][1];
    FS[1][2] = CbnTurn[1][2];
    FS[2][0] = CbnTurn[2][0];
    FS[2][1] = CbnTurn[2][1];
    FS[2][2] = CbnTurn[2][2];

    FS[0][3] = CbnTurn[0][0];
    FS[0][4] = CbnTurn[0][1];
    FS[0][5] = CbnTurn[0][2];
    FS[1][3] = CbnTurn[1][0];
    FS[1][4] = CbnTurn[1][1];
    FS[1][5] = CbnTurn[1][2];
    FS[2][3] = CbnTurn[2][0];
    FS[2][4] = CbnTurn[2][1];
    FS[2][5] = CbnTurn[2][2];

    FS[3][6] = CbnTurn[0][0];
    FS[3][7] = CbnTurn[0][1];
    FS[3][8] = CbnTurn[0][2];
    FS[4][6] = CbnTurn[1][0];
    FS[4][7] = CbnTurn[1][1];
    FS[4][8] = CbnTurn[1][2];
    FS[5][6] = CbnTurn[2][0];
    FS[5][7] = CbnTurn[2][1];
    FS[5][8] = CbnTurn[2][2];

    double Tgx = 3600.0;
    double Tgy = 3600.0;
    double Tgz = 3600.0;
    double Tax = 1800.0;
    double Tay = 1800.0;
    double Taz = 1800.0;

    vector<vector<double>> FM(9, vector<double>(9, 0.0));
    FM[0][0] = 0.0;
    FM[1][1] = 0.0;
    FM[2][2] = 0.0;
    FM[3][3] = -1.0 / Tgx;
    FM[4][4] = -1.0 / Tgy;
    FM[5][5] = -1.0 / Tgz;
    FM[6][6] = -1.0 / Tax;
    FM[7][7] = -1.0 / Tay;
    FM[8][8] = -1.0 / Taz;

    vector<vector<double>> FI(18, vector<double>(18, 0.0));

    for (int i = 0; i < 9; i++)
    {
        for (int j = 0; j < 9; j++)
        {
            FI[i][j] = FN[i][j];
        }
    }

    for (int i = 0; i < 9; i++)
    {
        for (int j = 9; j < 18; j++)
        {
            FI[i][j] = FS[i][j - 9];
        }
    }

    for (int i = 9; i < 18; i++)
    {
        for (int j = 9; j < 18; j++)
        {
            FI[i][j] = FM[i - 9][j - 9];
        }
    }

    vector<vector<double>> GI(18, vector<double>(9, 0.0));
    // GI[0][0] = Cbn[0][0];
    // GI[0][1] = Cbn[1][0];
    // GI[0][2] = Cbn[2][0];
    // GI[1][0] = Cbn[0][1];
    // GI[1][1] = Cbn[1][1];
    // GI[1][2] = Cbn[2][1];
    // GI[2][0] = Cbn[0][2];
    // GI[2][1] = Cbn[1][2];
    // GI[2][2] = Cbn[2][2];

    GI[0][0] = CbnTurn[0][0];
    GI[0][1] = CbnTurn[0][1];
    GI[0][2] = CbnTurn[0][2];
    GI[1][0] = CbnTurn[1][0];
    GI[1][1] = CbnTurn[1][1];
    GI[1][2] = CbnTurn[1][2];
    GI[2][0] = CbnTurn[2][0];
    GI[2][1] = CbnTurn[2][1];
    GI[2][2] = CbnTurn[2][2];

    GI[12][3] = 1.0;
    GI[13][4] = 1.0;
    GI[14][5] = 1.0;
    GI[15][6] = 1.0;
    GI[16][7] = 1.0;
    GI[17][8] = 1.0;
    // print_message(FI);
    // print_message(GI);

    vector<vector<double>> Unit_I(18, vector<double>(18, 0.0));
    // 自写函数
    // vector<vector<double>>FII;
    // vector<vector<double>>GLL(18, vector<double>(18, 0.0));
    // vector<vector<double>>FL(18, vector<double>(18, 0.0));
    // vector<vector<double>>GL;
    for (int i = 0; i < 18; i++)
    {
        Unit_I[i][i] = 1.0;
    }
    // 利用Eigen
    Eigen::MatrixXd I = convertToeigenmatrix(Unit_I);
    Eigen::MatrixXd FI_I = convertToeigenmatrix(FI);
    Eigen::MatrixXd GI_I = convertToeigenmatrix(GI);
    Eigen::MatrixXd FL = I + FI_I * T_D + FI_I * FI_I / 2.0 * T_D * T_D;
    Eigen::MatrixXd GL = (I + FI_I / 2.0 * T_D + FI_I * FI_I / 6.0 * T_D * T_D) * GI_I * T_D;

    // 自写函数
    // FII = cal_mat_mul(FI, FI);
    // for (int i = 0; i < 18; i++)
    //{
    //	for (int j = 0; j < 18; j++)
    //	{
    //		FL[i][j] = Unit_I[i][j] + FI[i][j] * T_D + FII[i][j] / 2.0 * T_D * T_D;
    //		GLL[i][j] = (Unit_I[i][j] + FI[i][j] / 2.0 * T_D + FII[i][j] / 6.0 * T_D * T_D) * T_D;
    //	}
    // }
    // GL = cal_mat_mul(GLL, GI);

    vector<double> W(9, 0.0);
    W[0] = 01 * pi / (3600.0 * 180.0);
    W[1] = 0.1 * pi / (3600.0 * 180.0);
    W[2] = 0.1 * pi / (3600.0 * 180.0);
    W[3] = sqrt(2 * T_D / Tgx) * 0.1 * pi / (3600.0 * 180.0);
    W[4] = sqrt(2 * T_D / Tgy) * 0.1 * pi / (3600.0 * 180.0);
    W[5] = sqrt(2 * T_D / Tgz) * 0.1 * pi / (3600.0 * 180.0);
    W[6] = sqrt(2 * T_D / Tax) * (1e-4) * g;
    W[7] = sqrt(2 * T_D / Tay) * (1e-4) * g;
    W[8] = sqrt(2 * T_D / Taz) * (1e-4) * g;

    vector<vector<double>> HG(3, vector<double>(18, 0.0));
    vector<double> VG(3, 0.0);
    VG[0] = 100.0;
    VG[1] = 100.0;
    VG[2] = 50.0;
    // VG[3] = 0.2;
    // VG[4] = 0.2;
    // VG[5] = 0.2;

    HG[0][6] = Rm;
    HG[1][7] = Rn * cos(lat);
    HG[2][8] = 1.0;
    // HG[3][3] = 1.0;
    // HG[4][4] = 1.0;
    // HG[5][5] = 1.0;

    Eigen::MatrixXd HG_G = convertToeigenmatrix(HG);

    vector<vector<double>> Q(9, vector<double>(9, 0.0));
    vector<vector<double>> RG(3, vector<double>(3, 0.0));
    for (int i = 0; i < 9; i++)
    {
        Q[i][i] = W[i] * W[i];
    }
    for (int i = 0; i < 3; i++)
    {
        RG[i][i] = VG[i] * VG[i];
    }
    // EIGEN
    Eigen::MatrixXd Q_Q = convertToeigenmatrix(Q);
    Eigen::MatrixXd RG_G = convertToeigenmatrix(RG);
    Eigen::MatrixXd PK_K = convertToeigenmatrix(PK);
    PK_K = FL * PK_K * FL.transpose() + GL * Q_Q * GL.transpose();
    // 自写函数
    // vector<vector<double>>FL_turn;
    // FL_turn = cal_mat_turn(FL);
    // vector<vector<double>> temp1;//temp1=FL*PK;
    // temp1 = cal_mat_mul(FL, PK);
    // vector<vector<double>>temp2;//temp2=(FL*PK)*FL'
    // temp2 = cal_mat_mul(temp1, FL_turn);

    // vector<vector<double>>GL_turn;
    // GL_turn = cal_mat_turn(GL);
    // vector<vector<double>>temp3;//temp3=GL*Q
    // temp3 = cal_mat_mul(GL, Q);
    // vector<vector<double>>temp4;//temp4=(GL*Q)*GL'
    // temp4 = cal_mat_mul(temp3, GL_turn);

    // PK = cal_mat_add(temp2, temp4);
    vector<vector<double>> Yc(3, vector<double>(1, 0.0));

    if (Kalman_flag == 1) // Kaiman
    {
        // 自写函数
        // vector<vector<double>>temp5;//temp5=HG*PK;
        // temp5 = cal_mat_mul(HG, PK);
        // vector<vector<double>>HG_turn;
        // HG_turn = cal_mat_turn(HG);
        // vector<vector<double>>temp6;//temp6=(HG*PK)*HG'
        // temp6 = cal_mat_mul(temp5, HG_turn);
        // vector<vector<double>>temp7;//temp7=((HG*PK)*HG')+RG
        // temp7 = cal_mat_add(temp6, RG);
        // vector<vector<double>>temp8;//temp8=inv(((HG*PK)*HG')+RG)
        // temp8 = cal_mat_inv(temp7);
        // vector<vector<double>>temp9;//temp9=PK*HG'
        // temp9 = cal_mat_mul(PK, HG_turn);
        // vector<vector<double>>KK;
        // KK = cal_mat_mul(temp9, temp8);
        //
        // vector<vector<double>>KK_turn;
        // KK_turn = cal_mat_turn(KK);
        // vector<vector<double>>temp10;//temp10=KK*HG
        // temp10 = cal_mat_mul(KK, HG);
        // vector<vector<double>>temp11;//temp11=I-(KK*HG)
        // temp11 = cal_mat_sub(Unit_I, temp10);
        ////vector<vector<double>>temp12;//temp12=inv(I-(KK*HG))
        ////temp12 = cal_mat_inv(temp11);
        // vector<vector<double>>temp12;//temp12= (I - (KK * HG))'
        // temp12 = cal_mat_turn(temp11);
        // vector<vector<double>>temp13;//temp13=(I-(KK*HG))*PK
        // temp13 = cal_mat_mul(temp11, PK);
        // vector<vector<double>>temp14;//temp14=(I-(KK*HG))*PK*(I - (KK * HG))'
        // temp14 = cal_mat_mul(temp13, temp12);
        // vector<vector<double>>temp15;//temp15=KK*RG;
        // temp15 = cal_mat_mul(KK, RG);
        // vector<vector<double>>temp16;//temp16=(KK*RG)*KK'
        // temp16 = cal_mat_mul(temp15, KK_turn);
        // PK = cal_mat_add(temp14, temp16);

        // eigen
        Eigen::MatrixXd KK = PK_K * HG_G.transpose() * (HG_G * PK_K * HG_G.transpose() + RG_G).inverse();
        // Eigen::MatrixXd data_t = (HG_G * PK_K * HG_G.transpose() + RG_G).inverse();
        // datatemp = convertTovector(data_t);
        PK_K = (I - KK * HG_G) * PK_K * (I - KK * HG_G).transpose() + KK * RG_G * KK.transpose();

        Yc[0][0] = (posiN[1] - posiT[1]) * pi / 180.0 * (Rm + heig);
        Yc[1][0] = (posiN[0] - posiT[0]) * pi / 180.0 * (Rn + heig) * cos(lat);
        Yc[2][0] = posiN[2] - posiT[2];
        // Yc[3][0] = veloN[0] - veloG[0];
        // Yc[4][0] = veloN[1] - veloG[1];
        // Yc[5][0] = veloN[2] - veloG[2];

        Eigen::MatrixXd Yc_c = convertToeigenmatrix(Yc);

        // 自写函数
        // vector<vector<double>>Xc_temp;
        // Xc_temp = cal_mat_mul(FL, Xc);
        // vector<vector<double>>temp17;//temp17=HG*Xc
        // temp17 = cal_mat_mul(HG, Xc_temp);
        // vector<vector<double>>temp18;//temp18=Yc-HG*Xc
        // temp18 = cal_mat_sub(Yc, temp17);
        // vector<vector<double>>temp19;//temp19=KK*(Yc-HG*Xc)
        // temp19 = cal_mat_mul(KK, temp18);
        // Xc = cal_mat_add(Xc_temp, temp19);
        Eigen::MatrixXd Xc_c = convertToeigenmatrix(Xc);
        Xc_c = FL * Xc_c;
        Xc_c = Xc_c + KK * (Yc_c - HG_G * Xc_c);
        Xc = convertTovector(Xc_c);
        PK = convertTovector(PK_K);
        // print_message(Xc);
        // print_message(PK);
    }
    Xerr[0] = sqrt(PK[0][0]) * 180.0 / pi; // deg
    Xerr[1] = sqrt(PK[1][1]) * 180.0 / pi;
    Xerr[2] = sqrt(PK[2][2]) * 180.0 / pi;
    Xerr[3] = sqrt(PK[3][3]); // m/s
    Xerr[4] = sqrt(PK[4][4]);
    Xerr[5] = sqrt(PK[5][5]);
    Xerr[6] = sqrt(PK[6][6]) * (Rm + heig); // m
    Xerr[7] = sqrt(PK[7][7]) * (Rn + heig) * cos(lat);
    Xerr[8] = sqrt(PK[8][8]);
    // INS的9个导航误差量

    Xerr[9] = sqrt(PK[9][9]) * 180.0 * 3600.0 / pi; // deg/h
    Xerr[10] = sqrt(PK[10][10]) * 180.0 * 3600.0 / pi;
    Xerr[11] = sqrt(PK[11][11]) * 180.0 * 3600.0 / pi;
    Xerr[12] = sqrt(PK[12][12]) * 180.0 * 3600.0 / pi;
    Xerr[13] = sqrt(PK[13][13]) * 180.0 * 3600.0 / pi;
    Xerr[14] = sqrt(PK[14][14]) * 180.0 * 3600.0 / pi;
    Xerr[15] = sqrt(PK[15][15]) / g; // g
    Xerr[16] = sqrt(PK[16][16]) / g;
    Xerr[17] = sqrt(PK[17][17]) / g;
    // IMU的9个误差量
}

void Kalman_TERCOM_modi(vector<double> &attiN, vector<double> &veloN, vector<double> &posiN, vector<vector<double>> &Xc)
{
    int kc = 0;
    double std_gain = 1;

    double lon = posiN[0] * pi / 180.0;
    double lat = posiN[1] * pi / 180.0;
    double heig = posiN[2];

    double Rm = (1 - f) * (1 - f) * Re / (((1 - f) * (1 - f) * sin(lat) * sin(lat) + cos(lat) * cos(lat)) * sqrt((1 - f) * (1 - f) * sin(lat) * sin(lat) + cos(lat) * cos(lat)));
    double Rn = Re / sqrt((1 - f) * (1 - f) * sin(lat) * sin(lat) + cos(lat) * cos(lat));

    veloN[0] = veloN[0] - Xc[3][0];
    veloN[1] = veloN[1] - Xc[4][0];
    veloN[2] = veloN[2] - Xc[5][0];

    posiN[0] = posiN[0] - Xc[7][0] * 180.0 / pi;
    posiN[1] = posiN[1] - Xc[6][0] * 180.0 / pi;
    //posiN[2] = posiN[2] - Xc[8][0];

    double roll = attiN[0] * pi / 180.0;
    double pitch = attiN[1] * pi / 180.0;
    double head = attiN[2] * pi / 180.0;

    vector<vector<double>> Cbc(3, vector<double>(3, 0.0));
    vector<vector<double>> Ccn(3, vector<double>(3, 0.0));

    Cbc[0][0] = cos(roll) * cos(head) + sin(roll) * sin(pitch) * sin(head);
    Cbc[0][1] = -cos(roll) * sin(head) + sin(roll) * sin(pitch) * cos(head);
    Cbc[0][2] = -sin(roll) * cos(pitch);
    Cbc[1][0] = cos(pitch) * sin(head);
    Cbc[1][1] = cos(pitch) * cos(head);
    Cbc[1][2] = sin(pitch);
    Cbc[2][0] = sin(roll) * cos(head) - cos(roll) * sin(pitch) * sin(head);
    Cbc[2][1] = -sin(roll) * sin(head) - cos(roll) * sin(pitch) * cos(head);
    Cbc[2][2] = cos(roll) * cos(pitch);
    // 坐标系C-->B

    Ccn[0][0] = 1.0;
    Ccn[0][1] = Xc[2][0];
    Ccn[0][2] = -Xc[1][0];
    Ccn[1][0] = -Xc[2][0];
    Ccn[1][1] = 1.0;
    Ccn[1][2] = Xc[0][0];
    Ccn[2][0] = Xc[1][0];
    Ccn[2][1] = -Xc[0][0];
    Ccn[2][2] = 1.0;
    // 修正矩阵N-->C

    Eigen::MatrixXd Cbc_c = convertToeigenmatrix(Cbc);
    Eigen::MatrixXd Ccn_n = convertToeigenmatrix(Ccn);
    Eigen::MatrixXd Cbn_n = (Ccn_n.transpose() * Cbc_c.transpose()).transpose();
    vector<vector<double>> Cbn = convertTovector(Cbn_n);
    // datatemp = Cbn;
    // cout << datatemp.size() << endl;
    // vector<vector<double>>Cbc_turn;
    // vector<vector<double>>Ccn_turn;
    // vector<vector<double>>temp1;//temp1=Ccn'*Cbc'
    // Cbc_turn = cal_mat_turn(Cbc);
    // Ccn_turn = cal_mat_turn(Ccn);
    // temp1 = cal_mat_mul(Ccn_turn, Cbc_turn);
    // vector<vector<double>>Cbn;
    // Cbn = cal_mat_turn(temp1);

    attiN[0] = atan((-Cbn[0][2]) / Cbn[2][2]);
    attiN[1] = atan(Cbn[1][2] / sqrt(Cbn[1][0] * Cbn[1][0] + Cbn[1][1] * Cbn[1][1]));
    attiN[2] = atan(Cbn[1][0] / Cbn[1][1]);

    attiN[0] = attiN[0] * 180.0 / pi;
    attiN[1] = attiN[1] * 180.0 / pi;
    attiN[2] = attiN[2] * 180.0 / pi;

    // if (Cbn[1][1] < 0)
    //	attiN[2] = 180.0 + attiN[2];
    // else if (Cbn[1][0] < 0) attiN[2] = 360.0 + attiN[2];

    // if (Cbn[2 ][ 2] < 0) {
    //	if (Cbn[0 ][ 2] > 0) attiN[0] = 180.0 - attiN[0];
    //	if (Cbn[0 ][ 2] < 0) attiN[0] = -(180.0 + attiN[0]);
    // }
    if (Cbn[1][1] < 0)
    {
        attiN[2] = 180.0 + attiN[2];
    }
    else
    {
        if (Cbn[1][0] < 0)
        {
            attiN[2] = 360.0 + attiN[2];
        }
    }

    if (Cbn[2][2] < 0)
    {
        if (Cbn[0][2] > 0)
        {
            attiN[0] = 180.0 - attiN[0];
        }
        if (Cbn[0][2] < 0)
        {
            attiN[0] = -(180.0 + attiN[0]);
        }
    }
}