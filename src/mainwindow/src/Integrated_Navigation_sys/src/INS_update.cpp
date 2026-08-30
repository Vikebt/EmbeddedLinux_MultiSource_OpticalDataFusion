#include "INS_update.h"
#include "Mat_Math.h"
// #include<random>
// #include<chrono>

void atti_cal_cq_modi(vector<double> &posiN, vector<double> &attiN, vector<double> &veloN, vector<vector<double>> &Wibb, vector<vector<double>> &WnbbA_old, double T)
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
	// vector<vector<double>>CbnTurn;
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

	double Ve = veloN[0];
	double Vn = veloN[1];
	double Vu = veloN[2];

	vector<vector<double>> Wenn(3, vector<double>(1, 0.0));
	vector<vector<double>> Wien(3, vector<double>(1, 0.0));

	Wenn[0][0] = -Vn / (Rm + heig);
	Wenn[1][0] = Ve / (Rn + heig);
	Wenn[2][0] = Ve / (Rn + heig) * tan(lat);

	Wien[0][0] = 0.0;
	Wien[1][0] = Wie * cos(lat);
	Wien[2][0] = Wie * sin(lat);

	Eigen::MatrixXd Cbn_n = convertToeigenmatrix(Cbn);
	Eigen::MatrixXd Wibb_b = convertToeigenmatrix(Wibb);
	Eigen::MatrixXd Wenn_n = convertToeigenmatrix(Wenn);
	Eigen::MatrixXd Wien_n = convertToeigenmatrix(Wien);
	Eigen::MatrixXd Wnbb = Wibb_b * pi / 180.0 - Cbn_n * (Wien_n + Wenn_n);

	////自写函数
	// vector<vector<double>> Wnbb;
	// vector<vector<double>>temp1;//temp1=Wien+Wenn
	// temp1 = cal_mat_add(Wien, Wenn);
	// vector<vector<double>>temp2;//temp2=Cbn*(Wien*Wenn)
	// temp2 = cal_mat_mul(Cbn, temp1);
	// vector<vector<double>>temp3(3, vector<double>(1, 0.0));//temp3=Wibb*pi/180
	// temp3[0][0] = Wibb[0][0] * pi / 180.0;
	// temp3[1][0] = Wibb[1][0] * pi / 180.0;
	// temp3[2][0] = Wibb[2][0] * pi / 180.0;
	// Wnbb = cal_mat_sub(temp3, temp2);

	// 四元数求姿态矩阵
	vector<vector<double>> Q(4, vector<double>(1, 0.0));
	Q[0][0] = cos(head / 2) * cos(pitch / 2) * cos(roll / 2) + sin(head / 2) * sin(pitch / 2) * sin(roll / 2);
	Q[1][0] = cos(head / 2) * sin(pitch / 2) * cos(roll / 2) + sin(head / 2) * cos(pitch / 2) * sin(roll / 2);
	Q[2][0] = cos(head / 2) * cos(pitch / 2) * sin(roll / 2) - sin(head / 2) * sin(pitch / 2) * cos(roll / 2);
	Q[3][0] = -1.0 * sin(head / 2) * cos(pitch / 2) * cos(roll / 2) + cos(head / 2) * sin(pitch / 2) * sin(roll / 2);

	Eigen::MatrixXd Q_Q = convertToeigenmatrix(Q);
	Eigen::MatrixXd WnbbA = Wnbb * T;

	// vector<vector<double>> WnbbA(3,vector<double>(1,0.0));
	// WnbbA[0][0] = Wnbb[0][0] * T;
	// WnbbA[1][0] = Wnbb[1][0] * T;
	// WnbbA[2][0] = Wnbb[2][0] * T;
	// double WnbbA0 = sqrt(WnbbA[0][0] * WnbbA[0][0] + WnbbA[1][0] * WnbbA[1][0] + WnbbA[2][0] * WnbbA[2][0]);
	double WnbbA0 = sqrt(WnbbA(0, 0) * WnbbA(0, 0) + WnbbA(1, 0) * WnbbA(1, 0) + WnbbA(2, 0) * WnbbA(2, 0));

	vector<vector<double>> WnbbX(4, vector<double>(4, 0.0));
	////自写函数
	// WnbbX[0][1] = -WnbbA[0][0];
	// WnbbX[0][2] = -WnbbA[1][0];
	// WnbbX[0][3] = -WnbbA[2][0];

	// WnbbX[1][0] = WnbbA[0][0];
	// WnbbX[1][2] = WnbbA[2][0];
	// WnbbX[1][3] = -WnbbA[1][0];

	// WnbbX[2][0] = WnbbA[1][0];
	// WnbbX[2][1] = -WnbbA[2][0];
	// WnbbX[2][3] = WnbbA[0][0];

	////WnbbX[2][0] = WnbbA[2][0];
	////WnbbX[2][1] = -WnbbA[1][0];
	////WnbbX[2][3] = WnbbA[0][0];

	// WnbbX[3][0] = WnbbA[2][0];
	// WnbbX[3][1] = WnbbA[1][0];
	// WnbbX[3][2] = -WnbbA[0][0];

	WnbbX[0][1] = -WnbbA(0, 0);
	WnbbX[0][2] = -WnbbA(1, 0);
	WnbbX[0][3] = -WnbbA(2, 0);

	WnbbX[1][0] = WnbbA(0, 0);
	WnbbX[1][2] = WnbbA(2, 0);
	WnbbX[1][3] = -WnbbA(1, 0);

	WnbbX[2][0] = WnbbA(1, 0);
	WnbbX[2][1] = -WnbbA(2, 0);
	WnbbX[2][3] = WnbbA(0, 0);

	WnbbX[3][0] = WnbbA(2, 0);
	WnbbX[3][1] = WnbbA(1, 0);
	WnbbX[3][2] = -WnbbA(0, 0);

	Eigen::MatrixXd WnbbX_X = convertToeigenmatrix(WnbbX);

	double c_q = cos(WnbbA0 / 2);
	// c_q = cos(WnbbA0 / 2.0);
	double d_q = 0.0;
	if (WnbbA0 <= 1.0e-15)
	{
		d_q = 0.5;
	}
	else
	{
		d_q = sin(WnbbA0 / 2.0) / WnbbA0;
	}

	Eigen::MatrixXd WnbbA_old_old = convertToeigenmatrix(WnbbA_old);
	// Eigen::MatrixXd WnbbA_e = WnbbA_old_old.cross(WnbbA);

	////自写函数
	vector<vector<double>> WnbbA_old_temp(3, vector<double>(3, 0.0));
	WnbbA_old_temp[0][1] = -WnbbA_old[2][0];
	WnbbA_old_temp[0][2] = WnbbA_old[1][0];
	WnbbA_old_temp[1][0] = WnbbA_old[2][0];
	WnbbA_old_temp[1][2] = -WnbbA_old[0][0];
	WnbbA_old_temp[2][0] = -WnbbA_old[1][0];
	WnbbA_old_temp[2][1] = WnbbA_old[0][0];
	// vector<vector<double>>WnbbA_e;
	// WnbbA_e = cal_mat_mul(WnbbA_old_temp, WnbbA);
	Eigen::MatrixXd Wnbba_old_temo_temo = convertToeigenmatrix(WnbbA_old_temp);
	Eigen::MatrixXd WnbbA_e = Wnbba_old_temo_temo * WnbbA;

	vector<vector<double>> WnbbX_e(4, vector<double>(4, 0.0));
	// WnbbX_e[0][1] = -WnbbA_e[0][0];
	// WnbbX_e[0][2] = -WnbbA_e[1][0];
	// WnbbX_e[0][3] = -WnbbA_e[2][0];

	// WnbbX_e[1][0] = WnbbA_e[0][0];
	// WnbbX_e[1][2] = WnbbA_e[2][0];
	// WnbbX_e[1][3] = -WnbbA_e[1][0];

	// WnbbX_e[2][0] = WnbbA_e[1][0];
	// WnbbX_e[2][1] = -WnbbA_e[2][0];
	// WnbbX_e[2][3] = WnbbA_e[0][0];

	// WnbbX_e[3][0] = WnbbA_e[2][0];
	// WnbbX_e[3][1] = WnbbA_e[1][0];
	// WnbbX_e[3][2] = -WnbbA_e[0][0];

	WnbbX_e[0][1] = -WnbbA_e(0, 0);
	WnbbX_e[0][2] = -WnbbA_e(1, 0);
	WnbbX_e[0][3] = -WnbbA_e(2, 0);

	WnbbX_e[1][0] = WnbbA_e(0, 0);
	WnbbX_e[1][2] = WnbbA_e(2, 0);
	WnbbX_e[1][3] = -WnbbA_e(1, 0);

	WnbbX_e[2][0] = WnbbA_e(1, 0);
	WnbbX_e[2][1] = -WnbbA_e(2, 0);
	WnbbX_e[2][3] = WnbbA_e(0, 0);

	WnbbX_e[3][0] = WnbbA_e(2, 0);
	WnbbX_e[3][1] = WnbbA_e(1, 0);
	WnbbX_e[3][2] = -WnbbA_e(0, 0);

	Eigen::MatrixXd WnbbX_e_e = convertToeigenmatrix(WnbbX_e);

	Eigen::MatrixXd Unit_I = Eigen::MatrixXd::Identity(4, 4);

	Q_Q = (c_q * Unit_I + d_q * (WnbbX_X + 1 / 12 * WnbbX_e_e)) * Q_Q;

	WnbbA_old_old = WnbbA;

	// Q[0][0] = (c_q + d_q * (WnbbX[0][0] + 1.0 / 12.0 * WnbbX_e[0][0])) * Q[0][0] + d_q * (WnbbX[0][1] + 1.0 / 12.0 * WnbbX_e[0][1]) * Q[1][0] + d_q * (WnbbX[0][2] + 1.0 / 12.0 * WnbbX_e[0][2]) * Q[2][0] + d_q * (WnbbX[0][3] + 1.0 / 12.0 * WnbbX_e[0][3]) * Q[3][0];
	// Q[1][0] = d_q * (WnbbX[1][0] + 1.0 / 12.0 * WnbbX_e[1][0]) * Q[0][0] + (c_q + d_q * (WnbbX[1][1] + 1.0 / 12.0 * WnbbX_e[1][1])) * Q[1][0] + d_q * (WnbbX[1][2] + 1.0 / 12.0 * WnbbX_e[1][2]) * Q[2][0] + d_q * (WnbbX[1][3] + 1.0 / 12.0 * WnbbX_e[1][3]) * Q[3][0];
	// Q[2][0] = d_q * (WnbbX[2][0] + 1.0 / 12.0 * WnbbX_e[2][0]) * Q[0][0] + d_q * (WnbbX[2][1] + 1.0 / 12.0 * WnbbX_e[2][1]) * Q[1][0] + (c_q + d_q * (WnbbX[2][2] + 1.0 / 12.0 * WnbbX_e[2][2])) * Q[2][0] + d_q * (WnbbX[2][3] + 1.0 / 12.0 * WnbbX_e[2][3]) * Q[3][0];
	// Q[3][0] = d_q * (WnbbX[3][0] + 1.0 / 12.0 * WnbbX_e[3][0]) * Q[0][0] + d_q * (WnbbX[3][1] + 1.0 / 12.0 * WnbbX_e[3][1]) * Q[1][0] + d_q * (WnbbX[3][2] + 1.0 / 12.0 * WnbbX_e[3][2]) * Q[2][0] + (c_q + d_q * (WnbbX[3][3] + 1.0 / 12.0 * WnbbX_e[3][3])) * Q[3][0];

	// WnbbA_old[0][0] = WnbbA[0][0];
	// WnbbA_old[1][0] = WnbbA[1][0];
	// WnbbA_old[2][0] = WnbbA[2][0];

	// 四元数规范化
	Q = convertTovector(Q_Q);
	double temp_Q = sqrt(Q[0][0] * Q[0][0] + Q[1][0] * Q[1][0] + Q[2][0] * Q[2][0] + Q[3][0] * Q[3][0]);
	// temp_Q = sqrt(Q[0][0] * Q[0][0] + Q[1][0] * Q[1][0] + Q[2][0] * Q[2][0] + Q[3][0] * Q[3][0]);
	Q[0][0] = Q[0][0] / temp_Q;
	Q[1][0] = Q[1][0] / temp_Q;
	Q[2][0] = Q[2][0] / temp_Q;
	Q[3][0] = Q[3][0] / temp_Q;

	// 获取姿态矩阵
	Cbn[0][0] = Q[1][0] * Q[1][0] + Q[0][0] * Q[0][0] - Q[3][0] * Q[3][0] - Q[2][0] * Q[2][0];
	Cbn[0][1] = 2 * (Q[1][0] * Q[2][0] + Q[0][0] * Q[3][0]);
	Cbn[0][2] = 2 * (Q[1][0] * Q[3][0] - Q[0][0] * Q[2][0]);
	Cbn[1][0] = 2 * (Q[1][0] * Q[2][0] - Q[0][0] * Q[3][0]);
	Cbn[1][1] = Q[2][0] * Q[2][0] - Q[3][0] * Q[3][0] + Q[0][0] * Q[0][0] - Q[1][0] * Q[1][0];
	Cbn[1][2] = 2 * (Q[2][0] * Q[3][0] + Q[0][0] * Q[1][0]);
	Cbn[2][0] = 2 * (Q[1][0] * Q[3][0] + Q[0][0] * Q[2][0]);
	Cbn[2][1] = 2 * (Q[2][0] * Q[3][0] - Q[0][0] * Q[1][0]);
	Cbn[2][2] = Q[3][0] * Q[3][0] - Q[2][0] * Q[2][0] - Q[1][0] * Q[1][0] + Q[0][0] * Q[0][0];

	attiN[0] = atan(-Cbn[0][2] / Cbn[2][2]);
	attiN[1] = atan(Cbn[1][2] / sqrt(Cbn[1][0] * Cbn[1][0] + Cbn[1][1] * Cbn[1][1]));
	attiN[2] = atan(Cbn[1][0] / Cbn[1][1]);

	attiN[0] = attiN[0] * 180.0 / pi;
	attiN[1] = attiN[1] * 180.0 / pi;
	attiN[2] = attiN[2] * 180.0 / pi;

	// 象限判断
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
			attiN[0] = -(180.0 - attiN[0]);
		}
		if (Cbn[0][2] < 0)
		{
			attiN[0] = 180.0 + attiN[0];
		}
	}
	WnbbA_old = convertTovector(WnbbA_old_old);
}

void velo_cal(double T, vector<double> &attiN, vector<double> &posiN, vector<double> &veloN, vector<vector<double>> &Fb)
{
	double lon = posiN[0] * pi / 180.0;
	double lat = posiN[1] * pi / 180.0;
	double heig = posiN[2];

	double Rm = (1 - f) * (1 - f) * Re / (((1 - f) * (1 - f) * sin(lat) * sin(lat) + cos(lat) * cos(lat)) * sqrt((1 - f) * (1 - f) * sin(lat) * sin(lat) + cos(lat) * cos(lat)));
	double Rn = Re / sqrt((1 - f) * (1 - f) * sin(lat) * sin(lat) + cos(lat) * cos(lat));
	// cout << Rm << " " << Rn << endl;
	double roll = attiN[0] * pi / 180.0;
	double pitch = attiN[1] * pi / 180.0;
	double head = attiN[2] * pi / 180.0;

	vector<vector<double>> Cbn(3, vector<double>(3, 0.0));
	vector<vector<double>> Cbn_turn;
	// vector<vector<double>>CbnTurn;
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

	Cbn_turn = cal_mat_turn(Cbn);
	// Eigen::MatrixXd Fb_b = convertToeigenmatrix(Fb);
	// Eigen::MatrixXd Cbn_n = convertToeigenmatrix(Cbn);
	// Eigen::MatrixXd Fn = Cbn_n.transpose() * Fb_b;

	vector<vector<double>> Fn;
	Fn = cal_mat_mul(Cbn_turn, Fb);

	double Ve = veloN[0];
	double Vn = veloN[1];
	double Vu = veloN[2];

	vector<vector<double>> Wenn(3, vector<double>(1, 0.0));
	vector<vector<double>> Wien(3, vector<double>(1, 0.0));

	Wenn[0][0] = -Vn / (Rm + heig);
	Wenn[1][0] = Ve / (Rn + heig);
	Wenn[2][0] = Ve / (Rn + heig) * tan(lat);

	// Eigen::MatrixXd Wenn_n = convertToeigenmatrix(Wenn);

	Wien[0][0] = 0.0;
	Wien[1][0] = Wie * cos(lat);
	Wien[2][0] = Wie * sin(lat);

	// Eigen::MatrixXd Wien_n = convertToeigenmatrix(Wien);

	// vector<vector<double>>temp(3, vector<double>(1,0.0));
	// temp[0][0] = 0;
	// temp[1][0] = 0;
	// temp[2][0] = -1.0 * g;
	// Eigen::MatrixXd temp_p = convertToeigenmatrix(temp);
	// vector<vector<double>>velon_temp(3, vector<double>(1, 0.0));
	// velon_temp[0][0] = veloN[0];
	// velon_temp[1][0] = veloN[1];
	// velon_temp[2][0] = veloN[2];
	// Eigen::MatrixXd veloN_N = convertToeigenmatrix(velon_temp);
	// veloN_N = veloN_N + T * (Fn - (2 * Wien_n + Wenn_n).cross(veloN_N) + temp_p);
	// veloN[0] = veloN_N(0, 0);
	// veloN[1] = veloN_N(1, 0);
	// veloN[2] = veloN_N(2, 0);
	veloN[0] = veloN[0] + T * (Fn[0][0] - ((2 * Wien[1][0] + Wenn[1][0]) * veloN[2] - (2 * Wien[2][0] + Wenn[2][0]) * veloN[1]) + 0.0);
	veloN[1] = veloN[1] + T * (Fn[1][0] - ((2 * Wien[2][0] + Wenn[2][0]) * veloN[0] - (2 * Wien[0][0] + Wenn[0][0]) * veloN[2]) + 0.0);
	veloN[2] = veloN[2] + T * (Fn[2][0] - ((2 * Wien[0][0] + Wenn[0][0]) * veloN[1] - (2 * Wien[1][0] + Wenn[1][0]) * veloN[0]) - g);
}

void posi_cal(double T, vector<double> &veloN, vector<double> &posiN)
{
	double lon = posiN[0] * pi / 180.0;
	double lat = posiN[1] * pi / 180.0;
	double heig = posiN[2];

	double Rm = (1 - f) * (1 - f) * Re / (((1 - f) * (1 - f) * sin(lat) * sin(lat) + cos(lat) * cos(lat)) * sqrt((1 - f) * (1 - f) * sin(lat) * sin(lat) + cos(lat) * cos(lat)));
	double Rn = Re / sqrt((1 - f) * (1 - f) * sin(lat) * sin(lat) + cos(lat) * cos(lat));

	double Ve = veloN[0];
	double Vn = veloN[1];
	double Vu = veloN[2];

	heig = heig + T * Vu;
	lat = lat + T * (Vn / (Rm + heig));
	lon = lon + T * (Ve / ((Rn + heig) * cos(lat)));

	posiN[0] = lon * 180.0 / pi;
	posiN[1] = lat * 180.0 / pi;
	posiN[2] = heig;
}

void posi_compensation(int64_t &time_now, int64_t &time_last, vector<double> &veloN, vector<double> &posiT, double T)
{
	double lon = posiT[0] * pi / 180.0;
	double lat = posiT[1] * pi / 180.0;
	double heig = posiT[2];

	double Rm = (1 - f) * (1 - f) * Re / (((1 - f) * (1 - f) * sin(lat) * sin(lat) + cos(lat) * cos(lat)) * sqrt((1 - f) * (1 - f) * sin(lat) * sin(lat) + cos(lat) * cos(lat)));
	double Rn = Re / sqrt((1 - f) * (1 - f) * sin(lat) * sin(lat) + cos(lat) * cos(lat));

	double Ve = veloN[0];
	double Vn = veloN[1];
	double Vu = veloN[2];

	double t_temp = static_cast<double>(time_now - time_last);
	double time_compensation = 0;
	if (t_temp > 0)
	{
		int n = static_cast<int>(floor(t_temp / T)) + 1;
		time_compensation = n * T;
	}
	else
	{
		time_compensation = 0;
	}
	// 时间信息以毫秒为单位
	heig = heig + (time_compensation / 1000.0) * Vu;
	lat = lat + (time_compensation / 1000.0) * (Vn / (Rm + heig));
	lon = lon + (time_compensation / 1000.0) * (Ve / (Rn + heig));

	posiT[0] = lon * 180.0 / pi;
	posiT[1] = lat * 180.0 / pi;
	posiT[2] = heig;
}

void imu_err_random(double t, double T, vector<double> &Gyro_b, vector<double> &Gyro_r, vector<double> &Gyro_wg, vector<double> &Acc_r, vector<vector<double>> &Fb, vector<double> &Wibb)
{
	vector<double> Da_bias(3, 0.0);
	Da_bias[0] = 0.0001 * g;
	Da_bias[1] = 0.0001 * g;
	Da_bias[2] = 0.0001 * g;

	double Ta = 1800.0;
	double Tg = 3600.0;
	vector<double> Acc_wa(3, 0.0);
	vector<double> Gyro_wr(3, 0.0);
	// srand(time(nullptr));
	unsigned seed = chrono::system_clock::now().time_since_epoch().count();
	default_random_engine generator(seed);
	normal_distribution<double> distribute(0.0, 1.0);
	if (t == 0)
	{
		Acc_r[0] = Da_bias[0] * distribute(generator);
		Acc_r[1] = Da_bias[1] * distribute(generator);
		Acc_r[2] = Da_bias[2] * distribute(generator);
		// 加速度一阶马尔可夫过程

		Gyro_b[0] = 0.01 * (deg_rad / 3600.0) * distribute(generator);
		Gyro_b[1] = 0.01 * (deg_rad / 3600.0) * distribute(generator);
		Gyro_b[2] = 0.01 * (deg_rad / 3600.0) * distribute(generator);

		Gyro_r[0] = 0.01 * (deg_rad / 3600.0) * distribute(generator);
		Gyro_r[1] = 0.01 * (deg_rad / 3600.0) * distribute(generator);
		Gyro_r[2] = 0.01 * (deg_rad / 3600.0) * distribute(generator);

		Gyro_wg[0] = 0.01 * (deg_rad / 3600.0) * distribute(generator);
		Gyro_wg[1] = 0.01 * (deg_rad / 3600.0) * distribute(generator);
		Gyro_wg[2] = 0.01 * (deg_rad / 3600.0) * distribute(generator);

		// Acc_r[0] = Da_bias[0] * 0.0001;
		// Acc_r[1] = Da_bias[1] * 0.0001;
		// Acc_r[2] = Da_bias[2] * 0.0001;
		////加速度一阶马尔可夫过程

		// Gyro_b[0] = 0.01 * (deg_rad / 3600.0) * 0.0001;
		// Gyro_b[1] = 0.01 * (deg_rad / 3600.0) * 0.0001;
		// Gyro_b[2] = 0.01 * (deg_rad / 3600.0) * 0.0001;

		// Gyro_r[0] = 0.01 * (deg_rad / 3600.0) * 0.0001;
		// Gyro_r[1] = 0.01 * (deg_rad / 3600.0) * 0.0001;
		// Gyro_r[2] = 0.01 * (deg_rad / 3600.0) * 0.0001;

		// Gyro_wg[0] = 0.01 * (deg_rad / 3600.0) * 0.0001;
		// Gyro_wg[1] = 0.01 * (deg_rad / 3600.0) * 0.0001;
		// Gyro_wg[2] = 0.01 * (deg_rad / 3600.0) * 0.0001;
	}
	else
	{
		Acc_wa[0] = sqrt(2 * T / Ta) * Da_bias[0] * distribute(generator);
		Acc_wa[1] = sqrt(2 * T / Ta) * Da_bias[1] * distribute(generator);
		Acc_wa[2] = sqrt(2 * T / Ta) * Da_bias[2] * distribute(generator);
		// 加速度白噪声
		Acc_r[0] = exp(-1.0 * T / Ta) * Acc_r[0] + Acc_wa[0];
		Acc_r[1] = exp(-1.0 * T / Ta) * Acc_r[1] + Acc_wa[1];
		Acc_r[2] = exp(-1.0 * T / Ta) * Acc_r[2] + Acc_wa[2];
		// 加速度一阶马尔可夫过程

		Gyro_wr[0] = sqrt(2 * T / Tg) * 0.01 * deg_rad / 3600.0 * distribute(generator);
		Gyro_wr[1] = sqrt(2 * T / Tg) * 0.01 * deg_rad / 3600.0 * distribute(generator);
		Gyro_wr[2] = sqrt(2 * T / Tg) * 0.01 * deg_rad / 3600.0 * distribute(generator);
		// 陀螺一阶马尔可夫过程白噪声
		Gyro_r[0] = exp(-1.0 * T / Tg) * Gyro_r[0] + Gyro_wr[0];
		Gyro_r[1] = exp(-1.0 * T / Tg) * Gyro_r[1] + Gyro_wr[1];
		Gyro_r[2] = exp(-1.0 * T / Tg) * Gyro_r[2] + Gyro_wr[2];
		// 陀螺一阶马尔可夫过程
		Gyro_wg[0] = 0.01 * deg_rad / 3600.0 * distribute(generator);
		Gyro_wg[1] = 0.01 * deg_rad / 3600.0 * distribute(generator);
		Gyro_wg[2] = 0.01 * deg_rad / 3600.0 * distribute(generator);

		// Acc_wa[0] = sqrt(2 * T / Ta) * Da_bias[0] * 0.00015;
		// Acc_wa[1] = sqrt(2 * T / Ta) * Da_bias[1] * 0.00015;
		// Acc_wa[2] = sqrt(2 * T / Ta) * Da_bias[2] * 0.00015;
		////加速度白噪声
		// Acc_r[0] = exp(-1.0 * T / Ta) * Acc_r[0] + Acc_wa[0];
		// Acc_r[1] = exp(-1.0 * T / Ta) * Acc_r[1] + Acc_wa[1];
		// Acc_r[2] = exp(-1.0 * T / Ta) * Acc_r[2] + Acc_wa[2];
		////加速度一阶马尔可夫过程

		// Gyro_wr[0] = sqrt(2 * T / Tg) * 0.01 * deg_rad / 3600.0 * 0.00015;
		// Gyro_wr[1] = sqrt(2 * T / Tg) * 0.01 * deg_rad / 3600.0 * 0.00015;
		// Gyro_wr[2] = sqrt(2 * T / Tg) * 0.01 * deg_rad / 3600.0 * 0.00015;
		////陀螺一阶马尔可夫过程白噪声
		// Gyro_r[0] = exp(-1.0 * T / Tg) * Gyro_r[0] + Gyro_wr[0];
		// Gyro_r[1] = exp(-1.0 * T / Tg) * Gyro_r[1] + Gyro_wr[1];
		// Gyro_r[2] = exp(-1.0 * T / Tg) * Gyro_r[2] + Gyro_wr[2];
		////陀螺一阶马尔可夫过程
		// Gyro_wg[0] = 0.01 * deg_rad / 3600.0 * 0.00015;
		// Gyro_wg[1] = 0.01 * deg_rad / 3600.0 * 0.00015;
		// Gyro_wg[2] = 0.01 * deg_rad / 3600.0 * 0.00015;
	}
	Fb[0][0] = Fb[0][0] + Acc_r[0];
	Fb[1][0] = Fb[1][0] + Acc_r[0];
	Fb[2][0] = Fb[2][0] + Acc_r[0];

	Wibb[0] = Wibb[0] + Gyro_b[0] / deg_rad + Gyro_r[0] / deg_rad + Gyro_wg[0] / deg_rad;
	Wibb[1] = Wibb[1] + Gyro_b[1] / deg_rad + Gyro_r[1] / deg_rad + Gyro_wg[1] / deg_rad;
	Wibb[2] = Wibb[2] + Gyro_b[2] / deg_rad + Gyro_r[2] / deg_rad + Gyro_wg[2] / deg_rad;
	// cout << distribute(generator) << endl;
	// cout << distribute(generator) << endl;
	// cout << distribute(generator) << endl;
}
