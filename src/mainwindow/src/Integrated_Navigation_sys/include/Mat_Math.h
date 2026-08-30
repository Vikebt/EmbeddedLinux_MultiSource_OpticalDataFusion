#pragma once
#include <iostream>
#include <vector>
#include <cmath>
#include <eigen3/Eigen/Dense>
using namespace std;
using namespace Eigen;
vector<vector<double>> cal_mat_turn(vector<vector<double>> &Mat);
vector<vector<double>> cal_mat_mul(vector<vector<double>> &Mat1, vector<vector<double>> &Mat2);
vector<vector<double>> cal_mat_add(vector<vector<double>> &Mat1, vector<vector<double>> &Mat2);
vector<vector<double>> cal_mat_sub(vector<vector<double>> &Mat1, vector<vector<double>> &Mat2);
vector<vector<double>> cal_mat_inv(vector<vector<double>> &Mat);
Eigen::MatrixXd convertToeigenmatrix(vector<vector<double>> &Mat);
vector<vector<double>> convertTovector(Eigen::MatrixXd Mat);
void print_message(vector<vector<double>> &Mat);