#include "Mat_Math.h"

vector<vector<double>> cal_mat_turn(vector<vector<double>> &Mat)
{
    int row = Mat.size();
    int col = Mat[0].size();
    vector<vector<double>> Mat_Turn(col, vector<double>(row, 0.0));
    // Mat_Turn.resize(col, vector<double>(row));
    for (int i = 0; i < row; i++)
    {
        for (int j = 0; j < col; j++)
        {
            Mat_Turn[j][i] = Mat[i][j];
        }
    }
    return Mat_Turn;
}

vector<vector<double>> cal_mat_mul(vector<vector<double>> &Mat1, vector<vector<double>> &Mat2)
{
    int row1 = Mat1.size();
    int col1 = Mat1[0].size();
    int row2 = Mat2.size();
    int col2 = Mat2[0].size();
    vector<vector<double>> ans;
    ans.resize(row1, vector<double>(col2, 0.0));
    for (int i = 0; i < row1; i++)
    {
        for (int j = 0; j < col2; j++)
        {
            for (int k = 0; k < col1; k++)
            {
                ans[i][j] += Mat1[i][k] * Mat2[k][j];
            }
        }
    }
    return ans;
}

vector<vector<double>> cal_mat_add(vector<vector<double>> &Mat1, vector<vector<double>> &Mat2)
{
    int row = Mat1.size();
    int col = Mat1[0].size();
    vector<vector<double>> ans(row, vector<double>(col, 0.0));
    for (int i = 0; i < row; i++)
    {
        for (int j = 0; j < col; j++)
        {
            ans[i][j] = Mat1[i][j] + Mat2[i][j];
        }
    }
    return ans;
}

vector<vector<double>> cal_mat_sub(vector<vector<double>> &Mat1, vector<vector<double>> &Mat2)
{
    int row = Mat1.size();
    int col = Mat1[0].size();
    vector<vector<double>> ans(row, vector<double>(col, 0.0));
    for (int i = 0; i < row; i++)
    {
        for (int j = 0; j < col; j++)
        {
            ans[i][j] = Mat1[i][j] - Mat2[i][j];
        }
    }
    return ans;
}

vector<vector<double>> cal_mat_inv(vector<vector<double>> &Mat)
{
    Eigen::MatrixXd mat(Mat.size(), Mat[0].size());
    for (int i = 0; i < Mat.size(); i++)
    {
        for (int j = 0; j < Mat[0].size(); j++)
        {
            mat(i, j) = Mat[i][j];
        }
    }
    Eigen::MatrixXd mat_inv = mat.inverse();
    vector<vector<double>> ans(Mat.size(), vector<double>(Mat[0].size()));
    for (int i = 0; i < Mat.size(); i++)
    {
        for (int j = 0; j < Mat[0].size(); j++)
        {
            ans[i][j] = mat_inv(i, j);
        }
    }
    return ans;
}

Eigen::MatrixXd convertToeigenmatrix(vector<vector<double>> &Mat)
{
    int row = Mat.size();
    int col = Mat[0].size();
    Eigen::MatrixXd mat(row, col);
    for (int i = 0; i < row; i++)
    {
        for (int j = 0; j < col; j++)
        {
            mat(i, j) = Mat[i][j];
        }
    }
    return mat;
}

vector<vector<double>> convertTovector(Eigen::MatrixXd Mat)
{
    int row = Mat.rows();
    int col = Mat.cols();
    vector<vector<double>> ans(row, vector<double>(col));
    for (int i = 0; i < row; i++)
    {
        for (int j = 0; j < col; j++)
        {
            ans[i][j] = Mat(i, j);
        }
    }
    return ans;
}

void print_message(vector<vector<double>> &Mat)
{
    int row = Mat.size();
    int col = Mat[0].size();
    for (int i = 0; i < row; i++)
    {
        for (int j = 0; j < col; j++)
        {
            cout << Mat[i][j] << " ";
        }
        cout << endl;
    }
}
