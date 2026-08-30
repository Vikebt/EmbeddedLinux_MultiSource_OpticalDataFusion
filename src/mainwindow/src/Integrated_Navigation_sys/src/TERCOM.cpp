#include "TERCOM.h"

// double Map_process::Map_start_lat = 0.0;
// double Map_process::Map_start_lon = 0.0;
// int Map_process::Map_resolution = 0;
// int Map_process::Map_size = 0;

// vector<double> Map_process::readMapmessage(double lat, double lon)
// {
//     vector<double> ans;
//     double value = 0;
//     double rows = (floor((this->Map_start_lat - lat) * this->Map_resolution)) - 1;
//     double cols = (floor((lon - this->Map_start_lon) * this->Map_resolution)) - 1;
//     value = this->Map[static_cast<int>(rows)][static_cast<int>(cols)];
//     ans.push_back(rows);
//     ans.push_back(cols);
//     ans.push_back(value);
//     return ans;
// }

void map_generator(vector<int16_t> &map, const string &filename)
{
    ifstream file(filename, ios::binary | ios::ate);
    if (!file)
    {
        cout << "无法打开文件" << endl;
    }
    streamsize size = file.tellg();
    file.seekg(0, ios::beg);
    map.resize(size / sizeof(int16_t), 0);
    file.read(reinterpret_cast<char *>(map.data()), size);
    file.close();
} //

void map_generator_9(vector<Map_database> &Map, int map_count, vector<string> &filename)
{
    for (int i = 0; i < map_count; i++)
    {
        ifstream file(filename[i], ios::binary | ios::ate);
        if (!file)
        {
            cout << "文件打开失败！" << endl;
        }
        streamsize size = file.tellg();
        file.seekg(0, ios::beg);
        Map[i].Map.resize(size / sizeof(int16_t));
        file.read(reinterpret_cast<char *>(Map[i].Map.data()), size);
        file.close();
    }
}

// Class TERCOM start
// 初始化
void TERCOM::var_init(int map_reso, int map_cell, double radius, double ins_lat_start, double ins_lon_start, double ins_altitude_start, int max_size)
{
    this->time_info = 0;
    this->analysis_time = 0;
    this->analysis_lat = 0;
    this->analysis_lon = 0;
    this->match_row = 0;
    this->match_col = 0;
    this->map_resolution = map_reso;
    this->Ins_lat = ins_lat_start;
    this->Ins_lon = ins_lon_start;
    this->Ins_altitude = ins_altitude_start;
    this->max_match_size = max_size;
    this->serch_radius = floor(radius / map_cell);
    this->filename.resize(9);
    this->mapdata.resize(9, Map_database(3601, 3601));
    this->map_read();
    this->start_flag = 1;
}

// 设置队列
void TERCOM::Set_sequence(double lat, double lon, double height, int sequence_size)
{
    double cell = 1.0 / static_cast<double>(this->map_resolution);
    if (this->max_match_size != sequence_size)
    {
        this->max_match_size = sequence_size;
    }
    if (abs(lat - this->old_lat) > cell || abs(lon - this->old_lon) > cell)
    {
        // if (this->max_match_size != sequence_size)
        //{
        //	this->max_match_size = sequence_size;
        // }
        if (this->Match_sequence.size() >= this->max_match_size)
        {
            this->match_flag = 1;
            this->Match_sequence.erase(this->Match_sequence.begin());
            this->Match_sequence.push_back(Match_InFo(lat, lon, height));
        }
        else
        {
            this->Match_sequence.push_back(Match_InFo(lat, lon, height));
            if (this->Match_sequence.size() == this->max_match_size)
            {
                this->match_flag = 1;
            }
        }
        // this->old_lon = lon;
        // this->old_lat = lat;
        this->old_lon = this->Match_sequence[this->Match_sequence.size() - 1].ins_lon;
        this->old_lat = this->Match_sequence[this->Match_sequence.size() - 1].ins_lat;
        this->serch_map_update_flag = 1;
    }
    else
    {
        this->serch_map_update_flag = 0;
        // cout << "距离小于地图分辨率，地形匹配轮廓序列未更新！" << endl;
    }
}

// 生成搜索地图序列1
void TERCOM::Serch_map(const vector<int16_t> &Map, vector<Map_database> &Mapdata)
{
    int count = this->Match_sequence.size();
    // cout << count << endl;
    double cell = 1.0 / static_cast<double>(this->map_resolution);
    if (this->serch_map_update_flag == 1)
    {
        if (count <= this->max_match_size)
        {
            if (this->Match_sequence[count - 1].ins_lon >= this->map_start_lon && this->Match_sequence[count - 1].ins_lon <= (this->map_start_lon + 1) && this->Match_sequence[count - 1].ins_lat <= this->map_start_lat && this->Match_sequence[count - 1].ins_lat >= (this->map_start_lat - 1))
            {
                int row = floor((this->map_start_lat - this->Match_sequence[count - 1].ins_lat) * this->map_resolution);
                int col = floor((this->Match_sequence[count - 1].ins_lon - this->map_start_lon) * this->map_resolution);
                int row_temp_up = row - this->serch_radius;
                int row_temp_down = row + this->serch_radius;
                int col_temp_left = col - this->serch_radius;
                int col_temp_right = col + this->serch_radius;
                if (row_temp_up >= 0 && row_temp_down <= this->map_resolution && col_temp_left >= 0 && col_temp_right <= this->map_resolution)
                {
                    this->Serch_map_generator(Map, row_temp_up, row_temp_down, col_temp_left, col_temp_right);
                }
                else
                {
                    this->Serch_boundary_map_generator(Mapdata, row_temp_up, row_temp_down, col_temp_left, col_temp_right);
                }
                /*原方法**************************************
                double lon_temp_left = this->Match_sequence[count - 1].ins_lon - (this->serch_radius * cell);
                double lon_temp_right = this->Match_sequence[count - 1].ins_lon + (this->serch_radius * cell);
                double lat_temp_up = this->Match_sequence[count - 1].ins_lat + (this->serch_radius * cell);
                double lat_temp_down = this->Match_sequence[count - 1].ins_lat - (this->serch_radius * cell);
                if (lon_temp_left > (this->map_start_lon + cell) && lon_temp_right < (this->map_start_lon + 1.0 + cell) && lat_temp_up < (this->map_start_lat - cell) && lat_temp_down > (this->map_start_lat - 1.0 - cell))
                {
                    Serch_map_generator(count, Map, this->map_start_lat, this->map_start_lon, this->map_resolution);
                }
                else
                {
                    Serch_boundary_map_generator(count, Mapdata, lon_temp_left, lon_temp_right, lat_temp_up, lat_temp_down, this->map_start_lat, this->map_start_lon, this->map_resolution);
                }
                **********************************************/
            }
            else
            {
                // cout << this->map_start_lat << endl;
                // for (int i = 0; i < 9; i++)
                //{
                //	cout << this->mapdata[i].Map[101] << endl;
                // }
                // 更新中心地图位置信息
                this->map_read();
                // cout << this->map_start_lat << endl;
                // for (int i = 0; i < 9; i++)
                //{
                //	cout << this->mapdata[i].Map[101] << endl;
                //	cout << this->filename[i] << endl;
                //	cout << this->mapdata[i].Map.size() << endl;
                // }
                int row = floor((this->map_start_lat - this->Match_sequence[count - 1].ins_lat) * this->map_resolution);
                int col = floor((this->Match_sequence[count - 1].ins_lon - this->map_start_lon) * this->map_resolution);
                int row_temp_up = row - this->serch_radius;
                int row_temp_down = row + this->serch_radius;
                int col_temp_left = col - this->serch_radius;
                int col_temp_right = col + this->serch_radius;
                if (row_temp_up >= 0 && row_temp_down <= this->map_resolution && col_temp_left >= 0 && col_temp_right <= this->map_resolution)
                {
                    this->Serch_map_generator(Map, row_temp_up, row_temp_down, col_temp_left, col_temp_right);
                }
                else
                {
                    this->Serch_boundary_map_generator(Mapdata, row_temp_up, row_temp_down, col_temp_left, col_temp_right);
                }
                /*原方法******************************************************************
                double lon_temp_left = this->Match_sequence[count - 1].ins_lon - (this->serch_radius * cell);
                double lon_temp_right = this->Match_sequence[count - 1].ins_lon + (this->serch_radius * cell);
                double lat_temp_up = this->Match_sequence[count - 1].ins_lat + (this->serch_radius * cell);
                double lat_temp_down = this->Match_sequence[count - 1].ins_lat - (this->serch_radius * cell);
                if (lon_temp_left > (this->map_start_lon + cell) && lon_temp_right < (this->map_start_lon + 1.0 + cell) && lat_temp_up < (this->map_start_lat - cell) && lat_temp_down > (this->map_start_lat - 1.0 - cell))
                {
                    Serch_map_generator(count, Map, this->map_start_lat, this->map_start_lon, this->map_resolution);
                }
                else
                {
                    Serch_boundary_map_generator(count, Mapdata, lon_temp_left, lon_temp_right, lat_temp_up, lat_temp_down, this->map_start_lat, this->map_start_lon, this->map_resolution);
                }
                *************************************************************************/
            }
        }
        // cout << "时间戳：" << this->time_info << " " << "地形匹配轮廓序列更新！" << endl;
        this->serch_map_update_flag = 0;
    }
    this->serch_map_update_flag = 0;
    // double lon_temp_left = this->Match_sequence[count - 1].ins_lon - (this->serch_radius * cell);
    // double lon_temp_right = this->Match_sequence[count - 1].ins_lon + (this->serch_radius * cell);
    // double lat_temp_up = this->Match_sequence[count - 1].ins_lat + (this->serch_radius * cell);
    // double lat_temp_down = this->Match_sequence[count - 1].ins_lat - (this->serch_radius * cell);
    // if (lon_temp_left > (map_start_lon + cell) && lon_temp_right < (map_start_lon + 1.0 - cell) && lat_temp_up<(map_start_lat - cell) && lat_temp_down>(map_start_lat - 1.0 + cell))
    //{
    //	Serch_map_generator(count, Map, map_start_lat, map_start_lon, map_resolution);
    // }
    // else
    //{
    //	Serch_boundary_map_generator(count, Mapdata, lon_temp_left, lon_temp_right, lat_temp_up, lat_temp_down, map_start_lat, map_start_lon, map_resolution);
    // }
}

// 生成搜索地图序列2（重载）
void TERCOM::Serch_map_generator(const vector<int16_t> &Map, int &row_up, int &row_down, int &col_left, int &col_right)
{
    vector<vector<int16_t>> map_temp(2 * this->serch_radius + 1, vector<int16_t>(2 * this->serch_radius + 1, 0));
    int hang = 0;
    for (int i = row_up; i <= row_down; i++)
    {
        vector<int16_t> rows(Map.begin() + (i * (this->map_resolution + 1)) + col_left, Map.begin() + (i * (this->map_resolution + 1) + col_right + 1));
        map_temp[hang] = rows;
        hang++;
    }
    int size_temp = this->Map_match.size();
    if (size_temp < this->max_match_size)
    {
        this->Map_match.push_back(map_temp);
    }
    if (size_temp == this->max_match_size)
    {
        this->Map_match.erase(this->Map_match.begin());
        this->Map_match.push_back(map_temp);
    }
}

// 生成搜索地图序列3（越界搜索地图）（重载）
void TERCOM::Serch_boundary_map_generator(vector<Map_database> &Map, int &row_up, int &row_down, int &col_left, int &col_right)
{
    vector<vector<int16_t>> map_temp(2 * this->serch_radius + 1, vector<int16_t>(2 * this->serch_radius + 1, 0));
    if (col_left < 0 && col_right <= this->map_resolution && row_up >= 0 && row_down <= this->map_resolution)
    {
        int hang = 0;
        int d_col1 = 2 * this->serch_radius + 1 + col_left;
        for (int i = row_up; i <= row_down; i++)
        {
            vector<int16_t> temp;
            vector<int16_t> row_left(Map[4].Map.begin() + (i * (this->map_resolution + 1)) + (this->map_resolution + col_left + 1), Map[4].Map.begin() + (i * (this->map_resolution + 1)) + this->map_resolution + 1);
            vector<int16_t> row_right(Map[0].Map.begin() + (i * (this->map_resolution + 1)) + 0, Map[0].Map.begin() + (i * (this->map_resolution + 1)) + d_col1);
            temp.insert(temp.end(), row_left.begin(), row_left.end());
            temp.insert(temp.end(), row_right.begin(), row_right.end());
            map_temp[hang] = temp;
            hang++;
        }
    }
    else if (col_left >= 0 && col_right > this->map_resolution && row_up >= 0 && row_down <= this->map_resolution)
    {
        int hang = 0;
        int col1 = this->map_resolution - col_right;
        int d_col1 = 2 * this->serch_radius + 1 + col1;
        for (int i = row_up; i <= row_down; i++)
        {
            vector<int16_t> temp;
            vector<int16_t> row_left(Map[0].Map.begin() + (i * (this->map_resolution + 1)) + this->map_resolution + 1 - d_col1, Map[0].Map.begin() + (i * (this->map_resolution + 1)) + this->map_resolution + 1);
            vector<int16_t> row_right(Map[5].Map.begin() + (i * (this->map_resolution + 1)) + 0, Map[5].Map.begin() + (i * (this->map_resolution + 1)) - col1);
            temp.insert(temp.end(), row_left.begin(), row_left.end());
            temp.insert(temp.end(), row_right.begin(), row_right.end());
            map_temp[hang] = temp;
            hang++;
        }
    }
    else if (col_left >= 0 && col_right <= this->map_resolution && row_up < 0 && row_down <= this->map_resolution)
    {
        int hang = 0;
        int d_row1 = 2 * this->serch_radius + 1 + row_up;
        for (int i = this->map_resolution + 1 + row_up; i <= this->map_resolution; i++)
        {
            vector<int16_t> rows_up(Map[2].Map.begin() + (i * (this->map_resolution + 1)) + col_left, Map[2].Map.begin() + (i * (this->map_resolution + 1)) + col_right + 1);
            map_temp[hang] = rows_up;
            hang++;
        }
        for (int i = 0; i <= (d_row1 - 1); i++)
        {
            vector<int16_t> rows_down(Map[0].Map.begin() + (i * (this->map_resolution + 1)) + col_left, Map[0].Map.begin() + (i * (this->map_resolution + 1)) + col_right + 1);
            map_temp[hang] = rows_down;
            hang++;
        }
    }
    else if (col_left >= 0 && col_right <= this->map_resolution && row_up >= 0 && row_down > this->map_resolution)
    {
        int hang = 0;
        int row1 = this->map_resolution - row_down;
        int d_row1 = 2 * this->serch_radius + 1 + row1;
        for (int i = this->map_resolution + 1 - d_row1; i <= this->map_resolution; i++)
        {
            vector<int16_t> rows_up(Map[0].Map.begin() + (i * (this->map_resolution + 1)) + col_left, Map[0].Map.begin() + (i * (this->map_resolution + 1)) + col_right + 1);
            map_temp[hang] = rows_up;
            hang++;
        }
        for (int i = 0; i <= ((-row1) - 1); i++)
        {
            vector<int16_t> rows_down(Map[7].Map.begin() + (i * (this->map_resolution + 1)) + col_left, Map[7].Map.begin() + (i * (this->map_resolution + 1)) + col_right + 1);
            map_temp[hang] = rows_down;
            hang++;
        }
    }
    else if (col_left < 0 && col_right <= this->map_resolution && row_up < 0 && row_down <= this->map_resolution)
    {
        int hang = 0;
        int d_row1 = 2 * this->serch_radius + 1 + row_up;
        int d_col1 = 2 * this->serch_radius + 1 + col_left;
        for (int i = this->map_resolution + 1 + row_up; i <= this->map_resolution; i++)
        {
            vector<int16_t> temp;
            vector<int16_t> rows_up_cols_left(Map[1].Map.begin() + (i * (this->map_resolution + 1)) + this->map_resolution + 1 + col_left, Map[1].Map.begin() + (i * (this->map_resolution + 1)) + this->map_resolution + 1);
            vector<int16_t> rows_up_cols_right(Map[2].Map.begin() + (i * (this->map_resolution + 1)) + 0, Map[2].Map.begin() + (i * (this->map_resolution + 1)) + d_col1);
            temp.insert(temp.end(), rows_up_cols_left.begin(), rows_up_cols_left.end());
            temp.insert(temp.end(), rows_up_cols_right.begin(), rows_up_cols_right.end());
            map_temp[hang] = temp;
            hang++;
        }
        for (int i = 0; i <= (d_row1 - 1); i++)
        {
            vector<int16_t> temp;
            vector<int16_t> rows_down_cols_left(Map[4].Map.begin() + (i * (this->map_resolution + 1)) + this->map_resolution + 1 + col_left, Map[4].Map.begin() + (i * (this->map_resolution + 1)) + this->map_resolution + 1);
            vector<int16_t> rows_down_cols_right(Map[0].Map.begin() + (i * (this->map_resolution + 1)) + 0, Map[0].Map.begin() + (i * (this->map_resolution + 1)) + d_col1);
            temp.insert(temp.end(), rows_down_cols_left.begin(), rows_down_cols_left.end());
            temp.insert(temp.end(), rows_down_cols_right.begin(), rows_down_cols_right.end());
            map_temp[hang] = temp;
            hang++;
        }
    }
    else if (col_left < 0 && col_right <= this->map_resolution && row_up >= 0 && row_down > this->map_resolution)
    {
        int hang = 0;
        int row1 = this->map_resolution - row_down;
        int d_row1 = 2 * this->map_resolution + 1 + row1;
        int d_col1 = 2 * this->map_resolution + 1 + col_left;
        for (int i = this->map_resolution + 1 - d_row1; i <= this->map_resolution; i++)
        {
            vector<int16_t> temp;
            vector<int16_t> rows_up_cols_left(Map[4].Map.begin() + (i * (this->map_resolution + 1)) + this->map_resolution + 1 + col_left, Map[4].Map.begin() + (i * (this->map_resolution + 1)) + this->map_resolution + 1);
            vector<int16_t> rows_up_cols_right(Map[0].Map.begin() + (i * (this->map_resolution + 1)) + 0, Map[0].Map.begin() + (i * (this->map_resolution + 1)) + d_col1);
            temp.insert(temp.end(), rows_up_cols_left.begin(), rows_up_cols_left.end());
            temp.insert(temp.end(), rows_up_cols_right.begin(), rows_up_cols_right.end());
            map_temp[hang] = temp;
            hang++;
        }
        for (int i = 0; i <= ((-row1) - 1); i++)
        {
            vector<int16_t> temp;
            vector<int16_t> rows_down_cols_left(Map[6].Map.begin() + (i * (this->map_resolution + 1)) + this->map_resolution + 1 + col_left, Map[6].Map.begin() + (i * (this->map_resolution + 1)) + this->map_resolution + 1);
            vector<int16_t> rows_down_cols_right(Map[7].Map.begin() + (i * (this->map_resolution + 1)) + 0, Map[7].Map.begin() + (i * (this->map_resolution + 1)) + d_col1);
            temp.insert(temp.end(), rows_down_cols_left.begin(), rows_down_cols_left.end());
            temp.insert(temp.end(), rows_down_cols_right.begin(), rows_down_cols_right.end());
            map_temp[hang] = temp;
            hang++;
        }
    }
    else if (col_left >= 0 && col_right > this->map_resolution && row_up < 0 && row_down <= this->map_resolution)
    {
        int hang = 0;
        int col1 = this->map_resolution - col_right;
        int d_row1 = 2 * this->serch_radius + 1 + row_up;
        int d_col1 = 2 * this->serch_radius + 1 + col1;
        for (int i = this->map_resolution + 1 + row_up; i <= this->map_resolution; i++)
        {
            vector<int16_t> temp;
            vector<int16_t> rows_up_cols_left(Map[2].Map.begin() + (i * (this->map_resolution + 1)) + this->map_resolution + 1 - d_col1, Map[2].Map.begin() + (i * (this->map_resolution + 1)) + this->map_resolution + 1);
            vector<int16_t> rows_up_cols_right(Map[3].Map.begin() + (i * (this->map_resolution + 1)) + 0, Map[3].Map.begin() + (i * (this->map_resolution + 1)) - col1);
            temp.insert(temp.end(), rows_up_cols_left.begin(), rows_up_cols_left.end());
            temp.insert(temp.end(), rows_up_cols_right.begin(), rows_up_cols_right.end());
            map_temp[hang] = temp;
            hang++;
        }
        for (int i = 0; i <= (d_row1 - 1); i++)
        {
            vector<int16_t> temp;
            vector<int16_t> rows_down_cols_left(Map[0].Map.begin() + (i * (this->map_resolution + 1)) + this->map_resolution + 1 - d_col1, Map[0].Map.begin() + (i * (this->map_resolution + 1)) + this->map_resolution + 1);
            vector<int16_t> rows_down_cols_right(Map[5].Map.begin() + (i * (this->map_resolution + 1)) + 0, Map[5].Map.begin() + (i * (this->map_resolution + 1)) - col1);
            temp.insert(temp.end(), rows_down_cols_left.begin(), rows_down_cols_left.end());
            temp.insert(temp.end(), rows_down_cols_right.begin(), rows_down_cols_right.end());
            map_temp[hang] = temp;
            hang++;
        }
    }
    else if (col_left >= 0 && col_right > this->map_resolution && row_up >= 0 && row_down > this->map_resolution)
    {
        int hang = 0;
        int row1 = this->map_resolution - row_down;
        int col1 = this->map_resolution - col_right;
        int d_row1 = 2 * this->serch_radius + 1 + row1;
        int d_col1 = 2 * this->serch_radius + 1 + col1;
        for (int i = this->map_resolution + 1 - d_row1; i <= this->map_resolution; i++)
        {
            vector<int16_t> temp;
            vector<int16_t> rows_up_cols_left(Map[0].Map.begin() + (i * (this->map_resolution + 1)) + this->map_resolution + 1 - d_col1, Map[0].Map.begin() + (i * (this->map_resolution + 1)) + this->map_resolution + 1);
            vector<int16_t> rows_up_cols_right(Map[5].Map.begin() + (i * (this->map_resolution + 1)) + 0, Map[5].Map.begin() + (i * (this->map_resolution + 1)) - col1);
            temp.insert(temp.end(), rows_up_cols_left.begin(), rows_up_cols_left.end());
            temp.insert(temp.end(), rows_up_cols_right.begin(), rows_up_cols_right.end());
            map_temp[hang] = temp;
            hang++;
        }
        for (int i = 0; i <= ((-row1) - 1); i++)
        {
            vector<int16_t> temp;
            vector<int16_t> rows_down_cols_left(Map[7].Map.begin() + (i * (this->map_resolution + 1)) + this->map_resolution + 1 - d_col1, Map[7].Map.begin() + (i * (this->map_resolution + 1)) + this->map_resolution + 1);
            vector<int16_t> rows_down_cols_right(Map[8].Map.begin() + (i * (this->map_resolution + 1)) + 0, Map[8].Map.begin() + (i * (this->map_resolution + 1)) - col1);
            temp.insert(temp.end(), rows_down_cols_left.begin(), rows_down_cols_left.end());
            temp.insert(temp.end(), rows_down_cols_right.begin(), rows_down_cols_right.end());
            map_temp[hang] = temp;
            hang++;
        }
    }
    int size_temp = this->Map_match.size();
    if (size_temp < this->max_match_size)
    {
        this->Map_match.push_back(map_temp);
    }
    if (size_temp == this->max_match_size)
    {
        this->Map_match.erase(Map_match.begin());
        this->Map_match.push_back(map_temp);
    }
}

// 生成搜索地图序列2
void TERCOM::Serch_map_generator(int &count, const vector<int16_t> &Map, double &map_start_lat, double &map_start_lon, int &map_resolution)
{
    vector<vector<int16_t>> map_temp(2 * this->serch_radius + 1, vector<int16_t>(2 * this->serch_radius + 1, 0));
    int row = floor((map_start_lat - Match_sequence[count - 1].ins_lat) * map_resolution) - 1;
    int col = floor((Match_sequence[count - 1].ins_lon - map_start_lon) * map_resolution) - 1;
    // if (row < 0)
    // {
    //     row = 0;
    // }
    // if (col < 0)
    // {
    //     col = 0;
    // }
    // int height = Map[row * (map_resolution + 1) + col];
    // cout << height << endl;
    // cout << "生成方式1-时间戳：" << this->time_info << " ROW:" << row << " COL:" << col << endl;
    int hang = 0;
    for (int i = row - this->serch_radius; i <= row + this->serch_radius; i++)
    {
        vector<int16_t> rows(Map.begin() + (i * (map_resolution + 1)) + (col - this->serch_radius), Map.begin() + (i * (map_resolution + 1)) + (col + this->serch_radius) + 1);
        map_temp[hang] = rows;
        hang++;
    }
    int size_temp = this->Map_match.size();
    if (size_temp < this->max_match_size)
    {
        Map_match.push_back(map_temp);
    }
    if (size_temp == this->max_match_size)
    {
        Map_match.erase(this->Map_match.begin());
        Map_match.push_back(map_temp);
    }
}

// 生成搜索地图序列3（越界搜索地图）
void TERCOM::Serch_boundary_map_generator(int &count, vector<Map_database> &Map, double &lon_temp_left, double &lon_temp_right, double &lat_temp_up, double &lat_temp_down, double &map_start_lat, double &map_start_lon, int &map_resolution)
{
    double cell = 1.0 / static_cast<double>(map_resolution);
    vector<vector<int16_t>> map_temp(2 * this->serch_radius + 1, vector<int16_t>(2 * this->serch_radius + 1, 0));
    int row = floor((map_start_lat - Match_sequence[count - 1].ins_lat) * map_resolution) - 1;
    int col = floor((Match_sequence[count - 1].ins_lon - map_start_lon) * map_resolution) - 1;
    // if (row < 0)
    // {
    //     row = 0;
    // }
    // if (col < 0)
    // {
    //     col = 0;
    // }
    // int height = Map[0].Map[row * (map_resolution + 1) + col];
    // int hang = 0;
    // cout << "生成方式2-时间戳：" << this->time_info << " ROW:" << row << " COL:" << col << endl;
    if (lon_temp_left <= (map_start_lon + cell) && lon_temp_right < (map_start_lon + 1.0 + cell) && lat_temp_up < (map_start_lat - cell) && lat_temp_down > (map_start_lat - 1.0 - cell))
    {
        int hang = 0;
        int col1 = floor((Match_sequence[count - 1].ins_lon - map_start_lon) * map_resolution) - 1 + 1 + this->serch_radius;
        int d_col1 = 2 * this->serch_radius + 1 - col1;
        for (int i = row - this->serch_radius; i <= row + this->serch_radius; i++)
        {
            vector<int16_t> temp;
            vector<int16_t> row_left(Map[4].Map.begin() + (i * (map_resolution + 1)) + map_resolution - (d_col1 - 1), Map[4].Map.begin() + (i * (map_resolution + 1)) + map_resolution + 1);
            vector<int16_t> row_right(Map[0].Map.begin() + (i * (map_resolution + 1)), Map[0].Map.begin() + (i * (map_resolution + 1)) + (2 * this->serch_radius + 1) - d_col1);
            temp.insert(temp.end(), row_left.begin(), row_left.end());
            temp.insert(temp.end(), row_right.begin(), row_right.end());
            map_temp[hang] = temp;
            hang++;
        }
    }
    else if (lon_temp_left > (map_start_lon + cell) && lon_temp_right >= (map_start_lon + 1.0 + cell) && lat_temp_up < (map_start_lat - cell) && lat_temp_down > (map_start_lat - 1.0 - cell))
    {
        int hang = 0;
        int col1 = map_resolution - (floor((Match_sequence[count - 1].ins_lon - map_start_lon) * map_resolution) - 1) + 1 + this->serch_radius;
        int d_col1 = 2 * this->serch_radius + 1 - col1;
        for (int i = row - this->serch_radius; i <= row + this->serch_radius; i++)
        {
            vector<int16_t> temp;
            vector<int16_t> row_left(Map[0].Map.begin() + (i * (map_resolution + 1)) + map_resolution - (col1 - 1), Map[0].Map.begin() + (i * (map_resolution + 1)) + map_resolution + 1);
            vector<int16_t> row_right(Map[5].Map.begin() + (i * (map_resolution + 1)), Map[5].Map.begin() + (i * (map_resolution + 1)) + d_col1);
            temp.insert(temp.end(), row_left.begin(), row_left.end());
            temp.insert(temp.end(), row_right.begin(), row_right.end());
            map_temp[hang] = temp;
            hang++;
        }
    }
    else if (lon_temp_left > (map_start_lon + cell) && lon_temp_right < (map_start_lon + 1.0 + cell) && lat_temp_up >= (map_start_lat - cell) && lat_temp_down > (map_start_lat - 1.0 - cell))
    {
        int hang = 0;
        int row1 = floor((map_start_lat - Match_sequence[count - 1].ins_lat) * map_resolution) - 1 + 1 + this->serch_radius;
        int d_row1 = 2 * this->serch_radius + 1 - row1;
        for (int i = map_resolution - d_row1 + 1; i <= map_resolution; i++)
        {
            vector<int16_t> rows(Map[2].Map.begin() + (i * (map_resolution + 1)) + (col - this->serch_radius), Map[2].Map.begin() + (i * (map_resolution + 1)) + (col + this->serch_radius) + 1);
            map_temp[hang] = rows;
            hang++;
        }
        for (int i = 0; i <= row + this->serch_radius; i++)
        {
            vector<int16_t> rows(Map[0].Map.begin() + (i * (map_resolution + 1)) + (col - this->serch_radius), Map[0].Map.begin() + (i * (map_resolution + 1)) + (col + this->serch_radius) + 1);
            map_temp[hang] = rows;
            hang++;
        }
    }
    else if (lon_temp_left > (map_start_lon + cell) && lon_temp_right < (map_start_lon + 1.0 + cell) && lat_temp_up < (map_start_lat - cell) && lat_temp_down <= (map_start_lat - 1.0 - cell))
    {
        int hang = 0;
        int row1 = map_resolution - (floor((map_start_lat - Match_sequence[count - 1].ins_lat) * map_resolution) - 1) + 1 + this->serch_radius;
        int d_row1 = 2 * this->serch_radius + 1 - row1;
        for (int i = row - this->serch_radius; i <= map_resolution; i++)
        {
            vector<int16_t> rows(Map[0].Map.begin() + (i * (map_resolution + 1)) + (col - this->serch_radius), Map[0].Map.begin() + (i * (map_resolution + 1)) + (col + this->serch_radius) + 1);
            map_temp[hang] = rows;
            hang++;
        }
        for (int i = 0; i < d_row1; i++)
        {
            vector<int16_t> rows(Map[7].Map.begin() + (i * (map_resolution + 1)) + (col - this->serch_radius), Map[7].Map.begin() + (i * (map_resolution + 1)) + (col + this->serch_radius) + 1);
            map_temp[hang] = rows;
            hang++;
        }
    }
    else if (lon_temp_left <= (map_start_lon + cell) && lon_temp_right < (map_start_lon + 1.0 + cell) && lat_temp_up >= (map_start_lat - cell) && lat_temp_down > (map_start_lat - 1.0 - cell))
    {
        int hang = 0;
        int col1 = floor((Match_sequence[count - 1].ins_lon - map_start_lon) * map_resolution) - 1 + 1 + this->serch_radius;
        int row1 = floor((map_start_lat - Match_sequence[count - 1].ins_lat) * map_resolution) - 1 + 1 + this->serch_radius;
        int d_col1 = 2 * this->serch_radius + 1 - col1;
        int d_row1 = 2 * this->serch_radius + 1 - row1;
        for (int i = map_resolution - d_row1 + 1; i <= map_resolution; i++)
        {
            vector<int16_t> temp;
            vector<int16_t> row_left(Map[1].Map.begin() + (i * (map_resolution + 1)) + map_resolution - (d_col1 - 1), Map[1].Map.begin() + (i * (map_resolution + 1)) + map_resolution + 1);
            vector<int16_t> row_right(Map[2].Map.begin() + (i * (map_resolution + 1)), Map[2].Map.begin() + (i * (map_resolution + 1)) + (2 * this->serch_radius + 1) - d_col1);
            temp.insert(temp.end(), row_left.begin(), row_left.end());
            temp.insert(temp.end(), row_right.begin(), row_right.end());
            map_temp[hang] = temp;
            hang++;
        }
        for (int i = 0; i <= row + this->serch_radius; i++)
        {
            vector<int16_t> temp;
            vector<int16_t> row_left(Map[4].Map.begin() + (i * (map_resolution + 1)) + map_resolution - (d_col1 - 1), Map[4].Map.begin() + (i * (map_resolution + 1)) + map_resolution + 1);
            vector<int16_t> row_right(Map[0].Map.begin() + (i * (map_resolution + 1)), Map[0].Map.begin() + (i * (map_resolution + 1)) + (2 * this->serch_radius + 1) - d_col1);
            temp.insert(temp.end(), row_left.begin(), row_left.end());
            temp.insert(temp.end(), row_right.begin(), row_right.end());
            map_temp[hang] = temp;
            hang++;
        }
    }
    else if (lon_temp_left <= (map_start_lon + cell) && lon_temp_right < (map_start_lon + 1.0 + cell) && lat_temp_up < (map_start_lat - cell) && lat_temp_down <= (map_start_lat - 1.0 - cell))
    {
        int hang = 0;
        int col1 = floor((Match_sequence[count - 1].ins_lon - map_start_lon) * map_resolution) - 1 + 1 + this->serch_radius;
        // cout << col1 << endl;
        int row1 = map_resolution - (floor((map_start_lat - Match_sequence[count - 1].ins_lat) * map_resolution) - 1) + 1 + this->serch_radius;
        // cout << Match_sequence[count - 1].ins_lat << " " << Match_sequence[count - 1].ins_lon << endl;
        // cout << map_start_lat << endl;
        // cout << row1 << endl;
        int d_col1 = 2 * this->serch_radius + 1 - col1;
        // cout << d_col1 << endl;
        int d_row1 = 2 * this->serch_radius + 1 - row1;
        // cout << d_row1 << endl;
        for (int i = map_resolution - d_row1 + 1; i <= map_resolution; i++)
        {
            vector<int16_t> temp;
            vector<int16_t> row_left(Map[4].Map.begin() + (i * (map_resolution + 1)) + map_resolution - (d_col1 - 1), Map[4].Map.begin() + (i * (map_resolution + 1)) + map_resolution + 1);
            // cout << row_left.size() <<i<< endl;
            vector<int16_t> row_right(Map[0].Map.begin() + (i * (map_resolution + 1)), Map[0].Map.begin() + (i * (map_resolution + 1)) + (2 * this->serch_radius + 1) - d_col1);
            // cout << row_right.size() <<i<< endl;
            temp.insert(temp.end(), row_left.begin(), row_left.end());
            temp.insert(temp.end(), row_right.begin(), row_right.end());
            map_temp[hang] = temp;
            // cout << hang << endl;
            hang++;
        }
        for (int i = 0; i < d_row1; i++)
        {
            vector<int16_t> temp;
            vector<int16_t> row_left(Map[6].Map.begin() + (i * (map_resolution + 1)) + map_resolution - (d_col1 - 1), Map[6].Map.begin() + (i * (map_resolution + 1)) + map_resolution + 1);
            // cout << row_left.size() << "1" << endl;
            vector<int16_t> row_right(Map[7].Map.begin() + (i * (map_resolution + 1)), Map[7].Map.begin() + (i * (map_resolution + 1)) + (2 * this->serch_radius + 1) - d_col1);
            // cout << row_right.size() << "1" << endl;
            temp.insert(temp.end(), row_left.begin(), row_left.end());
            temp.insert(temp.end(), row_right.begin(), row_right.end());
            map_temp[hang] = temp;
            hang++;
        }
    }
    else if (lon_temp_left > (map_start_lon + cell) && lon_temp_right >= (map_start_lon + 1.0 + cell) && lat_temp_up >= (map_start_lat - cell) && lat_temp_down > (map_start_lat - 1.0 - cell))
    {
        int hang = 0;
        int col1 = map_resolution - (floor((Match_sequence[count - 1].ins_lon - map_start_lon) * map_resolution) - 1) + 1 + this->serch_radius;
        int row1 = floor((map_start_lat - Match_sequence[count - 1].ins_lat) * map_resolution) - 1 + 1 + this->serch_radius;
        int d_col1 = 2 * this->serch_radius + 1 - col1;
        int d_row1 = 2 * this->serch_radius + 1 - row1;
        for (int i = map_resolution - d_row1 + 1; i <= map_resolution; i++)
        {
            vector<int16_t> temp;
            vector<int16_t> row_left(Map[2].Map.begin() + (i * (map_resolution + 1)) + map_resolution - (col1 - 1), Map[2].Map.begin() + (i * (map_resolution + 1)) + map_resolution + 1);
            vector<int16_t> row_right(Map[3].Map.begin() + (i * (map_resolution + 1)), Map[3].Map.begin() + (i * (map_resolution + 1)) + d_col1);
            temp.insert(temp.end(), row_left.begin(), row_left.end());
            temp.insert(temp.end(), row_right.begin(), row_right.end());
            map_temp[hang] = temp;
            hang++;
        }
        for (int i = 0; i <= row + this->serch_radius; i++)
        {
            vector<int16_t> temp;
            vector<int16_t> row_left(Map[0].Map.begin() + (i * (map_resolution + 1)) + map_resolution - (col1 - 1), Map[0].Map.begin() + (i * (map_resolution + 1)) + map_resolution + 1);
            vector<int16_t> row_right(Map[5].Map.begin() + (i * (map_resolution + 1)), Map[5].Map.begin() + (i * (map_resolution + 1)) + d_col1);
            temp.insert(temp.end(), row_left.begin(), row_left.end());
            temp.insert(temp.end(), row_right.begin(), row_right.end());
            map_temp[hang] = temp;
            hang++;
        }
    }
    else if (lon_temp_left > (map_start_lon + cell) && lon_temp_right >= (map_start_lon + 1.0 + cell) && lat_temp_up < (map_start_lat - cell) && lat_temp_down <= (map_start_lat - 1.0 - cell))
    {
        int hang = 0;
        int col1 = map_resolution - (floor((Match_sequence[count - 1].ins_lon - map_start_lon) * map_resolution) - 1) + 1 + this->serch_radius;
        int row1 = map_resolution - (floor((map_start_lat - Match_sequence[count - 1].ins_lat) * map_resolution) - 1) + 1 + this->serch_radius;
        int d_col1 = 2 * this->serch_radius + 1 - col1;
        int d_row1 = 2 * this->serch_radius + 1 - row1;
        for (int i = map_resolution - d_row1 + 1; i <= map_resolution; i++)
        {
            vector<int16_t> temp;
            vector<int16_t> row_left(Map[0].Map.begin() + (i * (map_resolution + 1)) + map_resolution - (col1 - 1), Map[0].Map.begin() + (i * (map_resolution + 1)) + map_resolution + 1);
            vector<int16_t> row_right(Map[5].Map.begin() + (i * (map_resolution + 1)), Map[5].Map.begin() + (i * (map_resolution + 1)) + d_col1);
            temp.insert(temp.end(), row_left.begin(), row_left.end());
            temp.insert(temp.end(), row_right.begin(), row_right.end());
            map_temp[hang] = temp;
            hang++;
        }
        for (int i = 0; i < d_row1; i++)
        {
            vector<int16_t> temp;
            vector<int16_t> row_left(Map[7].Map.begin() + (i * (map_resolution + 1)) + map_resolution - (col1 - 1), Map[7].Map.begin() + (i * (map_resolution + 1)) + map_resolution + 1);
            vector<int16_t> row_right(Map[8].Map.begin() + (i * (map_resolution + 1)), Map[8].Map.begin() + (i * (map_resolution + 1)) + d_col1);
            temp.insert(temp.end(), row_left.begin(), row_left.end());
            temp.insert(temp.end(), row_right.begin(), row_right.end());
            map_temp[hang] = temp;
            hang++;
        }
    }
    int size_temp = this->Map_match.size();
    if (size_temp < this->max_match_size)
    {
        this->Map_match.push_back(map_temp);
    }
    if (size_temp == this->max_match_size)
    {
        this->Map_match.erase(Map_match.begin());
        this->Map_match.push_back(map_temp);
    }
}

// 读取地图（更新地图）
void TERCOM::map_read()
{
    int count = this->Match_sequence.size();
    int file_lat;
    int file_lon;
    if (this->start_flag == 0)
    {
        this->map_start_lat = floor(this->Ins_lat) + 1.0;
        this->map_start_lon = floor(this->Ins_lon);
        // this->map_resolution = 3600;
        file_lat = static_cast<int>(floor(this->Ins_lat)) + 1;
        file_lon = static_cast<int>(floor(this->Ins_lon));
    }
    else
    {
        this->map_start_lat = floor(this->Match_sequence[count - 1].ins_lat) + 1.0;
        this->map_start_lon = floor(this->Match_sequence[count - 1].ins_lon);
        // this->map_resolution=this
        file_lat = static_cast<int>(floor(this->Match_sequence[count - 1].ins_lat)) + 1;
        file_lon = static_cast<int>(floor(this->Match_sequence[count - 1].ins_lon));
    }
    // this->map_start_lat = floor(this->Ins_lat) + 1.0;
    // this->map_start_lon = floor(this->Ins_lon);
    // this->map_resolution = 3600;
    // int file_lat = static_cast<int>(floor(this->Ins_lat)) + 1;
    // int file_lon = static_cast<int>(floor(this->Ins_lon));
    vector<int> file_lat_9(9, 0);
    vector<int> file_lon_9(9, 0);
    // this->filename.resize(9);

    file_lat_9[0] = file_lat;
    file_lat_9[1] = file_lat + 1;
    file_lat_9[2] = file_lat + 1;
    file_lat_9[3] = file_lat + 1;
    file_lat_9[4] = file_lat;
    file_lat_9[5] = file_lat;
    file_lat_9[6] = file_lat - 1;
    file_lat_9[7] = file_lat - 1;
    file_lat_9[8] = file_lat - 1;

    file_lon_9[0] = file_lon;
    file_lon_9[1] = file_lon - 1;
    file_lon_9[2] = file_lon;
    file_lon_9[3] = file_lon + 1;
    file_lon_9[4] = file_lon - 1;
    file_lon_9[5] = file_lon + 1;
    file_lon_9[6] = file_lon - 1;
    file_lon_9[7] = file_lon;
    file_lon_9[8] = file_lon + 1;

    for (int i = 0; i < 9; i++)
    {
        ostringstream strs;
        strs << "integrated_navigation/"<<file_lat_9[i] << "_" << file_lon_9[i] << "_" << this->map_resolution;
        this->filename[i] = strs.str() + ".bin";
    }
    map_generator_9(this->mapdata, 9, this->filename);
}

// TERCOM
void TERCOM::Tercom_realization(int serch_mode)
{

    // PSO Start
    if (this->match_flag == 1) //) && this->serch_map_update_flag == 1)
    {
        int match_length = this->Match_sequence.size();
        vector<int> match_posi;
        if (serch_mode == 1)
        {
            int N = 100;
            int D = 2;
            int GEN = 150;

            match_posi = APSO(N, D, GEN);
        }
        if (serch_mode == 0)
        {
            match_posi = Ergodic();
        }
        // 定位反算,还需修改。
        double lon_result = static_cast<double>(match_posi[0] - this->serch_radius) / static_cast<double>(map_resolution);
        double lat_result = static_cast<double>(this->serch_radius - match_posi[1]) / static_cast<double>(map_resolution);
        this->match_lon = this->Match_sequence[match_length - 1].ins_lon + lon_result;
        this->match_lat = this->Match_sequence[match_length - 1].ins_lat + lat_result;
        this->match_altitude = match_posi[2];
        this->match_row = match_posi[1];
        this->match_col = match_posi[0];
    }
}

// TERCOM BY APSO
vector<int> TERCOM::APSO(int n, int D, int GEN)
{
    srand(time(nullptr));
    int match_length = this->Match_sequence.size();
    // vector<double>record(GEN);
    vector<vector<int>> posi(D, vector<int>(n));
    vector<vector<int>> posi_best(D, vector<int>(n));
    vector<vector<double>> v(2, vector<double>(n));
    vector<int> ans(3);
    int posilimitleft = 0;
    int posilimitright = 2 * this->serch_radius;
    double vlimitleft = -10;
    double vlimitright = 10;
    double w_max = 1.3;
    double w_min = 0.3;
    double w = 0;
    double c_start = 2.0;
    double c_end = 1.0;
    double c1 = 0;
    double c2 = 0;
    // 初始化种群位置
    for (int i = 0; i < D; i++)
    {
        for (int j = 0; j < n; j++)
        {
            posi[i][j] = posilimitleft + rand() % (posilimitright - posilimitleft);
        }
    }
    // 初始化种群速度
    for (int i = 0; i < n; i++)
    {
        v[0][i] = rand() % 10;
        v[1][i] = rand() % 10;
    }
    // 个体历史最优位置
    posi_best = posi;
    // 种群历史最优位置
    vector<int> posi_all_best;
    posi_all_best.push_back(0);
    posi_all_best.push_back(0);
    // 每个个体的历史最佳适应度
    vector<double> partical_best_fit(n);
    for (int i = 0; i < n; i++)
    {
        partical_best_fit[i] = -numeric_limits<double>::infinity();
    }
    // 种群历史最佳适应度
    double best_fit = -numeric_limits<double>::infinity();
    // 以上为APSO各参数初始化
    int iter = 1;
    while (iter <= GEN)
    {
        vector<double> fitness;
        fitness = cal_fitness(n, posi);
        double fitness_max = *max_element(fitness.begin(), fitness.end());
        double fitness_sum = accumulate(fitness.begin(), fitness.end(), 0.0);
        double fitness_ave = fitness_sum / n;
        // double fitness_min = *min_element(fitness.begin(), fitness.end());
        for (int i = 0; i < n; i++)
        {
            if (fitness[i] > partical_best_fit[i])
            {
                partical_best_fit[i] = fitness[i];
                posi_best[0][i] = posi[0][i];
                posi_best[1][i] = posi[1][i];
            }
        }
        double partical_best_fit_max = *max_element(partical_best_fit.begin(), partical_best_fit.end());
        auto max_posi = max_element(partical_best_fit.begin(), partical_best_fit.end());
        if (partical_best_fit_max > best_fit)
        {
            best_fit = partical_best_fit_max;
            posi_all_best[0] = posi[0][max_posi - partical_best_fit.begin()];
            posi_all_best[1] = posi[1][max_posi - partical_best_fit.begin()];
        }
        for (int i = 0; i < n; i++)
        {
            if (fitness[i] >= fitness_ave)
            {
                w = w_min + (w_max - w_min) * ((fitness_max - fitness[i]) / (fitness_max - fitness_ave));
            }
            else
            {
                w = w_max;
            }
            c1 = c_start - (c_start - c_end) * ((w - w_min) / (w_max - w_min));
            c2 = c_start + (c_start - c_end) * ((w - w_min) / (w_max - w_min));
            v[0][i] = v[0][i] * w + c1 * (1.0 - (static_cast<double>(iter) / GEN)) * static_cast<double>(posi_best[0][i] - posi[0][i]) + c2 * (static_cast<double>(iter) / GEN) * static_cast<double>(posi_all_best[0] - posi[0][i]);
            v[1][i] = v[1][i] * w + c1 * (1.0 - (static_cast<double>(iter) / GEN)) * static_cast<double>(posi_best[1][i] - posi[1][i]) + c2 * (static_cast<double>(iter) / GEN) * static_cast<double>(posi_all_best[1] - posi[1][i]);
            if (v[0][i] > vlimitright)
            {
                v[0][i] = vlimitright;
            }
            if (v[0][i] < vlimitleft)
            {
                v[0][i] = vlimitleft;
            }
            if (v[1][i] > vlimitright)
            {
                v[1][i] = vlimitright;
            }
            if (v[1][i] < vlimitleft)
            {
                v[1][i] = vlimitleft;
            }
            posi[0][i] = floor(posi[0][i] + v[0][i]);
            posi[1][i] = floor(posi[1][i] + v[1][i]);
            if (posi[0][i] > posilimitright)
            {
                posi[0][i] = posilimitright;
            }
            if (posi[0][i] < posilimitleft)
            {
                posi[0][i] = posilimitleft;
            }
            if (posi[1][i] > posilimitright)
            {
                posi[1][i] = posilimitright;
            }
            if (posi[1][i] < posilimitleft)
            {
                posi[1][i] = posilimitleft;
            }
        }
        // record[iter] = best_fit;
        iter++;
    }
    ans[0] = posi_all_best[0];
    ans[1] = posi_all_best[1];
    ans[2] = this->Map_match[match_length - 1][posi_all_best[1]][posi_all_best[0]];
    return ans;
}

// APSO CAL FITNESS
vector<double> TERCOM::cal_fitness(int n, vector<vector<int>> posi)
{
    vector<double> fitness(n);
    int match_size = this->Match_sequence.size();
    double height_mean = 0;
    for (int i = 1; i < n; i++)
    {
        height_mean = 0;
        // double/* measure_height = 0;
        // double real_height = 0;*/
        for (int j = 0; j < match_size; j++)
        {
            height_mean = height_mean + (this->Match_sequence[j].terrain_height - this->Map_match[j][posi[1][i]][posi[0][i]]) * (this->Match_sequence[j].terrain_height - this->Map_match[j][posi[1][i]][posi[0][i]]);
        }
        height_mean = height_mean / static_cast<double>(match_size);
        fitness[i] = 1.0 / height_mean;
    }
    return fitness;
}

// TERCOM BY ERGODIC
vector<int> TERCOM::Ergodic()
{
    int match_size = this->Match_sequence.size();
    vector<vector<double>> fitness(2 * this->serch_radius + 1, vector<double>(2 * this->serch_radius + 1));
    double height_sum = 0;
    vector<int> ans(3);
    for (int i = 0; i < (2 * this->serch_radius + 1); i++)
    {
        for (int j = 0; j < (2 * this->serch_radius + 1); j++)
        {
            height_sum = 0;
            for (int k = 0; k < match_size; k++)
            {
                height_sum = height_sum + (this->Match_sequence[k].terrain_height - this->Map_match[k][i][j]) * (this->Match_sequence[k].terrain_height - this->Map_match[k][i][j]);
            }
            height_sum = height_sum / static_cast<double>(match_size);
            fitness[i][j] = height_sum;
        }
    }
    int best_row = 0;
    int best_col = 0;
    double min_value = numeric_limits<double>::max();
    for (int i = 0; i < (2 * this->serch_radius + 1); i++)
    {
        for (int j = 0; j < (2 * this->serch_radius + 1); j++)
        {
            if (fitness[i][j] < min_value)
            {
                min_value = fitness[i][j];
                best_row = i;
                best_col = j;
            }
        }
    }

    ans[0] = best_col;
    ans[1] = best_row;
    ans[2] = this->Map_match[match_size - 1][best_row][best_col];
    return ans;
}

// 地形特征评价
double TERCOM::Match_analysis(int match_lon, int match_lat)
{
    double ans = 101.0;
    // 生成匹配所得地形序列
    int match_size = this->Match_sequence.size();
    vector<int16_t> Match_height(match_size, 0);
    for (int i = 0; i < match_size; i++)
    {
        Match_height[i] = this->Map_match[i][match_lat][match_lon];
    }

    // 实测地形标准差
    double height_real_sum = 0;
    for (int i = 0; i < match_size; i++)
    {
        height_real_sum += this->Match_sequence[i].terrain_height;
    }
    double height_real_mean = height_real_sum / match_size;
    double height_real_square = 0;
    for (int i = 0; i < match_size; i++)
    {
        height_real_square = height_real_square + ((this->Match_sequence[i].terrain_height - height_real_mean) * (this->Match_sequence[i].terrain_height - height_real_mean));
    }
    double height_std = sqrt(height_real_square / match_size);
    // 实测地形高程绝对值粗糙度
    double abs_sum = 0.0;
    for (int i = 0; i < (match_size - 1); i++)
    {
        abs_sum = abs_sum + abs(this->Match_sequence[i].terrain_height - this->Match_sequence[i + 1].terrain_height);
    }
    double abs_z = abs_sum / (match_size - 1);

    // 匹配地形标准差
    double height_match_sum = 0;
    for (int i = 0; i < match_size; i++)
    {
        height_match_sum += Match_height[i];
    }
    double height_match_mean = height_match_sum / match_size;
    double height_match_square = 0;
    for (int i = 0; i < match_size; i++)
    {
        height_match_square = height_match_square + ((Match_height[i] - height_match_mean) * (Match_height[i] - height_match_mean));
    }
    double height_match_std = sqrt(height_match_square / match_size);
    // 匹配地形高程绝对值粗糙度
    double abs_match_sum = 0;
    for (int i = 0; i < (match_size - 1); i++)
    {
        abs_match_sum = abs_match_sum + abs(Match_height[i] - Match_height[i + 1]);
    }
    double abs_match_z = abs_match_sum / (match_size - 1);

    ans = 0.1 * abs((height_match_mean / height_real_mean) - 1) + 0.5 * abs((height_match_std / height_std) - 1) + 0.4 * abs((abs_match_z / abs_z) - 1);
    return ans;
}

// 匹配评价
void TERCOM::Tercom_evaluation()
{
    int dist_flag = 0;
    int head_flag = 0;
    int k = 3;
    int count = this->Match_sequence.size();
    double dist_x_sequence;
    double dist_x_match;
    double dist_y_sequence;
    double dist_y_match;
    dist_x_sequence = (this->Match_sequence[count - 1].ins_lon - this->Match_sequence[count - 2].ins_lon) * 111000 * cos(this->Match_sequence[count - 1].ins_lat * 3.1415926 / 180.0);
    dist_y_sequence = (this->Match_sequence[count - 1].ins_lat - this->Match_sequence[count - 2].ins_lat) * 111000;
    dist_x_match = (this->match_lon - this->Match_sequence[count - 2].ins_lon) * 111000 * cos(this->match_lat * 3.1415926 / 180.0);
    dist_y_match = (this->match_lat - this->Match_sequence[count - 2].ins_lat) * 111000;
    double dist_sequence = sqrt(dist_x_sequence * dist_x_sequence + dist_y_sequence * dist_y_sequence);
    double dist_match = sqrt(dist_x_match * dist_x_match + dist_y_match * dist_y_match);
    if (dist_match < k * dist_sequence)
    {
        dist_flag = 1;
    }
    // 航向角象限转换
    double atan_sequence;
    double atan_match;
    if (dist_y_sequence > 0 && dist_x_sequence >= 0)
    {
        atan_sequence = (atan(dist_x_sequence / dist_y_sequence)) * 180.0 / 3.1415926;
    }
    else if (dist_y_sequence < 0 && dist_x_sequence >= 0)
    {
        atan_sequence = (atan(dist_x_sequence / dist_y_sequence)) * 180.0 / 3.1415926 + 180.0;
    }
    else if (dist_y_sequence > 0 && dist_x_sequence <= 0)
    {
        atan_sequence = (atan(dist_x_sequence / dist_y_sequence)) * 180.0 / 3.1415926 + 360.0;
    }
    else if (dist_y_sequence < 0 && dist_x_sequence <= 0)
    {
        atan_sequence = (atan(dist_x_sequence / dist_y_sequence)) * 180.0 / 3.1415926 + 180.0;
    }
    else if (dist_y_sequence == 0)
    {
        if (dist_x_sequence >= 0)
        {
            atan_sequence = 90.0;
        }
        else
        {
            atan_sequence = 270.0;
        }
    }
    if (dist_y_match > 0 && dist_x_match >= 0)
    {
        atan_match = (atan(dist_x_match / dist_y_match)) * 180.0 / 3.1415926;
    }
    else if (dist_y_match < 0 && dist_x_match >= 0)
    {
        atan_match = (atan(dist_x_match / dist_y_match)) * 180.0 / 3.1415926 + 180.0;
    }
    else if (dist_y_match > 0 && dist_x_match <= 0)
    {
        atan_match = (atan(dist_x_match / dist_y_match)) * 180.0 / 3.1415926 + 360.0;
    }
    else if (dist_y_match < 0 && dist_x_match <= 0)
    {
        atan_match = (atan(dist_x_match / dist_y_match)) * 180.0 / 3.1415926 + 180.0;
    }
    else if (dist_y_match == 0)
    {
        if (dist_x_match >= 0)
        {
            atan_match = 90.0;
        }
        else
        {
            atan_match = 270.0;
        }
    }
    if (abs(atan_sequence - atan_match) <= 15)
    {
        head_flag = 1;
    }
    if (dist_flag == 1 && head_flag == 1)
    {
        this->fusion_flag = 1;
    }
}

// 高度仿真
double height_simu(vector<double> &posi, vector<Map_database> &mapdata, int reso)
{
    srand(time(nullptr));
    double lat_boundary = floor(posi[1]) + 1.0;
    // cout << lat_boundary << endl;
    double lon_boundary = floor(posi[0]);
    // cout << lon_boundary << endl;
    int row = floor((lat_boundary - posi[1]) * reso - 1);
    int col = floor((posi[0] - lon_boundary) * reso - 1);
    int16_t height = mapdata[0].Map[row * (reso + 1) + col];
    unsigned seed = chrono::system_clock::now().time_since_epoch().count();
    default_random_engine generator(seed);
    normal_distribution<double> distribute(0.0, 5.0);
    double ans = static_cast<double>(height) + distribute(generator); // +rand() % 5;
    return ans;
}
