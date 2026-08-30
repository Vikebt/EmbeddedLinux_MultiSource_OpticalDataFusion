#include <unistd.h>
#include <iostream>
#include <random>
#include <chrono>
#include <ctime>
#include <fstream>
#include <filesystem>
#include <fstream>
#include<opencv2/opencv.hpp>
#include<opencv2/flann/flann.hpp>

using namespace std;
using namespace cv;



struct BaseImgInfo 
{

    cv::Mat baseImg;        // 基准图，正北
    int width = 0;        // 基准图像素宽度
    int height = 0;        // 基准图像素高度
    long int timestamp = 0; // 时间戳信息，相当于父结构
    int index_row=0;        // 在父结构中行号
    int index_col=0;        // 在父结构中列号


    //4个角的经纬度,左上，右上，左下，右下顺序   //左边2点，右边2点经度要相同，上边2点，下边2点纬度要相同
	cv::Point2d topLeft;           //x经度，y纬度
	cv::Point2d topRight;
	cv::Point2d bottomLeft;
	cv::Point2d bottomRight;


    double dlon=0;  //基准图横向，每像素表示的经度变化量
    double dlat=0;  //基准图纵向，每像素表示的纬度变化量


    double len_top=0;    //基准图左上、右上2点经度差
    double len_bot=0;    //基准图左下、右下2点经度差
    double len_lef=0;    //基准图左上、左下2点纬度差
    double len_rig=0;    //基准图右上、右下2点纬度差

    double bdRes=1;      //基准图纵向物面分辨率 pix/m，基准图纵向物面分辨率每一列都是相同的，同一条经线上，纬度差是固定的



};



int read_BaseImgInfo(std::string path, struct BaseImgInfo &bd)
{
     // 打开文本
    FILE *file = fopen(path.data(), "r");
    if (!file)
    {
        cout << "error! cant find baseImg info" << endl;
        return -1;
    }


    double px,py,lon,lat,height,res;
    double wid,hei;

    //左上
    std::fscanf(file, "%lf %lf %lf %lf", &px, &py, &lon, &lat);
    bd.topLeft = cv::Point2d(lon, lat);

    //右上
    std::fscanf(file, "%lf %lf %lf %lf", &wid, &py, &lon, &lat);
    bd.topRight = cv::Point2d(lon, lat);

    //左下
    std::fscanf(file, "%lf %lf %lf %lf", &px, &hei, &lon, &lat);
    bd.bottomLeft = cv::Point2d(lon, lat);

    //右下
    std::fscanf(file, "%lf %lf %lf %lf", &px, &py, &lon, &lat);
    bd.bottomRight = cv::Point2d(lon, lat);

    bd.width = wid;
    bd.height = hei;

    //基准图四条边的差值
    bd.len_top = bd.topRight.x - bd.topLeft.x;       // 基准图左上、右上2点经度差
    bd.len_bot = bd.bottomRight.x - bd.bottomLeft.x; // 基准图左下、右下2点经度差
    bd.len_lef = bd.topLeft.y - bd.bottomLeft.y;     // 基准图左上、左下2点纬度差
    bd.len_rig = bd.topRight.y - bd.bottomRight.y;   // 基准图右上、右下2点纬度差

    //
    bd.dlon=bd.len_top/wid;  //基准图横向，每像素表示的经度变化量
    bd.dlat=bd.len_lef/hei;  //基准图纵向，每像素表示的纬度变化量


    //基准图纵向物面分辨率
    double dlat = 111194.926644;             // 理想球体，同一条经线上，纬度每差一度时距离差，固定值
    bd.bdRes=dlat*bd.len_lef/hei;        //基准图纵向物面分辨率 pix/m，基准图纵向物面分辨率每一列都是相同的，同一条经线上，纬度差是固定的

    fclose(file);

    
    printf("giant baseImgInfo:\n");
    printf("baseImg size [%d %d] \n",int(wid),int(hei));
    printf("topLeft[%+3.12f %+3.12f]  topRight[%+3.12f %+3.12f]\n",bd.topLeft.x,bd.topLeft.y,bd.topRight.x,bd.topRight.y);
    printf("botLeft[%+3.12f %+3.12f]  botRight[%+3.12f %+3.12f]\n",bd.bottomLeft.x,bd.bottomLeft.y,bd.bottomRight.x,bd.bottomRight.y);
    printf("topWidth[%+3.12f]  lefHeight[%+3.12f]\n",bd.len_top,bd.len_lef);
    printf("botWidth[%+3.12f]  rigHeight[%+3.12f]\n",bd.len_bot,bd.len_rig);
    printf("imgdlon [%+3.12f]\n",bd.dlon);
    printf("imgdlat [%+3.12f]\n",bd.dlat);
    printf("baseImg Res[%.4fm/pix]\n\n\n",bd.bdRes);


    return 1;
}





//////////////////////////////////////////////////////////////////////////
//
//
//   microBaseInfo.txt
//   存放所有分块基准图信息，第一行数据是基准图的物面分辨率
//   从第二行起格式如下
//   自定义信息 左上点经度 左上点纬度 右上点经度 右上点纬度 左下点经度 左下点纬度 右下点经度 右下点纬度
//
//
/////////////////////////////////////////////////////////////////////////////////



int main()
{

    printf("======================================================================\n");
    printf("                           strat makeBaseImg\n"                          );
    printf("======================================================================\n\n\n");


    //用户设置参数
    int width=1800;      //小块基准图像素宽度
    int height=1300;     //小块基准图像素高度

    // 将时间点转换为自纪元以来的时间（秒）,根据系统时间生成时间戳，相当于此组基准图的父结构
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    auto seconds = std::chrono::duration_cast<std::chrono::seconds>(duration).count();
    long int timestamp = static_cast<double>(seconds);


    // 获取当前目录的绝对路径
    std::filesystem::path absolute_path = std::filesystem::current_path();


    //设置数据读取目录并判断是否存在
    std::string path=absolute_path.parent_path().string()+"/base";
    std::filesystem::path readfile(path.c_str()); //读取数据的文件目录
    if (!std::filesystem::exists(readfile))       //读取数据文件不存在则退出
    {
         printf("[error data path is not exist:%s\n",path.c_str());
         return -1;
    }


    // 基准图参数
    struct BaseImgInfo bd;
    std::string path_baseInfo = path + "/baseinfo.txt";
    read_BaseImgInfo(path_baseInfo, bd);


    // 加载基准图
    std::string path_baseImg = path + "/nanjing.tif";
    bd.baseImg = imread(path_baseImg, 1);
    if (!bd.baseImg.data) // 判断是否有数据
    {
        printf("[error] [%s]loading failed\n", path_baseImg.c_str());
        return -1;
    }
    //加载基准图后保存一个min版本用于看图

    // 压缩后输出
    cv::Size size;
    size.height = 3000;
    size.width = round(bd.baseImg.cols / (bd.baseImg.rows / 3000.0));
    cv::Mat img2;
    cv::resize(bd.baseImg, img2, size, INTER_CUBIC);

    // 输出分块图像
    char fname[800];
    sprintf(fname, "%s/minBaseImg.jpg", path.c_str());
    cv::imwrite(fname, img2);




    // 小块基准图数据
    printf("micro baseImgInfo:\n");
    printf("micro baseImg size       [%d %d]    \n", width, height);
    printf("micro baseImg res        [%.4fm/pix]\n", bd.bdRes);
    printf("micro baseImg height     [%.2fm]    \n", height * bd.bdRes);
    printf("micro baseImg timestamp  [%ld]     \n\n\n", timestamp);


    //存储文件
    std::ofstream ofresult; //结果文件流
    std::string path_microBaseInfo=path+"/microBaseInfo.txt";
    ofresult.open(path_microBaseInfo,ios::in);//读取文件的方式打开，判断文件是否存在
    if(!ofresult.is_open())//若文件不存在，则新建文件
    {

        printf("create new microBaseInfo.txt\n");
        ofresult.open(path_microBaseInfo);//新建文件 

    }
    else  //文件存在，再以追加方式打开
    {
        printf("open old microBaseInfo.txt\n");
        ofresult.close();//文件关闭，再以追加方式打开
        ofresult.open(path_microBaseInfo,ios::app);
    }

    printf("path_alldata        :%s\n",path.c_str());
    printf("path_baseinfo       :%s\n",path_baseInfo.c_str());
    printf("path_baseImg        :%s\n",path_baseImg.c_str());
    printf("path_microBaseInfo  :%s\n",path_microBaseInfo.c_str());
    printf("\n\n");


    //////////////////////////////////////
    //////////////////////////////////////



    //基准图宽度高度
    int allw=bd.baseImg.size().width;
    int allh=bd.baseImg.size().height;



    //划分数量
    int num_col=allw/width;    //划分行数,会丢弃最后无法划分的小块
    int num_row=allh/height;   //划分列数,会丢弃最后无法划分的小块


    // if(num_col*width!=allw||num_row*height!=allh)     
    // {
    //     printf("[error] please confirm BaseImg size[%d %d] and microBaseImg size[%d %d]\n",allw,allh,width,height);
    //     return -1;
    // }




    printf("col %d row %d\n",num_col,num_row);


    //开始拆分基准图
    //左上角点数据
    double slon=bd.topLeft.x ;  //左上角点经度
    double slat=bd.topLeft.y ;  //左上角点纬度
    int cnt=0;
    for (int i = 0; i < num_row; i++)       //第一行
    {
        for (int j = 0; j < num_col; j++)   //第一列
        {
            int w1 = j * width;
            int h1 = i * height;

            int w2 = j * width + width;
            int h2 = i * height + height;

            // 越界处理
            if (w2 > allw)
            {
                w2 = allw;
            }

            if (h2 > allh)
            {
                h2 = allh;
            }

            cv::Point pt1(w1, h1);  //micro 左上
            cv::Point pt2(w2, h1);  //micro 右上
            cv::Point pt3(w1, h2);  //micro 左下
            cv::Point pt4(w2, h2);  //micro 右下

            cv::Rect s = cv::Rect(pt1, pt4);
            cv::Mat img = bd.baseImg(s);

            int iw=img.size().width;
            int ih=img.size().height;



            //计算图像四点的经纬度，左上，右上，左下，右下的顺序,经纬度都只保留到小数点后7位
            cv::Point2d gpt1(slon+w1*bd.dlon,slat-h1*bd.dlat);
            cv::Point2d gpt2(slon+w2*bd.dlon,slat-h1*bd.dlat);
            cv::Point2d gpt3(slon+w1*bd.dlon,slat-h2*bd.dlat);
            cv::Point2d gpt4(slon+w2*bd.dlon,slat-h2*bd.dlat);


            // 输出分块图像
            char fname[800];    
            sprintf(fname, "%s/%.6f-%.6f.jpg", path.c_str(),gpt1.x,gpt1.y);
            cv::imwrite(fname, img);


            // 输出分块信息
            sprintf(fname, "%ld %d %d %.12f %.12f %.12f %.12f %.12f %.12f %.12f %.12f %d %d %.3f",timestamp,i,j,gpt1.x,gpt1.y,gpt2.x,gpt2.y,gpt3.x,gpt3.y,gpt4.x,gpt4.y,iw,ih,bd.bdRes);
            ofresult << fname << endl;

            cnt++;
        }
    }

    printf("create %d micro baseImg\n",cnt);
    printf("exit makeBaseImg\n");
    return 0;
}

