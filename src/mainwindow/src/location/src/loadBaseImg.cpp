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


//融合的大基准图信息
struct briefBaseInfo
{
    std::vector<struct BaseImgInfo> vec_bd; //所有基准图信息
    int width = 0;        // 基准图像素宽度
    int height = 0;        // 基准图像素高度

    int swidth=0;    //组合的小块基准图宽度
    int sheight=0;    //组合的小块基准图高度

    int num_col=0;   //列数，表示1行有多少列小基准图
    int num_row=0;   //行数，表示1列有多少行小基准图

    //左上角点经纬度
    cv::Point2d topLeft;           //x经度，y纬度


    double dlon=0;  //基准图横向，每像素表示的经度变化量
    double dlat=0;  //基准图纵向，每像素表示的纬度变化量
    double bdRes=1;      //基准图纵向物面分辨率 pix/m，基准图纵向物面分辨率每一列都是相同的，同一条经线上，纬度差是固定的

};


bool read_microBaseImgInfo(std::string path, std::vector<struct BaseImgInfo> &vec_bd,struct briefBaseInfo &gbd)
{


    // 打开文本
    FILE *file = fopen(path.data(), "r");
    if (!file)
    {
        cout << "error! cant find microBaseinfo.txt" << endl;
        return -1;
    }

    //
    vec_bd.clear();

    bool isFirst =true;

    // 小块基准图标准宽度，默认为第一张数据，读取后续数据若不符合，则报错
    int swidth = 0;
    int sheight = 0;
    double sdlon = 0;
    double sdlat = 0;


    //读基准图信息
    while (!feof(file))
    {


        long int timestamp;
        double p1,p2,p3,p4,p5,p6,p7,p8,res;
        int w,h;
        int id1,id2;
        std::fscanf(file, "%ld %d %d %lf %lf %lf %lf %lf %lf %lf %lf %d %d %lf",&timestamp,&id1,&id2, &p1, &p2,&p3,&p4,&p5,&p6,&p7,&p8,&w,&h,&res);




        struct BaseImgInfo bd;
        bd.topLeft = cv::Point2d(p1, p2);
        bd.topRight = cv::Point2d(p3, p4);
        bd.bottomLeft = cv::Point2d(p5, p6);
        bd.bottomRight = cv::Point2d(p7, p8);


        bd.bdRes=res;
        bd.width = w;
        bd.height = h;
        bd.timestamp=timestamp;
        bd.index_row=id1;
        bd.index_col=id2;
  


        // 基准图四条边的差值
        bd.len_top = bd.topRight.x - bd.topLeft.x;       // 基准图左上、右上2点经度差
        bd.len_bot = bd.bottomRight.x - bd.bottomLeft.x; // 基准图左下、右下2点经度差
        bd.len_lef = bd.topLeft.y - bd.bottomLeft.y;     // 基准图左上、左下2点纬度差
        bd.len_rig = bd.topRight.y - bd.bottomRight.y;   // 基准图右上、右下2点纬度差

        //
        bd.dlon = bd.len_top / w; // 基准图横向，每像素表示的经度变化量
        bd.dlat = bd.len_lef / h; // 基准图纵向，每像素表示的纬度变化量


        if(isFirst)
        {
            swidth=bd.width;
            sheight=bd.height;
            sdlon=bd.dlon;
            sdlat=bd.dlat;
            isFirst=false;

            if(bd.dlat==0&&bd.dlon==0)
            {
                cout << "error! microBaseImg.txt no data\n" << endl;
                return -1;
            }
        }
        else
        {
            if(bd.width!=swidth||bd.height!=sheight)
            {
    
                cout << "error! please check microBaseImg size\n" << endl;
                return -1;
            }
        }




        vec_bd.push_back(bd);


    }
    

    //判断最后一行是否重复读取，若读取则弹出最后一个
    int num = vec_bd.size();

    if(num==0)
    {
        cout << "error! microBaseImg.txt no data\n" << endl;
        return -1;
    }

    if (num >= 2)
    {
        if (vec_bd[num - 1].bottomRight.x == vec_bd[num - 2].bottomRight.x && vec_bd[num - 1].bottomRight.y == vec_bd[num - 2].bottomRight.y)
        {
            vec_bd.pop_back();
        }
    }

    fclose(file);
    printf("load %ld mirco baseImg\n",vec_bd.size());


    // 所有基准图信息，找到最边界四角点坐标
    double maxlon = -99999; // 最大经度
    double minlon = 99999;  // 最小经度
    double maxlat = -99999; // 最大纬度
    double minlat = 99999;  // 最小纬度
    for (int i = 0; i < vec_bd.size(); i++)
    {
        // 只取每个小块基准图左上角点和右下角点进行检测
        double lon = vec_bd[i].topLeft.x;
        double lat = vec_bd[i].topLeft.y;

        maxlon = max(maxlon, lon);
        minlon = min(minlon, lon);
        maxlat = max(maxlat, lat);
        minlat = min(minlat, lat);

        lon = vec_bd[i].bottomRight.x;
        lat = vec_bd[i].bottomRight.y;

        maxlon = max(maxlon, lon);
        minlon = min(minlon, lon);
        maxlat = max(maxlat, lat);
        minlat = min(minlat, lat);
    }

    // 计算大基准图的像素范围
    double gps_width = maxlon - minlon;
    double gps_height = maxlat - minlat;
    // 小块基准图的gps/pix
    double dlon = vec_bd[0].dlon;
    double dlat = vec_bd[0].dlat;
    // 小块基准图的像素宽度，高度
    int swid = vec_bd[0].width;
    int shei = vec_bd[0].height;
    // 大块基准图的像素宽度，高度
    int awid = round(gps_width / dlon);
    int ahei = round(gps_height / dlat);

    // 融合的大基准图信息
    gbd.vec_bd = vec_bd;
    gbd.width = awid;
    gbd.height = ahei;
    gbd.swidth = swid;
    gbd.sheight = shei;
    gbd.num_col = awid / swid;
    gbd.num_row = ahei / shei;
    gbd.dlon = dlon;
    gbd.dlat = dlat;
    gbd.bdRes = vec_bd[0].bdRes;
    gbd.topLeft = cv::Point2d(minlon, maxlat);

    printf("whole microBaseImg size   [%d %d]\n", awid, ahei);
    printf("whole microBaseImg row-col[%d %d]\n", gbd.num_row, gbd.num_col);
    printf("whole microBaseImg gps range:\n");
    printf("topLeft[%+3.12f %+3.12f]  topRight[%+3.12f %+3.12f]\n", minlon, maxlat, maxlon, maxlat);
    printf("botLeft[%+3.12f %+3.12f]  botRight[%+3.12f %+3.12f]\n", minlon, minlat, maxlon, minlat);
    printf("\n\n");

    return 1;
}



//输入：
//1  path,数据存放路径
//2  pt 光心点gps坐标
//3  大基准图基础信息
//4  返回值1：相机光心所属的基准图左上角点像素坐标
//5  返回值2：新建立的当前基准图
bool getNearArea(std::string path,cv::Point2d pt,briefBaseInfo gbd,cv::Point cpt,struct BaseImgInfo &cbd)
{
    

    // 和大块基准图左上角点的经纬度差
    double zdlon = pt.x - gbd.topLeft.x;
    double zdlat = gbd.topLeft.y - pt.y;

    // 距离大块基准图左上角点的像素距离,计算对应区域
    int sw = round(zdlon / gbd.dlon);
    int sh = round(zdlat / gbd.dlat);


    //光心gps坐标及像素坐标
    printf("camera centre  gps[%.12f %.12f] pix[%d %d] \n",pt.x,pt.y,sw,sh);


    //检查光心是否越界
    if (sw < 0 || sh < 0 || sw  > gbd.width-50 || sh  > gbd.height-50)
    {
        printf("[error] camera centre gps[%.12f %.12f] pix[%d %d] out of border\n",pt.x,pt.y,sw,sh);
        return -1;
    }


    //确定光心所属小块基准图在大基准图中的像素坐标及经纬度坐标
    int spx=sw/gbd.swidth;
    int spy=sh/gbd.sheight;
    spx=spx*gbd.swidth;
    spy=spy*gbd.sheight;      //像素坐标取整到左上角点
    cpt.x=spx;
    cpt.y=spy;


    double slon=gbd.topLeft.x+spx*gbd.dlon;
    double slat=gbd.topLeft.y-spy*gbd.dlat;


    //取基准图的行列号
    int col=spx/gbd.swidth;
    int row=spy/gbd.sheight;


    //光心所属小块基准图左上角点，gps坐标，像素坐标
    printf("camera topleft gps[%.12f %.12f] pix[%d %d] baseImg [%d %d] \n",slon,slat,spx,spy,row,col);


    //9块区域的起始行列号
    int srow=row-1;
    int scol=col-1;


    // 建立空图像，行数  列数
    cv::Mat img(3*gbd.sheight, 3*gbd.swidth, CV_8UC3, cv::Scalar(0, 0, 0));

    double minlon = 99999; // 最小经度
    double maxlat = -999999;
    for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
        {
            int crow=srow+i;
            int ccol=scol+j;
            //printf("micro base row-col[%d %d]\n",crow,ccol);

            //行号列号越界,越界就报错
            if(crow<0||ccol<0||crow>gbd.num_row-1||ccol>gbd.num_col-1)
            {
   
                printf("[error] row-col [%d %d] ouf of border\n",crow,ccol);
                return -1;
            }
            
            int ptx=ccol*gbd.swidth;
            int pty=crow*gbd.sheight;

            double gpsx = ptx * gbd.dlon;
            gpsx += gbd.topLeft.x;
            double gpsy = pty * gbd.dlat;
            gpsy = gbd.topLeft.y - gpsy;


            minlon=min(minlon,gpsx);
            maxlat=max(maxlat,gpsy);

            //printf("micro base gps[%.12f %.12f]\n",gpsx,gpsy);



            // 加载当前小块基准图
            char path_baseImg[500];
            sprintf(path_baseImg, "%s/%.6f-%.6f.jpg", path.c_str(), gpsx, gpsy);
            cv::Mat imgt = imread(path_baseImg, 1);
            if (!imgt.data) // 判断是否有数据
            {
                printf("[error] [%s]loading failed\n", path_baseImg);
                return -1;
            }

            int spx9=j*gbd.swidth;
            int spy9=i*gbd.sheight;


            // 复制到制定位置
            cv::Rect area = cv::Rect(spx9, spy9, gbd.swidth, gbd.sheight);
            imgt.copyTo(img(area));
        }
    }

    cbd.baseImg = img;
    cbd.width = 3*gbd.swidth;   // 基准图像素宽度
    cbd.height = 3*gbd.sheight; // 基准图像素高度
    cbd.dlon=gbd.dlon;  //基准图横向，每像素表示的经度变化量
    cbd.dlat=gbd.dlat;  //基准图纵向，每像素表示的纬度变化量
    cbd.bdRes=gbd.bdRes;
    cbd.topLeft=cv::Point2d(minlon,maxlat);           


    return 1;

}


//输入：
//1  path,数据存放路径
//2  v_pt  图像4角点gps坐标
//3  大基准图基础信息
//4  返回值1：相机光心所属的基准图左上角点像素坐标
//5  返回值2：新建立的当前基准图
bool getNearArea(std::string path,std::vector<cv::Point2d> v_pt,briefBaseInfo gbd,cv::Point cpt,struct BaseImgInfo &cbd)
{
    int minrow=99999999;
    int maxrow=-99999999;
    int mincol=99999999;
    int maxcol=-99999999;

    //遍历4角点，取行列号
    for(int k=0;k<v_pt.size();k++)
    {
        // 和大块基准图左上角点的经纬度差
        double zdlon = v_pt[k].x - gbd.topLeft.x;
        double zdlat = gbd.topLeft.y - v_pt[k].y;

        // 距离大块基准图左上角点的像素距离,计算对应区域
        int sw = round(zdlon / gbd.dlon);
        int sh = round(zdlat / gbd.dlat);

        //提供数据点的gps坐标及像素坐标
        //printf("%d point gps[%.12f %.12f] pix[%d %d] \n", k,v_pt[k].x, v_pt[k].y, sw, sh);

        // 检查点是否越界
        if (sw < 10 || sh < 10 || sw > gbd.width - 10 || sh > gbd.height - 10)
        {
            printf("[error] %d point gps[%.12f %.12f] pix[%d %d] out of border \n", k,v_pt[k].x, v_pt[k].y, sw, sh);
            return -1;
        }

        // 确定光心所属小块基准图在大基准图中的像素坐标及经纬度坐标
        int spx = sw / gbd.swidth;
        int spy = sh / gbd.sheight;
        spx = spx * gbd.swidth;
        spy = spy * gbd.sheight; // 像素坐标取整到左上角点
        cpt.x = spx;
        cpt.y = spy;

        double slon = gbd.topLeft.x + spx * gbd.dlon;
        double slat = gbd.topLeft.y - spy * gbd.dlat;

        // 取基准图的行列号
        int col = spx / gbd.swidth;
        int row = spy / gbd.sheight;

        // 光心所属小块基准图左上角点，gps坐标，像素坐标
        printf("%d point baseImg topleft gps[%.12f %.12f] pix[%d %d] baseImg [%d %d] \n", k, slon, slat, spx, spy, row, col);
   
        maxrow=max(maxrow,row);
		minrow=min(minrow,row);
		maxcol=max(maxcol,col);
		mincol=min(mincol,col);
   
    }

    printf("row[%d-%d] col[%d-%d]\n",minrow,maxrow,mincol,maxcol);


    //拼接区域起始行列号
    int srow=minrow;
    int scol=mincol;

    int numrow=maxrow-minrow+1;
    int numcol=maxcol-mincol+1;


    // 建立空图像，行数  列数
    cv::Mat img(numrow*gbd.sheight, numcol*gbd.swidth, CV_8UC3, cv::Scalar(0, 0, 0));

    //
    double minlon = 99999; // 最小经度
    double maxlat = -999999;
    for(int i=0;i<numrow;i++)
    {
        for(int j=0;j<numcol;j++)
        {
            int crow=srow+i;
            int ccol=scol+j;
           
            //printf("micro base row-col[%d %d]\n",crow,ccol);

        
            int ptx=ccol*gbd.swidth;
            int pty=crow*gbd.sheight;

            double gpsx = ptx * gbd.dlon;
            gpsx += gbd.topLeft.x;
            double gpsy = pty * gbd.dlat;
            gpsy = gbd.topLeft.y - gpsy;


            minlon=min(minlon,gpsx);
            maxlat=max(maxlat,gpsy);

            //printf("micro base gps[%.12f %.12f]\n",gpsx,gpsy);



            // 加载当前小块基准图
            char path_baseImg[500];
            sprintf(path_baseImg, "%s/%.6f-%.6f.jpg", path.c_str(), gpsx, gpsy);
            cv::Mat imgt = imread(path_baseImg, 1);
            if (!imgt.data) // 判断是否有数据
            {
                printf("[error] [%s]loading failed\n", path_baseImg);
                return -1;
            }

            int spx9=j*gbd.swidth;
            int spy9=i*gbd.sheight;


            // 复制到制定位置
            cv::Rect area = cv::Rect(spx9, spy9, gbd.swidth, gbd.sheight);
            imgt.copyTo(img(area));
        }
    }

    cbd.baseImg = img;
    cbd.width = numcol*gbd.swidth;   // 基准图像素宽度
    cbd.height = numrow*gbd.sheight; // 基准图像素高度
    cbd.dlon=gbd.dlon;  //基准图横向，每像素表示的经度变化量
    cbd.dlat=gbd.dlat;  //基准图纵向，每像素表示的纬度变化量
    cbd.bdRes=gbd.bdRes;
    cbd.topLeft=cv::Point2d(minlon,maxlat);           


    return 1;

}



int main()
{

    printf("======================================================================\n");
    printf("                           strat loadBaseImg\n"                          );
    printf("======================================================================\n\n\n");



    // 获取当前目录的绝对路径
    std::filesystem::path absolute_path = std::filesystem::current_path();


    //设置数据读取目录并判断是否存在
    std::string path=absolute_path.parent_path().string()+"/data-615-0/baseImg";        
    std::filesystem::path readfile(path.c_str()); //读取数据的文件目录
    if (!std::filesystem::exists(readfile))       //读取数据文件不存在则退出
    {
         printf("[error data path is not exist:%s\n",path.c_str());
         return -1;
    }



    //加载所有基准图参数
    std::vector<struct BaseImgInfo> vec_bd;
    struct briefBaseInfo gbd;
    std::string path_microBaseInfo=path+"/data-615-0/baseImg/microBaseInfo.txt";


    printf("path_alldata        :%s\n",path.c_str());
    printf("path_microBaseInfo  :%s\n",path_microBaseInfo.c_str());
    printf("\n\n");


    if(!read_microBaseImgInfo(path_microBaseInfo,vec_bd,gbd))
    {
        return -1;  //micro基准图数据有误
    }





    //测试功能1，加载所有基准图拼接后保存
    if(1)
    {

        // 建立空图像，行数  列数
        cv::Mat img(gbd.height, gbd.width, CV_8UC3, cv::Scalar(0, 0, 0));

        //遍历所有基准图整合成大图
        for(int i=0;i<vec_bd.size();i++)
        {
            //小块基准图左上角经纬度
            double lon=vec_bd[i].topLeft.x;
            double lat=vec_bd[i].topLeft.y;

            // 和大块基准图左上角点的经纬度差
            double zdlon = lon - gbd.topLeft.x;
            double zdlat = gbd.topLeft.y - lat;

            // 距离大块基准图左上角点的像素距离,计算对应区域
            int sw = round(zdlon / gbd.dlon);
            int sh = round(zdlat / gbd.dlat);
            cv::Rect area = cv::Rect(sw, sh, gbd.swidth, gbd.sheight);
           
            //printf("%4d topleft[%5d %5d]  bottomRight[%5d %5d] \n", i, sw, sh,sw+swid,sh+shei);

            //数据有误，图像越界
            if(sw<0||sh<0||sw+gbd.swidth>gbd.width||sh+gbd.sheight>gbd.height)
            {
                printf("[error] out of border %4d topleft[%5d %5d]  bottomRight[%5d %5d] \n", i, sw, sh,sw+gbd.swidth,sh+gbd.sheight);
                return -1;
            }

    
            //加载当前小块基准图
            char path_baseImg[500];
            sprintf(path_baseImg, "%s/%.6f-%.6f.jpg", path.c_str(),lon,lat);
            cv::Mat imgt = imread(path_baseImg, 1);
            if (!imgt.data) // 判断是否有数据
            {
                printf("[error] [%s]loading failed\n", path_baseImg);
                return -1;
            }


            //复制到制定区域
            imgt.copyTo(img(area));

        
        }


        //压缩后输出
		cv::Size imgs;
		imgs.height = 1000;
		imgs.width = round(img.cols / (img.rows / 1000.0));
		cv::Mat img2;
		cv::resize(img, img2, imgs, INTER_CUBIC);

        // 输出分块图像
        char fname[800];
        sprintf(fname, "%s/mixBaseImg.jpg", path.c_str());
        cv::imwrite(fname, img2);

        printf("function 1 over\n\n\n");
    }



    //测试功能2，根据光轴点的gps坐标，加载9张图像，
    if(1)
    {

        //5 5 114.260237216950 22.336728572846 114.262919425964 22.336728572846 114.262919425964 22.334046363831
        //用户设置光心点gps坐标
        double plon=114.2612379;
        double plat= 22.3354250;


        cv::Point2d pt(plon,plat);
        cv::Point cpt;
        struct BaseImgInfo cbd;   
        getNearArea(path,pt,gbd,cpt,cbd);

        // 输出分块图像
        char fname[800];
        sprintf(fname, "%s/aeraBaseImg.jpg", path.c_str());
        cv::imwrite(fname, cbd.baseImg);

        printf("function 2 over\n\n\n");
    }



    // 测试功能3，根据图像4角点坐标，加载所有外界矩形区域，
    if (1)
    {

        //5 3 114.254872798920 22.336728572846 114.257555007935 22.334046363831 
        double plon1=114.2562379;    //用户设置左上角点坐标
        double plat1= 22.3354250;
        cv::Point2d pt1(plon1, plat1);


        //4 6 114.262919425964 22.339410781860 114.265601634979 22.336728572846 
        double plon2=114.26432379;    //用户设置右上角点坐标
        double plat2= 22.3374250;
        cv::Point2d pt2(plon2, plat2);


        //9 2 114.252190589905 22.325999736786 114.254872798920 22.323317527771 
        double plon3=114.25362379;    //用户设置左下角点坐标
        double plat3= 22.3244250;
        cv::Point2d pt3(plon3, plat3);

        //7 10 114.273648262024 22.331364154816 114.276330471039 22.328681945801
        double plon4 = 114.27502379; // 用户设置左下角点坐标
        double plat4 = 22.3294250;
        cv::Point2d pt4(plon4, plat4);


        std::vector<cv::Point2d> v_pt;
        v_pt.push_back(pt1);
        v_pt.push_back(pt2);
        v_pt.push_back(pt3);
        v_pt.push_back(pt4);


        cv::Point cpt;
        struct BaseImgInfo cbd;
        getNearArea(path, v_pt, gbd, cpt, cbd);


        // 输出分块图像
        char fname[800];
        sprintf(fname, "%s/aera2BaseImg.jpg", path.c_str());
        cv::imwrite(fname, cbd.baseImg);

        printf("function 3 over\n\n\n");
    }

    printf("exit loadBaseImg\n");
    return 0;
}


