#pragma once
#include <fstream>
#include<opencv2/opencv.hpp>
#include<opencv2/flann/flann.hpp>
//#include"newMatch.hpp"


using namespace std;
using namespace cv;


#define BYTE unsigned char 
#define R2D				(180.0/CV_PI)  //度和弧度变换
#define Deg2Rad( deg )	(deg)/ R2D		
#define Rad2Deg( Rad )	(Rad)* R2D	

//根据惯导定义旋转旋转顺序
enum order { RxRyRz, RxRzRy, RyRxRz, RyRzRx, RzRxRy, RzRyRx };


//相机参数结构  
struct cameraData 
{
	
	//等效焦距、主点坐标、畸变系数、相机内参矩阵、相机畸变矩阵
	double fx, fy, cx, cy, k1, k2, p1, p2, k3;
	cv::Mat intri, distortion;
};


//实时图像参数结构
struct imageData {

	// 实时图  
	int index;            //序号,相当于图像简称
	std::string name;     //图像名称
	double timestamp;     //时间戳
	cv::Mat img;          //正常情况，img为原始图像，大异视角，img为原始图像下半截区域

	int meanGray=0;//图像平均灰度
	int stdGray=0; //图像灰度方差

	//大异视角使用
	bool isHugeDev=false;       //是否为大倾角，默认不是
	cv::Mat img2;               //大异视角情况，img2为原始图像，正常情况，矩阵为空
       

	//相机拍摄时的初始经、纬、高  
	double lon;
	double lat;
	double height;

	
	//相机正下视变换到当前状态的姿态角，基准状态X指东（俯仰），Y指南（滚转），Z指地（偏航）先偏航，再俯仰，再滚转
	double rx,ry,rz;
	double dev;                              //表示相机光轴 和 铅垂线夹角 单位度
	cv::Mat R = cv::Mat::eye(3, 3, CV_64F);  //相机正下视状态的旋转矩阵
	order ord=RyRxRz;                        //先偏航，再俯仰，再滚转

	//信息情况
	char info[1000]="no info";
	double usetime=0;//匹配程序用时


	//实时图正下视变换矩阵
	cv::Mat H;  //正下视透视变换矩阵
	double res=0; //正下视物面分辨率
	double value=0;  //匹配结果的可信度 0～1；


	//左上，右上、左下、右下、光心点，在基准图的经纬度坐标
	std::vector<cv::Point2d> v_pt;


	//未使用u
	double maxr,minr;  //最大物面分辨率，最小物面分辨率，计算得出
	double meadnr;//有效区域的平均物面分辨率；




};


//小块基准图
struct BaseImgInfo
{

    cv::Mat img;        // 基准图，正北
    int width = 0;          // 基准图像素宽度
    int height = 0;         // 基准图像素高度
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

	//基准图海拔高度，默认为0，则实时数据是相对地面高度，
	double planeheight=0;


	///////////////////
	int srow=-1;
	int scol=-1;
	int nrow=-1;
	int ncol=-1;

};


//大块基准图信息
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
// //匹配函数
cv::Point TensorMatching(Mat baseGImg, Mat realGimg, Mat realMask,cv::Mat &corr_result, int rs);
    
class matching
{
private:
    
public:
    matching();
    ~matching();

public:
    std::string path_load;     //加载基准图路径
    std::string path_save;     //存储文件夹


    bool debug_save=true;
    bool debug_show=false;


    //飞行高度及物面分辨率倍数
    const int para_maxheight = 1000; // 无人机相对地面高度，必须小于此高度，单位米
    const int para_minheight = 50; 	 // 无人机相对地面高度，必须大于此高度，单位米
    const int para_ratio = 30;       // 基准图物面分辨率 / 实时图像物面分辨率，最大倍数

    //实时图像灰度及方差
    const int para_minstd = 18;		// 图像最低方差
    const int para_mingray = 20;	// 图像最低平均灰度
    const int para_maxgray = 220;	// 图像最高平均灰度

    //光轴与铅垂线夹角
    const int para_maxdev = 80;		// 相机光轴相对铅垂线倾角，必须小于此值
    const int para_cutdev = 65;		// 相机光轴相对铅垂线倾角，大于此值，则只取图像下半部分参数

///////////////////////////////////////////////////////
//
//
// 若相机光轴与铅垂线夹角 0～para_cutdev  正常处理
// 若相机光轴与铅垂线夹角 para_cutdev ~ para_maxdev 大异视角情况，只取图像下半区域，
// 若相机光轴与铅垂线夹角 > para_maxdev  无法处理
//
///////////////////////////////////////////////////////
///////////////////////////////////////
    struct BaseImgInfo lastbd;
    bool loadBase;
///////////////////////////////////////

public:
    //基准图,经纬度转像素坐标
    void degree2pixel(struct BaseImgInfo bdata, Point2d Dpt, Point2d& Mpt);

    //基准图，像素座标2经纬度
    void pixel2degree(struct BaseImgInfo bdata, Point2d Mpt, Point2d& Dpt);


    //计算旋转矩阵，输入角度，及旋转顺序
    cv::Mat GetRotation(double rx, double ry, double rz, order rd);


    // void ErrorExit(const char* message)
    // {
    // 	fprintf(stderr, message);
    // 	fflush(stdin);
    // 	getchar();
    // 	exit(1);
    // }






    //功能：调整单应矩阵
    cv::Mat AdjustHomography2(struct imageData id, Size& imgSize, cv::Mat H, double ratio, std::vector<cv::Point2f>&v_cpt);



    //处理大异视情况
    struct cameraData dealHugeDev(struct cameraData cd,struct imageData &id);

    //根据光心点坐标及姿态角计算图像4角点及光心点GPS坐标
    bool getImgGps(struct cameraData cd,struct imageData &id);



    //从所有基准图信息中获取当前基准图范围信息
    bool getLocalBaseInfo(briefBaseInfo gbd,struct imageData &id,struct BaseImgInfo &cbd);


    bool loadLocalBaseImg(briefBaseInfo gbd,struct imageData id,struct BaseImgInfo &cbd);



    //基准图截取匹配区域
    bool subMatchArea(struct BaseImgInfo bd, struct imageData id, cv::Mat &area,cv::Rect &rect_area,cv::Mat &showimg);



    cv::Point3d slovePnP(struct cameraData cd, struct BaseImgInfo bd, struct imageData &id,std::vector<cv::Point2f> v_cpt,cv::Point rp);


    /////////////////////////
    /////////////////////////
    int ImageMatching(const struct cameraData cam, const struct briefBaseInfo gbd, struct imageData &id, cv::Point3d &rgps);

};













