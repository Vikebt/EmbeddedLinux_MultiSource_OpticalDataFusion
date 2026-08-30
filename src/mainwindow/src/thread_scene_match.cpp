#include "thread_scene_match.h"
#include <QDir>

ThreadSceneMatch::ThreadSceneMatch(QObject *parent)
{
}

ThreadSceneMatch::~ThreadSceneMatch()
{
}

void ThreadSceneMatch::match()
{
    printf("======================================================================\n");
    printf("                           strat demo\n"                                 );
    printf("======================================================================\n\n\n");


    //调试模式 ,若设置debug_save 为true,会新建 save 目录下保存每帧匹配中间数据：基准图匹配区域subBaseImg ，校正后的实时图remapImg
    double min_error=10; //匹配误差阈值10米

    // 获取当前目录的绝对路径
    //std::filesystem::path absolute_path = std::filesystem::current_path();
    //std::string path=absolute_path.parent_path().string()+"/data-615-0";

    QString exe_folder = QDir::currentPath();
    
    QDir dir(exe_folder);
    if (dir.cdUp()) dir.cdUp(); // 进入上一层，再进入上一层（共两次）
    if (dir.cdUp()); 
    QString src_folder = dir.absolutePath();
       
    //设置数据读取目录并判断是否存在
    std::string path = src_folder.toStdString()+"/src/mainwindow/data";
    std::filesystem::path readfile(path.c_str()); //读取数据的文件目录
    if (!std::filesystem::exists(readfile))       //读取数据文件不存在则退出
    {
         printf("[error data path is not exist:%s\n",path.c_str());
         return;
    }


    //设置存储路径
    path_save=path+"/save";
    std::filesystem::path savefile(path_save.c_str()); // 设置要创建的文件夹名称
    //判断文件夹是否已存在，存在则删除文件夹
    if(std::filesystem::exists(savefile))
    {
        
        std::filesystem::remove_all(savefile);
    }
    //创建save文件夹
    if (!std::filesystem::create_directories(savefile)) 
    {
        //若文件夹存在,删除后再创建
        printf("[error] save path can not created:%s\n",path_save.c_str());
        return;
    }
  

    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    //-start 下方内容甲方按需修改
    /////////////////////////////////////////////////////////////////////////////////////////////////////////



    //设置所有保存结果
    std::ofstream ofresult1;      //结果文件流
    std::string path_result1=path_save+"/1error.txt";
    ofresult1.open(path_result1);


    //定位成功的所有结果
    std::ofstream ofresult2;      
    std::string path_result2=path_save+"/1success.txt";
    ofresult2.open(path_result2);


    //读取相机参数
    struct cameraData cd;
    std::string path_cameraInfo = path+"/camerainfo.txt";
    read_cameraInfo(path_cameraInfo,cd);


    // 读取IMU,GPS等参数
    std::string path_imgdata = path + "/data.txt";
    std::vector<struct imageData> vec_id;
    read_imageData(path_imgdata, vec_id);


    //基准图参数
    std::vector<struct BaseImgInfo> vec_bd;
    struct briefBaseInfo gbd;
    path_load = path+"/baseImg0.25";
    std::string path_baseInfo = path_load+"/microBaseInfo.txt";
    if(!read_microBaseImgInfo(path_baseInfo,vec_bd,gbd))
    {
        return;  //micro基准图数据有误
    }


    


    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    //-end
    /////////////////////////////////////////////////////////////////////////////////////////////////////////


    printf("path_alldata      :%s\n",path.c_str());
    printf("path_savedata     :%s\n",path_save.c_str());
    printf("path_cameraInfo   :%s\n",path_cameraInfo.c_str());
    printf("path_baseinfo     :%s\n",path_baseInfo.c_str());
    printf("path_imgdata      :%s\n",path_imgdata.c_str());
    printf("path_result1      :%s\n",path_result1.c_str());
    printf("path_result2      :%s\n",path_result2.c_str());
    printf("\n\n");



  
    //根据读取的数据进行匹配
    int valid_cnt=0;  //匹配成功中误差小于阈值的次数统计
    int error_cnt=0;  //匹配成功中误差大于阈值的次数统计
    double aver_error=0; //误差统计
    std::vector<cv::Point2d> resultPs, GPSPs; //用于保存GPS和匹配结果在基准图上的位置



    //遍历数据
    for(int i=0;i<vec_id.size();i++)
    {

        //获取实时图数据
        struct imageData id = vec_id[i];


        // 读取实时图并判断 读615数据
        char img_path[500];
        sprintf(img_path, "%s/%s", path.c_str(), id.name.c_str());
        cv::Mat img = imread(img_path, 0);//必须按照灰度图加载
        
      

        if (img.empty()) // 判断是否有数据
        {
            printf("[error] [%s]loading failed skip this frame\n", img_path);
            continue;
        }
        id.img = img;

        result_image = img;


        cv::Point3d mps;
        
        int flag = ImageMatching(cd, gbd, id, mps);
        if (flag != 1)
        {
            // 匹配失败情况：
            printf("%s\n",id.info);
            ofresult1 << id.info << endl;
            continue;
        }
        else
        {


            // 简单计算版本，适用于距离100km以内
            double dlat = 111194.926644;                   // 同一条经线上，纬度每差一度时距离差，固定值
            double setlat = id.lat;                        // 指定纬度
            double dlon = dlat * cos(setlat / 180 * M_PI); // 同一纬线上，经度每差一度，距离差，需要指定纬度
          
            // 定位误差
            double dx = (mps.x - id.lon)*dlon; //正表示偏东
            double dy = (mps.y - id.lat)*dlat; //正表示偏北
            double dxy=sqrt(dx*dx+dy*dy);

            //输出定位结果
            char info1[5000];
            sprintf(info1, "%s gps[%.8f %.8f] pnp[%.8f %.8f] bias[%+7.2f %+7.2f %+7.2f]",id.info, id.lon,id.lat, mps.x, mps.y, dx,dy,dxy);
            printf("%s\n",info1);
            ofresult2<<info1<<endl;
        }
        QImage qimg;
        cvMat2QImage(result_image, qimg);
        emit sig_single_match_finished(qimg, result_lon, result_lat);
    }

    printf("exit demo \n");
    emit sig_match_finished();
  
}



bool ThreadSceneMatch::read_cameraInfo(std::string path, cameraData &cd)
{
    // 打开文本
    FILE *file = fopen(path.data(), "r");
    if (!file)
    {
        std::cout << "error! cant find camera info" << endl;
        return -1;
    }


    double fx, fy, cx, cy;
    double k1, k2, p1, p2, k3;
    std::fscanf(file, "%lf %lf %lf %lf", &fx, &fy, &cx, &cy);
    std::fscanf(file, "%lf %lf %lf %lf %lf", &k1, &k2, &p1, &p2, &k3);
	fclose(file);
    


    cd.fx=fx;
    cd.fy=fy;
    cd.cx=cx;
    cd.cy=cy;
    cd.k1=k1;
    cd.k2=k2;
    cd.p1=p1;
    cd.p2=p2;
    cd.k3=k3;

    cd.intri=(cv::Mat_<double>(3, 3) << fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0);
    cd.distortion=(cv::Mat_<double>(5, 1) << k1,k2,p1,p2,k3);

    printf("read cameraInfo:\n");
    printf("fx [%+8.3f] fy [%+8.3f]  \n", fx,fy);
    printf("cx [%+8.3f] cy [%+8.3f]  \n", cx,cy);
    printf("k1 [%+8.6f] k2 [%+8.6f] k3 [%+8.6f] k4 [%+8.6f] k5 [%+8.6f]\n", k1,k2,p1,p2,k3);
    printf("\n\n");
    return 1;
}

bool ThreadSceneMatch::read_imageData(std::string path, std::vector<struct imageData> &vec_id)
{
    // 打开文本
    FILE *file = fopen(path.data(), "r");
    if (!file)
    {
        cout << "error! cant find img data" << endl;
        return -1;
    }


    vec_id.clear();
    std::string buff;
    int index= -9999;

 

    //读无人机，吊舱，相机光轴初始指北模式
    while (!feof(file))
    {

        // 读取ins_img文件（包含图像姿态、初始位置等信息）
        int IMG_CNT;
        char name[500];
        double lon, lat, height; // 无人机经纬高
        double rx1, ry1, rz1;    // 吊舱姿态角
        double rx2, ry2, rz2;    // 无人机姿态角

     
        std::fscanf(file, "%d %s %lf %lf %lf %lf %lf %lf %lf %lf %lf",&IMG_CNT,&name,&lon,&lat,&height,&rx1,&ry1,&rz1,&rx2,&ry2,&rz2);


        // //输出看结果
        // printf("img_cnt [%u] name [%s]\n", IMG_CNT, name);
        // printf("img_cnt [%u] lon [%.9f] lat [%.9f] height [%+8.3f] \n", IMG_CNT, lon, lat, height);
        // printf("img_cnt [%u] rx1 [%+5.2f] ry1 [%+5.2f] rz1 [%+5.2f] \n",IMG_CNT,rx1,ry1,rz1);
        // printf("img_cnt [%u] rx2 [%+5.2f] ry2 [%+5.2f] rz2 [%+5.2f] \n\n",IMG_CNT,rx2,ry2,rz2);
      

        cv::Mat R3=GetRotation(0,0,-90,RxRyRz);      //相机正下视坐标系 变换到 无人机基准IMU  //内旋
        cv::Mat R2=GetRotation(rx2,ry2,rz2,RxRyRz);  //无人机基准IMU 变换到 无人机当前IMU   //内旋
        cv::Mat R1=GetRotation(rx1,ry1,rz1,RxRyRz);  //无人机当前IMU 变换到 吊舱当前IMU   //内旋



        cv::Mat R4=GetRotation(90,0,90,RxRyRz);      //吊舱当前IMU 变换到 相机当前坐标系   //内旋
        cv::Mat R=R4*R1*R2*R3;                       //正下视变换到当前状态的旋转矩阵

       

        double rx =atan2(R.at<double>(1, 2),sqrt(R.at<double>(0, 2)*R.at<double>(0, 2)+R.at<double>(2, 2)*R.at<double>(2, 2)))/3.141592654*180;
        double ry =atan2(-R.at<double>(0, 2),R.at<double>(2, 2))/3.141592654*180;
        double rz =atan2(-R.at<double>(1, 0),R.at<double>(1, 1))/3.141592654*180;

        struct imageData id;
        id.index = IMG_CNT;
        id.name = name;
        id.lon = lon;
        id.lat = lat;
        id.height = height;


        // 
		cv::Mat tmat = R * (cv::Mat_<double>(3, 1) << 0, 0, 1.0);
		cv::Point3d pt2 = cv::Point3d(tmat);

        double b1=acos(pt2.z)/3.141592654*180;
       

        id.ord=RyRxRz;
        id.R = R;
        id.dev=b1;
        id.rx=rx;
        id.ry=ry;
        id.rz=rz;

        vec_id.push_back(id);
    }


    //判断最后一行是否重复读取，若读取则弹出最后一个
    int num=vec_id.size();
    if(vec_id[num-1].index==vec_id[num-2].index)
    {
        vec_id.pop_back();
    }




    printf("read imgdata size [%lu]\n\n\n",vec_id.size());
	fclose(file);

    return 1;
}

bool ThreadSceneMatch::read_microBaseImgInfo(std::string path, std::vector<struct BaseImgInfo> &vec_bd, briefBaseInfo &gbd)
{
    // 打开文本
    FILE *file = fopen(path.data(), "r");
    if (!file)
    {
        cout << "error! cant find mircoBaseinfo.txt" << endl;
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
    printf("whole microBaseImg Res    [%.2f]\n", gbd.bdRes);
    printf("whole microBaseImg gps range:\n");
    printf("topLeft[%+3.12f %+3.12f]  topRight[%+3.12f %+3.12f]\n", minlon, maxlat, maxlon, maxlat);
    printf("botLeft[%+3.12f %+3.12f]  botRight[%+3.12f %+3.12f]\n", minlon, minlat, maxlon, minlat);

    printf("\n\n");

    return 1;
}

void ThreadSceneMatch::Drawps(Mat &img, std::vector<cv::Point2d> srcps, int r, Scalar sc)
{
    for (int i = 0; i < srcps.size(); i++)
	{
		Point p;
		p.x = cvRound(srcps[i].x);
		p.y = cvRound(srcps[i].y);
		cv::circle(img, p, r, sc, -1);
		// if (i > 1)
		// 	cv::line(img, srcps[i - 1], srcps[i], sc, 4);
	}
}

void ThreadSceneMatch::cvMat2QImage(const cv::Mat &mat, QImage& image)
{
    switch(mat.type())
    {
        case CV_8UC1:
            // QImage构造：数据，宽度，高度，每行多少字节，存储结构
            image = QImage((const unsigned char*)mat.data, mat.cols, mat.rows, mat.step, QImage::Format_Grayscale8);
            break;
        case CV_8UC3:
            image = QImage((const unsigned char*)mat.data, mat.cols, mat.rows, mat.step, QImage::Format_RGB888).rgbSwapped();
            //image = image.rgbSwapped(); // BRG转为RGB(颜色变成灰色)
            //Qt5.14增加了Format_BGR888
            //image = QImage((const unsigned char*)mat.data, mat.cols, mat.rows, mat.cols * 3, QImage::Format_BGR888);
            break;
        case CV_8UC4:
            image = QImage((const unsigned char*)mat.data, mat.cols, mat.rows, mat.step, QImage::Format_ARGB32);
            break;
        case CV_16UC4:
            image = QImage((const unsigned char*)mat.data, mat.cols, mat.rows, mat.step, QImage::Format_RGBA64);
            image = image.rgbSwapped(); // BRG转为RGB
            break;
    }
}