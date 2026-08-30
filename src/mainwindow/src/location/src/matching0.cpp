#include "matching.h"

matching::matching()
{

}

matching::~matching()
{

}


//基准图,经纬度转像素坐标
void matching::degree2pixel(struct BaseImgInfo bdata, Point2d Dpt, Point2d& Mpt)
{
	// 原本单应计算方法，废弃
	// //建立单应变换基准点（基准图4个角）
	// std::vector<cv::Point2f> baseMps, baseDps;
	// baseMps.push_back(Point2f(0.0f, 0.0f));
	// baseMps.push_back(Point2f(float(bdata.img.cols), 0.0f));
	// baseMps.push_back(Point2f(0.0f, float(bdata.img.rows)));
	// baseMps.push_back(Point2f(float(bdata.img.cols), float(bdata.img.rows)));
	
	// baseDps.push_back(Point2f(bdata.topLeft.x, bdata.topLeft.y));
	// baseDps.push_back(Point2f(bdata.topRight.x, bdata.topRight.y));
	// baseDps.push_back(Point2f(bdata.bottomLeft.x, bdata.bottomLeft.y));
	// baseDps.push_back(Point2f(bdata.bottomRight.x, bdata.bottomRight.y));
	// std::vector<cv::Point2f> srcps, dstps;
	// //利用基准点对应，解算变换矩阵H（度到像素）
	// Mat H = getPerspectiveTransform(baseDps, baseMps);
	// //将Dpt从度转换到像素（Mpt）
	// srcps.push_back(Dpt);
	// perspectiveTransform(srcps, dstps, H);
	// Mpt = dstps[0];
	// return;


	//直接计算法
	cv::Point2d pt=bdata.topLeft;   //基准图左上角点经纬度
	double dx=Dpt.x-pt.x;           //经度差
	double dy=pt.y-Dpt.y;           //纬度差

	double dpx=dx/bdata.dlon;
	double dpy=dy/bdata.dlat;

	Mpt.x=dpx;
	Mpt.y=dpy;

}


//基准图，像素座标2经纬度
void matching::pixel2degree(struct BaseImgInfo bdata, Point2d Mpt, Point2d& Dpt)
{

	// 原本单应计算方法，废弃
	// //建立单应变换基准点（基准图4个角）
	// std::vector<cv::Point2f> baseMps, baseDps;
	// baseMps.push_back(Point2f(0.0f, 0.0f));
	// baseMps.push_back(Point2f(float(bdata.img.cols), 0.0f));
	// baseMps.push_back(Point2f(0.0f, float(bdata.img.rows)));
	// baseMps.push_back(Point2f(float(bdata.img.cols), float(bdata.img.rows)));


	// baseDps.push_back(Point2f(bdata.topLeft.x, bdata.topLeft.y));
	// baseDps.push_back(Point2f(bdata.topRight.x, bdata.topRight.y));
	// baseDps.push_back(Point2f(bdata.bottomLeft.x, bdata.bottomLeft.y));
	// baseDps.push_back(Point2f(bdata.bottomRight.x, bdata.bottomRight.y));

	// std::vector<cv::Point2f> srcps, dstps;
	// //利用基准点对应，解算变换矩阵H（像素到度）
	// cv::Mat H = getPerspectiveTransform(baseMps, baseDps);
	// srcps.push_back(Mpt);
	// //将Mpt从像素转换到度（Dpt）
	// perspectiveTransform(srcps, dstps, H);
	// Dpt = dstps[0];
	// return;

	// 直接计算法
	cv::Point2d pt = bdata.topLeft; // 基准图左上角点经纬度

	double dlon = Mpt.x * bdata.dlon;
	double dlat = Mpt.y * bdata.dlat;

	double lon = pt.x + dlon;
	double lat = pt.y - dlat;

	Dpt.x = lon;
	Dpt.y = lat;
}


//计算旋转矩阵，输入角度，及旋转顺序
cv::Mat matching::GetRotation(double rx, double ry, double rz, order rd)
{
	rx = Deg2Rad(rx);
	ry = Deg2Rad(ry);
	rz = Deg2Rad(rz);
	Mat Rx = (Mat_<double>(3, 3) << 1.0, 0.0, 0.0, 0.0, cos(rx), sin(rx),
			  0.0, -sin(rx), cos(rx));
	Mat Ry = (Mat_<double>(3, 3) << cos(ry), 0.0, -sin(ry), 0.0, 1.0, 0.0,
			  sin(ry), 0.0, cos(ry));
	Mat Rz = (Mat_<double>(3, 3) << cos(rz), sin(rz), 0.0, -sin(rz),
			  cos(rz), 0.0, 0.0, 0.0, 1.0);
	Mat R;
	switch (rd)
	{
	case RxRyRz:
		R = (Rx * Ry * Rz);
		break;
	case RxRzRy:
		R = (Rx * Rz * Ry);
		break;
	case RyRxRz:
		R = (Ry * Rx * Rz);
		break;
	case RyRzRx:
		R = (Ry * Rz * Rx);
		break;
	case RzRxRy:
		R = (Rz * Rx * Ry);
		break;
	case RzRyRx:
		R = (Rz * Ry * Rx);
		break;
	}
	return R;
}





//功能：调整单应矩阵
cv::Mat matching::AdjustHomography2(struct imageData id, Size& imgSize, cv::Mat H, double ratio, std::vector<cv::Point2f>&v_cpt)
{


	std::vector<cv::Point2f> srcps, dstps;


	imgSize=id.img.size();
	cv::Point2f ptc(imgSize.width/2,imgSize.height/2); //光心点
	if(id.isHugeDev)
	{
		ptc.y=0;
	}

	cv::Point2f pt1(0,0); //左上
	cv::Point2f pt2(imgSize.width,0); //右上
	cv::Point2f pt3(0,imgSize.height);//左下
	cv::Point2f pt4(imgSize.width,imgSize.height);//右下

	srcps.push_back(pt1);
	srcps.push_back(pt2);
	srcps.push_back(pt3);
	srcps.push_back(pt4);



	//计算初次调整角点坐标，调整后会有角点不在原始分辨率范围内，
	perspectiveTransform(srcps, dstps, H);



	// 确定调整后角点坐标范围
	cv::Point2f topLeft = dstps[0];
	cv::Point2f topRight = dstps[1];
	cv::Point2f bottomLeft = dstps[2];
	cv::Point2f bottomRight = dstps[3];
	float minx = min(topLeft.x, min(topRight.x, min(bottomLeft.x, bottomRight.x)));
	float miny = min(topLeft.y, min(topRight.y, min(bottomLeft.y, bottomRight.y)));
	float maxx = max(topLeft.x, max(topRight.x, max(bottomLeft.x, bottomRight.x)));
	float maxy = max(topLeft.y, max(topRight.y, max(bottomLeft.y, bottomRight.y)));


	//调整角点坐标
	for(int i=0;i<dstps.size();i++)
	{
		//printf("ori-dpt[%8.3f  %8.3f]\n",dstps[i].x,dstps[i].y);
		dstps[i].x=(dstps[i].x-minx)*ratio;
		dstps[i].y=(dstps[i].y-miny)*ratio;
		//printf("new-dpt[%8.3f  %8.3f]\n",dstps[i].x,dstps[i].y);
	}


	//新的分辨率及机下点，
	cv::Point2f cpt;  //机下点
	cpt.x = round((imgSize.width / 2.0f - minx)*ratio); // 四舍五入
	cpt.y = round((imgSize.height / 2.0f - miny)*ratio);
	imgSize.width = cvRound((maxx - minx)*ratio);
	imgSize.height = cvRound((maxy - miny)*ratio);

	
	//printf("机下点：%.2f %.2f\n",cpt.x,cpt.y);
	//移到场景内后重新计算单应矩阵
	cv::Mat H2 = getPerspectiveTransform(srcps, dstps);  //必须point2f  并且仅仅支持4个点


	//根据H2矩阵再次计算,原始图像4角点，光心点，
	srcps.push_back(ptc);
	for(int i=0;i<srcps.size();i++)
	{
		cv::Point2f pt=srcps[i];
		cv::Mat tmat = H2 * (cv::Mat_<double>(3, 1) << pt.x,pt.y, 1.0);
		cv::Point3f pt2=cv::Point3f(tmat);
		cv::Point2f pt3(pt2.x/pt2.z,pt2.y/pt2.z);
		v_cpt.push_back(pt3);
	}
	v_cpt.push_back(cpt);//最后放入机下点


	//printf("光心点：%.2f %.2f\n",v_cpt[4].x,v_cpt[4].y);	

	// //
	// cv::Point2f pt5(0,0); //左上
	// cv::Point2f pt6(imgSize.width,0); //右上
	// cv::Point2f pt7(0,imgSize.height);//左下
	// cv::Point2f pt8(imgSize.width,imgSize.height);//右下
	// v_cpt.push_back(pt5);
	// v_cpt.push_back(pt6);
	// v_cpt.push_back(pt7);
	// v_cpt.push_back(pt8);






	return H2;
}


//处理大异视情况
struct cameraData matching::dealHugeDev(struct cameraData cd,struct imageData &id)
{
	cv::Mat img=id.img.clone();


	id.isHugeDev = true; // 将此图像标记为大倾角
	int wid = img.size().width;
	int hei = img.size().height;

	//取原始基准图下半区域
	cv::Rect rect(0, cvRound(hei / 2), wid, hei-cvRound(hei / 2));
	cv::Mat imghalf=img(rect);
	
	//更新
	id.img=imghalf;
	id.img2=img;
	


	//更新主点坐标
	struct cameraData newcd=cd;
	newcd.cy = cd.cy - cvRound(hei / 2);
	newcd.intri.at<double>(1,2)=newcd.cy;
	return newcd;
	
}


//根据光心点坐标及姿态角计算图像4角点及光心点GPS坐标
bool matching::getImgGps(struct cameraData cd,struct imageData &id)
{
	// 基本原理
	// 1、无人机GPS位置,计算当前4角点归一化平面坐标
	// 2、转到初始状态下四角点坐标，并计算直线方程
	// 3、根据无人机高度，计算直线与基准平面交点，
	// 4、转换为GPS坐标

 	// 简单计算版本，适用于距离100km以内 计算大基准图横向物面分辨率，纵向是固定的，横向在不同纬度不一样
    const double resdlat = 111194.926644;                        // 同一条经线上，纬度每差一度时距离差，固定值
    const double resdlon = resdlat * cos(id.lat / 180 * M_PI);   // 同一纬线上，经度每差一度，距离差，需要指定纬度
   
	// printf("pt  lon %.12f      lat %.12f\n",id.lon,id.lat);
	// printf("resdlon %.12f  resdlat %.12f\n",resdlon,resdlat);


	//计算归一化平面四角点，及主点在相机坐标系下坐标,并转换到基准图坐标系

	//普通四角点
	int W = id.img.cols, H = id.img.rows;
	//printf("W%d H%d\n",W,H);

	std::vector<cv::Point2d> imagePoints;
	cv::Point2d pt1(0,0);
	cv::Point2d pt2(W,0);
	cv::Point2d pt3(0,H);
	cv::Point2d pt4(W,H);
	imagePoints.push_back(pt1);
	imagePoints.push_back(pt2);
	imagePoints.push_back(pt3);
	imagePoints.push_back(pt4);

	//光心点坐标
	cv::Point2d pt5(W/2,H/2);           
	if (id.isHugeDev)   //大倾角情况,
	{
		pt5.y=0;
	}
	imagePoints.push_back(pt5);



	//实时图像，四角点的gps坐标，最后是光心点
	for (const auto &pt : imagePoints)
	{	

		//转到归一化平面坐标
		double tpx=(pt.x-cd.cx)/cd.fx;
		double tpy=(pt.y-cd.cy)/cd.fy;
		double tpz=1.0;

	
		
		//计算坐标系朝向 X-东 y-南 z-地面 此状态下四角点坐标
		cv::Mat tmat = id.R.inv() * (cv::Mat_<double>(3, 1) << tpx, tpy, 1.0);
		cv::Point3d pt2=cv::Point3d(tmat);
		// printf("4pt %.2f %.2f\n",pt.x,pt.y);
		// printf("4pt %.2f %.2f %.2f\n",tpx,tpy,tpz);
		// printf("4pt %.2f %.2f %.2f\n",pt2.x,pt2.y,pt2.z);
		
		//直线方向向量
		double lx=pt2.x;
		double ly=pt2.y;
		double lz=pt2.z;

		//计算直线与基准平面交点
		double z=id.height;
		double x=z*lx/lz;
		double y=z*ly/lz;

		// 以米为单位转成经纬度单位
		double lon2 = id.lon + x / resdlon;
		double lat2 = id.lat - y / resdlat;

		//存储
		cv::Point2d ptt(lon2,lat2);
		id.v_pt.push_back(ptt);
		
		//printf("gps %.12f %.12f  \n",ptt.x,ptt.y);

	}
	return 1;
}


//从所有基准图信息中获取当前基准图范围信息
bool matching::getLocalBaseInfo(briefBaseInfo gbd,struct imageData &id,struct BaseImgInfo &cbd)
{


	//取四角点及光心点坐标
	std::vector<cv::Point2d> v_pt = id.v_pt;

	//
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

        //gps坐标及像素坐标
        //printf("%d point gps[%.12f %.12f] pix[%d %d] \n", k,v_pt[k].x, v_pt[k].y, sw, sh);

        // 检查点是否越界
        if (sw < 20 || sh < 20 || sw > gbd.width - 20 || sh > gbd.height - 20)
        {
            sprintf(id.info,"errorType[%d] index [%d] name[%s] img gps[%.12f %.12f] pix[%d %d] out of whole BaseIMg border ",-7,id.index, id.name.c_str(),v_pt[k].x, v_pt[k].y, sw, sh);
			return false;
        }

        // 确定光心所属小块基准图在大基准图中的像素坐标及经纬度坐标
        int spx = sw / gbd.swidth;
        int spy = sh / gbd.sheight;
        spx = spx * gbd.swidth;
        spy = spy * gbd.sheight; 
  
        double slon = gbd.topLeft.x + spx * gbd.dlon;
        double slat = gbd.topLeft.y - spy * gbd.dlat;

        // 取基准图的行列号
        int col = spx / gbd.swidth;
        int row = spy / gbd.sheight;

        // 光心所属小块基准图左上角点，gps坐标，像素坐标
        // printf("%d point baseImg topleft gps[%.12f %.12f] pix[%d %d] baseImg [%d %d] \n", k, slon, slat, spx, spy, row, col);
   
        maxrow=max(maxrow,row);
		minrow=min(minrow,row);
		maxcol=max(maxcol,col);
		mincol=min(mincol,col);
   
    }

	//行列号区域
    //printf("row[%d-%d] col[%d-%d]\n",minrow,maxrow,mincol,maxcol);


    //拼接区域起始行列号
    int srow=minrow;
    int scol=mincol;

    int numrow=maxrow-minrow+1;
    int numcol=maxcol-mincol+1;


	// 拼接区域起始行列号
	cbd.srow = srow;
	cbd.scol = scol;
	cbd.nrow = numrow;
	cbd.ncol = numcol;

	//和上一帧基准图信息不符，需重新加载
	//printf("curr base %d %d %d %d\n",srow,scol,numrow,numcol);
	//printf("last base %d %d %d %d\n",lastbd.srow,lastbd.scol,lastbd.nrow,lastbd.ncol);
	if(srow!=lastbd.srow||scol!=lastbd.scol||numrow!=lastbd.nrow||numcol!=lastbd.ncol)
	{

		// printf("need load new\n");
		loadBase=true;
	}
	else
	{
	
		//printf("not need load new\n");
		loadBase=false;
		
	}


	return true;

}


bool matching::loadLocalBaseImg(briefBaseInfo gbd,struct imageData id,struct BaseImgInfo &cbd)
{

	//printf("load new baseImg\n");
	// 拼接区域起始行列号
	int srow = cbd.srow;
	int scol = cbd.scol;

	int numrow = cbd.nrow;
	int numcol = cbd.ncol;

	// 建立空图像，行数  列数
	cv::Mat img(numrow * gbd.sheight, numcol * gbd.swidth, CV_8UC1, cv::Scalar(0, 0, 0));

	//
	double minlon = 99999; // 最小经度
	double maxlat = -999999;
	for (int i = 0; i < numrow; i++)
	{
		for (int j = 0; j < numcol; j++)
		{
			int crow = srow + i;
			int ccol = scol + j;

			// printf("micro base row-col[%d %d]\n",crow,ccol);

			int ptx = ccol * gbd.swidth;
			int pty = crow * gbd.sheight;

			double gpsx = ptx * gbd.dlon;
			gpsx += gbd.topLeft.x;
			double gpsy = pty * gbd.dlat;
			gpsy = gbd.topLeft.y - gpsy;

			minlon = min(minlon, gpsx);
			maxlat = max(maxlat, gpsy);

			// printf("micro base gps[%.12f %.12f]\n",gpsx,gpsy);

			// 加载当前小块基准图
			char path_baseImg[500];
			sprintf(path_baseImg, "%s/%.6f-%.6f.jpg", path_load.c_str(), gpsx, gpsy);
			cv::Mat imgt = imread(path_baseImg, 0); // 必须灰度图模式加载
			if (!imgt.data)							// 判断是否有数据
			{
				sprintf(id.info, "errorType[%d] index [%d] [%s] loading baseImg failed", -8,id.index, path_baseImg);
				return false;
			}

			int spx9 = j * gbd.swidth;
			int spy9 = i * gbd.sheight;

			// 复制到制定位置
			cv::Rect area = cv::Rect(spx9, spy9, gbd.swidth, gbd.sheight);
			imgt.copyTo(img(area));
		}
	}

	double addlon=img.size().width*gbd.dlon;
	double addlat=img.size().height*gbd.dlat;
    cbd.width = numcol*gbd.swidth;   // 基准图像素宽度
    cbd.height = numrow*gbd.sheight; // 基准图像素高度
    cbd.dlon=gbd.dlon;  //基准图横向，每像素表示的经度变化量
    cbd.dlat=gbd.dlat;  //基准图纵向，每像素表示的纬度变化量
    cbd.bdRes=gbd.bdRes;
    cbd.topLeft=cv::Point2d(minlon,maxlat);
	cbd.topRight=cv::Point2d(minlon+addlon,maxlat);
	cbd.bottomLeft=cv::Point2d(minlon,maxlat-addlat);
	cbd.bottomRight=cv::Point2d(minlon+addlon,maxlat-addlat);
	cbd.img=img;

	
	return true;

}


//基准图截取匹配区域
bool matching::subMatchArea(struct BaseImgInfo bd, struct imageData id, cv::Mat &area,cv::Rect &rect_area,cv::Mat &showimg)
{

	//像素坐标
	int minx=99999999;
    int maxx=-99999999;
    int miny=99999999;
    int maxy=-99999999;

	//取四角点及光心点坐标
	std::vector<cv::Point2d> v_pt = id.v_pt;
	std::vector<cv::Point> v_pix;
	for(int i=0;i<v_pt.size();i++)
	{
		double lon=v_pt[i].x;
		double lat=v_pt[i].y;

		int x=(lon-bd.topLeft.x)/bd.dlon;
		int y=(bd.topLeft.y-lat)/bd.dlat;

		cv::Point pix(x,y);
		v_pix.push_back(pix);


		maxx=max(maxx,x);
		minx=min(minx,x);
		maxy=max(maxy,y);
		miny=min(miny,y);
	}


	int w1=maxx-minx;
	int h1=maxy-miny;
	int aw=2*w1/3;  //扩充边界
	int ah=2*h1/3;  //扩充边界



	//基准图边界扩充
	int BW = bd.img.cols, BH = bd.img.rows;// 实时图尺寸及基准图尺

	

	//printf("BW %d BH %d \n",BW,BH);
	if(minx>aw)
	{
		minx -= aw;
	}	
	else
	{
		minx=0;
	}


	if ( miny > ah)
	{
	
		miny -= ah;
	}
	else
	{
	
		miny = 0;
	}

	if (maxx + aw < BW )
	{
		maxx += aw;

	}
	else
	{
		maxx = BW;
	
	}

	if ( maxy + ah < BH)
	{
	
		maxy += ah;
	}
	else
	{
	
		maxy = BH;
	}
	



	//截取
	cv::Point pt1(minx, miny);
	cv::Point pt2(maxx, maxy);
	rect_area=cv::Rect(pt1, pt2);
	area = bd.img(rect_area);




	//printf("add pt1 %d %d pt2 %d %d\n",pt1.x,pt1.y,pt2.x,pt2.y);
	//printf("area w %d h %d\n",area.size().width,area.size().height);

	//输出showImg  
	cv::Mat img_color;
	cvtColor(area.clone(), img_color, cv::COLOR_GRAY2BGR);
	for (int i=0;i<v_pix.size();i++)
	{
		cv::Point p(v_pix[i].x-minx,v_pix[i].y-miny);
		cv::circle(img_color, p, 7, Scalar(0,255,0), -1);
		if(i==v_pix.size()-1)
		{
			cv::circle(img_color, p, 7, Scalar(0,255,0), -1);
		}
	}
	showimg=img_color;
	return true;
	
}



cv::Point3d matching::slovePnP(struct cameraData cd, struct BaseImgInfo bd, struct imageData &id,std::vector<cv::Point2f> v_cpt,cv::Point rp)
{
	/////////////////////////////////////////////////
	//根据光轴实际指向，多点PNP计算相机位姿
	/////////////////////////////////////////////////
	std::vector<cv::Point3f> v_wpt;         //物点坐标
	double allx=0;                         
	double ally=0;
	double allz=0;

	//物点坐标处理
	for (int i = 0; i < v_cpt.size(); i++)
	{
		cv::Point2f pt=v_cpt[i];
		pt.x+=rp.x;
		pt.y+=rp.y;

		cv::Point3f wpt;
		wpt.x=pt.x*bd.bdRes;
		wpt.y=pt.y*bd.bdRes;
		wpt.z=0;
		v_wpt.push_back(wpt);
		allx+=wpt.x;
		ally+=wpt.y;
		allz+=wpt.z;
	}

	//去质心归一化
	allx=allx/v_wpt.size();
	ally=ally/v_wpt.size();
	allz=allz/v_wpt.size();
	for (int i = 0; i < v_wpt.size(); i++)
	{
		v_wpt[i].x=v_wpt[i].x-allx;
		v_wpt[i].y=v_wpt[i].y-ally;
		v_wpt[i].z=v_wpt[i].z-allz;
	}



	//像点坐标坐标处理
	std::vector<cv::Point2f> v_pix; 
	for(int i=0;i<v_cpt.size();i++)
	{
		cv::Point2f pt=v_cpt[i];
		cv::Mat tmat = id.H.inv() * (cv::Mat_<double>(3, 1) << pt.x,pt.y, 1.0);
		cv::Point3f pt2=cv::Point3f(tmat);
		cv::Point2f pt3(pt2.x/pt2.z,pt2.y/pt2.z);
		v_pix.push_back(pt3);

		//printf("%.2f %.2f \n",pt3.x,pt3.y);
	}


	// for(int i=0;i<v_cpt.size();i++)
	// {
	// 	printf("wpt %.3f %.3f %.2f\n",v_wpt[i].x,v_wpt[i].y,v_wpt[i].z);
	// 	printf("ipx %.3f %.3f \n",v_pix[i].x,v_pix[i].y);
	// }
	


	//效果不好，
	//if(solvePnPRansac(v_wpt, v_pix, cd.intri, cd.distortion, rvec, tvec, false, 100, 1))

	//PNP计算
	cv::Mat rvec;  // 输出的旋转向量
    cv::Mat tvec;  // 输出的平移向量              //SOLVEPNP_ITERATIVE  //迭代法
	if(cv::solvePnP(v_wpt,v_pix,cd.intri,cd.distortion,rvec,tvec,false,SOLVEPNP_ITERATIVE))  //PNP普通迭代法
	{	
		cv::Mat R;
		cv::Rodrigues(rvec, R);
		cv::Mat T=-R.inv()*tvec;

		//基准图左下角坐标系下坐标
		cv::Point3f pt=cv::Point3d(T);
		pt.x+=allx;
		pt.y+=ally;
		pt.z+=allz;
		pt.z=-pt.z;

		//printf("id real height %.3f  %.6f\n",id.index,id.height,pt.z);

		//像素，像素 米
		cv::Point3d zpt(pt.x/bd.bdRes,pt.y/bd.bdRes,pt.z);
	

		return zpt;

	}
	else
	{
		cv::Point3d zpt(0, 0, 0);
		return zpt;
	}
}





/////////////////////////
/////////////////////////
int matching::ImageMatching(const struct cameraData cam, const struct briefBaseInfo gbd, struct imageData &id, cv::Point3d &rgps)
{


	// 获取开始时间
	auto timestamp_start = chrono::high_resolution_clock::now();


	//复制相机参数
	struct cameraData cd=cam;


	//判断高度，不在工作区间则报错
	if(id.height<para_minheight || id.height>para_maxheight)
	{
		//实时图像基础信息
	    sprintf(id.info,"errorType[%d] index [%d] name[%s] img height %.1fm out of [%d %dm]",-1,id.index,id.name.c_str(), id.height,para_minheight,para_maxheight);
		return -1;      
	}


	//判断倾角,默认80度
	if(id.dev>para_maxdev) 
	{
		//实时图像基础信息
	    sprintf(id.info,"errorType[%d] index [%d] name[%s] rxyz[%7.2f %7.2f %7.2f] dev[%6.2f]>%d",-2,id.index,id.name.c_str(), id.rx,id.ry,id.rz,id.dev,para_maxdev);
		return -2;
	}


	//判断图像灰度均值及方差
	cv::Scalar mgray, stdd;
	cv::meanStdDev(id.img, mgray, stdd);
	double mv = mgray.val[0];
	double st=stdd.val[0];
	id.meanGray=mv;
	id.stdGray=st;
	if (mv < para_mingray) // 图像过暗，平均灰度过低
	{
		sprintf(id.info, "errorType[%d] index [%d] name[%s] meangray[%.1f] < %d", -3,id.index,id.name.c_str(), mv, para_mingray);
		return -3;
	}
	if (mv > para_maxgray) // 图像过曝，平均灰度高
	{
		sprintf(id.info, "errorType[%d] index [%d] name[%s] meangray[%.1f] > %d", -4,id.index, id.name.c_str(),mv, para_maxgray);
		return -4;
	}
	if (st < para_minstd) // 图像场景单一，方差过小
	{
		sprintf(id.info, "errorType[%d] index [%d] name[%s] graystd [%.1f] < %d", -5,id.index,id.name.c_str(), st, para_minstd);
		return -5;
	}
	

	//////////////////////////////////////////////////////////////////////////
	//   实时图物面分辨率/基准图物面分辨率 = 缩放系数  例如 实时图（1m/pix）基准图（2m/pix），则缩放0.5倍
	//   实时图物面分辨率   = 相机相对高度/fx   相似三角形正下视情况
	//   基准图物面分辨率   已知
	/////////////////////////////////////////////////////////////////////////////

	//计算缩放因子
	double img_res=(id.height)*2/(cd.fx+cd.fy);
	double ratio=img_res/gbd.bdRes;
	if (ratio > 1 || ratio<(1./para_ratio)) // 实时图像与基准图物面分辨率相差过大
	{
		sprintf(id.info, "errorType[%d] index [%d] name[%s] H[%.1f] imgRes [%.3f] baseImgRes [%.3f] ratio %.2f", -6,id.index, id.name.c_str(),id.height, img_res, gbd.bdRes,1./ratio);
		return -6;
	}
	id.res=img_res;
	//printf("index [%4d] Res[%.3f] gary[%3d %3d] rxyz[%+7.2f %+7.2f %+7.2f] dev[%5.2f] H[%6.1f]", id.index, id.res, id.meanGray,id.stdGray, id.rx, id.ry, id.rz, id.dev,id.height);


	
	///////////////////////////////////////////////////////////////////////////////
	//    1 基准图处理环节
	//////////////////////////////////////////////////////////////////////////////




	//判断倾角,若为大异视角情况，只取下半截区域
	if(id.dev>para_cutdev) 
	{
		cd=dealHugeDev(cd,id);
	}


	//根据无人机gps坐标，及姿态倾角，计算四角点及光心点与基准图交点的gps坐标
	getImgGps(cd,id);



	//根据无人机GPS坐标，及相机姿态倾角加载局部基准图
	struct BaseImgInfo bd=lastbd;
	if(!getLocalBaseInfo(gbd,id,bd))
	{
		return -7;
	};


	if(loadBase)
	{
		if (!loadLocalBaseImg(gbd,id, bd))
		{

			return -8;
		}
		else
		{
			lastbd=bd;
		}
	}


	/////////////////////////////////////////////////////////////////////////////
	//   正下视初步校正
	//  【实时图坐标系下同名点集坐标】= R * 【基准状态坐标系下同名点集坐标】
	//  【实时图图像坐标】= 相机内参矩阵intri * 【实时图坐标系下同名点集坐标】
	//  【基准状态图像坐标】= 相机内参矩阵intri * 【基准状态坐标系下同名点集坐标】
	//  【基准状态图像坐标】= intri * R.inv() * intri.inv()
	/////////////////////////////////////////////////////////////////////////////



	//正下视校正
	cv::Mat img=id.img.clone();
	cv::Size imgsize = img.size();
	std::vector<cv::Point2f> v_cpt; // 四角点加光心点，左上，右上，左下，右下，光心点,
	cv::Mat H=cd.intri*id.R.inv()*cd.intri.inv();   //旋转矩阵变换得到的姿态角
	id.H = AdjustHomography2(id, imgsize, H, ratio, v_cpt);


	//图像校正
	cv::Mat remapImg;
	warpPerspective(img, remapImg, id.H, imgsize, INTER_CUBIC);


	// 实时图掩模
	cv::Mat mask = Mat(id.img.rows, id.img.cols, CV_8U, cv::Scalar::all(0));
	cv::Rect trect = cv::Rect(30, 30, id.img.cols - 60, id.img.rows - 60);
	mask(trect).setTo(1);
	warpPerspective(mask, mask, id.H, imgsize, INTER_CUBIC);




	//基准图截取匹配区域
	cv::Mat subBaseImg;
	cv::Mat showsubBaseImg;
	cv::Rect rect_area;
	subMatchArea(bd, id, subBaseImg, rect_area,showsubBaseImg);








	if (debug_save)
	{
		char fname[500];

		if(id.isHugeDev)
		{
			// 输出原始图像
			sprintf(fname, "%s/%d-0Img.jpg", path_save.c_str(), id.index);
			cv::imwrite(fname, id.img2);
		}


		//输出原始图像
		sprintf(fname, "%s/%d-1Img.jpg", path_save.c_str(), id.index);
		cv::imwrite(fname, id.img);

		
		//输出正下视矫正
		cv::Mat img_color1;
		cvtColor(remapImg, img_color1, cv::COLOR_GRAY2BGR);	  // 转为彩色

		for (int i = 0; i < v_cpt.size()-1; i++)  //画四角点，及光心点
		{
			cv::Point pt(int(v_cpt[i].x),int(v_cpt[i].y));
			cv::circle(img_color1, pt, 5, Scalar(0, 255, 0), -1); // 画出点,绿色
		}
		//cv::circle(img_color1, v_cpt[4], 5, Scalar(0, 0, 255), -1); // 画光轴点

		sprintf(fname, "%s/%d-2remapImg.jpg", path_save.c_str(), id.index);
		cv::imwrite(fname, img_color1);


		// 输出截取基准图区域
		sprintf(fname, "%s/%d-3subBaseImg.jpg", path_save.c_str(), id.index);
		cv::imwrite(fname, showsubBaseImg);

	}


	

	//验证基准图部分区域和校正后实时图分辨率大小
	if (remapImg.cols >= subBaseImg.cols-12 || remapImg.rows >= subBaseImg.rows-12)
	{
		//基本不会发生
		sprintf(id.info,"errorType[%d] index [%d] name[%s] remapImg size[%d %d] >= subBaseImg size[%d %d]",-9,id.index,id.name.c_str(),remapImg.cols,remapImg.rows,subBaseImg.cols,subBaseImg.rows);
		return -9;
	}
	

	//匹配环节，返回reamp图像左上角点在subBaseImg中坐标
	cv::Mat corr_mat;
    cv::Point rp1 = TensorMatching(subBaseImg, remapImg, Mat(), corr_mat,360);   //rp1 表示实时图像左上角点在局部基准图中坐标


	
	cv::Point rp2; //rp2表示实时图像左上角点在大基准图的像素坐标
	rp2.x=rp1.x+rect_area.x;
	rp2.y=rp1.y+rect_area.y;



	//基准图的定位结果，像素坐标 + 高度
	cv::Point3d ppt=slovePnP(cd, bd, id, v_cpt, rp2);


	//无人机gps坐标转像素坐标
	cv::Point2d gpt;
	degree2pixel(bd, cv::Point2d(id.lon,id.lat), gpt);

	// printf("gps定位结果  : %.2f %.2f pix\n",gpt.x,gpt.y);
	// printf("PNP定位结果  : %.2f %.2f  %.2f %.2f pix\n",ppt.x,ppt.y,gpt.x-ppt.x,gpt.y-ppt.y);
	

	//PNP定位结果转经纬度坐标
	cv::Point2d dpt;
	pixel2degree(bd, cv::Point2d(ppt.x,ppt.y), dpt);	
	rgps.x = dpt.x;
	rgps.y = dpt.y;
	rgps.z = ppt.z;



	if (debug_save||debug_show)
	{

		// subBaseImg 上画出定位结果和匹配框区域
		cv::Mat img_color1, img_color2;
		cvtColor(remapImg, img_color1, cv::COLOR_GRAY2BGR);	  // 转为彩色
		cvtColor(subBaseImg, img_color2, cv::COLOR_GRAY2BGR); // 转为彩色


		//定位框，并在img_color2画出
		cv::Rect rectimg=cv::Rect(rp1.x,rp1.y, remapImg.cols, remapImg.rows);
		cv::rectangle(img_color2, rectimg, Scalar(0, 0, 255), 2);  //画出红色匹配框
		


		//矫正后图像叠加显示
		cv::Mat timg=img_color2(rectimg);
		cv::Mat timg2;
		cv::addWeighted(timg,0.5,img_color1,0.5,1,timg2);
		timg2.copyTo(img_color2(rectimg));
		
	
		//无人机gps真值，及PNP定位结果
		cv::Point pt_gps=cv::Point(gpt.x-rect_area.x,gpt.y-rect_area.y);
		cv::Point pt_pnp=cv::Point(ppt.x-rect_area.x,ppt.y-rect_area.y);
	
		
		//img_color2上画定位结果
		cv::circle(img_color2, pt_gps, 5, Scalar(0, 255, 0), -1);//画出gps定位点，绿色，
		cv::circle(img_color2, pt_pnp, 5, Scalar(0, 0, 255), -1);//画出PNP定位点，红色
	

		//拼接	
		int imgw = subBaseImg.cols + remapImg.cols;
		int imgh = max(subBaseImg.rows, remapImg.rows);
		cv::Mat showImg = Mat(imgh, imgw, CV_8UC3);
		showImg.setTo(Scalar(255,255,255));
		img_color1.copyTo(showImg(Rect(0, 0, remapImg.cols, remapImg.rows)));
		img_color2.copyTo(showImg(Rect(remapImg.cols, 0, subBaseImg.cols, subBaseImg.rows)));

		//缩放显示
		cv::Size imgsize;
		imgsize.height = 800;
		imgsize.width = round(showImg.cols / (showImg.rows / 800.0));
		cv::Mat resizeImg;
		cv::resize(showImg, resizeImg, imgsize, INTER_CUBIC);

	
		//写图像信息
		char fname[500];
		sprintf(fname, "%d   dev[%.1f] H[%.1f]", id.index, id.dev, id.height);
	
	

		
		if(debug_show)
		{
			cv::imshow(" 按空格显示下一张,控制台ctrl+c 退出程序", resizeImg);
			while (cv::waitKey(0) == 32)break;
		}
		
		
		if (debug_save)
		{
			// 输出基准图区域
			char name[1000];
			sprintf(name, "%s/%s.jpg", path_save.c_str(), fname);
			cv::imwrite(name, resizeImg);
		}
	}



	// 获取结束时间
	auto timestamp_end = chrono::high_resolution_clock::now();
	chrono::duration<double, milli> duration = timestamp_end - timestamp_start;
	id.usetime = duration.count(); // 单位ms
	
	//定位成功的情况
	sprintf(id.info,"index [%d] name[%s] val[%.1f] Res[%.3f] gary[%d %d] rxyz[%.1f %.1f %.1f] dev[%.1f] H[%.1f] usetime[%.1fms]", id.index, id.name.c_str(),id.value, id.res, id.meanGray,id.stdGray, id.rx, id.ry, id.rz, id.dev,id.height,id.usetime);

	return true;


}
