#include<iostream>
#include<core.hpp>
#include<imgproc.hpp>
#include<opencv.hpp>
#include<highgui.hpp>
#include<cstring>
#include<cmath>
using namespace std;
using namespace cv;
vector<vector<Point2f>>imgPointsL,imgPointsR;//所有图像角点储存
vector<vector<Point3f>>objectPoints;//标定板世界坐标，标定板为XoY平面
vector<Mat>rMatL, tMatL, rMatR, tMatR;//所有图像的旋转平移矩阵。
Mat R, T, E, F;//双目相机的旋转、平移、本征、基础矩阵
Mat R1, R2, P1, P2, Q;//R1和R2分别为校正变换矩阵，P1和P2为新的投影矩阵（即校正后的）。
Mat map1L, map2L;//左相机仿射变换矩阵
Mat map1R, map2R;//右相机仿射变换矩阵
Mat distCoeffsL(1, 5, CV_64FC1, Scalar::all(0));//左相机的畸变参数矩阵
Mat distCoeffsR(1, 5, CV_64FC1, Scalar::all(0));//右相机的畸变参数矩阵
Mat cameraMatL(3, 3, CV_64FC1, Scalar::all(0)), cameraMatR(3, 3, CV_64FC1, Scalar::all(0));//相机的内参矩阵。
Mat xyz;//重投影矩阵得到的世界坐标。
const Size imgSize(640, 480);//图像大小
const Size cornersSize(8, 5);//标定板上角点矩阵
const Size squareSize(28.91, 28.91);//测量标定板每个格子的边长/mm，以得到世界坐标
ofstream fout("caliResults.txt");//存储标定结果
/*get角点像素坐标*/
void getimgPoints(vector<Mat>imgSet, vector<vector<Point2f>> &imgPoints)/*角点提取得到左右相机的角点图*/
{
	namedWindow("cornersShow", WINDOW_AUTOSIZE);
	for (int t = 0; t < imgSet.size(); t++)
	{
		vector<Point2f>cornersBuf;//临时该图像储存角点
		Mat img_gray;//灰度图，用以亚像素精确化用
		//转化为灰度图
		cvtColor(imgSet[t], img_gray, COLOR_BGR2GRAY);
		//角点提取，若失败直接退出
		if (!findChessboardCorners(imgSet[t], cornersSize, cornersBuf))
		{
			cout << "角点检测第"<<t+1<<"副图像时失败~~~~~~~~";
			exit(0);
		}
		//亚像素精确化
		find4QuadCornerSubpix(img_gray, cornersBuf, Size(5, 5));
		imgPoints.push_back(cornersBuf);
		//画图展示
		drawChessboardCorners(imgSet[t], cornersSize, cornersBuf, true);//true为连接
		imshow("cornersShow", imgSet[t]);
		waitKey(1);
	}
}
/*get标准世界坐标，标定板为XoY*/
void getObjectPoints()
{
	for (int t = 0; t < imgPointsL.size(); t++)
	{
		vector<Point3f>worldPointsSetBuf;
		for (int i = 0; i < cornersSize.height; i++)//！！一定注意根据角点的顺序以得到世界坐标，这里是先枚举宽度再枚举高度。
		{
			for (int j = 0; j < cornersSize.width; j++)
			{
				Point3f tmp;
				tmp.x = j * squareSize.width;
				tmp.y = i * squareSize.height;
				tmp.z = 0; 
				worldPointsSetBuf.push_back(tmp);
			}
		}objectPoints.push_back(worldPointsSetBuf);
	}
}
/*单目标定得到参数后对imgPoints进行重投影并计算误差，采用二范数*/
void calSingleCameraCliErr(string Name, vector<vector<Point2f>>imgPoints,vector<Mat>rMat,vector<Mat>tMat,Mat distCoeffs,Mat cameraMat)
{
	int pointsNum = cornersSize.width * cornersSize.height;//点的数目
	vector<Point2f>proPointsBuf;//重投影后的像素点
	double err, totErr = 0;//误差
	cout << "以下为" << Name << "标定图像后重投影计算得到的误差：" << endl;
	fout << "以下为" << Name << "标定图像后重投影计算得到的误差：" << endl;
	for (int t = 0; t < imgPoints.size(); t++)
	{
		projectPoints(objectPoints[t], rMat[t], tMat[t], cameraMat, distCoeffs,proPointsBuf);
		err = norm(imgPoints[t], proPointsBuf, NORM_L2)/pointsNum;//2范数即x、y差值平方和加起来开根号。
		cout << "第" << t + 1 << "副图像的平均误差为：    " << err << "像素" << endl;
		fout << "第" << t + 1 << "副图像的平均误差为：    " << err << "像素" << endl;
		totErr += err;
	}totErr /= imgPoints.size();
	cout << "所有图像的平均误差为：    " << totErr << "像素" << endl;
	fout << "所有图像的平均误差为：    " << totErr << "像素" << endl;
}
/*SGBM计算视差图并返回*/
Point3f lastP(0,0,0);
int mouse = 0;
void onMouse(int event, int x, int y, int, void*)
{
	Point origin;
	switch (event)
	{
	case cv::EVENT_LBUTTONDOWN:   //鼠标左按钮按下的事件
		origin = Point(x, y);
		++mouse;
		Vec3f rP = xyz.at<Vec3f>(origin);
		cout << origin << "in world coordinate is: " << rP << endl;
		if(mouse==1)break;
		cout << "该点与上一坐标点的距离为" << sqrt((lastP.x - rP[0]) * (lastP.x - rP[0]) +
			(lastP.y - rP[1]) * (lastP.y - rP[1]) + (lastP.z - rP[2]) * (lastP.z - rP[2])) / 1000 << "m" << endl;
		lastP = Point3f(rP[0], rP[1], rP[2]);
		break;
	}
}
void fOut()//输出结果函数
{
	fout << "一.下面为单目标定得到的参数（已与双目标定处进行最大似然估计）:" << endl;
	fout << "1.左侧相机：" << endl;
	fout << "①内参矩阵cameraMatL为：" << endl;
	fout << cameraMatL << endl;
	fout << "②畸变系数为：" << endl;
	fout << distCoeffsL << endl;
	fout << "2.右侧相机：" << endl;
	fout << "①内参矩阵cameraMatR为：" << endl;
	fout << cameraMatR << endl;
	fout << "②畸变系数为(k1,k2,k3,p1,p2)：" << endl;
	fout << distCoeffsR << endl;

	/*重投影误差计算*/
	fout << "二.单目标定重投影误差：" << endl;
	cout << endl << "正在进行重投影及其误差计算" << endl;
	fout << "1.";
	calSingleCameraCliErr("leftCamera", imgPointsL, rMatL, tMatL, distCoeffsL, cameraMatL);
	cout << endl;
	fout << "2.";
	calSingleCameraCliErr("rightCamera", imgPointsR, rMatR, tMatR, distCoeffsR, cameraMatR);

	fout << "三.双目标定（未校正）得到的参数：" << endl;
	fout << "1.R和T（相对于cL的）：" << endl << "①R:" << R << endl << "②T:" << T << endl;
	fout << "2.①E:" << E << endl << "②F:" << F << endl;
	fout << "四.双目校正得到的参数：" << endl;
	fout << "1.校正变换矩阵R1和R2：" << endl << "①R1:" << R1 << endl << "②R2:" << R2 << endl;
	fout << "2.新投影矩阵P1和P2" << endl << "①P1:" << P1 << endl << "②P2:" << P2 << endl;
	fout << "3.重投影矩阵Q：" << endl << Q << endl;
}
void SGBM(Mat RectL, Mat RectR, Mat Q)
{
	//对SGBM影响较大的参数为NumDisparity、BlockSize、UniquenessRatio。其它按默认设置即可。
	cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(0, 16, 3);
	int sgbmWinSize = 9;//SAD窗口大小，根据实际情况自己设定。
	int UniquenessRatio = 5;
	int numDisparities = ((imgSize.width / 12) + 15) & -16;//视差窗口，即匹配时最大视差与最小视差值之差且窗口大小必须是16的整数倍
	int cn = RectL.channels();

	sgbm->setPreFilterCap(11);//preFilterCap为一个常数参数，openCv默认取15。预处理实际上是得到图像的梯度信息。经预处理的图像保存起来，将会用于计算代价。 
	sgbm->setBlockSize(sgbmWinSize);
	sgbm->setP1(8 * cn * sgbmWinSize * sgbmWinSize);//P1和P2是动态规划中很重要的参数，即代价聚合过程中。略
	sgbm->setP2(32 * cn * sgbmWinSize * sgbmWinSize);
	sgbm->setMinDisparity(0);//最小视差设置为0
	sgbm->setNumDisparities(numDisparities);
	sgbm->setUniquenessRatio(UniquenessRatio);//最低匹配代价达到次低匹配代价的(1-UniquenessRatio/100)%时，该点视差才被确定为是正确的，即唯一性检测。
	sgbm->setSpeckleWindowSize(50);
	sgbm->setSpeckleRange(32);
	sgbm->setDisp12MaxDiff(1);//左右一致性检测的误差阈值
	sgbm->setMode(StereoSGBM::MODE_SGBM);

	cv::Mat disp, disp8;
	sgbm->compute(RectL, RectR, disp);
	disp.convertTo(disp8, CV_8U, 255 / 16.0/numDisparities);
	reprojectImageTo3D(disp, xyz, Q, true);
	xyz = xyz * 16; //由于算disp的时候视差多乘了16，且Z=bf/d(B是角点间的距离)，即Z应该乘以16乘回来。
	imshow("dispShow", disp8);
	setMouseCallback("dispShow", onMouse, 0);//对窗口dispShow设置回调函数
	waitKey(0);
}
int main()
{
	//打开样本名文件
	ifstream finL("L.txt"), finR("R.txt");
	if (!finL.is_open())
	{
		cout << "未能打开文件~~~~~";
		return 0;
	}
	string filenameL,filenameR;//读入的文件名左右相机
	
	/*下面为图像的读取*/ 
	vector<Mat>imgSetL, imgSetR;//存储图像集
	vector<Mat> imgSetLCopy, imgSetRCopy;//拷贝图像集，最后显示校正效果图时用
	cout << "正在读取文件" << endl;
	while (getline(finL, filenameL)&&getline(finR,filenameR))
	{
		//读入图片且，-1表示不改变通道数
		Mat imgL = imread(filenameL,-1),imgR = imread(filenameR,-1);
		Mat Lc, Rc;//imgL和imgR的copy
		Lc = imgL.clone(); Rc = imgR.clone();
		if (imgL.empty())
		{
			cout << "读取图片失败~~~~~~";
			return 0;
		}
		imgSetL.push_back(imgL);
		imgSetR.push_back(imgR);
		imgSetLCopy.push_back(Lc);//若不clone()还是共享一个矩阵头
		imgSetRCopy.push_back(Rc);
	}
	cout << "读取完毕" << endl << endl;
	/*获取图像角点坐标。*/
	cout << "正在获取左侧相机imgPoints" << endl;
	getimgPoints(imgSetL, imgPointsL);
	cout << "正在获取右侧相机imgPoints" << endl;
	getimgPoints(imgSetR, imgPointsR);
	/*获取世界坐标*/
	cout << "正在获取objectPoints" << endl;
	getObjectPoints(); 
	/*相机标定*/
	cout << endl << "正在进行单目相机标定" << endl;
	calibrateCamera(objectPoints, imgPointsL, imgSize, cameraMatL, distCoeffsL, rMatL, tMatL,0);
	calibrateCamera(objectPoints, imgPointsR, imgSize, cameraMatR, distCoeffsR, rMatR, tMatR,0);
	cout << "两个相机单目标定完成" << endl;
	
	/*双目标定*/
	cout << endl << "正在进行双目标定" << endl;
	stereoCalibrate(objectPoints, imgPointsL, imgPointsR, cameraMatL, distCoeffsL, cameraMatR, distCoeffsR, imgSize, R, T, E, F, CV_CALIB_USE_INTRINSIC_GUESS);
	cout << "双目标定完成" << endl;
	
	/*计算校正变换矩阵*/
	cout << endl << "正在计算校正参数" << endl;
	stereoRectify(cameraMatL, distCoeffsL, cameraMatR, distCoeffsR, imgSize, R, T, R1, R2, P1, P2, Q, 0);
	
	/*计算仿射变换矩阵*/
	cout << endl << "正在计算仿射变换矩阵map" << endl;
	initUndistortRectifyMap(cameraMatL, distCoeffsL, R1, P1, imgSize, CV_16SC2, map1L, map2L);
	initUndistortRectifyMap(cameraMatR, distCoeffsR, R2, P2, imgSize, CV_16SC2, map1R, map2R);

	/*得到校正后的图，并画线，检验校正效果，查看校正前后图像的区别*/
	cout << endl << "正在得到校正后图像" << endl;
	namedWindow("RectifyImgShow", WINDOW_NORMAL);
	for (int t = 0; t < imgSetL.size(); t++)
	{
		Mat imgLRec, imgRRec;//校正后的左右图像

		//使用校正映射
		remap(imgSetLCopy[t], imgLRec, map1L, map2L, INTER_LINEAR);
		remap(imgSetRCopy[t], imgRRec, map1R, map2R, INTER_LINEAR);

		//拼接矩阵以看的更清楚，并非共用矩阵头而是复制。
		Mat LinkMat;
		hconcat(imgLRec, imgRRec, LinkMat);

		//绘制直线，看是否校正成功
		for (int i = 0; i < cornersSize.height; i++)
		{
			line(LinkMat, Point2i(0, imgPointsL[t][i*cornersSize.width].y), Point2i(2 * imgSize.width - 1, imgPointsL[t][i * cornersSize.width].y), Scalar(255, 0, 0), 1);
		}
		imshow("RectifyImgShow", LinkMat);
		waitKey(0);
		/*下面可利用校正后的标定板图像做立体匹配SGBM，得到视差图，函数后续可以对视差图进行重投影以得到真实坐标*/
		//Mat recGrayL; cvtColor(imgLRec, recGrayL, CV_BGR2GRAY);
		//Mat recGrayR; cvtColor(imgRRec, recGrayR, CV_BGR2GRAY);
		//SGBM(recGrayL, recGrayR,Q);//SGBM后得到视差图利用重投影矩阵Q得到真实坐标并设置回调函数便可以实现查看坐标了。
	}
	/*输出标定结果*/
	fOut();
	return 0; 
}