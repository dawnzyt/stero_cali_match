#include<iostream>
#include<core.hpp>
#include<imgproc.hpp>
#include<opencv.hpp>
#include<highgui.hpp>
#include<cstring>
#include<cmath>
#include<ctime>
#include<Windows.h>//包含高精度计时的库
using namespace std;
using namespace cv;
Mat R1, R2, P1, P2, Q;//R1和R2分别为校正变换矩阵，P1和P2为新的投影矩阵（即校正后的）。
Mat map1L, map2L;//左相机仿射变换矩阵
Mat map1R, map2R;//右相机仿射变换矩阵

/*左右相机单目标定后的参数*/
Mat cameraMatL = (Mat_<double>(3, 3) << 733.8717636998138, 0, 310.2623615442249,
	0, 732.9430546834244, 244.035542862459,
0, 0, 1);
Mat cameraMatR = (Mat_<double>(3, 3) << 722.3469510932143, 0, 309.7534364355943,
	0, 720.8755187273044, 270.3514045476137,
0, 0, 1);
Mat distCoeffsL = (Mat_<double>(1, 5) << 0.05592363960911046, 1.482078314973658, 0.00250936244370585, 0.00506077757879516, -11.35413769231045);
Mat distCoeffsR = (Mat_<double>(1, 5) << 0.2866259141282117, -1.915437535711202, 0.006506222135938714, 0.001768655778660644, 2.576672625749777);
/*双目标定R、T*/
Mat R = (Mat_<double>(3, 3) << 0.9999395960917631, -0.001257500432343772, 0.01091892213106632,
	0.001338678267355275, 0.9999714975591566, -0.007430478436429543,
	-0.01090926708528788, 0.007444646530251721, 0.9999127787610784);
Mat T = (Mat_<double>(3, 1) << -25.64851828934408,
	-0.9424207144780784,
	-3.697534412881244);
Mat xyz;//重投影矩阵得到的世界坐标。
const Size imgSize(640, 480);//图像大小
const Size cornersSize(8, 5);//标定板上角点矩阵
/*SGBM计算视差图并返回*/
Point3f lastP(0, 0, 0);
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
		if (mouse == 1)break;
		cout << "该点与上一坐标点的距离为" << sqrt((lastP.x - rP[0]) * (lastP.x - rP[0]) +
			(lastP.y - rP[1]) * (lastP.y - rP[1]) + (lastP.z - rP[2]) * (lastP.z - rP[2])) / 1000 << "m" << endl;
		lastP = Point3f(rP[0], rP[1], rP[2]);
		break;
	}
}
/*进行视差图的空洞填充*/
int f[1281][721][2] = { 0 };//0存的是有视差值的点，1则存矩阵前缀和。
int filter(int x1, int y1, int x2, int y2) {//均值滤波。矩形的对顶点
	x1++; y1++; x2++; y2++;
	int num = (f[x2][y2][0] - f[x2][y1 - 1][0] - f[x1 - 1][y2][0] + f[x1 - 1][y1 - 1][0]);
	return !num?0:(f[x2][y2][1] - f[x2][y1 - 1][1] - f[x1 - 1][y2][1] + f[x1 - 1][y1 - 1][1]) / num;
}
int filter(int x, int y, int a) {//a为边长，保证为奇数。参数为mat中下标从0开始的行列数。
	return filter(max(x - a / 2, 0), max(y - a / 2, 0), min(x + a / 2, imgSize.height - 1), min(y + a / 2, imgSize.width - 1));
}
void fillEmpty(Mat &disp) {
	struct node {//链表记录仍可以进行滤波的三元组。
		int x, y;
		node* next;
	}; node *front=new node,*now=front,*tmp;
	for (int i = 1,t1,t2; i <= imgSize.height; i++) {
		t1 = t2 = 0;
		uchar* v = disp.ptr(i - 1);
		for (int j=1; j <= imgSize.width; j++) {
			t1 += (v[j - 1] > 0); t2 += v[j - 1];
			f[i][j][0] = f[i - 1][j][0] + t1;
			f[i][j][1] = f[i - 1][j][1] + t2;
			if (!v[j - 1]) {
				tmp = new node;
				tmp->next = NULL; tmp->x = i - 1; tmp->y = j - 1;
				now->next = tmp; now = tmp;
			}
		}
	}
	int a = 11;//a为初始窗口。
	while (a != 1) {
		now = front;
		while (now->next != NULL) {
			tmp = now; now = now->next;
			int fil = filter(now->x, now->y, a);
			if (!fil) {//可以删除该点了。因为滤波不了了。
				tmp->next = now->next;
				delete(now);
				now = tmp;
			}
			else {
				disp.at<uchar>(now->x, now->y) = fil;
			}
		}
		a /= 2; a += ((a % 2) ^ 1);
	}
}
int cnt = 0;
clock_t clos;
LARGE_INTEGER st, et, freq;
void SGBM(Mat RectL, Mat RectR, Mat Q)
{
	//对SGBM影响较大的参数为NumDisparity、BlockSize、UniquenessRatio。其它按默认设置即可。
	cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(0, 16, 3);
	int sgbmWinSize = 9;//SAD窗口大小，根据实际情况自己设定。
	int UniquenessRatio = 5;
	int numDisparities = (imgSize.width/8+15)&(-16);//视差窗口，即匹配时最大视差与最小视差值之差且窗口大小必须是16的整数倍
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
	Mat disp, disp8U;
	/*先以左图为标准再以右图为标准计算视差图*/
	sgbm->compute(RectL, RectR, disp);
	disp.convertTo(disp8U, CV_8U, 255 / (numDisparities * 16.0));
	reprojectImageTo3D(disp, xyz, Q, true);
	xyz = xyz * 16; //由于算disp的时候视差多乘了16，且Z=bf/d(B是角点间的距离)，即Z应该乘以16乘回来。
	//fillEmpty(disp8U);
	imshow("left_dispShow", disp8U);
	cnt++;
	if (cnt % 30==0) {
		QueryPerformanceCounter(&et);
		cout << cnt << "次花了：" << (et.QuadPart - st.QuadPart) / freq.QuadPart << "s （query函数）和"<<(clock()-clos)/(double)CLOCKS_PER_SEC	<<"s (clock函数)"<<endl;
	}
	setMouseCallback("left_dispShow", onMouse, 0);//对窗口dispShow设置回调函数
	int x=waitKey(1);
	if (x == 32) { system("pause"); }

	/*此为以右图为标准，要修改参数
	sgbm->setNumDisparities(numDisparities);//这个是正的,枚举视差时范围为：minDis-minDis+numDis
	sgbm->setMinDisparity(-numDisparities);
	sgbm->compute(RectR, RectL, disp);
	disp=abs(disp);
	disp.convertTo(disp8, CV_8U, 255 / (numDisparities * 16.0));
	reprojectImageTo3D(disp, xyz, Q, true);
	xyz = xyz * 16; //由于算disp的时候视差多乘了16，且Z=bf/d(B是角点间的距离)，即Z应该乘以16乘回来。
	imshow("right_dispShow", disp8);
	setMouseCallback("right_dispShow", onMouse, 0);//对窗口dispShow设置回调函数
	waitKey(0);*/
}
/*函数作用：视差图转深度图输入：　　dispMap ----视差图，8位单通道，CV_8UC1　
　K       ----内参矩阵，float类型输出：
　　depthMap ----深度图，16位无符号单通道，CV_16UC1
void disp2Depth(cv::Mat dispMap, cv::Mat& depthMap, cv::Mat K)
{
	int type = dispMap.type();
	float fx = K.at(0, 0);
	float fy = K.at(1, 1);
	float cx = K.at(0, 2);
	float cy = K.at(1, 2);
	float baseline = 65; //基线距离65mm  
	if (type == CV_8U)
	{
		const float PI = 3.14159265358;
		int height = dispMap.rows;
		int width = dispMap.cols;
		uchar* dispData = (uchar*)dispMap.data;
		ushort* depthData = (ushort*)depthMap.data;
		for (int i = 0; i < height; i++)
		{
			for (int j = 0; j < width; j++)
			{
				int id = i * width + j;
				if (!dispData[id])
					continue;  //防止0除           
				depthData[id] = ushort((float)fx * baseline / ((float)dispData[id]));
			}
		}
	}
	else
	{
		cout << "please confirm dispImg's type!" << endl;
		cv::waitKey(0);
	}
}*/
void BM(Mat RectL, Mat RectR, Mat Q)
{
	cv::Ptr<cv::StereoBM>bm = cv::StereoBM::create(0, 21);//参数和SGBM类似
	int numDisparities = ((imgSize.width / 8) + 15) & -16;
	int uniquenessRatio = 10;
	int SADSize = 15;

	Rect ROI1, ROI2;//若在BM处定义，将用来除去黑边（无法匹配的位置）  若需要重投影则不能启用，若启用那么坐标将不对应，重投影的是错误值。
	bm->setBlockSize(SADSize);
	bm->setDisp12MaxDiff(1);//遮挡点（唯一性检测）检测阈值
	bm->setMinDisparity(0);
	bm->setNumDisparities(numDisparities);
	bm->setPreFilterCap(63);//预处理滤波器的截断阈值
	bm->setPreFilterSize(11);//预处理滤波器的窗口大小
	bm->setPreFilterType(CV_STEREO_BM_XSOBEL);//预处理类型标识，有水平的sobel算子算法 CV_STEREO_BM_XSOBEL 和 CV_STEREO_BM_NORMALIZED_RESPONSE（归一化响应）
	//bm->setROI1(ROI1); bm->setROI2(ROI2);
	bm->setSpeckleRange(32);//允许视差变化阈值
	bm->setSpeckleWindowSize(50);//连通点数目阈值
	bm->setTextureThreshold(10);//低纹理区域的判断阈值,当该SAD窗口内像素点x的导数和小于该阈值，可将视差设置为0
	bm->setUniquenessRatio(uniquenessRatio);
	Mat disp16S;
	bm->compute(RectL, RectR, disp16S);
	Mat disp8U;
	disp16S.convertTo(disp8U, CV_8U, 255 / 16.0 / numDisparities);
	reprojectImageTo3D(disp16S, xyz, Q, true);
	xyz = xyz * 16;
	imshow("dispShow", disp8U);
	fillEmpty(disp8U);
	setMouseCallback("dispShow", onMouse, 0);
	waitKey(0);
}
void fileMatch()//读取文件进行匹配
{
	string filenameL, filenameR;//读入的文件名左右相机
	ifstream inL("L.txt"), inR("R.txt");
	namedWindow("linePhoto", WINDOW_NORMAL);
	while (getline(inL, filenameL) && getline(inR, filenameR))
	{
		Mat L, R, rL, rR;
		L = imread(filenameL, -1); R = imread(filenameR, -1);
		remap(L, rL, map1L, map2L, INTER_LINEAR);
		remap(R, rR, map1R, map2R, INTER_LINEAR);
		Mat Link; hconcat(rL, rR, Link);
		for (int i = 1; i < imgSize.height / 30; i++)
		{
			line(Link, Point2i(0, i * 30), Point2i(2 * imgSize.width - 1, i * 30), Scalar(255, 0, 255), 1);
		}
		imshow("linePhoto", Link);
		//waitKey(0);
		SGBM(rL, rR, Q);
	}
}
void cameraMatch()//实时拍照进行匹配
{
	Mat L, R, rL, rR;//拍照得到的左右图像
	/*下为单设备号*/
	cout << "正在连接摄像头，请稍等" << endl;
	VideoCapture cap(1);
	cap.set(CV_CAP_PROP_FRAME_WIDTH, imgSize.width * 2);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, imgSize.height);
	cout << "连接成功" << endl;
	namedWindow("showCamera", WINDOW_NORMAL);
	QueryPerformanceFrequency(&freq);
	QueryPerformanceCounter(&st);
	clos = clock();
	while (1)
	{
		//cout << "按空格键进行拍照,esc键退出" << endl;
		Mat frame;
		//while (1)
		//{
			cap >> frame;
			//imshow("showCamera", frame);
			//int pre = waitKey(50);
			//if (pre == 32) break;
			//else if (pre == 27) { return; }
		//}
		L = frame(Rect(0, 0, imgSize.width, imgSize.height));
		R = frame(Rect(imgSize.width, 0, imgSize.width, imgSize.height));
		//cvtColor(L, L, CV_BGR2GRAY);
		//cvtColor(R, R, CV_BGR2GRAY);
		remap(L, rL, map1L, map2L, INTER_LINEAR);
		remap(R, rR, map1R, map2R, INTER_LINEAR);
		Mat Link; hconcat(rL, rR, Link);
		//namedWindow("originShow", WINDOW_NORMAL);
		//imshow("originShow", Link);
		//waitKey(0);
		for (int i = 1; i < imgSize.height / 30; i++)
		{
			line(Link, Point2i(0, i * 30), Point2i(2 * imgSize.width - 1, i * 30), Scalar(255, 0, 255), 1);
		}
		//namedWindow("rectImgShow", WINDOW_NORMAL);
		//imshow("rectImgShow", Link);
		//waitKey(0);
		SGBM(rL, rR, Q);
	}
}
void cameraMatch2()
{
	/*双设备号*/
	cout << "正在连接摄像头，请稍等" << endl;
	VideoCapture c1(1), c2(2);
	cout << "连接成功" << endl;
	while (1)
	{
		cout << "按空格键进行拍照,esc键退出" << endl;
		Mat L, R, Link, gL, gR, rL, rR;
		/*while (1)
		{
			c1 >> L; c2 >> R;
			hconcat(L, R, Link);
			imshow("showCamera", Link);
			int pre = waitKey(50);
			if (pre == 32) break;
			else if (pre == 27) { return; }
		}*/
		c1 >> L; c2 >> R;
		hconcat(L, R, Link);
		imshow("showCamera", Link);
		cvtColor(L, gL, CV_BGR2GRAY);
		cvtColor(R, gR, CV_BGR2GRAY);
		remap(gL, rL, map1L, map2L, INTER_LINEAR);
		remap(gR, rR, map1R, map2R, INTER_LINEAR);
		hconcat(rL, rR, Link);
		for (int i = 1; i < imgSize.height / 30; i++)
		{
			line(Link, Point2i(0, i * 30), Point2i(2 * imgSize.width - 1, i * 30), Scalar(255, 0, 255), 1);
		}
		//imshow("rectImgShow", Link);
		//waitKey(1);
		SGBM(gL, gR, Q);
	}
}
int main()
{
	stereoRectify(cameraMatL, distCoeffsL, cameraMatR, distCoeffsR, imgSize, R, T, R1, R2, P1, P2, Q, 0);//校正
	cout << Q;
	initUndistortRectifyMap(cameraMatL, distCoeffsL, R1, P1, imgSize, CV_32FC1, map1L, map2L);//计算映射关系
	initUndistortRectifyMap(cameraMatR, distCoeffsR, R2, P2, imgSize, CV_32FC1, map1R, map2R);
	//fileMatch();
	//cameraMatch();//单
	cameraMatch2();//双
	return 0;
}
