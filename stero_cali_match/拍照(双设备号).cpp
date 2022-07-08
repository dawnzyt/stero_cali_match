#include<iostream>
#include<highgui.hpp>
#include<opencv.hpp>
#include<imgproc.hpp>
#include<core.hpp>
#include<cstring>
#include<cstdio>
using namespace std;
using namespace cv;
vector<Point2f>bufL, bufR;
const Size cornersSize(8, 5);//标定板上角点矩阵（宽乘以高）
/*拍标定图*/
void Sam()
{
	//使用前请连接摄像头
	cout << "正在连接摄像头，请稍等" << endl;
	VideoCapture c1(2), c2(1);
	cout << "连接成功" << endl;
	int tot = 0;
	cout << "请输入图像目前下标最大值以编号，例如0" << endl;
	cin >> tot;
	cout << "1.视频展示时按任意键可开始进行拍照,按Esc退出;" << endl;
	cout << "2.拍完会展示角点分布,若失败则返回1;" << endl;
	cout << "3.按space键为保存并返回1，按esc退出程序且不保存，按其它任意键返回1" << endl;
	namedWindow("showCamera", WINDOW_NORMAL);
	namedWindow("cornersShow", WINDOW_NORMAL);
	for (; ;)
	{
		Mat A, B, Link;
		string p1 = "left", p2 = "right", t1, t2 = ".png";
		/*角点提取*/
		for (; ;)
		{
			c1 >> A;
			c2 >> B;
			hconcat(A, B, Link);
			imshow("showCamera", Link);
			int tmp = waitKey(50);
			if (tmp == 27) { return ; }
			if (tmp != -1)break;

		}
		Mat imgL = A.clone(), imgR = B.clone();
		Mat grayL, grayR;//灰度图，用以亚像素精确化用
		//转化为灰度图
		cvtColor(imgL, grayL, COLOR_BGR2GRAY);
		cvtColor(imgR, grayR, COLOR_BGR2GRAY);
		//角点提取，若失败直接退出
		cout << "正在提取角点~~~~" << endl;
		bool judge = (!findChessboardCorners(imgL, cornersSize, bufL, 3)) & (!findChessboardCorners(imgR, cornersSize, bufR, 3));
		if (judge==true)
		{
			cout << "角点检测时失败，请重新换个角度" << endl;
			continue;
		}
		//亚像素精确化
		find4QuadCornerSubpix(grayL, bufL, Size(6, 6));
		find4QuadCornerSubpix(grayR, bufR, Size(6, 6));
		drawChessboardCorners(imgL, cornersSize, bufL, true);//true为连接
		drawChessboardCorners(imgR, cornersSize, bufR, true);
		Mat Link_; hconcat(imgL, imgR, Link_);
		imshow("cornersShow", Link_);
		int press = waitKey(0);
		if (press == 32)
		{
			++tot; 
			cout << "photo" << tot << " successfully saved!" << endl;
			t1 = to_string(tot);
			imwrite(p1 + t1 + t2, A); imwrite(p2 + t1 + t2, B);
		}
		else if (press == 27) { return ; }
	}
}
/*拍普通图*/
void Nor()
{
	cout << "正在连接摄像头，请稍等" << endl;
	VideoCapture c1(2), c2(1);
	cout << "连接成功" << endl;
	int tot = 0;
	cout << "请输入图像目前下标最大值以编号，例如0" << endl;
	cin >> tot;
	cout << "1.视频展示时按任意键可开始进行拍照,按Esc退出;" << endl;
	cout << "2.按space键为保存并返回1，按esc退出程序且不保存，按其它任意键返回1" << endl;
	namedWindow("showCamera", WINDOW_NORMAL);
	for (; ;)
	{
		Mat A, B, Link;
		string p1 = "leftP", p2 = "rightP", t1, t2 = ".bmp";
		/*角点提取*/
		for (; ;)
		{
			c1 >> A;
			c2 >> B;
			Mat gA, gB;
			//cvtColor(A, gA, CV_RGB2GRAY);
			cvtColor(B, gB, CV_BGR2GRAY);
			hconcat(A, B, Link);
			imshow("showCamera", Link);
			int tmp = waitKey(50);
			if (tmp == 27) { return; }
			if (tmp != -1)break;

		}
		Mat Link_; hconcat(A, B, Link_);
		imshow("cornersShow", Link_);
		int press = waitKey(0);
		if (press == 32)
		{
			++tot;
			cout << "photo" << tot << " successfully saved!" << endl;
			t1 = to_string(tot);
			imwrite(p1 + t1 + t2, A); imwrite(p2 + t1 + t2, B);
		}
		else if (press == 27) { return; }
	}
}
/*设置相机参数*/
void alter()
{

	cout << "正在连接摄像头，请稍等" << endl;
	VideoCapture c1(0), c2(1);
	cout << "连接成功" << endl<<endl;
	printf("width = %.2f\n", c1.get(CV_CAP_PROP_FRAME_WIDTH));
	printf("height = %.2f\n", c1.get(CV_CAP_PROP_FRAME_HEIGHT));
	printf("fbs = %.2f\n", c1.get(CV_CAP_PROP_FPS));
	printf("brightness = %.2f\n", c1.get(CV_CAP_PROP_BRIGHTNESS));
	printf("contrast = %.2f\n", c1.get(CV_CAP_PROP_CONTRAST));
	printf("saturation = %.2f\n", c1.get(CV_CAP_PROP_SATURATION));
	printf("hue = %.2f\n", c1.get(CV_CAP_PROP_HUE));
	printf("exposure = %.2f\n", c1.get(CV_CAP_PROP_EXPOSURE));
	cout << endl;
	printf("width = %.2f\n", c2.get(CV_CAP_PROP_FRAME_WIDTH));
	printf("height = %.2f\n", c2.get(CV_CAP_PROP_FRAME_HEIGHT));
	printf("fbs = %.2f\n", c2.get(CV_CAP_PROP_FPS));
	printf("brightness = %.2f\n", c2.get(CV_CAP_PROP_BRIGHTNESS));
	printf("contrast = %.2f\n", c2.get(CV_CAP_PROP_CONTRAST));
	printf("saturation = %.2f\n", c2.get(CV_CAP_PROP_SATURATION));
	printf("hue = %.2f\n", c2.get(CV_CAP_PROP_HUE));
	printf("exposure = %.2f\n", c2.get(CV_CAP_PROP_EXPOSURE));
}
int main()
{
	//alter();
	//Sam();
	Nor();
	return 0;
}