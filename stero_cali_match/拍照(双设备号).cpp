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
const Size cornersSize(8, 5);//�궨���Ͻǵ���󣨿���Ըߣ�
/*�ı궨ͼ*/
void Sam()
{
	//ʹ��ǰ����������ͷ
	cout << "������������ͷ�����Ե�" << endl;
	VideoCapture c1(2), c2(1);
	cout << "���ӳɹ�" << endl;
	int tot = 0;
	cout << "������ͼ��Ŀǰ�±����ֵ�Ա�ţ�����0" << endl;
	cin >> tot;
	cout << "1.��Ƶչʾʱ��������ɿ�ʼ��������,��Esc�˳�;" << endl;
	cout << "2.�����չʾ�ǵ�ֲ�,��ʧ���򷵻�1;" << endl;
	cout << "3.��space��Ϊ���沢����1����esc�˳������Ҳ����棬���������������1" << endl;
	namedWindow("showCamera", WINDOW_NORMAL);
	namedWindow("cornersShow", WINDOW_NORMAL);
	for (; ;)
	{
		Mat A, B, Link;
		string p1 = "left", p2 = "right", t1, t2 = ".png";
		/*�ǵ���ȡ*/
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
		Mat grayL, grayR;//�Ҷ�ͼ�����������ؾ�ȷ����
		//ת��Ϊ�Ҷ�ͼ
		cvtColor(imgL, grayL, COLOR_BGR2GRAY);
		cvtColor(imgR, grayR, COLOR_BGR2GRAY);
		//�ǵ���ȡ����ʧ��ֱ���˳�
		cout << "������ȡ�ǵ�~~~~" << endl;
		bool judge = (!findChessboardCorners(imgL, cornersSize, bufL, 3)) & (!findChessboardCorners(imgR, cornersSize, bufR, 3));
		if (judge==true)
		{
			cout << "�ǵ���ʱʧ�ܣ������»����Ƕ�" << endl;
			continue;
		}
		//�����ؾ�ȷ��
		find4QuadCornerSubpix(grayL, bufL, Size(6, 6));
		find4QuadCornerSubpix(grayR, bufR, Size(6, 6));
		drawChessboardCorners(imgL, cornersSize, bufL, true);//trueΪ����
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
/*����ͨͼ*/
void Nor()
{
	cout << "������������ͷ�����Ե�" << endl;
	VideoCapture c1(2), c2(1);
	cout << "���ӳɹ�" << endl;
	int tot = 0;
	cout << "������ͼ��Ŀǰ�±����ֵ�Ա�ţ�����0" << endl;
	cin >> tot;
	cout << "1.��Ƶչʾʱ��������ɿ�ʼ��������,��Esc�˳�;" << endl;
	cout << "2.��space��Ϊ���沢����1����esc�˳������Ҳ����棬���������������1" << endl;
	namedWindow("showCamera", WINDOW_NORMAL);
	for (; ;)
	{
		Mat A, B, Link;
		string p1 = "leftP", p2 = "rightP", t1, t2 = ".bmp";
		/*�ǵ���ȡ*/
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
/*�����������*/
void alter()
{

	cout << "������������ͷ�����Ե�" << endl;
	VideoCapture c1(0), c2(1);
	cout << "���ӳɹ�" << endl<<endl;
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