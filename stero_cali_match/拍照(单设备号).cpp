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
const Size imgSize(1280, 720);//�и� ����ͼ���size
const Size cornersSize(8, 5);//�궨���Ͻǵ���󣨿���Ըߣ�
Rect rectL(0, 0, imgSize.width, imgSize.height), rectR(imgSize.width, 0, imgSize.width, imgSize.height);//�и�ͼ��
/*�ı궨ͼ*/
void Sam()
{
	//ʹ��ǰ����������ͷ
	cout << "������������ͷ�����Ե�" << endl;
	VideoCapture cap(0);
	cap.set(CV_CAP_PROP_FRAME_WIDTH, imgSize.width*2);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, imgSize.height);
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
		Mat capL, capR,frame;
		string p1 = "left", p2 = "right", t1, t2 = ".png";
		/*�ǵ���ȡ*/
		for (; ;)
		{
			cap >> frame;
			capL = frame(rectL);
			capR = frame(rectR);
			imshow("showCamera", frame);
			int tmp = waitKey(50);
			if (tmp == 27) { return; }
			if (tmp != -1)break;

		}
		Mat imgL = capL.clone(), imgR = capR.clone();
		Mat grayL, grayR;//�Ҷ�ͼ�����������ؾ�ȷ����
		//ת��Ϊ�Ҷ�ͼ
		cvtColor(imgL, grayL, COLOR_BGR2GRAY);
		cvtColor(imgR, grayR, COLOR_BGR2GRAY);
		//�ǵ���ȡ����ʧ��ֱ���˳�
		cout << "������ȡ�ǵ�~~~~" << endl;
		bool judge = (!findChessboardCorners(imgL, cornersSize, bufL,3)) || (!findChessboardCorners(imgR, cornersSize, bufR, 3));
		if (judge == true)
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
			imwrite(p1 + t1 + t2, capL); imwrite(p2 + t1 + t2, capR);
		}
		else if (press == 27) { return; }
	}
}
/*����ͨͼ*/
void Nor()
{
	cout << "������������ͷ�����Ե�" << endl;
	VideoCapture cap(0);
	cap.set(CV_CAP_PROP_FRAME_WIDTH, imgSize.width * 2);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, imgSize.height);
	cout << "���ӳɹ�" << endl;
	int tot = 0;
	cout << "������ͼ��Ŀǰ�±����ֵ�Ա�ţ�����0" << endl;
	cin >> tot;
	cout << "1.��Ƶչʾʱ��������ɿ�ʼ��������,��Esc�˳�;" << endl;
	cout << "2.��space��Ϊ���沢����1����esc�˳������Ҳ����棬���������������1" << endl;
	namedWindow("showCamera", WINDOW_AUTOSIZE);
	namedWindow("capShow", WINDOW_NORMAL);
	for (; ;)
	{
		Mat frame;
		Mat gL,gR;
		Mat capL, capR;//���ڵ��豸�ţ���ֻ��һ��USB�ӿڵ�ͼ��ָ
		string p1 = "leftP", p2 = "rightP", t1, t2 = ".png";
		/*չʾ��ͷ����*/
		for (; ;)
		{
			cap >> frame;
			imshow("showCamera", frame);
			int tmp = waitKey(50);
			if (tmp == 27) { return; }
			if (tmp != -1)break;

		}

		imshow("capShow", frame);
		int press = waitKey(0);
		if (press == 32)
		{
			++tot;
			cout << "photo" << tot << " successfully saved!" << endl;
			capL = frame(rectL);
			capR = frame(rectR);
			t1 = to_string(tot);
			imwrite(p1 + t1 + t2, capL); imwrite(p2 + t1 + t2, capR);
		}
		else if (press == 27) { return; }
	}
}
/*�����������*/
void alter()
{

	cout << "������������ͷ�����Ե�" << endl;
	VideoCapture  cap(1);
	cout << "���ӳɹ�" << endl << endl;
	printf("width = %.2f\n", cap.get(CV_CAP_PROP_FRAME_WIDTH));
	printf("height = %.2f\n", cap.get(CV_CAP_PROP_FRAME_HEIGHT));
	printf("fbs = %.2f\n", cap.get(CV_CAP_PROP_FPS));
	printf("brightness = %.2f\n", cap.get(CV_CAP_PROP_BRIGHTNESS));
	printf("contrast = %.2f\n", cap.get(CV_CAP_PROP_CONTRAST));
	printf("saturation = %.2f\n", cap.get(CV_CAP_PROP_SATURATION));
	printf("hue = %.2f\n", cap.get(CV_CAP_PROP_HUE));
	printf("exposure = %.2f\n", cap.get(CV_CAP_PROP_EXPOSURE));
}
int main()
{
	//alter();
	Sam();
	//Nor();
	return 0;
}
