#include<iostream>
#include<core.hpp>
#include<imgproc.hpp>
#include<opencv.hpp>
#include<highgui.hpp>
#include<cstring>
#include<cmath>
#include<ctime>
#include<Windows.h>//�����߾��ȼ�ʱ�Ŀ�
using namespace std;
using namespace cv;
Mat R1, R2, P1, P2, Q;//R1��R2�ֱ�ΪУ���任����P1��P2Ϊ�µ�ͶӰ���󣨼�У����ģ���
Mat map1L, map2L;//���������任����
Mat map1R, map2R;//���������任����

/*���������Ŀ�궨��Ĳ���*/
Mat cameraMatL = (Mat_<double>(3, 3) << 733.8717636998138, 0, 310.2623615442249,
	0, 732.9430546834244, 244.035542862459,
0, 0, 1);
Mat cameraMatR = (Mat_<double>(3, 3) << 722.3469510932143, 0, 309.7534364355943,
	0, 720.8755187273044, 270.3514045476137,
0, 0, 1);
Mat distCoeffsL = (Mat_<double>(1, 5) << 0.05592363960911046, 1.482078314973658, 0.00250936244370585, 0.00506077757879516, -11.35413769231045);
Mat distCoeffsR = (Mat_<double>(1, 5) << 0.2866259141282117, -1.915437535711202, 0.006506222135938714, 0.001768655778660644, 2.576672625749777);
/*˫Ŀ�궨R��T*/
Mat R = (Mat_<double>(3, 3) << 0.9999395960917631, -0.001257500432343772, 0.01091892213106632,
	0.001338678267355275, 0.9999714975591566, -0.007430478436429543,
	-0.01090926708528788, 0.007444646530251721, 0.9999127787610784);
Mat T = (Mat_<double>(3, 1) << -25.64851828934408,
	-0.9424207144780784,
	-3.697534412881244);
Mat xyz;//��ͶӰ����õ����������ꡣ
const Size imgSize(640, 480);//ͼ���С
const Size cornersSize(8, 5);//�궨���Ͻǵ����
/*SGBM�����Ӳ�ͼ������*/
Point3f lastP(0, 0, 0);
int mouse = 0;
void onMouse(int event, int x, int y, int, void*)
{
	Point origin;
	switch (event)
	{
	case cv::EVENT_LBUTTONDOWN:   //�����ť���µ��¼�
		origin = Point(x, y);
		++mouse;
		Vec3f rP = xyz.at<Vec3f>(origin);
		cout << origin << "in world coordinate is: " << rP << endl;
		if (mouse == 1)break;
		cout << "�õ�����һ�����ľ���Ϊ" << sqrt((lastP.x - rP[0]) * (lastP.x - rP[0]) +
			(lastP.y - rP[1]) * (lastP.y - rP[1]) + (lastP.z - rP[2]) * (lastP.z - rP[2])) / 1000 << "m" << endl;
		lastP = Point3f(rP[0], rP[1], rP[2]);
		break;
	}
}
/*�����Ӳ�ͼ�Ŀն����*/
int f[1281][721][2] = { 0 };//0��������Ӳ�ֵ�ĵ㣬1������ǰ׺�͡�
int filter(int x1, int y1, int x2, int y2) {//��ֵ�˲������εĶԶ���
	x1++; y1++; x2++; y2++;
	int num = (f[x2][y2][0] - f[x2][y1 - 1][0] - f[x1 - 1][y2][0] + f[x1 - 1][y1 - 1][0]);
	return !num?0:(f[x2][y2][1] - f[x2][y1 - 1][1] - f[x1 - 1][y2][1] + f[x1 - 1][y1 - 1][1]) / num;
}
int filter(int x, int y, int a) {//aΪ�߳�����֤Ϊ����������Ϊmat���±��0��ʼ����������
	return filter(max(x - a / 2, 0), max(y - a / 2, 0), min(x + a / 2, imgSize.height - 1), min(y + a / 2, imgSize.width - 1));
}
void fillEmpty(Mat &disp) {
	struct node {//�����¼�Կ��Խ����˲�����Ԫ�顣
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
	int a = 11;//aΪ��ʼ���ڡ�
	while (a != 1) {
		now = front;
		while (now->next != NULL) {
			tmp = now; now = now->next;
			int fil = filter(now->x, now->y, a);
			if (!fil) {//����ɾ���õ��ˡ���Ϊ�˲������ˡ�
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
	//��SGBMӰ��ϴ�Ĳ���ΪNumDisparity��BlockSize��UniquenessRatio��������Ĭ�����ü��ɡ�
	cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(0, 16, 3);
	int sgbmWinSize = 9;//SAD���ڴ�С������ʵ������Լ��趨��
	int UniquenessRatio = 5;
	int numDisparities = (imgSize.width/8+15)&(-16);//�Ӳ�ڣ���ƥ��ʱ����Ӳ�����С�Ӳ�ֵ֮���Ҵ��ڴ�С������16��������
	int cn = RectL.channels();

	sgbm->setPreFilterCap(11);//preFilterCapΪһ������������openCvĬ��ȡ15��Ԥ����ʵ�����ǵõ�ͼ����ݶ���Ϣ����Ԥ�����ͼ�񱣴��������������ڼ�����ۡ� 
	sgbm->setBlockSize(sgbmWinSize);
	sgbm->setP1(8 * cn * sgbmWinSize * sgbmWinSize);//P1��P2�Ƕ�̬�滮�к���Ҫ�Ĳ����������۾ۺϹ����С���
	sgbm->setP2(32 * cn * sgbmWinSize * sgbmWinSize);
	sgbm->setMinDisparity(0);//��С�Ӳ�����Ϊ0
	sgbm->setNumDisparities(numDisparities);
	sgbm->setUniquenessRatio(UniquenessRatio);//���ƥ����۴ﵽ�ε�ƥ����۵�(1-UniquenessRatio/100)%ʱ���õ��Ӳ�ű�ȷ��Ϊ����ȷ�ģ���Ψһ�Լ�⡣
	sgbm->setSpeckleWindowSize(50);
	sgbm->setSpeckleRange(32);
	sgbm->setDisp12MaxDiff(1);//����һ���Լ��������ֵ
	sgbm->setMode(StereoSGBM::MODE_SGBM);
	Mat disp, disp8U;
	/*������ͼΪ��׼������ͼΪ��׼�����Ӳ�ͼ*/
	sgbm->compute(RectL, RectR, disp);
	disp.convertTo(disp8U, CV_8U, 255 / (numDisparities * 16.0));
	reprojectImageTo3D(disp, xyz, Q, true);
	xyz = xyz * 16; //������disp��ʱ���Ӳ�����16����Z=bf/d(B�ǽǵ��ľ���)����ZӦ�ó���16�˻�����
	//fillEmpty(disp8U);
	imshow("left_dispShow", disp8U);
	cnt++;
	if (cnt % 30==0) {
		QueryPerformanceCounter(&et);
		cout << cnt << "�λ��ˣ�" << (et.QuadPart - st.QuadPart) / freq.QuadPart << "s ��query��������"<<(clock()-clos)/(double)CLOCKS_PER_SEC	<<"s (clock����)"<<endl;
	}
	setMouseCallback("left_dispShow", onMouse, 0);//�Դ���dispShow���ûص�����
	int x=waitKey(1);
	if (x == 32) { system("pause"); }

	/*��Ϊ����ͼΪ��׼��Ҫ�޸Ĳ���
	sgbm->setNumDisparities(numDisparities);//���������,ö���Ӳ�ʱ��ΧΪ��minDis-minDis+numDis
	sgbm->setMinDisparity(-numDisparities);
	sgbm->compute(RectR, RectL, disp);
	disp=abs(disp);
	disp.convertTo(disp8, CV_8U, 255 / (numDisparities * 16.0));
	reprojectImageTo3D(disp, xyz, Q, true);
	xyz = xyz * 16; //������disp��ʱ���Ӳ�����16����Z=bf/d(B�ǽǵ��ľ���)����ZӦ�ó���16�˻�����
	imshow("right_dispShow", disp8);
	setMouseCallback("right_dispShow", onMouse, 0);//�Դ���dispShow���ûص�����
	waitKey(0);*/
}
/*�������ã��Ӳ�ͼת���ͼ���룺����dispMap ----�Ӳ�ͼ��8λ��ͨ����CV_8UC1��
��K       ----�ڲξ���float���������
����depthMap ----���ͼ��16λ�޷��ŵ�ͨ����CV_16UC1
void disp2Depth(cv::Mat dispMap, cv::Mat& depthMap, cv::Mat K)
{
	int type = dispMap.type();
	float fx = K.at(0, 0);
	float fy = K.at(1, 1);
	float cx = K.at(0, 2);
	float cy = K.at(1, 2);
	float baseline = 65; //���߾���65mm  
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
					continue;  //��ֹ0��           
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
	cv::Ptr<cv::StereoBM>bm = cv::StereoBM::create(0, 21);//������SGBM����
	int numDisparities = ((imgSize.width / 8) + 15) & -16;
	int uniquenessRatio = 10;
	int SADSize = 15;

	Rect ROI1, ROI2;//����BM�����壬��������ȥ�ڱߣ��޷�ƥ���λ�ã�  ����Ҫ��ͶӰ�������ã���������ô���꽫����Ӧ����ͶӰ���Ǵ���ֵ��
	bm->setBlockSize(SADSize);
	bm->setDisp12MaxDiff(1);//�ڵ��㣨Ψһ�Լ�⣩�����ֵ
	bm->setMinDisparity(0);
	bm->setNumDisparities(numDisparities);
	bm->setPreFilterCap(63);//Ԥ�����˲����Ľض���ֵ
	bm->setPreFilterSize(11);//Ԥ�����˲����Ĵ��ڴ�С
	bm->setPreFilterType(CV_STEREO_BM_XSOBEL);//Ԥ�������ͱ�ʶ����ˮƽ��sobel�����㷨 CV_STEREO_BM_XSOBEL �� CV_STEREO_BM_NORMALIZED_RESPONSE����һ����Ӧ��
	//bm->setROI1(ROI1); bm->setROI2(ROI2);
	bm->setSpeckleRange(32);//�����Ӳ�仯��ֵ
	bm->setSpeckleWindowSize(50);//��ͨ����Ŀ��ֵ
	bm->setTextureThreshold(10);//������������ж���ֵ,����SAD���������ص�x�ĵ�����С�ڸ���ֵ���ɽ��Ӳ�����Ϊ0
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
void fileMatch()//��ȡ�ļ�����ƥ��
{
	string filenameL, filenameR;//������ļ����������
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
void cameraMatch()//ʵʱ���ս���ƥ��
{
	Mat L, R, rL, rR;//���յõ�������ͼ��
	/*��Ϊ���豸��*/
	cout << "������������ͷ�����Ե�" << endl;
	VideoCapture cap(1);
	cap.set(CV_CAP_PROP_FRAME_WIDTH, imgSize.width * 2);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, imgSize.height);
	cout << "���ӳɹ�" << endl;
	namedWindow("showCamera", WINDOW_NORMAL);
	QueryPerformanceFrequency(&freq);
	QueryPerformanceCounter(&st);
	clos = clock();
	while (1)
	{
		//cout << "���ո����������,esc���˳�" << endl;
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
	/*˫�豸��*/
	cout << "������������ͷ�����Ե�" << endl;
	VideoCapture c1(1), c2(2);
	cout << "���ӳɹ�" << endl;
	while (1)
	{
		cout << "���ո����������,esc���˳�" << endl;
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
	stereoRectify(cameraMatL, distCoeffsL, cameraMatR, distCoeffsR, imgSize, R, T, R1, R2, P1, P2, Q, 0);//У��
	cout << Q;
	initUndistortRectifyMap(cameraMatL, distCoeffsL, R1, P1, imgSize, CV_32FC1, map1L, map2L);//����ӳ���ϵ
	initUndistortRectifyMap(cameraMatR, distCoeffsR, R2, P2, imgSize, CV_32FC1, map1R, map2R);
	//fileMatch();
	//cameraMatch();//��
	cameraMatch2();//˫
	return 0;
}