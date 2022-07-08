#include<iostream>
#include<core.hpp>
#include<imgproc.hpp>
#include<opencv.hpp>
#include<highgui.hpp>
#include<cstring>
#include<cmath>
using namespace std;
using namespace cv;
vector<vector<Point2f>>imgPointsL,imgPointsR;//����ͼ��ǵ㴢��
vector<vector<Point3f>>objectPoints;//�궨���������꣬�궨��ΪXoYƽ��
vector<Mat>rMatL, tMatL, rMatR, tMatR;//����ͼ�����תƽ�ƾ���
Mat R, T, E, F;//˫Ŀ�������ת��ƽ�ơ���������������
Mat R1, R2, P1, P2, Q;//R1��R2�ֱ�ΪУ���任����P1��P2Ϊ�µ�ͶӰ���󣨼�У����ģ���
Mat map1L, map2L;//���������任����
Mat map1R, map2R;//���������任����
Mat distCoeffsL(1, 5, CV_64FC1, Scalar::all(0));//������Ļ����������
Mat distCoeffsR(1, 5, CV_64FC1, Scalar::all(0));//������Ļ����������
Mat cameraMatL(3, 3, CV_64FC1, Scalar::all(0)), cameraMatR(3, 3, CV_64FC1, Scalar::all(0));//������ڲξ���
Mat xyz;//��ͶӰ����õ����������ꡣ
const Size imgSize(640, 480);//ͼ���С
const Size cornersSize(8, 5);//�궨���Ͻǵ����
const Size squareSize(28.91, 28.91);//�����궨��ÿ�����ӵı߳�/mm���Եõ���������
ofstream fout("caliResults.txt");//�洢�궨���
/*get�ǵ���������*/
void getimgPoints(vector<Mat>imgSet, vector<vector<Point2f>> &imgPoints)/*�ǵ���ȡ�õ���������Ľǵ�ͼ*/
{
	namedWindow("cornersShow", WINDOW_AUTOSIZE);
	for (int t = 0; t < imgSet.size(); t++)
	{
		vector<Point2f>cornersBuf;//��ʱ��ͼ�񴢴�ǵ�
		Mat img_gray;//�Ҷ�ͼ�����������ؾ�ȷ����
		//ת��Ϊ�Ҷ�ͼ
		cvtColor(imgSet[t], img_gray, COLOR_BGR2GRAY);
		//�ǵ���ȡ����ʧ��ֱ���˳�
		if (!findChessboardCorners(imgSet[t], cornersSize, cornersBuf))
		{
			cout << "�ǵ����"<<t+1<<"��ͼ��ʱʧ��~~~~~~~~";
			exit(0);
		}
		//�����ؾ�ȷ��
		find4QuadCornerSubpix(img_gray, cornersBuf, Size(5, 5));
		imgPoints.push_back(cornersBuf);
		//��ͼչʾ
		drawChessboardCorners(imgSet[t], cornersSize, cornersBuf, true);//trueΪ����
		imshow("cornersShow", imgSet[t]);
		waitKey(1);
	}
}
/*get��׼�������꣬�궨��ΪXoY*/
void getObjectPoints()
{
	for (int t = 0; t < imgPointsL.size(); t++)
	{
		vector<Point3f>worldPointsSetBuf;
		for (int i = 0; i < cornersSize.height; i++)//����һ��ע����ݽǵ��˳���Եõ��������꣬��������ö�ٿ����ö�ٸ߶ȡ�
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
/*��Ŀ�궨�õ��������imgPoints������ͶӰ�����������ö�����*/
void calSingleCameraCliErr(string Name, vector<vector<Point2f>>imgPoints,vector<Mat>rMat,vector<Mat>tMat,Mat distCoeffs,Mat cameraMat)
{
	int pointsNum = cornersSize.width * cornersSize.height;//�����Ŀ
	vector<Point2f>proPointsBuf;//��ͶӰ������ص�
	double err, totErr = 0;//���
	cout << "����Ϊ" << Name << "�궨ͼ�����ͶӰ����õ�����" << endl;
	fout << "����Ϊ" << Name << "�궨ͼ�����ͶӰ����õ�����" << endl;
	for (int t = 0; t < imgPoints.size(); t++)
	{
		projectPoints(objectPoints[t], rMat[t], tMat[t], cameraMat, distCoeffs,proPointsBuf);
		err = norm(imgPoints[t], proPointsBuf, NORM_L2)/pointsNum;//2������x��y��ֵƽ���ͼ����������š�
		cout << "��" << t + 1 << "��ͼ���ƽ�����Ϊ��    " << err << "����" << endl;
		fout << "��" << t + 1 << "��ͼ���ƽ�����Ϊ��    " << err << "����" << endl;
		totErr += err;
	}totErr /= imgPoints.size();
	cout << "����ͼ���ƽ�����Ϊ��    " << totErr << "����" << endl;
	fout << "����ͼ���ƽ�����Ϊ��    " << totErr << "����" << endl;
}
/*SGBM�����Ӳ�ͼ������*/
Point3f lastP(0,0,0);
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
		if(mouse==1)break;
		cout << "�õ�����һ�����ľ���Ϊ" << sqrt((lastP.x - rP[0]) * (lastP.x - rP[0]) +
			(lastP.y - rP[1]) * (lastP.y - rP[1]) + (lastP.z - rP[2]) * (lastP.z - rP[2])) / 1000 << "m" << endl;
		lastP = Point3f(rP[0], rP[1], rP[2]);
		break;
	}
}
void fOut()//����������
{
	fout << "һ.����Ϊ��Ŀ�궨�õ��Ĳ���������˫Ŀ�궨�����������Ȼ���ƣ�:" << endl;
	fout << "1.��������" << endl;
	fout << "���ڲξ���cameraMatLΪ��" << endl;
	fout << cameraMatL << endl;
	fout << "�ڻ���ϵ��Ϊ��" << endl;
	fout << distCoeffsL << endl;
	fout << "2.�Ҳ������" << endl;
	fout << "���ڲξ���cameraMatRΪ��" << endl;
	fout << cameraMatR << endl;
	fout << "�ڻ���ϵ��Ϊ(k1,k2,k3,p1,p2)��" << endl;
	fout << distCoeffsR << endl;

	/*��ͶӰ������*/
	fout << "��.��Ŀ�궨��ͶӰ��" << endl;
	cout << endl << "���ڽ�����ͶӰ����������" << endl;
	fout << "1.";
	calSingleCameraCliErr("leftCamera", imgPointsL, rMatL, tMatL, distCoeffsL, cameraMatL);
	cout << endl;
	fout << "2.";
	calSingleCameraCliErr("rightCamera", imgPointsR, rMatR, tMatR, distCoeffsR, cameraMatR);

	fout << "��.˫Ŀ�궨��δУ�����õ��Ĳ�����" << endl;
	fout << "1.R��T�������cL�ģ���" << endl << "��R:" << R << endl << "��T:" << T << endl;
	fout << "2.��E:" << E << endl << "��F:" << F << endl;
	fout << "��.˫ĿУ���õ��Ĳ�����" << endl;
	fout << "1.У���任����R1��R2��" << endl << "��R1:" << R1 << endl << "��R2:" << R2 << endl;
	fout << "2.��ͶӰ����P1��P2" << endl << "��P1:" << P1 << endl << "��P2:" << P2 << endl;
	fout << "3.��ͶӰ����Q��" << endl << Q << endl;
}
void SGBM(Mat RectL, Mat RectR, Mat Q)
{
	//��SGBMӰ��ϴ�Ĳ���ΪNumDisparity��BlockSize��UniquenessRatio��������Ĭ�����ü��ɡ�
	cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(0, 16, 3);
	int sgbmWinSize = 9;//SAD���ڴ�С������ʵ������Լ��趨��
	int UniquenessRatio = 5;
	int numDisparities = ((imgSize.width / 12) + 15) & -16;//�Ӳ�ڣ���ƥ��ʱ����Ӳ�����С�Ӳ�ֵ֮���Ҵ��ڴ�С������16��������
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

	cv::Mat disp, disp8;
	sgbm->compute(RectL, RectR, disp);
	disp.convertTo(disp8, CV_8U, 255 / 16.0/numDisparities);
	reprojectImageTo3D(disp, xyz, Q, true);
	xyz = xyz * 16; //������disp��ʱ���Ӳ�����16����Z=bf/d(B�ǽǵ��ľ���)����ZӦ�ó���16�˻�����
	imshow("dispShow", disp8);
	setMouseCallback("dispShow", onMouse, 0);//�Դ���dispShow���ûص�����
	waitKey(0);
}
int main()
{
	//���������ļ�
	ifstream finL("L.txt"), finR("R.txt");
	if (!finL.is_open())
	{
		cout << "δ�ܴ��ļ�~~~~~";
		return 0;
	}
	string filenameL,filenameR;//������ļ����������
	
	/*����Ϊͼ��Ķ�ȡ*/ 
	vector<Mat>imgSetL, imgSetR;//�洢ͼ��
	vector<Mat> imgSetLCopy, imgSetRCopy;//����ͼ�񼯣������ʾУ��Ч��ͼʱ��
	cout << "���ڶ�ȡ�ļ�" << endl;
	while (getline(finL, filenameL)&&getline(finR,filenameR))
	{
		//����ͼƬ�ң�-1��ʾ���ı�ͨ����
		Mat imgL = imread(filenameL,-1),imgR = imread(filenameR,-1);
		Mat Lc, Rc;//imgL��imgR��copy
		Lc = imgL.clone(); Rc = imgR.clone();
		if (imgL.empty())
		{
			cout << "��ȡͼƬʧ��~~~~~~";
			return 0;
		}
		imgSetL.push_back(imgL);
		imgSetR.push_back(imgR);
		imgSetLCopy.push_back(Lc);//����clone()���ǹ���һ������ͷ
		imgSetRCopy.push_back(Rc);
	}
	cout << "��ȡ���" << endl << endl;
	/*��ȡͼ��ǵ����ꡣ*/
	cout << "���ڻ�ȡ������imgPoints" << endl;
	getimgPoints(imgSetL, imgPointsL);
	cout << "���ڻ�ȡ�Ҳ����imgPoints" << endl;
	getimgPoints(imgSetR, imgPointsR);
	/*��ȡ��������*/
	cout << "���ڻ�ȡobjectPoints" << endl;
	getObjectPoints(); 
	/*����궨*/
	cout << endl << "���ڽ��е�Ŀ����궨" << endl;
	calibrateCamera(objectPoints, imgPointsL, imgSize, cameraMatL, distCoeffsL, rMatL, tMatL,0);
	calibrateCamera(objectPoints, imgPointsR, imgSize, cameraMatR, distCoeffsR, rMatR, tMatR,0);
	cout << "���������Ŀ�궨���" << endl;
	
	/*˫Ŀ�궨*/
	cout << endl << "���ڽ���˫Ŀ�궨" << endl;
	stereoCalibrate(objectPoints, imgPointsL, imgPointsR, cameraMatL, distCoeffsL, cameraMatR, distCoeffsR, imgSize, R, T, E, F, CV_CALIB_USE_INTRINSIC_GUESS);
	cout << "˫Ŀ�궨���" << endl;
	
	/*����У���任����*/
	cout << endl << "���ڼ���У������" << endl;
	stereoRectify(cameraMatL, distCoeffsL, cameraMatR, distCoeffsR, imgSize, R, T, R1, R2, P1, P2, Q, 0);
	
	/*�������任����*/
	cout << endl << "���ڼ������任����map" << endl;
	initUndistortRectifyMap(cameraMatL, distCoeffsL, R1, P1, imgSize, CV_16SC2, map1L, map2L);
	initUndistortRectifyMap(cameraMatR, distCoeffsR, R2, P2, imgSize, CV_16SC2, map1R, map2R);

	/*�õ�У�����ͼ�������ߣ�����У��Ч�����鿴У��ǰ��ͼ�������*/
	cout << endl << "���ڵõ�У����ͼ��" << endl;
	namedWindow("RectifyImgShow", WINDOW_NORMAL);
	for (int t = 0; t < imgSetL.size(); t++)
	{
		Mat imgLRec, imgRRec;//У���������ͼ��

		//ʹ��У��ӳ��
		remap(imgSetLCopy[t], imgLRec, map1L, map2L, INTER_LINEAR);
		remap(imgSetRCopy[t], imgRRec, map1R, map2R, INTER_LINEAR);

		//ƴ�Ӿ����Կ��ĸ���������ǹ��þ���ͷ���Ǹ��ơ�
		Mat LinkMat;
		hconcat(imgLRec, imgRRec, LinkMat);

		//����ֱ�ߣ����Ƿ�У���ɹ�
		for (int i = 0; i < cornersSize.height; i++)
		{
			line(LinkMat, Point2i(0, imgPointsL[t][i*cornersSize.width].y), Point2i(2 * imgSize.width - 1, imgPointsL[t][i * cornersSize.width].y), Scalar(255, 0, 0), 1);
		}
		imshow("RectifyImgShow", LinkMat);
		waitKey(0);
		/*���������У����ı궨��ͼ��������ƥ��SGBM���õ��Ӳ�ͼ�������������Զ��Ӳ�ͼ������ͶӰ�Եõ���ʵ����*/
		//Mat recGrayL; cvtColor(imgLRec, recGrayL, CV_BGR2GRAY);
		//Mat recGrayR; cvtColor(imgRRec, recGrayR, CV_BGR2GRAY);
		//SGBM(recGrayL, recGrayR,Q);//SGBM��õ��Ӳ�ͼ������ͶӰ����Q�õ���ʵ���겢���ûص����������ʵ�ֲ鿴�����ˡ�
	}
	/*����궨���*/
	fOut();
	return 0; 
}