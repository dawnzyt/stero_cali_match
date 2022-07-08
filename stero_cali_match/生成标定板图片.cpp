#include<iostream>
#include<highgui.hpp>
#include<opencv.hpp>
#include<imgproc.hpp>
#include<core.hpp>
#include<cstring>
#include<cstdio>
using namespace std;
using namespace cv;
int main()
{
	int a = 130;//∆Â≈Ã±ﬂ≥§
	int dx = 9;
	int dy = 6;
	int edge = 65;
	Mat img=Mat::zeros(edge*2+dy * a, edge*2+dx * a, CV_8UC1);
	for(int i=0;i<edge*2+dy*a;i++)for(int j=0;j<edge*2+dx*a;j++)img.at<uchar>(i, j) = 255;
	for (int i = 0; i < dy; i++) {
		for (int j = 0; j < dx; j++) {
			if ((i * dx + j) % 2 == 1 ) {
				for (int k = edge+i * a; k < edge+(i + 1) * a; k++) {
					for (int l = edge+j * a; l < edge+(j + 1) * a; l++) {
						img.at<uchar>(k, l) = 0;
					}
						
				}
			}
		}
	}imshow("test", img);
	waitKey(0);
	imwrite("8_5board_.png", img);
	return 0;
}