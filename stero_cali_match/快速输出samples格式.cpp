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
	int tot = 0;
	string t1 = "left", t2 = ".png",t3="right";
	ofstream foutR("samplesR.txt"), foutL("samples L.txt");
	int num = 20;
	for (tot = 1; tot <= num; tot++)foutL << t1 + to_string(tot) + t2 << endl, foutR << t3 + to_string(tot) + t2 << endl;
	return 0;
}
