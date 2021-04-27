#pragma once
#include <opencv2/opencv.hpp>
using namespace std;

/// <summary>
/// �����
/// ���ԣ��ֱ��ʣ����࣬��������С���Խ��߳��ȣ�Ӣ��Ϊ��λ),����ڲξ���
/// �����������ڲ��󣨲������Զ����㣬����궨�˺���Ҫ�ֶ����ã�
/// </summary>
class Camera
{
public:
	Camera() {};
	int row;
	int col;
	double focalLength;
	double diagonal_inch;
	cv::Mat_<double> K;
	Camera(int width, int height, double focalLength, double diagonal_inch);
	void setK();
};

