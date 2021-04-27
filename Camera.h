#pragma once
#include <opencv2/opencv.hpp>
using namespace std;

/// <summary>
/// 相机类
/// 属性：分辨率，焦距，传感器大小（对角线长度，英寸为单位),相机内参矩阵
/// 方法：设置内参阵（不设置自动计算，如果标定了后需要手动设置）
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

