#pragma once
#include <opencv2/opencv.hpp>
#include "SIFTFeatureMatcher.h"
#include "Camera.h"
using namespace std;

/// <summary>
/// SfM解算
///	根据siftmatcher匹配结果计算基础矩阵与粗差剔除，恢复旋转矩阵和平移矩阵
/// </summary>
class SfMSolver
{
public:
	cv::Mat F; // fundamental matrix
	cv::Mat E; // essential matrix
	cv::Mat K; // camera params
	cv::Mat R; 
	cv::Mat t;
	vector<cv::Point2f> points1; // 左图的特征点xy坐标
	vector<cv::Point2f> points2; // 右图的特征点xy坐标
	SIFTFeatureMatcher matcher;
	Camera camera;
public:
	SfMSolver() {}
	SfMSolver(SIFTFeatureMatcher &matcher, Camera &c);
	cv::Mat solveF();
	cv::Mat solveE();
	// 恢复姿态，即求解F,E,R,t
	void recoverPose();
	// 将siftmatcher匹配的特征点的xy坐标存放到属性points1与points2中
	void fillPoints();
	// 像素坐标转换到相机作弊哦
	cv::Point2d pixel2cam(const cv::Point2d& p, const cv::Mat& K);
	// 验证基础矩阵解算精度
	void Verify();
};

