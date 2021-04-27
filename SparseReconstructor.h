#pragma once
#include <opencv2/opencv.hpp>
#include "SIFTFeatureMatcher.h"
#include "SfMSolver.h"
using namespace std;

/// <summary>
/// 稀疏重建类，根据sfm解算结果与匹配特征点进行稀疏重建
/// </summary>
class SparseReconstructor
{
public:
	vector<cv::Point3d> points3D;
	SIFTFeatureMatcher matcher;
	SfMSolver sfmsolver;
	cv::Mat R;
	cv::Mat t;
	cv::Mat K;
public:
	SparseReconstructor() {};
	void triangulate();
	void viewPoints3D();
};

