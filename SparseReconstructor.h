#pragma once
#include <opencv2/opencv.hpp>
#include "SIFTFeatureMatcher.h"
#include "SfMSolver.h"
using namespace std;

/// <summary>
/// ϡ���ؽ��࣬����sfm��������ƥ�����������ϡ���ؽ�
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

