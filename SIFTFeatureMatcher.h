#pragma once
#include <opencv2/opencv.hpp>
#include <vector>
#include <opencv2/features2d/features2d.hpp>
using namespace std;

/// <summary>
/// SIFT特征匹配类，封装匹配影像，提取的特征点，描述符以及最终匹配对（DMatch）
/// </summary>
class SIFTFeatureMatcher
{

public:
	cv::Mat imageL;
	cv::Mat imageR;
	cv::Ptr<cv::Feature2D> detector;
	vector<cv::KeyPoint> keyPointsL, keyPointsR; // 特征点
	cv::Mat descriptors_L, descriptors_R; // 特特征点的描述符
	vector<cv::DMatch> matches; // 存放匹配的结果

public:
	SIFTFeatureMatcher(); 
	SIFTFeatureMatcher(cv::Mat image1, cv::Mat image2);
	void setImagePair(cv::Mat image1, cv::Mat image2); // 设置匹影像

	// 进行匹配，返回cv::DMatch的匹配结果
	vector<cv::DMatch> match();
	// 比率检测法
	void ratioTest(vector<vector<cv::DMatch>>& matches, vector<cv::DMatch>& goodMatches);
	// 对称性检测法
	void symmetryTest(const vector<cv::DMatch>& matches1, const vector<cv::DMatch>& matches2, vector<cv::DMatch>& symMatches);
	// 绘制匹配结果图
	void drawMatchImage();
};


