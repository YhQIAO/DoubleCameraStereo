#pragma once
#include <opencv2/opencv.hpp>
#include <vector>
#include <opencv2/features2d/features2d.hpp>
using namespace std;

/// <summary>
/// SIFT����ƥ���࣬��װƥ��Ӱ����ȡ�������㣬�������Լ�����ƥ��ԣ�DMatch��
/// </summary>
class SIFTFeatureMatcher
{

public:
	cv::Mat imageL;
	cv::Mat imageR;
	cv::Ptr<cv::Feature2D> detector;
	vector<cv::KeyPoint> keyPointsL, keyPointsR; // ������
	cv::Mat descriptors_L, descriptors_R; // ���������������
	vector<cv::DMatch> matches; // ���ƥ��Ľ��

public:
	SIFTFeatureMatcher(); 
	SIFTFeatureMatcher(cv::Mat image1, cv::Mat image2);
	void setImagePair(cv::Mat image1, cv::Mat image2); // ����ƥӰ��

	// ����ƥ�䣬����cv::DMatch��ƥ����
	vector<cv::DMatch> match();
	// ���ʼ�ⷨ
	void ratioTest(vector<vector<cv::DMatch>>& matches, vector<cv::DMatch>& goodMatches);
	// �Գ��Լ�ⷨ
	void symmetryTest(const vector<cv::DMatch>& matches1, const vector<cv::DMatch>& matches2, vector<cv::DMatch>& symMatches);
	// ����ƥ����ͼ
	void drawMatchImage();
};


