#pragma once
#include <opencv2/opencv.hpp>
#include "SIFTFeatureMatcher.h"
#include "Camera.h"
using namespace std;

/// <summary>
/// SfM����
///	����siftmatcherƥ�����������������ֲ��޳����ָ���ת�����ƽ�ƾ���
/// </summary>
class SfMSolver
{
public:
	cv::Mat F; // fundamental matrix
	cv::Mat E; // essential matrix
	cv::Mat K; // camera params
	cv::Mat R; 
	cv::Mat t;
	vector<cv::Point2f> points1; // ��ͼ��������xy����
	vector<cv::Point2f> points2; // ��ͼ��������xy����
	SIFTFeatureMatcher matcher;
	Camera camera;
public:
	SfMSolver() {}
	SfMSolver(SIFTFeatureMatcher &matcher, Camera &c);
	cv::Mat solveF();
	cv::Mat solveE();
	// �ָ���̬�������F,E,R,t
	void recoverPose();
	// ��siftmatcherƥ����������xy�����ŵ�����points1��points2��
	void fillPoints();
	// ��������ת�����������Ŷ
	cv::Point2d pixel2cam(const cv::Point2d& p, const cv::Mat& K);
	// ��֤����������㾫��
	void Verify();
};

