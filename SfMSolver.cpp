#include "SfMSolver.h"
#include "Camera.h"

SfMSolver::SfMSolver(SIFTFeatureMatcher &matcher, Camera &c)
{
	this->matcher = matcher;
	this->camera = c;
	this->solveF();
	this->solveE();
}

cv::Mat SfMSolver::solveF()
{
	this->fillPoints();
	vector<uchar> inliers(points1.size(), 0); // 内点为1，外点为0
	cv::Mat fundamentalMatrix = cv::findFundamentalMat(points1, points2, inliers, cv::FM_RANSAC, 3.0, 0.99);
	vector<cv::DMatch> reMatch;
	vector<cv::DMatch>::const_iterator itM = this->matcher.matches.begin();
	for (vector<uchar>::const_iterator itIn = inliers.begin(); itIn != inliers.end(); ++itIn, ++itM)
	{
		if (*itIn) // 若为内点
			reMatch.push_back(*itM); // 用于计算F的最终匹配对
	}
	this->points1.clear();
	this->points2.clear();
	this->matcher.matches = reMatch;
	cout << matcher.matches.size() << endl;
	this->fillPoints();

	// 计算基本矩阵F
	fundamentalMatrix = findFundamentalMat(points1, points2,cv::FM_RANSAC);
	this->F = fundamentalMatrix;
	return F;
}

cv::Mat SfMSolver::solveE() 
{
	K = this->camera.K;
	cv::Mat_<double> essential = K.t() * F * K;
	this->E = essential;
	return E;
}

void SfMSolver::recoverPose() {
	cv::Mat R;
	cv::Mat t;
	cv::recoverPose(E, points1, points2, K, R, t);
	this->R = R;
	this->t = t;
}

void SfMSolver::fillPoints()
{
	vector<cv::DMatch> goodMatches = this->matcher.matches;
	vector<cv::KeyPoint> keypoints1 = this->matcher.keyPointsL;
	vector<cv::KeyPoint> keypoints2 = this->matcher.keyPointsR;
	for (vector<cv::DMatch>::const_iterator it = goodMatches.begin();
		it != goodMatches.end(); ++it)
	{
		points1.push_back(keypoints1[it->queryIdx].pt);
		points2.push_back(keypoints2[it->trainIdx].pt);
	}
}

cv::Point2d SfMSolver::pixel2cam(const cv::Point2d& p, const cv::Mat& K) {
	return cv::Point2d
	(
		(p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
		(p.y - K.at<double>(1, 2)) / K.at<double>(1, 1)
	);
};

void SfMSolver::Verify() {
	cv::Mat t_x = (
		cv::Mat_<double>(3, 3) <<
		0, -t.at<double>(2, 0), t.at<double>(1, 0),
		t.at<double>(2, 0), 0, -t.at<double>(0, 0),
		-t.at<double>(1, 0), t.at<double>(0, 0), 0
		);

	for (cv::DMatch m : this->matcher.matches) {
		cv::Point2d pt1 = pixel2cam(this->matcher.keyPointsL[m.queryIdx].pt, K);
		cv::Mat y1 = (cv::Mat_<double>(3, 1) << pt1.x, pt1.y, 1);
		cv::Point2d pt2 = pixel2cam(this->matcher.keyPointsR[m.queryIdx].pt, K);
		cv::Mat y2 = (cv::Mat_<double>(3, 1) << pt2.x, pt2.y, 1);
		cv::Mat d = y2.t() * t_x * R * y1;
		cout << "epipolar constraint = " << d << endl;
	}

}