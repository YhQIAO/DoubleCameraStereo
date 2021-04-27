#include "SIFTFeatureMatcher.h"

void SIFTFeatureMatcher::setImagePair(cv::Mat image1, cv::Mat image2) 
{
	this->imageL = image1;
	this->imageR = image2;
}

SIFTFeatureMatcher::SIFTFeatureMatcher()
{
	this->detector = cv::SIFT::create();
}

SIFTFeatureMatcher::SIFTFeatureMatcher(cv::Mat image1, cv::Mat image2)
{
	this->detector = cv::SIFT::create();
	this->setImagePair(image1, image2);
}

vector<cv::DMatch> SIFTFeatureMatcher::match()
{
	detector->detectAndCompute(imageL, cv::noArray(), keyPointsL, descriptors_L);
	detector->detectAndCompute(imageR, cv::noArray(), keyPointsR, descriptors_R);
	vector<vector<cv::DMatch>> matches1, matches2;
	vector<cv::DMatch> goodMatches1, goodMatches2, outMatches;
	cv::FlannBasedMatcher matcher;
	matcher.knnMatch(descriptors_L, descriptors_R, matches1, 2);
	matcher.knnMatch(descriptors_R, descriptors_L, matches2, 2);
	ratioTest(matches1, goodMatches1);
	ratioTest(matches2, goodMatches2);
	symmetryTest(goodMatches1, goodMatches2, outMatches);
	this->matches = outMatches;
	return outMatches;
}

void SIFTFeatureMatcher::ratioTest(vector<vector<cv::DMatch>>& matches, vector<cv::DMatch>& goodMatches)
{
	for (vector<vector<cv::DMatch>>::iterator matchIterator = matches.begin(); matchIterator != matches.end(); ++matchIterator)
		if (matchIterator->size() > 1)
			if ((*matchIterator)[0].distance < (*matchIterator)[1].distance * 0.8)
				goodMatches.push_back((*matchIterator)[0]);
}

void SIFTFeatureMatcher::symmetryTest(const vector<cv::DMatch>& matches1, const vector<cv::DMatch>& matches2, vector<cv::DMatch>& symMatches)
{
	symMatches.clear();
	for (vector<cv::DMatch>::const_iterator matchIterator1 = matches1.begin(); matchIterator1 != matches1.end(); ++matchIterator1)
		for (vector<cv::DMatch>::const_iterator matchIterator2 = matches2.begin(); matchIterator2 != matches2.end(); ++matchIterator2)
			if ((*matchIterator1).queryIdx == (*matchIterator2).trainIdx && (*matchIterator1).trainIdx == (*matchIterator2).queryIdx)
				symMatches.push_back(*matchIterator1);
}

void SIFTFeatureMatcher::drawMatchImage() {
	cv::Mat matchImage;
	cv::drawMatches(imageL, keyPointsL,
		imageR, keyPointsR,
		matches, matchImage,
		cv::Scalar(0, 0, 255), cv::Scalar(0, 0, 255));
	
	cv::namedWindow("matchImage", 0);
	cv::resizeWindow("matchImage", cv::Size(400*2*2, 300*2));
	cv::imshow("matchImage", matchImage);
	cv::waitKey(0);
}