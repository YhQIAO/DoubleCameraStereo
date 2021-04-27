#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "SIFTFeatureMatcher.h"
#include "SfMSolver.h"
#include "Camera.h"
#include "SparseReconstructor.h"

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/point_types.h>

#include <boost/thread/thread.hpp>

using namespace std;
using namespace pcl;
using namespace cv;

int main(){
	
	// read left and right image
	cv::Mat imageL = cv::imread("../images/image_L.jpg");
	cv::Mat imageR = cv::imread("../images/image_R.jpg");


	Camera camera(6016, 4512,7, 1 / 1.33);
	cout << "K = " << camera.K << endl;

	SIFTFeatureMatcher siftMatcher(imageL, imageR);
	siftMatcher.match();

	SfMSolver sfmSolver(siftMatcher, camera);
	sfmSolver.recoverPose();
	cout << "解算得到的基础矩阵F = " << endl << sfmSolver.F << endl;
	cout << "解算得到的旋转矩阵R = " << endl << sfmSolver.R << endl;
	cout << "解算得到的平移向量t = " << endl << sfmSolver.t << endl;
	siftMatcher.matches = sfmSolver.matcher.matches;
	cout << "显示特征匹配结果" << endl;
	siftMatcher.drawMatchImage();

	SparseReconstructor sr;
	sr.matcher = siftMatcher;
	sr.sfmsolver = sfmSolver;
	sr.R = sfmSolver.R;
	sr.t = sfmSolver.t;
	sr.K = sfmSolver.K;
	sr.triangulate(); // 三角化
	sr.viewPoints3D(); // 显示稀疏点云
}