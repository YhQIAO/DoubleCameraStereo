#include "Camera.h"
Camera::Camera(int width, int height, double focalLength, double diagonal_inch)
{
	this->focalLength = focalLength;
	this->row = height;
	this->col = width;
	this->diagonal_inch = diagonal_inch;
	double u0 = col / 2.0;
	double v0 = row / 2.0;
	double widthmm = diagonal_inch * 0.8 * 2.54 * 10; 
	double heightmm = diagonal_inch * 0.6 * 2.54 * 10;
	double fx = focalLength / (widthmm / col);
	double fy = focalLength / (heightmm / row);
	cv::Mat k = (cv::Mat_<double>(3, 3) << fx, 0, u0, 0, fy, v0, 0, 0, 1);
	K = k;
}

