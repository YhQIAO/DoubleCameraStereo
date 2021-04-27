#include "SparseReconstructor.h"
#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/point_types.h>
#include <boost/thread/thread.hpp>
#include <Eigen/Core>

#include "SfMSolver.h"
#include "SIFTFeatureMatcher.h"

void SparseReconstructor::triangulate() {

    cv::Mat T1 = (cv::Mat_<double>(3, 4) <<
        1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0); // 视为世界坐标系，投影矩阵为标准型矩阵：[I 0]
    cv::Mat T2 = (cv::Mat_<double>(3, 4) <<
        R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), t.at<double>(0, 0),
        R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), t.at<double>(1, 0),
        R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), t.at<double>(2, 0));
    T1 = K * T1;
    T2 = K * T2;
    
    cv::Mat points4D; // 齐次坐标形式：4*N：[x y z w].t()
    cv::triangulatePoints(T1, T2, this->sfmsolver.points1, this->sfmsolver.points2, points4D);

    for (size_t i = 0; i < points4D.cols; i++)
    {
        cv::Mat_<float> c = points4D.col(i);
        c /= c(3);
        cv::Point3d p(c(0), c(1), c(2));
        points3D.push_back(p);
    }
}

// 显示重建的稀疏点云
void SparseReconstructor::viewPoints3D()
{
    // 获取特征点在图像中的RGB
    vector<cv::Vec3b> points_Colors;
    for (size_t i = 0; i < sfmsolver.points1.size(); i++)
    {
        cv::Point2f p = sfmsolver.points1[i];
        if (p.x <= matcher.imageL.cols && p.y <= matcher.imageL.rows)
            points_Colors.push_back(matcher.imageL.at<cv::Vec3b>(int(p.y), int(p.x)));
    }

    
    // 建立一个点云
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (size_t i = 0; i < points3D.size(); i++)
    {
        pcl::PointXYZRGB p;
        p.x = points3D[i].x;
        p.y = points3D[i].y;
        p.z = points3D[i].z;

        p.b = points_Colors[i][0];
        p.g = points_Colors[i][1];
        p.r = points_Colors[i][2];

        cloud->points.push_back(p);
    }

    // 在每个相机处显示坐标轴
    // cam1（世界坐标系）
    Eigen::Matrix4f transformationMatrix;
    transformationMatrix <<
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;

    Eigen::Affine3f postion1;
    postion1.matrix() = transformationMatrix.inverse();

    // cam2
    transformationMatrix <<
        R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), t.at<double>(0,0),
        R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), t.at<double>(1,0),
        R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), t.at<double>(2,0),
        0, 0, 0, 1;
    Eigen::Affine3f postion2;
    postion2.matrix() = transformationMatrix.inverse();

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer);

    viewer->addCoordinateSystem(0.4, postion1);
    viewer->addCoordinateSystem(0.4, postion2);

    viewer->setBackgroundColor(0, 0, 0); // 设置背景颜色
    viewer->addPointCloud<pcl::PointXYZRGB>(cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 2, "sample cloud");

    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(1000));
    } 
}
