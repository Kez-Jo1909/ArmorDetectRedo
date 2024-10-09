//
// Created by kezjo on 24-10-9.
//
#include "Camera.h"
#include<opencv2/opencv.hpp>
#include<eigen3/Eigen/Dense>

// 相机内参矩阵
cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) <<
                         1777.41, 0, 710.76,
                         0, 1775.42, 534.72,
                         0, 0, 1);

// 畸变系数
cv::Mat distCoeffs = (cv::Mat_<double>(5, 1) <<
                      -0.5631, 0.1830, 0.00197, 0.00096, 0.5688);


const double BulletVelocity=25000;

cv::Point2f projectTo2D(const Eigen::VectorXd& predictX) {
    double x_=predictX(0)/predictX(2);
    double y_=predictX(1)/predictX(2);
    double x=cameraMatrix.at<double>(0,0)*x_+cameraMatrix.at<double>(0,2);
    double y=cameraMatrix.at<double>(1,1)*y_+cameraMatrix.at<double>(1,2);
    return cv::Point2f(x,y);
}
