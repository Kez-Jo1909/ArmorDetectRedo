//
// Created by kezjo on 24-10-5.
//

#ifndef CAMERA_H
#define CAMERA_H

#endif //CAMERA_H

#include<opencv2/opencv.hpp>
#include<Eigen/Dense>

// 声明相机内参矩阵
extern cv::Mat cameraMatrix;

// 声明畸变系数
extern cv::Mat distCoeffs;

// 声明常量
extern const double BulletVelocity;

cv::Point2f projectTo2D(const Eigen::VectorXd& predictX);