//
// Created by kezjo on 24-10-5.
//

#ifndef CAMERA_H
#define CAMERA_H

#endif //CAMERA_H

#include<opencv2/opencv.hpp>

// 相机内参矩阵
cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) <<
                         1777.41, 0, 710.76,
                         0, 1775.42, 534.72,
                         0, 0, 1);

// 畸变系数
cv::Mat distCoeffs = (cv::Mat_<double>(5, 1) <<
                      -0.5631, 0.1830, 0.00197, 0.00096, 0.5688);


const double BulletVelocity=25000;