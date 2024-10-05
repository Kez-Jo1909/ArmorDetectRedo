//
// Created by kezjo on 24-10-3.
//

#include "PreProcess.h"
cv::Mat PreProcess::Process(cv::Mat image,int Color){
    cv::Mat imgHsv,mask,imgBlur,imgCanny,imgDil;
    int hmin,smin,vmin;
    int hmax,smax,vmax;
    if(Color==1) {
        hmin=52,smin=0,vmin=223;//vmin:255->223
        hmax=179,smax=255,vmax=255;
    }
    else if(Color==2) {
        hmin=0,smin=170,vmin=147;
        hmax=127,smax=255,vmax=255;
    }
    else if(Color==3) {
        hmin=109,hmax=179;
        smin=91,smax=151;
        vmin=65,vmax=255;
    }
    cv::cvtColor(image,imgHsv,cv::COLOR_BGR2HSV);
    cv::Scalar upper(hmax,smax,vmax);
    cv::Scalar lower(hmin,smin,vmin);
    cv::inRange(imgHsv,lower,upper,mask);
    //cv::imshow("mask",mask);
    GaussianBlur(mask,imgBlur,cv::Size(3,3),3,0);
    cv::Canny(imgBlur,imgCanny,25,75);
    //cv::imshow("imgCanny",imgCanny);
    cv::Mat kernel=cv::getStructuringElement(cv::MORPH_RECT,cv::Size(3,3));
    cv::dilate(imgCanny,imgDil,kernel);
    return imgDil;
};