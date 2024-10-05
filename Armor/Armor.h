//
// Created by kezjo on 24-10-3.
//

#ifndef ARMOR_H
#define ARMOR_H
#include "../Defines.h"
#include<opencv2/opencv.hpp>

struct ArmorInfo {
    int id;
    LightRect left, right;
    cv::Point center;
    double distance;
    cv::Mat rvec;
    cv::Mat tvec;
    cv::RotatedRect roi;
};


class ArmorToPair {
private:
    LightRect leftLight;
    LightRect rightLight;
    cv::Point ArmorCenter;
    int height, width;
    double angle;
    int predict;
    double distance;
    cv::Mat rvec;
    cv::Mat tvec;
    cv::RotatedRect rrect;//roi
public:
    ArmorToPair(LightRect lr1, LightRect lr2);
    void ArmorDraw(cv::Mat img);
    cv::Mat getRoiOfInterest(cv::Mat img);
    int RecognizeArmor(cv::Mat src);
    double GetDistance(cv::Mat img);
    ArmorInfo GetArmorInfo();
    int getPre();
};

class Armor {
private:
    int isDetectedThisFrame;
    ArmorInfo info;
public:
    Armor();
    Armor(ArmorInfo ArIf);
    void Calculate();
    void ArmorDraw(cv::Mat img,int whetherT);
    void renew();
    double getDistance();
    int getID();
    void renewArmor(ArmorInfo ArIF);
    int whetherFound();
};

#endif //ARMOR_H
