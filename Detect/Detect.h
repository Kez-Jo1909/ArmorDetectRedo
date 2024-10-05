//
// Created by kezjo on 24-10-3.
//

#ifndef DETECT_H
#define DETECT_H
#include<iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include"../Defines.h"
#include"../Armor/Armor.h"

class Contours {
public:
    std::vector<LightRect> getContours(cv::Mat imgDil);
};

class LightPair {
public:
    std::vector<ArmorToPair> PairProcess(std::vector<LightRect> lr,cv::Mat img);
};



#endif //DETECT_H
