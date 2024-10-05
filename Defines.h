//
// Created by kezjo on 24-10-3.
//

#ifndef DEFINES_H
#define DEFINES_H

#include<opencv2/opencv.hpp>

struct LightRect {
    cv::RotatedRect rR;
    double k;
    double ratio;
    double AreaSize;
    cv::Point2f vertices[4];
    double DeltaAngelWithPair;
    double DeltaKWithPair;
    int PairNumber;
};


inline void DrawResult(cv::Mat img,std::vector<LightRect> lr) {
    for(int i=0;i<lr.size();i++) {
        for (int j = 0; j < 4; j++) {
            cv::line(img, lr[i].vertices[j], lr[i].vertices[(j + 1) % 4], cv::Scalar(0, 255, 0), 2);
        }
    }
}
#endif //DEFINES_H
