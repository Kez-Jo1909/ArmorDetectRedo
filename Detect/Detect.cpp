//
// Created by kezjo on 24-10-3.
//
#include <opencv2/opencv.hpp>
#include "Detect.h"
#include"../Armor/Armor.h"

double CalRatio(cv::RotatedRect rR){
    double r;
    if(rR.size.height >rR.size.width ) {
        r=rR.size.height/rR.size.width;
    }
    else
        r=rR.size.width/rR.size.height;
    return r;
}

int JudgeRotatedCons(LightRect* cons) {
    cons->rR.points(cons->vertices);

    cons->k=(cons->vertices[0].y-cons->vertices[2].y)/(cons->vertices[0].x-cons->vertices[2].x); //对角线斜率绝对值应该大于1
    cons->AreaSize=cons->rR.size.area();
    cons->ratio=CalRatio(cons->rR);
    if(cons->k<0) {
        cons->k=0-cons->k;
    }

    if(cons->k>1 && cons->ratio>2.5){
        return 1;
    }
    else{
        return 0;
    }
}

std::vector<LightRect> Contours::getContours(cv::Mat imgDil) {
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(imgDil,contours,hierarchy,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_SIMPLE);

    std::vector<LightRect> lightRect(contours.size());
    int n=0;
    for(int i=0;i<contours.size();i++){
        LightRect CRR;
        CRR.PairNumber=-1;
        CRR.DeltaAngelWithPair=4;
        CRR.rR= cv::minAreaRect(contours[i]);
        if(JudgeRotatedCons(&CRR)){
            lightRect[n]=CRR;
            n++;
        }
    }
    lightRect.resize(n);

    //按照其中旋转矩形的x坐标排序
    std::sort(lightRect.begin(), lightRect.end(), [](const LightRect& a, const LightRect& b) {
        return a.rR.center.x < b.rR.center.x;
    });

    return lightRect;
}

int JudgeLightPair(LightRect lr1,LightRect lr2,cv::Mat img) {
    //cv::line(img,lr1.rR.center,lr2.rR.center,cv::Scalar(0,0,255),2);
    // std::cout<<"k:"<<std::abs(lr1.k-lr2.k)<<std::endl;
    // std::cout<<"delta x:"<<std::abs(lr1.rR.center.x-lr2.rR.center.x)<<std::endl;
    // std::cout<<"angle1:"<<lr1.rR.angle<<std::endl;
    // std::cout<<"angle2:"<<lr2.rR.angle<<std::endl;
    // std::cout<<"angle:"<<std::abs(lr1.rR.angle-lr2.rR.angle)<<std::endl;
    // std::cout<<"delta length:"<<std::abs(std::max(lr1.rR.size.width,lr1.rR.size.height)-std::max(lr2.rR.size.width,lr2.rR.size.height))<<std::endl;
    // std::cout<<std::abs(lr1.rR.center.x-lr2.rR.center.x)<<std::endl;
    // std::cout<<2*std::max(std::max(lr1.rR.size.width,lr1.rR.size.height),std::max(lr2.rR.size.width,lr2.rR.size.height))<<std::endl;

    if(std::abs(lr1.k-lr2.k)>2.03) {
        // std::cerr<<std::abs(lr1.k-lr2.k)<<">2.03"<<std::endl;
        return 0;
    }
    if(std::abs(lr1.rR.center.x-lr2.rR.center.x)>2.5*std::max(std::max(lr1.rR.size.width,lr1.rR.size.height),std::max(lr2.rR.size.width,lr2.rR.size.height))) {
        //std::cerr<<std::abs(lr1.rR.center.x-lr2.rR.center.x)<<">"<<2.5*std::max(std::max(lr1.rR.size.width,lr1.rR.size.height),std::max(lr2.rR.size.width,lr2.rR.size.height))<<std::endl;
        return 0;
    }
    if(std::abs(lr1.rR.angle-lr2.rR.angle)>10) {
        //std::cerr<<std::abs(lr1.rR.angle-lr2.rR.angle)<<">10"<<std::endl;
        return 0;
    }
    if(std::abs(std::max(lr1.rR.size.width,lr1.rR.size.height)-std::max(lr2.rR.size.width,lr2.rR.size.height))>15) {
        //std::cout<<std::abs(std::max(lr1.rR.size.width,lr1.rR.size.height)-std::max(lr2.rR.size.width,lr2.rR.size.height))<<">15"<<std::endl;
        return 0;
    }
    if(std::abs(lr1.rR.center.y-lr2.rR.center.y)/std::abs(lr1.rR.center.x-lr2.rR.center.x)>5) {
        return 0;
    }
    if(std::abs(lr1.rR.center.y-lr2.rR.center.y)>std::max(std::max(lr1.rR.size.width,lr1.rR.size.height),std::max(lr2.rR.size.width,lr2.rR.size.height))) {
        //std::cout<<std::abs(lr1.rR.center.y-lr2.rR.center.y)<<">"<<std::max(std::max(lr1.rR.size.width,lr1.rR.size.height),std::max(lr2.rR.size.width,lr2.rR.size.height))<<std::endl;
        return 0;
    }
    return 1;
}

std::vector<ArmorToPair> LightPair::PairProcess(std::vector<LightRect> lr,cv::Mat img) {
    //std::cout<<std::endl<<"lr size:"<<lr.size()<<std::endl;
    //cv::imshow("video",img);
    std::vector<ArmorToPair> ArmorPaired;
    for(int i=0;i<lr.size()-1;i++) {
        if(lr[i].PairNumber!=-1) {
            continue;
        }
        else {
            for(int j=i+1;j<lr.size();j++) {
                if(lr[j].PairNumber==-1) {
                    //std::cout<<"Now is"<<i<<"and"<<j;
                    int ret=JudgeLightPair(lr[i],lr[j],img);
                    //std::cout<<" ret:"<<ret<<std::endl;
                    if(ret==1){
                        //std::cout<<"yes"<<std::endl;
                        //cv::line(img,lr[i].rR.center,lr[j].rR.center,cv::Scalar(0,255,0),2);
                        //cv::imshow("video",img);
                        lr[i].PairNumber=j;
                        lr[j].PairNumber=i;
                        ArmorToPair NewArmor(lr[i],lr[j]);
                        ArmorPaired.push_back(NewArmor);
                        break;
                    }
                }
            }
        }
    }
    return ArmorPaired;
}
