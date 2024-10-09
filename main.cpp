#include <iostream>
#include<opencv2/opencv.hpp>

#include "camera/Camera.h"
#include"Defines.h"
#include "Detect/Detect.h"
#include"PreProcess/PreProcess.h"
#include"Armor/Armor.h"
#include "Kalman/Kalman.h"

/*
 * todo
 * 1画出预测和实际的轨迹线
 * 2优化目标选择
 * 3给卡尔曼加条件
 */

int main()
{
    std::vector<cv::Point> predictPoint;
    std::vector<Kalman> kalmanList;
    Kalman iniKalman;
    kalmanList.push_back(iniKalman);
    int JudgeColor;
    std::cin>>JudgeColor;
    std::string FilePath;
    PreProcess pre;
    if(JudgeColor==1) {
        FilePath = "/home/kezjo/项目/mission1/Infantry_blue.avi";
    }
    else if(JudgeColor==2) {
        FilePath = "/home/kezjo/项目/mission1/Infantry_red.avi";
    }
    else {
        std::cerr<<"Error Input"<<std::endl;
        return 0;
    }
    int minDistanceID=0;//开始选择目标
    double minDistance=10000;
    cv::VideoCapture cap(FilePath);
    cv::Mat frame;
    std::vector<LightRect> DetectedLight;
    std::vector<Armor> TargetArmors;
    Contours Cons;
    LightPair LightPair;
    Armor LastTarget;
    double fps=1000/cap.get(cv::CAP_PROP_FPS);
    //std::cout<<LastTarget.getID()<<std::endl;//应当是-1
    while(1) {
        cap>>frame;
        auto start = std::chrono::steady_clock::now();
        if(frame.empty()) {
            break;
        }
        cv::Mat imgDil=pre.Process(frame,JudgeColor);
        DetectedLight=Cons.getContours(imgDil);
        if(DetectedLight.size()>0) {
            std::vector<ArmorToPair> DetectedArmor=LightPair.PairProcess(DetectedLight,frame);
            if(DetectedArmor.size()>0) {
                for(int i=0;i<DetectedArmor.size();i++) {
                    cv::Mat ROI=DetectedArmor[i].getRoiOfInterest(frame);
                    if(!ROI.empty()) {
                        cv::Mat binaryMean, binaryGaussian,ROIGRAY;
                        cv::cvtColor(ROI,ROIGRAY,cv::COLOR_BGR2GRAY);
                        //cv::resize(ROIGRAY,ROIGRAY,cv::Size(200,280));
                        // 使用平均自适应阈值化
                        cv::adaptiveThreshold(ROIGRAY, binaryMean, 156, cv::ADAPTIVE_THRESH_MEAN_C,
                                              cv::THRESH_BINARY, 17, 2);
                        DetectedArmor[i].RecognizeArmor(binaryMean);
                        cv::resize(binaryMean,binaryMean,cv::Size(200,280));
                        //cv::imshow("Mean",binaryMean);
                        double dis=DetectedArmor[i].GetDistance(frame);
                        int isJudged=0;//用于判断是否已经存在数字类
                        for(int j=0;j<TargetArmors.size();j++) {
                            if(TargetArmors[j].getID()==DetectedArmor[i].getPre()) {
                                if(TargetArmors[j].whetherFound()==1) {
                                    if(dis<TargetArmors[j].getDistance()) {
                                        TargetArmors[j].renewArmor(DetectedArmor[i].GetArmorInfo());
                                    }
                                }
                                else {
                                    TargetArmors[j].renewArmor(DetectedArmor[i].GetArmorInfo());
                                }
                                isJudged=1;
                                break;
                            }
                        }
                        if(isJudged==0) {
                            Armor newTarget(DetectedArmor[i].GetArmorInfo());
                            TargetArmors.push_back(newTarget);
                        }
                        DetectedArmor[i].ArmorDraw(frame);
                    }
                }
                minDistanceID=-1;//开始选择目标
                minDistance=10000;
                if(LastTarget.getID()!=-1) {
                    for(int i=0;i<TargetArmors.size();i++) {
                        if(TargetArmors[i].getDistance()<minDistance && TargetArmors[i].whetherFound()==1) {
                            if((minDistance-TargetArmors[i].getDistance())<300) {//小于阈值，作判定
                                if(TargetArmors[i].getID()==LastTarget.getID()) {//如果近的id与上一帧目标相同
                                    minDistance=TargetArmors[i].getDistance();
                                    minDistanceID=i;
                                }
                                else {//如果近的id与原目标不同
                                    if(TargetArmors[minDistanceID].getID()==LastTarget.getID()) {//远的相同,不切换
                                        continue;
                                    }
                                    else {
                                        minDistance=TargetArmors[i].getDistance();
                                        minDistanceID=i;
                                    }
                                }
                            }
                            else {//大于200,切换
                                minDistance=TargetArmors[i].getDistance();
                                minDistanceID=i;
                            }
                        }
                        TargetArmors[i].ArmorDraw(frame,0);
                    }
                }
                else {
                    for(int i=0;i<TargetArmors.size();i++) {
                        if(TargetArmors[i].getDistance()<minDistance && TargetArmors[i].whetherFound()==1) {
                            minDistance=TargetArmors[i].getDistance();
                            minDistanceID=i;
                        }
                        TargetArmors[i].ArmorDraw(frame,0);
                    }
                }
                if(minDistanceID>=0 && minDistanceID<TargetArmors.size())
                    TargetArmors[minDistanceID].ArmorDraw(frame,1);
                if (LastTarget.getID() != -1) {
                    if(std::abs(LastTarget.getArmorInfo().center.x-TargetArmors[minDistanceID].getArmorInfo().center.x)>100) {//如果切换了目标，重建卡尔曼
                        Kalman newKalman;
                        kalmanList.erase(kalmanList.begin());
                        kalmanList.push_back(newKalman);
                    }
                    cv::Point predictP=usingKalman(kalmanList,fps,&TargetArmors[minDistanceID],&LastTarget);
                    predictPoint.push_back(predictP);
                    //std::cout<<predictP<<std::endl;
                    cv::circle(frame,predictP,5,cv::Scalar(0,0,255),-1);
                }
                else {
                        std::cerr << "Invalid minDistanceID: " << minDistanceID << std::endl;
                }
            }
        }
        for(int i=0;i<TargetArmors.size();i++) {
            TargetArmors[i].renew();
        }
        //std::cout<<TargetArmors.size()<<std::endl;
        std::string FPS="FPS:"+std::to_string(fps);
        cv::putText(frame,FPS,cv::Point(10,45),cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(255, 255, 255), 2);
        cv::imshow("video", frame);
        //int ret=cv::waitKey();
        if(minDistanceID >= 0 && minDistanceID < TargetArmors.size()) {
            LastTarget=TargetArmors[minDistanceID];
        }
        int ret=cv::waitKey(1000/cap.get(cv::CAP_PROP_FPS));
        if(ret==80 || ret==112) {//按p/P暂停
            cv::waitKey();   //此处应该有判定但是无所谓了
        }
        else if(ret==81 || ret==113) {//q/Q退出
            return 0;
        }
        auto end = std::chrono::steady_clock::now();
        std::chrono::duration<double, std::milli> elapsedTime = end - start;  // 以毫秒为单位计算间隔时间
        //std::cout<<elapsedTime.count()<<std::endl;
        if (elapsedTime.count() > 0) {
            fps = 1000 / elapsedTime.count();
        }
    }

}