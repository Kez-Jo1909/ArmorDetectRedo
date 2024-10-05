#include <iostream>
#include<opencv2/opencv.hpp>
#include"Defines.h"
#include "Detect/Detect.h"
#include"PreProcess/PreProcess.h"
#include"Armor/Armor.h"

/*
 * todo
 * 1给目标加上卡尔曼
 * 2优化目标选择
 */

int main()
{
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
    cv::VideoCapture cap(FilePath);
    cv::Mat frame;
    std::vector<LightRect> DetectedLight;
    std::vector<Armor> TargetArmors;
    Contours Cons;
    LightPair LightPair;
    Armor LastTarget;
    double fps=1000/cap.get(cv::CAP_PROP_FPS);
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
                                        //std::cout<<"Found old target class and replaced bad target"<<std::endl;
                                        TargetArmors[j].renewArmor(DetectedArmor[i].GetArmorInfo());
                                    }
                                }
                                else {
                                    //std::cout<<"Found old target class and reset target"<<std::endl;
                                    TargetArmors[j].renewArmor(DetectedArmor[i].GetArmorInfo());
                                }
                                isJudged=1;
                                break;
                            }
                        }
                        if(isJudged==0) {
                            Armor newTarget(DetectedArmor[i].GetArmorInfo());
                            //std::cout<<"create new target class"<<std::endl;
                            TargetArmors.push_back(newTarget);
                        }
                        DetectedArmor[i].ArmorDraw(frame);
                    }
                }
                int minDistanceID=0;//开始选择目标
                double minDistance=10000;
                //std::cout<<TargetArmors.size()<<std::endl;
                for(int i=0;i<TargetArmors.size();i++) {
                    if(TargetArmors[i].getDistance()<minDistance && TargetArmors[i].whetherFound()==1) {
                        //std::cout<<(minDistance-TargetArmors[i].getDistance())<<std::endl;
                        if((minDistance-TargetArmors[i].getDistance())<300) {//小于阈值，作判定
                            //std::cout<<"<200,进入判定"<<std::endl;
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
                        // if((minDistance-TargetArmors[i].getDistance())>200 || TargetArmors[i].getID()==LastTarget.getID()) {
                        //     /*
                        //      * 如果距离相差>200或者距离较近的id与上一帧的id相同，则切换
                        //      */
                        //     minDistance=TargetArmors[i].getDistance();
                        //     minDistanceID=i;
                        // }
                        // else {
                        //     if(TargetArmors[minDistanceID].getID()==LastTarget.getID()) {
                        //         /*
                        //          * 如果距离稍远的id与上一帧的目标id相同，较近的id不同，不切换目标
                        //          */
                        //         continue;
                        //     }
                        //     else{
                        //         /*
                        //          * 如果距离稍远的id与上一帧的目标id不同，切换目标
                        //          */
                        //         minDistance=TargetArmors[i].getDistance();
                        //         minDistanceID=i;
                        //     }
                        // }
                    }
                    LastTarget=TargetArmors[minDistanceID];
                    TargetArmors[i].ArmorDraw(frame,0);
                }
                TargetArmors[minDistanceID].ArmorDraw(frame,1);
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
