//
// Created by kezjo on 24-10-6.
//

#include "Kalman.h"
#include<Eigen/Dense>
#include<iostream>
#include"../Armor/Armor.h"
#include"../camera/Camera.h"

void Kalman::Initialize(const Eigen::VectorXd& X_in) {
    X_=X_in;
}

void Kalman::setF(double dt) {
    F_<<1,0,0,dt,0,0,
        0,1,0,0,dt,0,
        0,0,1,0,0,dt,
        0,0,0,1,0,0,
        0,0,0,0,1,0,
        0,0,0,0,0,1;
}

void Kalman::Predict() {
    if (X_.size() != 6 || F_.cols() != 6 || P_.cols() != 6) {
        std::cerr << "Error: Size mismatch in Predict() - X_: " << X_.size()
                  << ", F_: (" << F_.rows() << " x " << F_.cols()
                  << "), P_: (" << P_.rows() << " x " << P_.cols() << ")" << std::endl;
    }
    X_=F_*X_;
    Eigen::MatrixXd Ft=F_.transpose();
    P_=F_*P_*Ft+Q_;
}

Kalman::Kalman() {
    initialized = false;
    Q_.setIdentity(6,6);
    P_.setIdentity(6,6);
    F_.setIdentity(6,6);
    X_ = Eigen::VectorXd(6);
    H_ << 1,0,0,0,0,0,
         0,1,0,0,0,0,
         0,0,1,0,0,0;
    R_ << 0.01, 0,0,
        0,0.01,0,
      0,0,0.02;
    P_<<1e-1,0,0,0,0,0,
    0,1e-1,0,0,0,0,
    0,0,1e-1,0,0,0,
    0,0,0,1e-1,0,0,
    0,0,0,0,1e-1,0,
    0,0,0,0,0,1e-1;

}

void Kalman::MeasurementUpdate(const Eigen::VectorXd &z) {
    Eigen::VectorXd y=z-H_*X_;
    Eigen::MatrixXd S=H_*P_*H_.transpose()+R_;
    Eigen::MatrixXd K=P_*H_.transpose()*S.inverse();
    X_=X_+K*y;
    int size=X_.size();
    Eigen::MatrixXd I=Eigen::MatrixXd::Identity(size,size);
    P_=(I-K*H_)*P_;
}

Eigen::VectorXd Kalman::getX() {
    return X_;
}

cv::Point usingKalman(std::vector<Kalman>& kl,double fps,Armor* TargetArmors,Armor* LastTarget) {
    Eigen::VectorXd X_in(6);
    double dt = 1 / fps;
    double vx = (TargetArmors->getArmorInfo().tvec.at<double>(0) - LastTarget->getArmorInfo().tvec.at<double>(0)) / dt;
    double vy = (TargetArmors->getArmorInfo().tvec.at<double>(1) - LastTarget->getArmorInfo().tvec.at<double>(1)) / dt;
    double vz = (TargetArmors->getArmorInfo().tvec.at<double>(2) - LastTarget->getArmorInfo().tvec.at<double>(2)) / dt;

    X_in << LastTarget->getArmorInfo().tvec.at<double>(0), LastTarget->getArmorInfo().tvec.at<double>(1), LastTarget->getArmorInfo().tvec.at<double>(2), vx, vy, vz;
    kl[0].Initialize(X_in);
    kl[0].setF(dt);

    kl[0].Predict();

    Eigen::VectorXd z(3);
    z << TargetArmors->getArmorInfo().tvec.at<double>(0),
         TargetArmors->getArmorInfo().tvec.at<double>(1),
         TargetArmors->getArmorInfo().tvec.at<double>(2);

    kl[0].MeasurementUpdate(z);
    Eigen::VectorXd predictX = kl[0].getX();
    cv::Point predictP=projectTo2D(predictX);
    return predictP;
}
