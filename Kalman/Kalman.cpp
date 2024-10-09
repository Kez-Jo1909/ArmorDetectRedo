//
// Created by kezjo on 24-10-6.
//

#include "Kalman.h"
#include<Eigen/Dense>
#include<iostream>

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

