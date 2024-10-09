//
// Created by kezjo on 24-10-6.
//

#ifndef KALMAN_H
#define KALMAN_H

#include<Eigen/Dense>


class Kalman {
private:
    bool initialized;
    Eigen::VectorXd X_;//状态向量
    Eigen::MatrixXd F_;
    Eigen::MatrixXd Q_;
    Eigen::Matrix<double, 3, 3> R_;
    Eigen::MatrixXd P_;
    Eigen::Matrix<double, 3, 6> H_;
public:
    Kalman();
    void Initialize(const Eigen::VectorXd& X_in);
    void setF(double dt);
    void Predict();
    void MeasurementUpdate(const Eigen::VectorXd& z);
    Eigen::VectorXd getX();
};



#endif //KALMAN_H