// Written by Sinan Çimen, 2025. https://github.com/sinancimen

#pragma once

#include "ProcessModel.hpp"
#include "Helper.hpp"

class GyroProcessModel : public ProcessModel {
public:
    GyroProcessModel() = default;
    ~GyroProcessModel() = default;

    virtual Eigen::MatrixXd GetStateTransitionMatrix(const Eigen::Vector3d& gyro_measurement) const {
        Eigen::MatrixXd F(6, 6);
        F.setZero();

        // top-left: -[ω x]
        F.block<3,3>(0,0) = -skew(gyro_measurement);

        // top-right: -I3
        F.block<3,3>(0,3) = -Eigen::Matrix3d::Identity();
        return F;
    };

    virtual Eigen::MatrixXd GetProcessNoiseCovarince() const {
        Eigen::MatrixXd Q(6, 6);
        Q.setZero();

        // top-left: σ_gyro^2 * I3
        double sigma_gyro = 0.01; // Example gyro noise standard deviation
        Q.block<3,3>(0,0) = sigma_gyro * sigma_gyro * Eigen::Matrix3d::Identity();

        // bottom-right: σ_bias^2 * I3
        double sigma_bias = 0.001; // Example gyro bias noise standard deviation
        Q.block<3,3>(3,3) = sigma_bias * sigma_bias * Eigen::Matrix3d::Identity();

        return Q;
    }

    virtual Eigen::MatrixXd GetProcessNoiseInputMatrix() const {
        Eigen::MatrixXd G(6, 6);
        G.setZero();
        G.block<3,3>(0,0) = -1 * Eigen::Matrix3d::Identity();
        G.block<3,3>(3,3) = Eigen::Matrix3d::Identity();
        return G;
    }
};