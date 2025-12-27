// Written by Sinan Çimen, 2025. https://github.com/sinancimen

#pragma once

#include "Eigen/Dense"

class ProcessModel {
 public:
     virtual ~ProcessModel() = default;
     virtual Eigen::MatrixXd GetStateTransitionMatrix(const Eigen::Vector3d& gyro_measurement) const = 0;
     virtual Eigen::MatrixXd GetProcessNoiseCovarince() const = 0;
     virtual Eigen::MatrixXd GetProcessNoiseInputMatrix() const = 0;
 };