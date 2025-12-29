// Written by Sinan Çimen, 2025. https://github.com/sinancimen

#pragma once

#include "Eigen/Dense"

class MeasurementModel {
 public:
     virtual ~MeasurementModel() = default;
     virtual Eigen::MatrixXd GetMeasurementMatrix(Eigen::Matrix<double, Eigen::Dynamic, 3> ref_vecs, Eigen::Matrix3d attMatrix) const = 0;
     virtual Eigen::MatrixXd GetMeasurementNoiseCovariance(Eigen::Matrix<double, Eigen::Dynamic, 3> ref_vecs) const = 0;
 };