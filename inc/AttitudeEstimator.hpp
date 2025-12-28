// Written by Sinan Çimen, 2025. https://github.com/sinancimen

#pragma once

#include "KalmanFilter.hpp"
#include "Quaternion.hpp"

class AttitudeEstimator {
public:
    AttitudeEstimator(Eigen::VectorXd state_vector, Eigen::MatrixXd covariance_matrix, ProcessModel* process_model, MeasurementModel* measurement_model)
        : _full_state(state_vector, covariance_matrix),
          _error_state(Eigen::VectorXd::Zero(state_vector.size()-1), covariance_matrix),
          _kalman_filter(process_model, measurement_model) {};
    ~AttitudeEstimator() = default;
    Quaternion GetQuaternion() const;
    Eigen::Vector3d GetGyroBias() const;
    void TimeUpdate(const Eigen::Vector3d& gyro_measurement, double dt);
    void MeasurementUpdate(const Eigen::Matrix<double, Eigen::Dynamic, 3> ref_vecs, const Eigen::MatrixXd& measurement);
    Eigen::MatrixXd GetCovarianceMatrix() const;

private:
    void ResetErrorState();
    KalmanFilter _kalman_filter;
    State _full_state;
    State _error_state;
};