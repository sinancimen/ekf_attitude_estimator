// Written by Sinan Çimen, 2025. https://github.com/sinancimen

#include "AttitudeEstimator.hpp"

Quaternion AttitudeEstimator::GetQuaternion() const{
    return Quaternion(_full_state.GetStateVector().head<4>());
}
Eigen::Vector3d AttitudeEstimator::GetGyroBias() const {
    return _full_state.GetStateVector().tail<3>();
}

void AttitudeEstimator::ResetErrorState() {
    
}

void AttitudeEstimator::TimeUpdate(const Eigen::Vector3d& gyro_measurement, double dt) {
    _kalman_filter.ProcessGyro(gyro_measurement, dt, &_error_state, &_full_state);
}

void AttitudeEstimator::MeasurementUpdate(const Eigen::Matrix<double, Eigen::Dynamic, 3> ref_vecs, const Eigen::MatrixXd& measurement) {
    Eigen::Matrix3d attMatrix = GetQuaternion().GetRotationMatrix();
    _kalman_filter.ProcessMeasurement(ref_vecs, attMatrix, &_error_state, measurement);
    ResetErrorState();
}