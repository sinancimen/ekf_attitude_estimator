// Written by Sinan Çimen, 2025. https://github.com/sinancimen

#include "AttitudeEstimator.hpp"

Quaternion AttitudeEstimator::GetQuaternion() const{
    return Quaternion(_full_state.GetStateVector().head<4>());
}
Eigen::Vector3d AttitudeEstimator::GetGyroBias() const {
    return _full_state.GetStateVector().tail<3>();
}

void AttitudeEstimator::ResetErrorState() {
    Quaternion new_quat(this->GetQuaternion().GetQuaternion() + 0.5 * this->GetQuaternion().GetRightMultiplicationMatrix() * _error_state.GetStateVector().head<3>());
    new_quat.Normalize();
    Eigen::VectorXd new_full_state(_full_state.GetStateSize());
    new_full_state << new_quat.GetQuaternion(), _error_state.GetStateVector().tail<3>();
    _full_state.SetStateVector(new_full_state);
    Eigen::VectorXd new_state(6);
    new_state << Eigen::VectorXd::Zero(3), _error_state.GetStateVector().tail<3>();
    _error_state.SetStateVector(new_state);
}

void AttitudeEstimator::TimeUpdate(const Eigen::Vector3d& gyro_measurement, double dt) {
    _kalman_filter.ProcessGyro(gyro_measurement, dt, &_error_state, &_full_state);
}

void AttitudeEstimator::MeasurementUpdate(const Eigen::Matrix<double, Eigen::Dynamic, 3> ref_vecs, const Eigen::MatrixXd& measurement) {
    Eigen::Matrix3d attMatrix = GetQuaternion().GetRotationMatrix();
    _kalman_filter.ProcessMeasurement(ref_vecs, attMatrix, &_error_state, measurement);
    ResetErrorState();
}