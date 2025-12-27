// Written by Sinan Çimen, 2025. https://github.com/sinancimen

#include "AttitudeEstimator.hpp"

Quaternion AttitudeEstimator::GetQuaternion() const{
    return Quaternion(_full_state.GetStateVector().head<4>());
}
Eigen::Vector3d AttitudeEstimator::GetGyroBias() const {
    return _full_state.GetStateVector().tail<3>();
}