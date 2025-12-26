#include "Quaternion.hpp"

void Quaternion::Normalize() {
    double norm = _q.norm();
    if (norm > 0) {
        _q /= norm;
    }
}

Eigen::Matrix<double, 4, 3> Quaternion::GetLeftMultiplicationMatrix() const {
    Eigen::Matrix<double, 4, 3> R;
    R <<  _q(3),  _q(2), -_q(1),
         -_q(2),  _q(3),  _q(0),
          _q(1), -_q(0),  _q(3),
         -_q(0), -_q(1), -_q(2);
    return R;
}

Eigen::Matrix<double, 4, 3> Quaternion::GetRightMultiplicationMatrix() const {
    Eigen::Matrix<double, 4, 3> R;
    R <<  _q(3),  -_q(2), _q(1),
         _q(2),  _q(3),  -_q(0),
         -_q(1),  _q(0),  _q(3),
         -_q(0), -_q(1), -_q(2);
    return R;
}

void Quaternion::SumWithError(const Eigen::Vector3d& error) {
    _q += this->GetRightMultiplicationMatrix() * error * 0.5;
    this->Normalize();
}

Eigen::Matrix3d Quaternion::GetRotationMatrix() const {
    Eigen::Matrix3d R = this->GetRightMultiplicationMatrix().transpose() *
                        this->GetLeftMultiplicationMatrix();
    return R;
}

Eigen::Vector3d Quaternion::GetEulerAngles() const
{
    return Eigen::Vector3d(); // Placeholder implementation
}