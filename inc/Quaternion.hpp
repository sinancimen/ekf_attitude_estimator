// Written by Sinan Çimen, 2025. https://github.com/sinancimen

#pragma once

#include <Eigen/Dense>

class Quaternion {
public:
    Quaternion() {
        _q << 0.0, 0.0, 0.0, 1.0; // Initialize as unit quaternion
    };
    Quaternion(const Eigen::Vector4d& q) : _q(q) {}
    ~Quaternion() = default;

    Eigen::Vector4d GetQuaternion() const { return _q; }
    void SetQuaternion(const Eigen::Vector4d& q) { _q = q; }
    void Normalize();
    void SumWithError(const Eigen::Vector3d& error); // Eq. 6.27
    Eigen::Matrix3d GetRotationMatrix() const; // Eq. 2.129
    Eigen::Vector3d GetEulerAngles() const;

private:
    Eigen::Vector4d _q; // Quaternion components: [q1, q2, q3, q4], q4 being the scalar part
    Eigen::Matrix<double, 4, 3> GetRightMultiplicationMatrix() const; // Eq. 2.87
    Eigen::Matrix<double, 4, 3> GetLeftMultiplicationMatrix() const; // Eq. 2.88
};