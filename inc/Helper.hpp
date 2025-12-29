// Written by Sinan Çimen, 2025. https://github.com/sinancimen

#pragma once

#include "Eigen/Dense"
#include "Quaternion.hpp"

inline Eigen::Matrix3d skew(const Eigen::Vector3d& w)
{
    Eigen::Matrix3d W;
    W <<     0.0,  -w.z(),  w.y(),
          w.z(),    0.0,  -w.x(),
         -w.y(),  w.x(),   0.0;
    return W;
}

// propagate quaternion with body rate w (rad/s) using small-angle approx
inline void propagate_quat(Quaternion& q, const Eigen::Vector3d& w, double dt)
{
    Eigen::Vector3d half_dt_w = 0.5 * dt * w;
    Quaternion dq;
    dq.SetQuaternion(Eigen::Vector4d(half_dt_w.x(), half_dt_w.y(), half_dt_w.z(), 1.0));
    dq.Normalize();

    Eigen::Vector4d q_vec = q.GetQuaternion();
    Eigen::Vector4d dq_vec = dq.GetQuaternion();
    Eigen::Quaterniond q_e(q_vec(3), q_vec(0), q_vec(1), q_vec(2));
    Eigen::Quaterniond dq_e(dq_vec(3), dq_vec(0), dq_vec(1), dq_vec(2));
    Eigen::Quaterniond q_new = q_e * dq_e;
    Eigen::Vector4d q_new_vec;
    q_new_vec << q_new.x(), q_new.y(), q_new.z(), q_new.w();
    q.SetQuaternion(q_new_vec);
    q.Normalize();
}