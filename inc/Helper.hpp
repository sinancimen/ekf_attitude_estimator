// Written by Sinan Çimen, 2025. https://github.com/sinancimen

#pragma once

#include "Eigen/Dense"

inline Eigen::Matrix3d skew(const Eigen::Vector3d& w)
{
    Eigen::Matrix3d W;
    W <<     0.0,  -w.z(),  w.y(),
          w.z(),    0.0,  -w.x(),
         -w.y(),  w.x(),   0.0;
    return W;
}