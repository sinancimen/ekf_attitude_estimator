// Written by Sinan Çimen, 2025. https://github.com/sinancimen

#pragma once

#include "MeasurementModel.hpp"
#include "Helper.hpp"

class QuestMeasurementModel : public MeasurementModel {
public:
    QuestMeasurementModel() = default;
    ~QuestMeasurementModel() = default;

    virtual Eigen::MatrixXd GetMeasurementMatrix(Eigen::Matrix<double, Eigen::Dynamic, 3> ref_vecs, Eigen::Matrix3d attMatrix) const {
        int num_vectors = ref_vecs.rows();
        Eigen::MatrixXd H(num_vectors * 3, 6);
        H.setZero();

        for (int i = 0; i < num_vectors; ++i) {
            Eigen::Vector3d v_b = attMatrix * ref_vecs.row(i).transpose();
            H.block<3,3>(i * 3, 0) = skew(v_b);
            H.block<3,3>(i * 3, 3) = Eigen::Matrix3d::Zero();
        }

        return H;
    }
    virtual Eigen::MatrixXd GetMeasurementNoiseCovariance(Eigen::Matrix<double, Eigen::Dynamic, 3> ref_vecs, Eigen::Matrix3d attMatrix) const {
        int num_vectors = ref_vecs.rows();
        double measurement_noise = 0.01; // Example measurement noise standard deviation
        Eigen::MatrixXd R(num_vectors * 3, num_vectors * 3);
        R.setZero();
        
        for (int i = 0; i < num_vectors; ++i) {
            Eigen::Vector3d v_b = attMatrix * ref_vecs.row(i).transpose();
            R.block<3,3>(i * 3, i * 3) = -measurement_noise * measurement_noise * (skew(v_b) * skew(v_b));
        }

        return R;
    }
};