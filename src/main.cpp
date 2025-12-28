// Written by Sinan Çimen, 2025. https://github.com/sinancimen

#include <iostream>
#include <string>
#include <random>
#include <vector>
#include <cmath>
#include <iomanip>
#include "AttitudeEstimator.hpp"
#include "GyroProcessModel.hpp"
#include "QuestMeasurementModel.hpp"

int main() {
    std::cout << "a" << std::endl;
    MeasurementModel* measurement_model = new QuestMeasurementModel();
    ProcessModel* process_model = new GyroProcessModel();
    
    // --------- simulation settings ---------
    double dt      = 0.01;        // 100 Hz
    double T       = 100.0;       // seconds
    int    Nsteps  = static_cast<int>(T / dt);

    int star_update_stride = 20;  // star tracker at 5 Hz (every 20 steps)
    const int Nstars = 3;         // number of reference stars

    // true angular velocity (body frame, rad/s)
    Eigen::Vector3d omega_true(0.0, 0.0, 0.5);

    // true constant gyro bias (rad/s)
    Eigen::Vector3d beta_true(0.01, -0.005, 0.02);

    // --------- random number generators ---------
    std::mt19937 rng(1234);
    std::normal_distribution<double> n_g(0.0, 0.001); // gyro noise
    std::normal_distribution<double> n_b_rw(0.0, 0.0001); // bias random walk
    std::normal_distribution<double> n_s(0.0, 0.001*M_PI/180.0); // star direction noise 0.001 deg

    // --------- true state ---------
    Quaternion q_true(Eigen::Vector4d(0.0, 0.0, 0.0, 1.0)); // initial true quaternion
    Eigen::Vector3d beta_rw_true = beta_true;  // constant here, but could be RW

    // --------- initial guesses ---------
    Quaternion q_hat(Eigen::Vector4d(0.0, 0.05, 0.0, 1.0)); // initial true quaternion
    q_hat.Normalize();
    Eigen::Vector3d beta_hat(0.0, 0.0, 0.0); // initial bias estimate

    // --------- inertial star reference directions ---------
    std::vector<Eigen::Vector3d> r_inertial;
    r_inertial.push_back(Eigen::Vector3d(1, 0, 0).normalized());
    r_inertial.push_back(Eigen::Vector3d(0, 1, 0).normalized());
    r_inertial.push_back(Eigen::Vector3d(0.3, 0.8, 0.5).normalized());

    // initial covariance P (6x6: [delta_theta; delta_beta])
    Eigen::Matrix<double, 6, 6> P;
    P.setZero();
    double sigma_att0_deg = 5.0;
    double sigma_att0 = sigma_att0_deg * M_PI / 180.0;
    double sigma_bias0 = 0.05;   // rad/s
    P.block<3,3>(0,0) = (sigma_att0 * sigma_att0) * Eigen::Matrix3d::Identity();
    P.block<3,3>(3,3) = (sigma_bias0 * sigma_bias0) * Eigen::Matrix3d::Identity();

    Eigen::VectorXd initial_state(7);
    initial_state << q_hat.GetQuaternion(), beta_hat;
    AttitudeEstimator estimator(initial_state, P, process_model, measurement_model);

        for (int k = 0; k < Nsteps; ++k)
    {
        double t = k * dt;

        // ---------- TRUE STATE PROPAGATION ----------
        // (bias constant here; you could add random walk with n_b_rw)

        propagate_quat(q_true, omega_true, dt);

        // ---------- SENSOR SIMULATION ----------

        // gyro measurement
        Eigen::Vector3d gyro_meas = omega_true + beta_true;
        for (int i = 0; i < 3; ++i)
            gyro_meas[i] += n_g(rng);

        // ---------- EKF PROPAGATION STEP ----------
        estimator.TimeUpdate(gyro_meas, dt);

        // ---------- MEASUREMENT UPDATE (STAR TRACKER) ----------
        if (k % star_update_stride == 0)
        {
            const int m = 3 * Nstars; // measurement dimension

            // true and measured star directions
            std::vector<Eigen::Vector3d> b_meas(Nstars);
            Eigen::Matrix3d A_true = q_true.GetRotationMatrix();

            for (int i = 0; i < Nstars; ++i)
            {
                Eigen::Vector3d b_true = A_true * r_inertial[i];
                Eigen::Vector3d noise;
                for (int j = 0; j < 3; ++j)
                    noise[j] = n_s(rng);

                b_meas[i] = (b_true + noise).normalized();
            }

            Eigen::MatrixXd measurements(b_meas.size(), 3);
            for(int i = 0; i < measurements.rows(); ++i) measurements.row(i) = b_meas[i].transpose();
            Eigen::MatrixXd references(b_meas.size(), 3);
            for(int i = 0; i < references.rows(); ++i) references.row(i) = r_inertial[i].transpose();
            estimator.MeasurementUpdate(references,measurements);
        }

        // ---------- LOG ERRORS ----------
        /*if (k % 100 == 0) // every 1 second
        {
            double att_err = quat_error_deg(q_true, q_hat);
            double bias_err = (beta_hat - beta_true).norm();
            std::cout << t << "  " << att_err << "  " << bias_err << "\n";
        }*/
    }

    std::cout << "Quat True: " << std::setprecision(15) << q_true.GetQuaternion().transpose() << std::endl;
    std::cout << "Quat Est: " << std::setprecision(15) << estimator.GetQuaternion().GetQuaternion().transpose() << std::endl;
    std::cout << "Gyro Bias True: " << std::setprecision(15) << beta_true.transpose() << std::endl;
    std::cout << "Gyro Bias Est: " << std::setprecision(15) << estimator.GetGyroBias().transpose() << std::endl;

    return 0;
}