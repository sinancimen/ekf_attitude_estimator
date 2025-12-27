// Written by Sinan Çimen, 2025. https://github.com/sinancimen

#include "KalmanFilter.hpp"
#include "Quaternion.hpp"

void KalmanFilter::TimeUpdate(const Eigen::MatrixXd F, const Eigen::MatrixXd Q, const Eigen::MatrixXd G, State* error_state, State* full_state, Eigen::Vector3d rate_meas, double dt) {
    int state_size = error_state->GetStateSize();
    Eigen::MatrixXd stateTransition = Eigen::MatrixXd::Identity(state_size, state_size) + F * dt; 
    Eigen::Vector3d unbiased_rate = rate_meas - full_state->GetStateVector().tail<3>();
    Quaternion q_prev(full_state->GetStateVector().head<4>());
    Quaternion q_new(q_prev.GetQuaternion() + 0.5 * q_prev.GetRightMultiplicationMatrix() * unbiased_rate * dt);
    q_new.Normalize();
    Eigen::VectorXd new_state_vector(state_size+1);
    new_state_vector << q_new.GetQuaternion(), full_state->GetStateVector().tail<3>();
    full_state->SetStateVector(new_state_vector);
    error_state->SetCovarianceMatrix(stateTransition * error_state->GetCovarianceMatrix() * stateTransition.transpose() + G*Q*G.transpose()*dt);
}

void KalmanFilter::MeasurementUpdate(const Eigen::MatrixXd H, const Eigen::MatrixXd R, State* error_state, const Eigen::MatrixXd& measurement)
{
    Eigen::MatrixXd K_k = error_state->GetCovarianceMatrix() * H.transpose() *
                             (H * error_state->GetCovarianceMatrix() * H.transpose() + R).inverse();
    error_state->SetStateVector(error_state->GetStateVector() + K_k * (measurement - H * error_state->GetStateVector()));
    error_state->SetCovarianceMatrix((Eigen::MatrixXd::Identity(error_state->GetStateSize(), error_state->GetStateSize()) - K_k * H) * error_state->GetCovarianceMatrix());
}

void KalmanFilter::ProcessGyro(const Eigen::Vector3d& gyro_measurement, double dt, State* error_state, State* full_state) {
    Eigen::MatrixXd F = _process_model->GetStateTransitionMatrix(gyro_measurement);
    Eigen::MatrixXd Q = _process_model->GetProcessNoiseCovarince();
    Eigen::MatrixXd G = _process_model->GetProcessNoiseInputMatrix();
    TimeUpdate(F, Q, G, error_state, full_state, gyro_measurement, dt);
}

void KalmanFilter::ProcessMeasurement(const Eigen::Matrix<double, Eigen::Dynamic, 3> ref_vecs, Eigen::Matrix3d attMatrix, State* error_state, const Eigen::MatrixXd& measurement) {
    Eigen::MatrixXd H = _measurement_model->GetMeasurementMatrix(ref_vecs, attMatrix);
    Eigen::MatrixXd R = _measurement_model->GetMeasurementNoiseCovariance(ref_vecs, attMatrix);
    MeasurementUpdate(H, R, error_state, measurement);
}