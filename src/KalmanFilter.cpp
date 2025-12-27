// Written by Sinan Çimen, 2025. https://github.com/sinancimen

#include "KalmanFilter.hpp"

State KalmanFilter::TimeUpdate(const Eigen::MatrixXd F, const Eigen::MatrixXd Q, const Eigen::MatrixXd G, const State& prev_state, double dt) {
    State predicted_state(prev_state.GetStateVector(), prev_state.GetCovarianceMatrix());
    int state_size = prev_state.GetStateSize();
    Eigen::MatrixXd stateTransition = Eigen::MatrixXd::Identity(state_size, state_size) + F * dt;
    predicted_state.SetStateVector(stateTransition * prev_state.GetStateVector());   
    predicted_state.SetCovarianceMatrix(stateTransition * prev_state.GetCovarianceMatrix() * stateTransition.transpose() + G*Q*G.transpose()*dt);
    return predicted_state;
}

State KalmanFilter::MeasurementUpdate(const Eigen::MatrixXd H, const Eigen::MatrixXd R, const State& prev_state, const Eigen::VectorXd& measurement)
{
    State updated_state(prev_state.GetStateVector(), prev_state.GetCovarianceMatrix());
    Eigen::MatrixXd K_k = prev_state.GetCovarianceMatrix() * H.transpose() *
                             (H * prev_state.GetCovarianceMatrix() * H.transpose() + R).inverse();
    updated_state.SetStateVector(prev_state.GetStateVector() + K_k * (measurement - H * prev_state.GetStateVector()));
    updated_state.SetCovarianceMatrix((Eigen::MatrixXd::Identity(prev_state.GetStateSize(), prev_state.GetStateSize()) - K_k * H) * prev_state.GetCovarianceMatrix());
    return updated_state;
}

State KalmanFilter::ProcessGyro(const Eigen::Vector3d& gyro_measurement, double dt, const State& prev_state) {
    Eigen::MatrixXd F = _process_model->GetStateTransitionMatrix(gyro_measurement);
    Eigen::MatrixXd Q = _process_model->GetProcessNoiseCovarince();
    Eigen::MatrixXd G = _process_model->GetProcessNoiseInputMatrix();
    return TimeUpdate(F, Q, G, prev_state, dt);
}

State KalmanFilter::ProcessMeasurement(const Eigen::Matrix<double, Eigen::Dynamic, 3> ref_vecs, Eigen::Matrix3d attMatrix, const State& prev_state, const Eigen::VectorXd& measurement) {
    Eigen::MatrixXd H = _measurement_model->GetMeasurementMatrix(ref_vecs, attMatrix);
    Eigen::MatrixXd R = _measurement_model->GetMeasurementNoiseCovariance(ref_vecs, attMatrix);
    return MeasurementUpdate(H, R, prev_state, measurement);
}