#include "KalmanFilter.hpp"

State KalmanFilter::TimeUpdate(const Eigen::MatrixXd F, const Eigen::MatrixXd Q, const State& state, double dt) {
    State predicted_state(state.GetStateVector(), state.GetCovarianceMatrix());
    return predicted_state;
}

State KalmanFilter::MeasurementUpdate(const Eigen::MatrixXd H, const Eigen::MatrixXd R, const State& state, const Eigen::VectorXd& measurement)
{
    State updated_state(state.GetStateVector(), state.GetCovarianceMatrix());
    return updated_state;
}