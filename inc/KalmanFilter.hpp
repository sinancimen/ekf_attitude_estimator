// Written by Sinan Çimen, 2025. https://github.com/sinancimen

#include "State.hpp"
#include "MeasurementModel.hpp"
#include "ProcessModel.hpp"

class KalmanFilter {
public:
    KalmanFilter(ProcessModel* process_model,
                MeasurementModel* measurement_model)
        : _process_model(process_model),
        _measurement_model(measurement_model) {}
    ~KalmanFilter() = default;
    State ProcessGyro(const Eigen::Vector3d& gyro_measurement, double dt, const State& prev_state);
    State ProcessMeasurement(const Eigen::Matrix<double, Eigen::Dynamic, 3> ref_vecs, Eigen::Matrix3d attMatrix, const State& prev_state, const Eigen::VectorXd& measurement);

private:
    ProcessModel* _process_model;
    MeasurementModel* _measurement_model;
    State TimeUpdate(const Eigen::MatrixXd F, const Eigen::MatrixXd Q, const Eigen::MatrixXd G, const State& state, double dt);
    State MeasurementUpdate(const Eigen::MatrixXd H, const Eigen::MatrixXd R, const State& state, const Eigen::VectorXd& measurement);
};