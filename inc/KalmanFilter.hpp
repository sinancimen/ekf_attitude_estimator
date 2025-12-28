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
    void ProcessGyro(const Eigen::Vector3d& gyro_measurement, double dt, State* error_state, State* full_state);
    void ProcessMeasurement(const Eigen::Matrix<double, Eigen::Dynamic, 3> ref_vecs, Eigen::Matrix3d attMatrix, State* error_state, const Eigen::MatrixXd& measurement);

private:
    ProcessModel* _process_model;
    MeasurementModel* _measurement_model;
    void TimeUpdate(const Eigen::MatrixXd F, const Eigen::MatrixXd Q, const Eigen::MatrixXd G, State* error_state, State* full_state, Eigen::Vector3d rate_meas, double dt);
    void MeasurementUpdate(const Eigen::MatrixXd H, const Eigen::MatrixXd expected_y, const Eigen::MatrixXd R, State* error_state, const Eigen::MatrixXd& measurement);
};