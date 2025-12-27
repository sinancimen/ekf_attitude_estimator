// Written by Sinan Çimen, 2025. https://github.com/sinancimen

#include <iostream>
#include <string>
#include "AttitudeEstimator.hpp"
#include "GyroProcessModel.hpp"
#include "QuestMeasurementModel.hpp"

int main() {
    MeasurementModel* measurement_model = new QuestMeasurementModel();
    ProcessModel* process_model = new GyroProcessModel();
    AttitudeEstimator estimator(Eigen::VectorXd::Zero(6), Eigen::MatrixXd::Identity(6, 6), process_model, measurement_model);
    return 0;
}