// Written by Sinan Çimen, 2025. https://github.com/sinancimen

#include "State.hpp"

class KalmanFilter {
 public:
  KalmanFilter() = default;
  ~KalmanFilter() = default;

  State TimeUpdate(const Eigen::MatrixXd F, const Eigen::MatrixXd Q, const Eigen::MatrixXd G, const State& state, double dt);
  State MeasurementUpdate(const Eigen::MatrixXd H, const Eigen::MatrixXd R, const State& state, const Eigen::VectorXd& measurement);

 private:
};