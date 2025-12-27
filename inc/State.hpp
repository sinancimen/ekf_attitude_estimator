// Written by Sinan Çimen, 2025. https://github.com/sinancimen

#pragma once

#include <Eigen/Dense>

class State {
 public:
  State(Eigen::VectorXd state_vector, Eigen::MatrixXd covariance_matrix)
      : _state_vector(state_vector), _covariance_matrix(covariance_matrix) {}

  Eigen::VectorXd GetStateVector() const { return _state_vector; }
  Eigen::MatrixXd GetCovarianceMatrix() const { return _covariance_matrix; }

  void SetStateVector(const Eigen::VectorXd& state_vector) { _state_vector = state_vector; }
  void SetCovarianceMatrix(const Eigen::MatrixXd& covariance_matrix) { _covariance_matrix = covariance_matrix; }

  int GetStateSize() const { return _state_vector.size(); }

 private:
  Eigen::VectorXd _state_vector;
  Eigen::MatrixXd _covariance_matrix;
};