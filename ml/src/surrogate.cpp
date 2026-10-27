#include "robotarm/surrogate.hpp"

#include <fstream>
#include <stdexcept>

#include <nlohmann/json.hpp>

namespace robotarm {

namespace {

Eigen::MatrixXd to_matrix(const nlohmann::json &rows) {
  const size_t num_rows = rows.size();
  const size_t num_cols = rows.at(0).size();
  Eigen::MatrixXd matrix(num_rows, num_cols);
  for (size_t i = 0; i < num_rows; i++) {
    for (size_t j = 0; j < num_cols; j++) {
      matrix(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(j)) = rows.at(i).at(j).get<double>();
    }
  }
  return matrix;
}

Eigen::VectorXd to_vector(const nlohmann::json &values) {
  Eigen::VectorXd vector(values.size());
  for (size_t i = 0; i < values.size(); i++) {
    vector(static_cast<Eigen::Index>(i)) = values.at(i).get<double>();
  }
  return vector;
}

} // namespace

Surrogate::Surrogate(const std::filesystem::path &weights_path) {
  std::ifstream file(weights_path);
  if (!file) {
    throw std::runtime_error("surrogate: could not open weights file '" + weights_path.string() + "'");
  }

  nlohmann::json document;
  file >> document;

  _num_joints = document.at("num_joints").get<size_t>();
  _val_mean_abs_error = document.value("val_mean_abs_error_nm", 0.0);
  _val_max_abs_error = document.value("val_max_abs_error_nm", 0.0);

  _w1 = to_matrix(document.at("w1"));
  _b1 = to_vector(document.at("b1"));
  _w2 = to_matrix(document.at("w2"));
  _b2 = to_vector(document.at("b2"));
  _w3 = to_matrix(document.at("w3"));
  _b3 = to_vector(document.at("b3"));
}

std::vector<double> Surrogate::predict(const std::vector<double> &joint_values, double payload) const {
  if (joint_values.size() != _num_joints) {
    throw std::invalid_argument("surrogate: expected " + std::to_string(_num_joints) + " joint values, got " +
                                 std::to_string(joint_values.size()));
  }

  std::vector<double> input(joint_values);
  input.push_back(payload);
  return predict_raw(input);
}

std::vector<double> Surrogate::predict_raw(const std::vector<double> &input) const {
  if (input.size() != input_dim()) {
    throw std::invalid_argument("surrogate: expected " + std::to_string(input_dim()) + " input values, got " +
                                 std::to_string(input.size()));
  }

  Eigen::VectorXd input_vector(static_cast<Eigen::Index>(input.size()));
  for (size_t i = 0; i < input.size(); i++) input_vector(static_cast<Eigen::Index>(i)) = input[i];

  // The hand-written forward pass the plan calls for: two ReLU hidden
  // layers, one linear output layer. Normalization is already folded into
  // w1/b1 and w3/b3 (see train_surrogate.py), so this runs directly on the
  // raw input and produces raw torques - no separate normalization step.
  const Eigen::VectorXd h1 = (_w1 * input_vector + _b1).cwiseMax(0.0);
  const Eigen::VectorXd h2 = (_w2 * h1 + _b2).cwiseMax(0.0);
  const Eigen::VectorXd output = _w3 * h2 + _b3;

  std::vector<double> result(static_cast<size_t>(output.size()));
  for (Eigen::Index i = 0; i < output.size(); i++) result[static_cast<size_t>(i)] = output(i);
  return result;
}

} // namespace robotarm
