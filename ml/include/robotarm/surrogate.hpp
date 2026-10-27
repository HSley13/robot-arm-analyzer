#pragma once

#include <filesystem>
#include <vector>

#include <Eigen/Dense>

namespace robotarm {

// Hand-written inference for the small MLP trained by
// python/train_surrogate.py - no ONNX Runtime, per the plan: for a model
// this small, the extra dependency isn't worth it. The exported weights
// already have input/output normalization folded in (see the training
// script), so this is a plain raw-in/raw-out forward pass: two ReLU hidden
// layers, one linear output layer. Deliberately has no dependency on
// robotarm_core - this layer only knows about plain numbers in and out,
// matching the plan's "none of the three pieces know about the others'
// internals" architecture.
class Surrogate {
public:
  explicit Surrogate(const std::filesystem::path &weights_path);

  // joint_values (one per joint) followed by payload mass (kg) - the
  // static surrogate's input layout (Module 5). Returns one torque per
  // joint. Delegates to predict_raw() after assembling this specific
  // layout - see that method for the dynamic surrogate's layout
  // (Module P2-M4), which has more inputs than just joint_values+payload.
  std::vector<double> predict(const std::vector<double> &joint_values, double payload) const;

  // Runs the forward pass directly on an already-assembled input vector,
  // which must have exactly input_dim() entries. Any trained MLP of this
  // shape (two ReLU hidden layers, one linear output layer, normalization
  // folded into the weights) works through this one generic path -
  // predict() above is just a convenience for the static surrogate's
  // specific [joint_values..., payload] layout.
  std::vector<double> predict_raw(const std::vector<double> &input) const;

  size_t num_joints() const { return _num_joints; }
  size_t input_dim() const { return static_cast<size_t>(_w1.cols()); }
  double val_mean_abs_error() const { return _val_mean_abs_error; }
  double val_max_abs_error() const { return _val_max_abs_error; }

private:
  size_t _num_joints = 0;
  double _val_mean_abs_error = 0.0;
  double _val_max_abs_error = 0.0;

  Eigen::MatrixXd _w1, _w2, _w3;
  Eigen::VectorXd _b1, _b2, _b3;
};

} // namespace robotarm
