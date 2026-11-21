"""Measure a trained surrogate per joint, against that joint's own limit.

One mean or max error over all joints hides what matters for safety: a
13 N*m miss is 15 % of the shoulder's 90 N*m limit but more than the
wrist's whole 12 N*m. This reads the shipped (normalization-folded)
weights, runs them on a validation CSV with plain NumPy - the same three
matrix multiplies the C++ side does - and reports each joint's mean and
max absolute error in N*m and as a percentage of its torque limit from
the arm config.

Usage (from the project root, after generating the validation CSV - see
train_surrogate.py / train_dynamic_surrogate.py):

    python3 python/evaluate_surrogate.py ml/models/humanoid_arm.surrogate.json \
        python/data/humanoid_arm_val.csv
    python3 python/evaluate_surrogate.py ml/models/humanoid_arm_dynamic.surrogate.json \
        python/data/humanoid_arm_dynamic_val.csv
"""

import argparse
import json
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parent.parent


def predict(weights: dict, inputs: np.ndarray) -> np.ndarray:
    hidden = np.maximum(inputs @ np.array(weights["w1"]).T + np.array(weights["b1"]), 0.0)
    hidden = np.maximum(hidden @ np.array(weights["w2"]).T + np.array(weights["b2"]), 0.0)
    return hidden @ np.array(weights["w3"]).T + np.array(weights["b3"])


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("weights", type=Path, help="a *.surrogate.json file")
    parser.add_argument("validation_csv", type=Path, help="inputs, then one torque column per joint")
    args = parser.parse_args()

    weights = json.loads(args.weights.read_text())
    config = json.loads((PROJECT_ROOT / weights["arm_config"]).read_text())
    input_dim = weights["input_dim"]
    data = np.loadtxt(args.validation_csv, delimiter=",", skiprows=1)
    inputs, torques = data[:, :input_dim], data[:, input_dim:]

    errors = np.abs(predict(weights, inputs) - torques)
    print(f"{args.weights.name} on {len(data)} validation samples")
    print(f"{'joint':<10}{'limit':>8}{'mean err':>11}{'max err':>10}{'mean %':>9}{'max %':>9}")
    for i, joint in enumerate(config["joints"]):
        limit = joint["torque_limit"]
        mean, worst = errors[:, i].mean(), errors[:, i].max()
        name = joint.get("name", f"joint {i}")
        print(f"{name:<10}{limit:>8.1f}{mean:>11.2f}{worst:>10.2f}{100 * mean / limit:>8.1f}%{100 * worst / limit:>8.1f}%")


if __name__ == "__main__":
    main()
