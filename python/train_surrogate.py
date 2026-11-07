#!/usr/bin/env python3
"""Trains a small MLP surrogate for a planar arm's static torque
calculation, then exports weights with input/output normalization
folded directly into the first/last layers - so the C++ inference code
(ml/src/surrogate.cpp) runs a plain raw-in/raw-out forward pass with no
separate normalization step, per the plan's "~20 lines of C++" intent.

Generic over which arm/config: --arm-name picks both the training CSVs
(python/data/<arm-name>_{train,val}.csv) and the output weights file
(ml/models/<arm-name>.surrogate.json). Every argument defaults to the
original two-link arm's values, so running with no arguments reproduces
that exact invocation - P5-M1 added the arguments (to also train a
three_dof_arm surrogate) without duplicating this whole script.

Regenerate the training data this script expects with (from the project
root, after `cb`) - the last argument is the payload range in kg:
    build/bin/generate_training_data config/humanoid_arm.json \
        python/data/humanoid_arm_train.csv 30000 1 0 -9.81 0 40
    build/bin/generate_training_data config/humanoid_arm.json \
        python/data/humanoid_arm_val.csv 3000 2 0 -9.81 0 40

Usage (from the project root) - the humanoid arm the app ships with:
    python/.venv/bin/python python/train_surrogate.py --arm-name humanoid_arm --num-joints 3 --hidden-dim 32

(The defaults still describe the two-link test fixture, whose weights now
live in tests/fixtures/.)
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import torch
from torch import nn

PROJECT_ROOT = Path(__file__).resolve().parent.parent
DATA_DIR = PROJECT_ROOT / "python" / "data"

SEED = 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--arm-name", default="two_link_planar", help="selects config/<arm-name>.json's data/output files")
    parser.add_argument("--num-joints", type=int, default=2)
    parser.add_argument("--hidden-dim", type=int, default=16)
    parser.add_argument("--epochs", type=int, default=8000)
    parser.add_argument("--learning-rate", type=float, default=1e-3)
    return parser.parse_args()


def load_csv(path: Path, num_joints: int) -> tuple[np.ndarray, np.ndarray]:
    data = np.loadtxt(path, delimiter=",", skiprows=1)
    inputs = data[:, : num_joints + 1]  # joint values, then payload
    outputs = data[:, num_joints + 1 :]  # torques
    return inputs.astype(np.float32), outputs.astype(np.float32)


class Surrogate(nn.Module):
    def __init__(self, input_dim: int, hidden_dim: int, output_dim: int) -> None:
        super().__init__()
        self.layer1 = nn.Linear(input_dim, hidden_dim)
        self.layer2 = nn.Linear(hidden_dim, hidden_dim)
        self.layer3 = nn.Linear(hidden_dim, output_dim)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        x = torch.relu(self.layer1(x))
        x = torch.relu(self.layer2(x))
        return self.layer3(x)


def fold_normalization(model: Surrogate, x_mean, x_std, y_mean, y_std):
    """Folds x_norm=(x-mean)/std into layer1 and y=y_norm*std+mean into
    layer3, so the returned weights operate directly on raw inputs and
    produce raw outputs - see the module docstring for why."""
    w1 = model.layer1.weight.detach().numpy()
    b1 = model.layer1.bias.detach().numpy()
    w2 = model.layer2.weight.detach().numpy()
    b2 = model.layer2.bias.detach().numpy()
    w3 = model.layer3.weight.detach().numpy()
    b3 = model.layer3.bias.detach().numpy()

    w1_folded = w1 / x_std[np.newaxis, :]
    b1_folded = b1 - w1 @ (x_mean / x_std)

    w3_folded = w3 * y_std[:, np.newaxis]
    b3_folded = b3 * y_std + y_mean

    return w1_folded, b1_folded, w2, b2, w3_folded, b3_folded


def main() -> None:
    args = parse_args()
    num_joints = args.num_joints
    hidden_dim = args.hidden_dim
    epochs = args.epochs
    output_path = PROJECT_ROOT / "ml" / "models" / f"{args.arm_name}.surrogate.json"

    torch.manual_seed(SEED)
    np.random.seed(SEED)

    train_x, train_y = load_csv(DATA_DIR / f"{args.arm_name}_train.csv", num_joints)
    val_x, val_y = load_csv(DATA_DIR / f"{args.arm_name}_val.csv", num_joints)

    x_mean = train_x.mean(axis=0)
    x_std = train_x.std(axis=0)
    x_std[x_std < 1e-8] = 1.0
    y_mean = train_y.mean(axis=0)
    y_std = train_y.std(axis=0)
    y_std[y_std < 1e-8] = 1.0

    train_x_norm = (train_x - x_mean) / x_std
    train_y_norm = (train_y - y_mean) / y_std

    model = Surrogate(num_joints + 1, hidden_dim, num_joints)
    optimizer = torch.optim.Adam(model.parameters(), lr=args.learning_rate)
    scheduler = torch.optim.lr_scheduler.StepLR(optimizer, step_size=epochs // 4, gamma=0.3)
    loss_fn = nn.MSELoss()

    train_x_t = torch.from_numpy(train_x_norm)
    train_y_t = torch.from_numpy(train_y_norm)

    for epoch in range(epochs):
        optimizer.zero_grad()
        pred = model(train_x_t)
        loss = loss_fn(pred, train_y_t)
        loss.backward()
        optimizer.step()
        scheduler.step()
        if epoch % 500 == 0 or epoch == epochs - 1:
            print(f"epoch {epoch:4d}  train_loss(normalized)={loss.item():.6f}")

    # Evaluate in RAW units (N*m) - the number that actually matters, not
    # the normalized training loss.
    model.eval()
    with torch.no_grad():
        val_pred_norm = model(torch.from_numpy((val_x - x_mean) / x_std)).numpy()
    val_pred_raw = val_pred_norm * y_std + y_mean
    abs_err = np.abs(val_pred_raw - val_y)
    print(f"val mean abs error: {abs_err.mean():.4f} N*m")
    print(f"val max abs error:  {abs_err.max():.4f} N*m")

    w1, b1, w2, b2, w3, b3 = fold_normalization(model, x_mean, x_std, y_mean, y_std)

    # Sanity-check the fold itself: a raw-input forward pass through the
    # folded weights must match the normalized-pipeline prediction above,
    # since it's the exact same function - just re-parameterized.
    folded_pred = np.maximum(val_x @ w1.T + b1, 0)
    folded_pred = np.maximum(folded_pred @ w2.T + b2, 0)
    folded_pred = folded_pred @ w3.T + b3
    fold_err = np.abs(folded_pred - val_pred_raw).max()
    print(f"fold sanity check (should be ~0): {fold_err:.8f}")
    assert fold_err < 1e-3, "normalization fold does not match the trained model"

    output_path.parent.mkdir(parents=True, exist_ok=True)
    weights = {
        "arm_config": f"config/{args.arm_name}.json",
        "gravity": [0.0, -9.81, 0.0],
        "num_joints": num_joints,
        "input_dim": num_joints + 1,
        "hidden_dim": hidden_dim,
        "output_dim": num_joints,
        "val_mean_abs_error_nm": float(abs_err.mean()),
        "val_max_abs_error_nm": float(abs_err.max()),
        # Largest payload in the training data. The C++ side never asks the
        # network about heavier payloads - an MLP's extrapolation outside its
        # training range is meaningless (see ArmController::runPayloadScan).
        "payload_max_kg": float(train_x[:, -1].max()),
        "w1": w1.tolist(),
        "b1": b1.tolist(),
        "w2": w2.tolist(),
        "b2": b2.tolist(),
        "w3": w3.tolist(),
        "b3": b3.tolist(),
    }
    output_path.write_text(json.dumps(weights, indent=2))
    print(f"wrote {output_path}")


if __name__ == "__main__":
    main()
