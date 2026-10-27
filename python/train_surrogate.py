#!/usr/bin/env python3
"""Trains a small MLP surrogate for the two-link planar arm's static
torque calculation, then exports weights with input/output normalization
folded directly into the first/last layers - so the C++ inference code
(ml/src/surrogate.cpp) runs a plain raw-in/raw-out forward pass with no
separate normalization step, per the plan's "~20 lines of C++" intent.

Regenerate the training data this script expects with (from the project
root, after `cb`):
    build/bin/generate_training_data config/two_link_planar.json \
        python/data/two_link_planar_train.csv 20000 1 0 -9.81 0
    build/bin/generate_training_data config/two_link_planar.json \
        python/data/two_link_planar_val.csv 2000 2 0 -9.81 0

Usage (from the project root):
    python/.venv/bin/python python/train_surrogate.py
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import torch
from torch import nn

PROJECT_ROOT = Path(__file__).resolve().parent.parent
DATA_DIR = PROJECT_ROOT / "python" / "data"
OUTPUT_PATH = PROJECT_ROOT / "ml" / "models" / "two_link_planar.surrogate.json"

NUM_JOINTS = 2
HIDDEN_DIM = 16
EPOCHS = 8000
LEARNING_RATE = 1e-3
SEED = 0


def load_csv(path: Path) -> tuple[np.ndarray, np.ndarray]:
    data = np.loadtxt(path, delimiter=",", skiprows=1)
    inputs = data[:, : NUM_JOINTS + 1]  # joint values, then payload
    outputs = data[:, NUM_JOINTS + 1 :]  # torques
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
    torch.manual_seed(SEED)
    np.random.seed(SEED)

    train_x, train_y = load_csv(DATA_DIR / "two_link_planar_train.csv")
    val_x, val_y = load_csv(DATA_DIR / "two_link_planar_val.csv")

    x_mean = train_x.mean(axis=0)
    x_std = train_x.std(axis=0)
    x_std[x_std < 1e-8] = 1.0
    y_mean = train_y.mean(axis=0)
    y_std = train_y.std(axis=0)
    y_std[y_std < 1e-8] = 1.0

    train_x_norm = (train_x - x_mean) / x_std
    train_y_norm = (train_y - y_mean) / y_std

    model = Surrogate(NUM_JOINTS + 1, HIDDEN_DIM, NUM_JOINTS)
    optimizer = torch.optim.Adam(model.parameters(), lr=LEARNING_RATE)
    scheduler = torch.optim.lr_scheduler.StepLR(optimizer, step_size=EPOCHS // 4, gamma=0.3)
    loss_fn = nn.MSELoss()

    train_x_t = torch.from_numpy(train_x_norm)
    train_y_t = torch.from_numpy(train_y_norm)

    for epoch in range(EPOCHS):
        optimizer.zero_grad()
        pred = model(train_x_t)
        loss = loss_fn(pred, train_y_t)
        loss.backward()
        optimizer.step()
        scheduler.step()
        if epoch % 500 == 0 or epoch == EPOCHS - 1:
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

    OUTPUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    weights = {
        "arm_config": "config/two_link_planar.json",
        "gravity": [0.0, -9.81, 0.0],
        "num_joints": NUM_JOINTS,
        "input_dim": NUM_JOINTS + 1,
        "hidden_dim": HIDDEN_DIM,
        "output_dim": NUM_JOINTS,
        "val_mean_abs_error_nm": float(abs_err.mean()),
        "val_max_abs_error_nm": float(abs_err.max()),
        "w1": w1.tolist(),
        "b1": b1.tolist(),
        "w2": w2.tolist(),
        "b2": b2.tolist(),
        "w3": w3.tolist(),
        "b3": b3.tolist(),
    }
    OUTPUT_PATH.write_text(json.dumps(weights, indent=2))
    print(f"wrote {OUTPUT_PATH}")


if __name__ == "__main__":
    main()
