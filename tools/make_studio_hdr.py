#!/usr/bin/env python3
"""Generates app/assets/studio.hdr - the image-based-lighting environment
the 3D view lights the robot with.

Why: metal (the wrist clevis, screw heads, the dumbbell) has no color of
its own - it only reflects its surroundings - so with plain directional
lights it renders nearly black. A light probe gives every material
something to reflect. This is a procedural "photo studio": dark floor,
soft gradient walls, a large overhead softbox, a key softbox and a thin
rim strip. Generated (not downloaded) so the repo has no third-party
image asset; rerun to tweak it.

Writes a standard Radiance RGBE (.hdr) file with plain (uncompressed)
scanlines, equirectangular, 512x256.

Usage (from the project root):  python3 tools/make_studio_hdr.py
"""
from pathlib import Path

import numpy as np

W, H = 512, 256
OUT = Path(__file__).resolve().parent.parent / "app" / "assets" / "studio.hdr"


def softbox(lon, lat, center_lon, center_lat, half_w, half_h, intensity, softness=0.06):
    """A rounded rectangle of light on the sphere (angles in radians)."""
    dlon = np.angle(np.exp(1j * (lon - center_lon)))
    dx = np.abs(dlon) * np.cos(lat) - half_w
    dy = np.abs(lat - center_lat) - half_h
    d = np.maximum(dx, 0) ** 2 + np.maximum(dy, 0) ** 2
    return intensity * np.exp(-d / (softness ** 2))


def main() -> None:
    u = (np.arange(W) + 0.5) / W
    v = (np.arange(H) + 0.5) / H
    lon = (u - 0.5) * 2 * np.pi            # -pi..pi
    lat = (0.5 - v) * np.pi                # +pi/2 (up) .. -pi/2 (down)
    LON, LAT = np.meshgrid(lon, lat)

    # Base: dim blue-grey walls, brighter toward the horizon, dark floor.
    up = np.clip(np.sin(LAT), 0, 1)
    down = np.clip(-np.sin(LAT), 0, 1)
    wall = 0.20 + 0.10 * (1 - up)
    rgb = np.stack([wall * 0.85, wall * 0.92, wall * 1.08], axis=-1)
    floor = 0.06 + 0.04 * (1 - down)
    rgb = np.where(LAT[..., None] < 0, np.stack([floor, floor * 1.02, floor * 1.12], axis=-1), rgb)

    # Lights.
    light = (softbox(LON, LAT, 0.0, 1.25, 0.9, 0.18, 7.0, 0.15)      # large overhead box
             + softbox(LON, LAT, 0.7, 0.45, 0.28, 0.22, 9.0)        # key, front-right
             + softbox(LON, LAT, -1.0, 0.30, 0.22, 0.18, 3.0)       # fill, front-left
             + softbox(LON, LAT, 2.8, 0.25, 0.05, 0.45, 6.0, 0.03))  # thin rim strip behind
    warm = np.array([1.0, 0.97, 0.92])
    rgb = rgb + light[..., None] * warm

    # RGBE encode.
    maxc = rgb.max(axis=-1)
    exponent = np.ceil(np.log2(np.maximum(maxc, 1e-32)))
    scale = np.where(maxc > 1e-32, 256.0 / np.exp2(exponent), 0.0)
    mant = np.clip(np.floor(rgb * scale[..., None]), 0, 255).astype(np.uint8)
    e = np.where(maxc > 1e-32, exponent + 128, 0).astype(np.uint8)
    rgbe = np.concatenate([mant, e[..., None]], axis=-1)

    OUT.parent.mkdir(parents=True, exist_ok=True)
    with open(OUT, "wb") as f:
        f.write(b"#?RADIANCE\nFORMAT=32-bit_rle_rgbe\n\n")
        f.write(f"-Y {H} +X {W}\n".encode())
        f.write(rgbe.tobytes())
    print(f"wrote {OUT} ({W}x{H})")


if __name__ == "__main__":
    main()
