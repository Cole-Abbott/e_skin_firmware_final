#!/usr/bin/env python3
"""Generate simulated RF data and plot it over time."""

import argparse
from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def simulate_rf_data(
    n_samples: int,
    fs: float,
    c: float,
    f_c: float,
    pitch: float,
    n_elements: int,
    scatter_x: float,
    scatter_z: float,
    burst_cycles: float,
) -> tuple[np.ndarray, np.ndarray]:
    t = np.arange(n_samples, dtype=np.float64) / fs
    burst_duration = burst_cycles / f_c

    x_el = (np.arange(n_elements) - (n_elements - 1) / 2.0) * pitch
    rf_data = np.zeros((n_samples, n_elements), dtype=np.float64)

    for ch in range(n_elements):
        dist = np.sqrt((scatter_x - x_el[ch]) ** 2 + scatter_z**2)
        t_delay = 2.0 * dist / c
        burst = np.sin(2.0 * np.pi * f_c * (t - t_delay))
        burst *= np.exp(-((t - t_delay) / (burst_duration / 2.5)) ** 2)
        peak = np.max(np.abs(burst))
        if peak > 0:
            burst /= peak
        rf_data[:, ch] = burst

    return t, rf_data


def plot_traces(t: np.ndarray, rf_data: np.ndarray, out_file: Path):
    t_us = t * 1e6
    fig, ax = plt.subplots(figsize=(10, 6))
    for ch in range(rf_data.shape[1]):
        ax.plot(t_us, rf_data[:, ch], linewidth=1.0, alpha=0.8, label=f"Ch {ch + 1}")
    ax.set_xlabel("Time (us)")
    ax.set_ylabel("Amplitude (normalized)")
    ax.set_title("Simulated RF Data Over Time")
    ax.grid(True, alpha=0.3)
    ax.legend(ncol=4, fontsize=8)
    fig.tight_layout()
    fig.savefig(out_file, dpi=180)
    plt.close(fig)


def plot_heatmap(t: np.ndarray, rf_data: np.ndarray, out_file: Path):
    t_us = t * 1e6
    fig, ax = plt.subplots(figsize=(10, 6))
    im = ax.imshow(
        rf_data.T,
        aspect="auto",
        origin="lower",
        extent=[t_us[0], t_us[-1], 1, rf_data.shape[1]],
        cmap="seismic",
    )
    ax.set_xlabel("Time (us)")
    ax.set_ylabel("Channel")
    ax.set_title("Simulated RF Data (Channel vs Time)")
    fig.colorbar(im, ax=ax, label="Amplitude")
    fig.tight_layout()
    fig.savefig(out_file, dpi=180)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description="Plot simulated RF data over time.")
    parser.add_argument("--n-samples", type=int, default=2048)
    parser.add_argument("--fs", type=float, default=6.25e6)
    parser.add_argument("--c", type=float, default=1540.0)
    parser.add_argument("--f-c", dest="f_c", type=float, default=1e6)
    parser.add_argument("--pitch", type=float, default=3e-3)
    parser.add_argument("--n-elements", type=int, default=16)
    parser.add_argument("--scatter-x", type=float, default=-10e-3)
    parser.add_argument("--scatter-z", type=float, default=45e-3)
    parser.add_argument("--burst-cycles", type=float, default=1.0)
    parser.add_argument("--out-dir", type=Path, default=Path("beamforming_out/simulated_time"))
    args = parser.parse_args()

    args.out_dir.mkdir(parents=True, exist_ok=True)
    t, rf_data = simulate_rf_data(
        n_samples=args.n_samples,
        fs=args.fs,
        c=args.c,
        f_c=args.f_c,
        pitch=args.pitch,
        n_elements=args.n_elements,
        scatter_x=args.scatter_x,
        scatter_z=args.scatter_z,
        burst_cycles=args.burst_cycles,
    )

    traces_path = args.out_dir / "simulated_rf_traces.png"
    heatmap_path = args.out_dir / "simulated_rf_heatmap.png"
    npz_path = args.out_dir / "simulated_rf_data.npz"

    plot_traces(t=t, rf_data=rf_data, out_file=traces_path)
    plot_heatmap(t=t, rf_data=rf_data, out_file=heatmap_path)
    np.savez(npz_path, t=t, rf_data=rf_data)

    print(f"Saved: {traces_path}")
    print(f"Saved: {heatmap_path}")
    print(f"Saved: {npz_path}")


if __name__ == "__main__":
    main()
