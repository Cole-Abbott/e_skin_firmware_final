#!/usr/bin/env python3
"""Delay-and-sum (DAS) beamforming for simulated and USB-captured RF data."""

import argparse
from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def analytic_signal_hilbert(x: np.ndarray, axis: int = 0) -> np.ndarray:
    """Hilbert analytic signal using only NumPy FFT."""
    n = x.shape[axis]
    h = np.zeros(n, dtype=np.float64)
    if n % 2 == 0:
        h[0] = 1.0
        h[n // 2] = 1.0
        h[1 : n // 2] = 2.0
    else:
        h[0] = 1.0
        h[1 : (n + 1) // 2] = 2.0

    shape = [1] * x.ndim
    shape[axis] = n
    h = h.reshape(shape)
    return np.fft.ifft(np.fft.fft(x, axis=axis) * h, axis=axis)


def das_beamform(
    rf_data: np.ndarray,
    fs: float,
    c: float,
    x_el: np.ndarray,
    x_img: np.ndarray,
    z_img: np.ndarray,
):
    n_samples, _ = rf_data.shape

    xg, zg = np.meshgrid(x_img, z_img)
    img = np.zeros_like(xg, dtype=np.float64)

    for ch, x_i in enumerate(x_el):
        dist = np.sqrt((xg - x_i) ** 2 + zg**2)
        sample_idx = np.rint((2.0 * dist / c) * fs).astype(np.int64)
        np.clip(sample_idx, 0, n_samples - 1, out=sample_idx)
        img += rf_data[:, ch][sample_idx]

    env = np.abs(analytic_signal_hilbert(img, axis=0))
    env /= np.max(env) + np.finfo(np.float64).eps
    env_db = 20.0 * np.log10(env + np.finfo(np.float64).eps)
    return img, env_db, x_el


def save_bmode(
    env_db: np.ndarray,
    x_img: np.ndarray,
    z_img: np.ndarray,
    out_png: Path,
    title: str,
):
    fig, ax = plt.subplots(figsize=(8, 6))
    im = ax.imshow(
        env_db,
        extent=[x_img[0] * 1e3, x_img[-1] * 1e3, z_img[-1] * 1e3, z_img[0] * 1e3],
        cmap="gray",
        vmin=-60.0,
        vmax=0.0,
        aspect="auto",
    )
    ax.set_xlabel("Lateral (mm)")
    ax.set_ylabel("Depth (mm)")
    ax.set_title(title)
    fig.colorbar(im, ax=ax, label="dB")
    fig.tight_layout()
    fig.savefig(out_png, dpi=180)
    plt.close(fig)


def run_simulated(args):
    x0 = args.scatter_x
    z0 = args.scatter_z
    fs = args.fs
    c = args.c
    n_samples = args.n_samples
    f_c = args.f_c
    pitch = args.pitch
    n_elements = args.n_elements

    t = np.arange(n_samples, dtype=np.float64) / fs
    burst_duration = args.burst_cycles / f_c
    elem_idx = np.arange(n_elements) - (n_elements - 1) / 2.0
    x_el = elem_idx * pitch

    rf_data = np.zeros((n_samples, n_elements), dtype=np.float64)
    for ch in range(n_elements):
        dist = np.sqrt((x0 - x_el[ch]) ** 2 + z0**2)
        t_delay = 2.0 * dist / c
        burst = np.sin(2.0 * np.pi * f_c * (t - t_delay))
        burst *= np.exp(-((t - t_delay) / (burst_duration / 2.5)) ** 2)
        peak = np.max(np.abs(burst))
        if peak > 0:
            burst /= peak
        rf_data[:, ch] = burst

    x_img = np.linspace(np.min(x_el) - args.x_margin, np.max(x_el) + args.x_margin, args.nx)
    z_img = np.linspace(args.z_min, args.z_max, args.nz)

    img, env_db, _ = das_beamform(rf_data, fs=fs, c=c, x_el=x_el, x_img=x_img, z_img=z_img)

    args.out_dir.mkdir(parents=True, exist_ok=True)
    np.savez(
        args.out_dir / "simulated_das.npz",
        rf_data=rf_data,
        img=img,
        env_db=env_db,
        x_img=x_img,
        z_img=z_img,
        fs=fs,
        c=c,
        pitch=pitch,
    )
    save_bmode(
        env_db=env_db,
        x_img=x_img,
        z_img=z_img,
        out_png=args.out_dir / "simulated_bmode.png",
        title="Synthetic Aperture DAS (Simulated Data)",
    )


def load_usb_csv(csv_path: Path) -> np.ndarray:
    data = np.loadtxt(csv_path, delimiter=",", skiprows=1)
    if data.ndim == 1:
        data = data[:, np.newaxis]
    return data.astype(np.float64)


def select_channels_from_iterations(
    data: np.ndarray,
    n_elements: int,
    start_col: int,
    frame_step: int,
) -> tuple[np.ndarray, np.ndarray]:
    indices = start_col + np.arange(n_elements) * frame_step
    if np.max(indices) >= data.shape[1]:
        raise ValueError(
            f"Not enough captures in CSV: requested max column {np.max(indices)}, "
            f"but only {data.shape[1]} captures are available."
        )
    rf = data[:, indices]
    return rf, indices


def auto_crop_rf(
    rf_data: np.ndarray,
    threshold: float,
    offset: int,
    crop_samples: int,
) -> np.ndarray:
    n_samples, n_elements = rf_data.shape
    cropped = np.zeros((crop_samples, n_elements), dtype=np.float64)
    for ch in range(n_elements):
        wf = rf_data[:, ch]
        idxs = np.where(np.abs(wf) > threshold)[0]
        start = 0
        if idxs.size > 0:
            start = int(idxs[0]) + int(offset)
        start = max(0, min(start, n_samples - 1))
        end = min(start + crop_samples, n_samples)
        seg = wf[start:end]
        cropped[: seg.shape[0], ch] = seg
    return cropped


def run_real(args):
    raw = load_usb_csv(args.csv)
    if args.capture_indices:
        indices = np.array([int(tok) for tok in args.capture_indices.split(",")], dtype=np.int64)
        if np.any(indices < 0) or np.any(indices >= raw.shape[1]):
            raise ValueError(
                f"capture-indices out of range for CSV with {raw.shape[1]} captures: {indices.tolist()}"
            )
        rf_data = raw[:, indices]
    else:
        rf_data, indices = select_channels_from_iterations(
            data=raw,
            n_elements=args.n_elements,
            start_col=args.start_col,
            frame_step=args.frame_step,
        )

    if args.subtract_dc:
        rf_data = rf_data - np.median(rf_data, axis=0, keepdims=True)

    # Build element positions from selected capture order.
    n_slots = rf_data.shape[1]
    slot_idx = np.arange(n_slots, dtype=np.float64)
    x_el = (slot_idx - (n_slots - 1) / 2.0) * args.pitch

    if args.empty_std_frac is not None:
        ch_std = np.std(rf_data, axis=0)
        max_std = np.max(ch_std) + np.finfo(np.float64).eps
        keep = ch_std >= (args.empty_std_frac * max_std)
        if np.sum(keep) < 2:
            raise ValueError(
                f"empty-std-frac={args.empty_std_frac} removes too many channels. "
                f"Kept {int(np.sum(keep))} of {len(keep)}."
            )
        rf_data = rf_data[:, keep]
        x_el = x_el[keep]
        indices = indices[keep]

    if args.crop_samples is not None:
        rf_data = auto_crop_rf(
            rf_data=rf_data,
            threshold=args.crop_threshold,
            offset=args.crop_offset,
            crop_samples=args.crop_samples,
        )

    fs = args.fs
    c = args.c
    n_samples = rf_data.shape[0]
    z_limit_data = (n_samples - 1) / fs * c / 2.0
    z_max = min(args.z_max, z_limit_data)
    if z_max <= args.z_min:
        raise ValueError(
            f"Invalid depth range. Data supports up to {z_limit_data * 1e3:.2f} mm, "
            f"but z_min={args.z_min * 1e3:.2f} mm and z_max={args.z_max * 1e3:.2f} mm."
        )

    x_img = np.linspace(np.min(x_el) - args.x_margin, np.max(x_el) + args.x_margin, args.nx)
    z_img = np.linspace(args.z_min, z_max, args.nz)

    img, env_db, _ = das_beamform(rf_data, fs=fs, c=c, x_el=x_el, x_img=x_img, z_img=z_img)

    args.out_dir.mkdir(parents=True, exist_ok=True)
    np.savez(
        args.out_dir / "real_das.npz",
        rf_data=rf_data,
        selected_capture_indices=indices,
        img=img,
        env_db=env_db,
        x_img=x_img,
        z_img=z_img,
        fs=fs,
        c=c,
        pitch=args.pitch,
        x_el=x_el,
    )
    save_bmode(
        env_db=env_db,
        x_img=x_img,
        z_img=z_img,
        out_png=args.out_dir / "real_bmode.png",
        title="Synthetic Aperture DAS (USB Data)",
    )

    print(f"Loaded CSV: {args.csv}")
    print(f"Input matrix shape (samples x captures): {raw.shape}")
    print(f"Selected captures for channels: {indices.tolist()}")
    print(f"Beamforming RF shape (samples x elements): {rf_data.shape}")
    print(f"Depth range used: {args.z_min * 1e3:.2f} mm to {z_max * 1e3:.2f} mm")
    print(f"Saved: {args.out_dir / 'real_das.npz'}")
    print(f"Saved: {args.out_dir / 'real_bmode.png'}")


def build_parser():
    parser = argparse.ArgumentParser(description="DAS beamforming for simulated and USB CSV data.")
    sub = parser.add_subparsers(dest="mode", required=True)

    sim = sub.add_parser("simulate", help="Run DAS with synthetic RF data.")
    sim.add_argument("--fs", type=float, default=6.25e6)
    sim.add_argument("--c", type=float, default=1540.0)
    sim.add_argument("--f-c", dest="f_c", type=float, default=1e6)
    sim.add_argument("--pitch", type=float, default=3e-3)
    sim.add_argument("--n-elements", type=int, default=16)
    sim.add_argument("--n-samples", type=int, default=2048)
    sim.add_argument("--burst-cycles", type=float, default=1.0)
    sim.add_argument("--scatter-x", type=float, default=-10e-3)
    sim.add_argument("--scatter-z", type=float, default=45e-3)
    sim.add_argument("--x-margin", type=float, default=10e-3)
    sim.add_argument("--z-min", type=float, default=5e-3)
    sim.add_argument("--z-max", type=float, default=100e-3)
    sim.add_argument("--nx", type=int, default=256)
    sim.add_argument("--nz", type=int, default=1024)
    sim.add_argument("--out-dir", type=Path, default=Path("beamforming_out/simulated"))
    sim.set_defaults(func=run_simulated)

    real = sub.add_parser("real", help="Run DAS with data saved by usb_file.py CSV format.")
    real.add_argument("--csv", type=Path, required=True)
    real.add_argument("--fs", type=float, default=14.296e6)
    real.add_argument("--c", type=float, default=1540.0)
    real.add_argument("--pitch", type=float, default=5e-3)
    real.add_argument("--n-elements", type=int, default=16)
    real.add_argument("--start-col", type=int, default=0)
    real.add_argument("--frame-step", type=int, default=1)
    real.add_argument(
        "--capture-indices",
        type=str,
        default=None,
        help="Comma-separated explicit capture indices, e.g. '3,4,5,6,7,8'.",
    )
    real.add_argument("--subtract-dc", action="store_true", default=True)
    real.add_argument("--no-subtract-dc", dest="subtract_dc", action="store_false")
    real.add_argument(
        "--empty-std-frac",
        type=float,
        default=None,
        help="Drop channels with std < (empty_std_frac * max_channel_std).",
    )
    real.add_argument("--crop-threshold", type=float, default=80.0)
    real.add_argument("--crop-offset", type=int, default=0)
    real.add_argument("--crop-samples", type=int, default=None)
    real.add_argument("--x-margin", type=float, default=10e-3)
    real.add_argument("--z-min", type=float, default=5e-3)
    real.add_argument("--z-max", type=float, default=100e-3)
    real.add_argument("--nx", type=int, default=256)
    real.add_argument("--nz", type=int, default=1024)
    real.add_argument("--out-dir", type=Path, default=Path("beamforming_out/real"))
    real.set_defaults(func=run_real)

    return parser


def main():
    parser = build_parser()
    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
