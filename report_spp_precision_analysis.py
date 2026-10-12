from __future__ import annotations

from pathlib import Path

import matplotlib
import matplotlib.dates as mdates
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

try:
    import seaborn as sns
except ImportError:  # pragma: no cover
    sns = None


DEFAULT_INPUT_FILE = Path(r"D:\gnss_positioning\output\network4-29\spp_solution.txt")
DEFAULT_OUTPUT_DIR = Path(r"D:\gnss_positioning\报告撰写")

FIXED_COLUMNS = [
    "gps_week",
    "sow",
    "ecef_x_m",
    "ecef_y_m",
    "ecef_z_m",
    "ref_ecef_x_m",
    "ref_ecef_y_m",
    "ref_ecef_z_m",
    "east_m",
    "north_m",
    "up_m",
    "lat_deg",
    "lon_deg",
    "hgt_m",
    "vx_mps",
    "vy_mps",
    "vz_mps",
    "pdop",
    "sigma_p",
    "sigma_v",
    "gps_count",
    "bds_count",
]

COLORS = {
    "gps": "#4C78A8",
    "bds": "#59A14F",
    "pdop": "#B22222",
    "east": "#4C78A8",
    "north": "#B22222",
    "up": "#2E6F40",
    "sigma": "#B8C4D6",
    "velocity": "#6B5B95",
    "fit": "#2F2F2F",
    "gray": "#6B7280",
}


def load_spp_solution(file_path: Path) -> pd.DataFrame:
    rows: list[list[str]] = []
    with file_path.open("r", encoding="utf-8") as f:
        header_seen = False
        for raw_line in f:
            line = raw_line.strip()
            if not line:
                continue
            if not header_seen:
                header_seen = True
                continue
            tokens = line.split()
            if len(tokens) < len(FIXED_COLUMNS):
                continue
            rows.append(tokens[: len(FIXED_COLUMNS)])

    if not rows:
        raise ValueError(f"No valid data rows were found in {file_path}")

    df = pd.DataFrame(rows, columns=FIXED_COLUMNS)
    for col in FIXED_COLUMNS:
        df[col] = pd.to_numeric(df[col], errors="coerce")

    df["time"] = gps_week_sow_to_datetime(df["gps_week"], df["sow"])
    df["sat_total"] = df["gps_count"].fillna(0) + df["bds_count"].fillna(0)
    df["error_3d_m"] = np.sqrt(df["east_m"] ** 2 + df["north_m"] ** 2 + df["up_m"] ** 2)
    df["horizontal_error_m"] = np.sqrt(df["east_m"] ** 2 + df["north_m"] ** 2)
    df["speed_mps"] = np.sqrt(df["vx_mps"] ** 2 + df["vy_mps"] ** 2 + df["vz_mps"] ** 2)
    return df


def gps_week_sow_to_datetime(gps_week: pd.Series, sow: pd.Series) -> pd.Series:
    gps_epoch = pd.Timestamp("1980-01-06 00:00:00")
    week_delta = pd.to_timedelta(gps_week.fillna(0), unit="W")
    sow_delta = pd.to_timedelta(sow.fillna(0), unit="s")
    return gps_epoch + week_delta + sow_delta


def setup_matplotlib() -> None:
    if sns is not None:
        sns.set_theme(style="whitegrid", context="paper")
    matplotlib.rcParams["font.family"] = "sans-serif"
    matplotlib.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "SimSun", "Arial", "DejaVu Sans"]
    matplotlib.rcParams["mathtext.fontset"] = "stix"
    matplotlib.rcParams["axes.unicode_minus"] = False
    matplotlib.rcParams["figure.dpi"] = 600
    matplotlib.rcParams["savefig.dpi"] = 600
    matplotlib.rcParams["legend.frameon"] = True


def style_axis(ax: plt.Axes) -> None:
    ax.grid(True, alpha=0.3, linestyle="--", linewidth=0.7)
    ax.tick_params(direction="in")


def save_figure(fig: plt.Figure, output_dir: Path, stem: str) -> None:
    fig.savefig(output_dir / f"{stem}.png", dpi=600, bbox_inches="tight")
    fig.savefig(output_dir / f"{stem}.pdf", bbox_inches="tight")
    plt.close(fig)


def rms(series: pd.Series) -> float:
    s = pd.to_numeric(series, errors="coerce").dropna().to_numpy(dtype=float)
    if s.size == 0:
        return float("nan")
    return float(np.sqrt(np.mean(s**2)))


def summarize_components(df: pd.DataFrame, columns: list[tuple[str, str]]) -> pd.DataFrame:
    rows = []
    for label, col in columns:
        s = pd.to_numeric(df[col], errors="coerce").dropna()
        if s.empty:
            continue
        rows.append(
            {
                "Component": label,
                "Count": int(s.size),
                "Mean": float(s.mean()),
                "RMS": float(np.sqrt(np.mean(s.to_numpy(dtype=float) ** 2))),
                "STD": float(s.std(ddof=1)),
                "MaxAbs": float(s.abs().max()),
            }
        )
    return pd.DataFrame(rows)


def plot_constellation(df: pd.DataFrame, output_dir: Path) -> None:
    work = df.dropna(subset=["time"]).copy()
    fig, ax_left = plt.subplots(figsize=(11.2, 4.8), constrained_layout=True)
    ax_right = ax_left.twinx()

    ax_left.stackplot(
        work["time"],
        work["gps_count"].fillna(0),
        work["bds_count"].fillna(0),
        labels=["GPS (GS)", "BDS (BS)"],
        colors=[COLORS["gps"], COLORS["bds"]],
        alpha=0.85,
    )
    ax_right.plot(
        work["time"],
        work["pdop"],
        color=COLORS["pdop"],
        linewidth=1.6,
        label="PDOP",
    )

    ax_left.set_xlabel("Time")
    ax_left.set_ylabel("Satellite count")
    ax_right.set_ylabel("PDOP")
    ax_left.xaxis.set_major_formatter(mdates.DateFormatter("%H:%M"))
    style_axis(ax_left)
    ax_right.grid(False)

    handles_left, labels_left = ax_left.get_legend_handles_labels()
    handles_right, labels_right = ax_right.get_legend_handles_labels()
    ax_left.legend(handles_left + handles_right, labels_left + labels_right, loc="upper right")
    save_figure(fig, output_dir, "plotA_multisystem_constellation")


def plot_reliability(df: pd.DataFrame, output_dir: Path) -> None:
    work = df.dropna(subset=["time", "sigma_p"]).copy()
    fig, axes = plt.subplots(3, 1, figsize=(11.0, 7.5), sharex=True, constrained_layout=True)
    components = [
        ("east_m", r"$\Delta E$ (m)", COLORS["east"]),
        ("north_m", r"$\Delta N$ (m)", COLORS["north"]),
        ("up_m", r"$\Delta U$ (m)", COLORS["up"]),
    ]

    for ax, (col, ylabel, color) in zip(axes, components):
        valid = work[["time", "sigma_p", col]].dropna()
        ax.fill_between(
            valid["time"],
            -valid["sigma_p"],
            valid["sigma_p"],
            color=COLORS["sigma"],
            alpha=0.45,
            label=r"Formal envelope $\pm \sigma_P$",
        )
        ax.scatter(
            valid["time"],
            valid[col],
            s=10,
            color=color,
            alpha=0.65,
            label="Actual error",
        )
        ax.axhline(0.0, color=COLORS["gray"], linestyle=":", linewidth=0.9)
        ax.set_ylabel(ylabel)
        style_axis(ax)
        ax.legend(loc="upper right")

    axes[-1].set_xlabel("Time")
    axes[-1].xaxis.set_major_formatter(mdates.DateFormatter("%H:%M"))
    save_figure(fig, output_dir, "plotB_internal_vs_external_reliability")


def plot_velocity_stability(df: pd.DataFrame, output_dir: Path) -> None:
    work = df.dropna(subset=["time"]).copy()
    fig, axes = plt.subplots(3, 1, figsize=(11.0, 7.2), sharex=True, constrained_layout=True)
    components = [
        ("vx_mps", r"$V_X$ (m/s)", COLORS["east"]),
        ("vy_mps", r"$V_Y$ (m/s)", COLORS["north"]),
        ("vz_mps", r"$V_Z$ (m/s)", COLORS["up"]),
    ]

    for ax, (col, ylabel, color) in zip(axes, components):
        valid = work[["time", col]].dropna()
        comp_rms = rms(valid[col])
        ax.plot(valid["time"], valid[col], color=color, linewidth=1.05)
        ax.axhline(0.0, color=COLORS["gray"], linestyle=":", linewidth=0.9)
        ax.text(
            0.015,
            0.92,
            f"RMS = {comp_rms:.4f} m/s",
            transform=ax.transAxes,
            fontsize=10,
            va="top",
            ha="left",
            bbox={"facecolor": "white", "edgecolor": "0.8", "alpha": 0.85},
        )
        ax.set_ylabel(ylabel)
        style_axis(ax)

    axes[-1].set_xlabel("Time")
    axes[-1].xaxis.set_major_formatter(mdates.DateFormatter("%H:%M"))
    save_figure(fig, output_dir, "plotC_velocity_stability")


def fit_regression(x: np.ndarray, y: np.ndarray) -> tuple[np.ndarray, float]:
    coeff = np.polyfit(x, y, deg=1)
    y_fit = np.polyval(coeff, x)
    ss_res = np.sum((y - y_fit) ** 2)
    ss_tot = np.sum((y - y.mean()) ** 2)
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else np.nan
    return y_fit, float(r2)


def plot_geometry_correlation(df: pd.DataFrame, output_dir: Path) -> float:
    valid = df[["pdop", "error_3d_m"]].dropna()
    x = valid["pdop"].to_numpy(dtype=float)
    y = valid["error_3d_m"].to_numpy(dtype=float)
    y_fit, r2 = fit_regression(x, y)

    order = np.argsort(x)
    fig, ax = plt.subplots(figsize=(7.4, 5.8), constrained_layout=True)
    ax.scatter(x, y, s=18, alpha=0.55, color=COLORS["velocity"], edgecolors="none", label="Epochs")
    ax.plot(x[order], y_fit[order], color=COLORS["fit"], linewidth=1.6, label=fr"Linear fit, $R^2$ = {r2:.3f}")
    ax.set_xlabel("PDOP")
    ax.set_ylabel(r"3D position error (m)")
    style_axis(ax)
    ax.legend(loc="upper left")
    save_figure(fig, output_dir, "plotD_geometry_correlation")
    return r2


def write_summary(
    output_dir: Path,
    position_stats: pd.DataFrame,
    velocity_stats: pd.DataFrame,
    df: pd.DataFrame,
    r2: float,
) -> None:
    md_path = output_dir / "Statistics_Summary.md"

    avg_pdop = float(df["pdop"].dropna().mean())
    avg_sat_total = float(df["sat_total"].dropna().mean())
    avg_speed = float(df["speed_mps"].dropna().mean())

    pos = position_stats.copy()
    vel = velocity_stats.copy()
    for table in (pos, vel):
        numeric_cols = ["Mean", "RMS", "STD", "MaxAbs"]
        table[numeric_cols] = table[numeric_cols].round(4)

    lines = [
        "# Statistics Summary",
        "",
        "## Dataset Overview",
        "",
        f"- Input file: `{DEFAULT_INPUT_FILE}`",
        f"- Valid epochs: `{len(df)}`",
        f"- Mean PDOP: `{avg_pdop:.4f}`",
        f"- Mean tracked satellites (GS + BS): `{avg_sat_total:.2f}`",
        f"- Mean speed norm: `{avg_speed:.6f} m/s`",
        f"- PDOP vs. 3D error regression $R^2$: `{r2:.4f}`",
        "",
        "## Position Statistics",
        "",
        pos.to_markdown(index=False),
        "",
        "## Velocity Statistics",
        "",
        vel.to_markdown(index=False),
        "",
    ]
    md_path.write_text("\n".join(lines), encoding="utf-8")


def main() -> None:
    setup_matplotlib()
    DEFAULT_OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    df = load_spp_solution(DEFAULT_INPUT_FILE)
    df = df.dropna(subset=["time", "east_m", "north_m", "up_m"]).copy()

    position_stats = summarize_components(
        df,
        [
            ("East", "east_m"),
            ("North", "north_m"),
            ("Up", "up_m"),
            ("3D", "error_3d_m"),
        ],
    )
    velocity_stats = summarize_components(
        df,
        [
            ("Vx", "vx_mps"),
            ("Vy", "vy_mps"),
            ("Vz", "vz_mps"),
            ("Speed", "speed_mps"),
        ],
    )

    plot_constellation(df, DEFAULT_OUTPUT_DIR)
    plot_reliability(df, DEFAULT_OUTPUT_DIR)
    plot_velocity_stability(df, DEFAULT_OUTPUT_DIR)
    r2 = plot_geometry_correlation(df, DEFAULT_OUTPUT_DIR)
    write_summary(DEFAULT_OUTPUT_DIR, position_stats, velocity_stats, df, r2)

    print(f"Processed epochs: {len(df)}")
    print(f"Output directory: {DEFAULT_OUTPUT_DIR}")
    print("\nPosition statistics:")
    print(position_stats.to_string(index=False, float_format=lambda x: f"{x:0.4f}"))
    print("\nVelocity statistics:")
    print(velocity_stats.to_string(index=False, float_format=lambda x: f"{x:0.4f}"))
    print(f"\nAverage PDOP: {df['pdop'].dropna().mean():0.4f}")
    print(f"Regression R^2: {r2:0.4f}")


if __name__ == "__main__":
    main()
