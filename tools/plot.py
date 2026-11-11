import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd


def main():
    parser = argparse.ArgumentParser(description="Plot columns of a landing-sim CSV log against time.")
    parser.add_argument("csv", type=Path)
    parser.add_argument("--cols", nargs="+", default=None)
    parser.add_argument("--out", type=Path, default=None)
    parser.add_argument("--show", action="store_true")
    args = parser.parse_args()

    df = pd.read_csv(args.csv)
    if "t" not in df.columns:
        raise SystemExit("log has no 't' column")

    cols = args.cols or [c for c in df.columns if c != "t"]
    missing = [c for c in cols if c not in df.columns]
    if missing:
        raise SystemExit(f"unknown columns: {missing}")

    fig, axes = plt.subplots(len(cols), 1, sharex=True, figsize=(8, 2.2 * len(cols) + 1), squeeze=False)
    for ax, col in zip(axes[:, 0], cols):
        ax.plot(df["t"], df[col])
        ax.set_ylabel(col)
        ax.grid(True, alpha=0.3)
    axes[-1, 0].set_xlabel("t [s]")
    fig.tight_layout()

    out = args.out or args.csv.with_suffix(".png")
    fig.savefig(out, dpi=150)
    print(f"saved {out}")
    if args.show:
        plt.show()


if __name__ == "__main__":
    main()