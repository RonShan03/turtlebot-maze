import argparse
from collections import Counter
import csv
from dataclasses import asdict
import json
from pathlib import Path
import platform
import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import yaml
from .dataset import KittiSequence, save_poses
from .evaluation import align_poses, metrics
from .pipeline import Config, VisualOdometry
from .synthetic import generate


def run(args):
    config = Config(**yaml.safe_load(Path(args.config).read_text())) if args.config else Config()
    sequence = KittiSequence(args.root, args.sequence, args.max_frames)
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    if (output / "metrics.json").exists():
        raise ValueError("output already contains results; choose a new directory")
    tracker = VisualOdometry(sequence.K, config)
    poses, records, clouds = [], [], []
    for index, stamp, image in sequence:
        pose, row, landmarks = tracker.process(index, stamp, image)
        poses.append(pose)
        records.append(row)
        if len(landmarks):
            clouds.append(landmarks)
        if tracker.last_visualization is not None and (index % args.visualize_every == 0 or index == 1):
            cv2.imwrite(str(output / f"matches_{index:06d}.jpg"), tracker.last_visualization)
        if index % 20 == 0:
            print(f"frame={index} status={row['status']} features={row['features']} inliers={row['inliers']}", flush=True)
    poses = np.asarray(poses)
    save_poses(output / "estimated_poses.txt", poses)
    np.savetxt(output / "timestamps.txt", sequence.times)
    with (output / "frames.csv").open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(records[0]))
        writer.writeheader()
        writer.writerows(records)
    valid = np.array([r["status"] != "lost" for r in records])
    report = dict(dataset="synthetic" if (Path(args.root) / "synthetic.json").exists() else "KITTI odometry",
                  sequence=args.sequence, frames=len(poses), tracked_frames=int(valid.sum()),
                  tracking_failures=int((~valid).sum()), status_counts=dict(Counter(r["status"] for r in records)),
                  mean_features=float(np.mean([r["features"] for r in records])),
                  mean_inlier_ratio=float(np.mean([r["inlier_ratio"] for r in records if r["status"] not in ("anchor", "lost")])) if any(r["inliers"] for r in records) else 0.,
                  elapsed_tracking_seconds=float(sum(r["seconds"] for r in records)),
                  config=asdict(config), versions=dict(python=platform.python_version(), opencv=cv2.__version__, numpy=np.__version__),
                  scale_policy="unit initialization baseline, landmark PnP; fallback reuses last step, global alignment only for evaluation")
    aligned = poses
    if sequence.ground_truth is not None and valid.sum() >= 3 and np.var(poses[valid, :3, 3]) > 1e-12:
        _, alignment = align_poses(poses[valid], sequence.ground_truth[valid], args.alignment == "sim3")
        R, t, s = np.array(alignment["rotation"]), np.array(alignment["translation"]), alignment["scale"]
        aligned = poses.copy()
        aligned[:, :3, :3] = R @ poses[:, :3, :3]
        aligned[:, :3, 3] = s * poses[:, :3, 3] @ R.T + t
        report.update(alignment=alignment, **metrics(aligned, sequence.ground_truth, args.rpe_delta, valid))
        save_poses(output / "aligned_poses.txt", aligned)
        save_poses(output / "ground_truth_poses.txt", sequence.ground_truth)
    else:
        report["evaluation_unavailable"] = "missing truth or fewer than three nonstationary valid poses"
    (output / "metrics.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    fig, axes = plt.subplots(1, 2, figsize=(11, 4))
    if sequence.ground_truth is not None:
        gt = sequence.ground_truth[:, :3, 3]
        axes[0].plot(gt[:, 0], gt[:, 2], label="ground truth")
    axes[0].plot(aligned[:, 0, 3], aligned[:, 2, 3], label="estimated (globally aligned)")
    axes[0].scatter(aligned[~valid, 0, 3], aligned[~valid, 2, 3], marker="x", c="red", label="held lost poses")
    axes[0].set(xlabel="X (m after alignment)", ylabel="Z (m after alignment)", title=f"{report['dataset']} sequence {args.sequence}")
    axes[0].axis("equal")
    axes[0].legend(fontsize=8)
    axes[1].plot([r["features"] for r in records], label="ORB features")
    axes[1].plot([r["inliers"] for r in records], label="geometric inliers")
    axes[1].set(xlabel="frame", ylabel="count", title="Tracking diagnostics")
    axes[1].legend()
    fig.tight_layout()
    fig.savefig(output / "trajectory_xz.png", dpi=150)
    plt.close(fig)
    fig = plt.figure(figsize=(7, 5))
    ax = fig.add_subplot(projection="3d")
    ax.plot(*aligned[:, :3, 3].T, label="estimated aligned")
    if sequence.ground_truth is not None:
        ax.plot(*sequence.ground_truth[:, :3, 3].T, label="ground truth")
    ax.set(xlabel="X", ylabel="Y", zlabel="Z", title=report["dataset"])
    ax.legend()
    fig.savefig(output / "trajectory_3d.png", dpi=130)
    plt.close(fig)
    if clouds:
        X = np.concatenate(clouds)[::3]
        np.savetxt(output / "landmarks.xyz", X, fmt="%.5f")
        fig = plt.figure(figsize=(7, 5))
        ax = fig.add_subplot(projection="3d")
        ax.scatter(*X.T, s=1, alpha=0.3)
        ax.plot(*poses[:, :3, 3].T, c="red")
        ax.set(xlabel="X", ylabel="Y", zlabel="Z", title="Triangulated landmarks (arbitrary monocular units)")
        fig.savefig(output / "triangulation.png", dpi=130)
        plt.close(fig)
    print(json.dumps(report, indent=2))


def main():
    parser = argparse.ArgumentParser(description="DS685 monocular ORB visual odometry; no ground-truth scale during tracking")
    sub = parser.add_subparsers(dest="command", required=True)
    synth = sub.add_parser("synthetic", help="render a deterministic, explicitly synthetic KITTI-format sequence")
    synth.add_argument("--output", required=True)
    synth.add_argument("--frames", type=int, default=45)
    synth.add_argument("--seed", type=int, default=11)
    run_parser = sub.add_parser("run", help="track KITTI rectified grayscale image_0 and evaluate")
    run_parser.add_argument("--root", required=True)
    run_parser.add_argument("--sequence", default="00")
    run_parser.add_argument("--max-frames", type=int)
    run_parser.add_argument("--output", required=True)
    run_parser.add_argument("--config")
    run_parser.add_argument("--alignment", choices=["sim3", "se3"], default="sim3")
    run_parser.add_argument("--rpe-delta", type=int, default=1)
    run_parser.add_argument("--visualize-every", type=int, default=20)
    args = parser.parse_args()
    if args.command == "synthetic":
        generate(args.output, args.frames, args.seed)
    else:
        if args.visualize_every < 1 or args.rpe_delta < 1:
            parser.error("visualize-every and rpe-delta must be positive")
        run(args)


if __name__ == "__main__":
    main()
