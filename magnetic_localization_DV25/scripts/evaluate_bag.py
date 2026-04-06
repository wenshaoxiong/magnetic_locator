import argparse
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np
import yaml


@dataclass
class PoseSample:
    t: float
    p: np.ndarray
    q: np.ndarray


def quat_to_yaw(q: np.ndarray) -> float:
    x, y, z, w = q
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return float(np.arctan2(siny_cosp, cosy_cosp))


def read_bag(bag_path: Path, pose_topic: str, gt_topic: str):
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    storage_options = StorageOptions(uri=str(bag_path), storage_id="sqlite3")
    converter_options = ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr")
    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    type_map = {}
    for info in reader.get_all_topics_and_types():
        type_map[info.name] = info.type

    if pose_topic not in type_map:
        raise RuntimeError(f"missing topic: {pose_topic}")
    if gt_topic not in type_map:
        raise RuntimeError(f"missing topic: {gt_topic}")

    pose_cls = get_message(type_map[pose_topic])
    gt_cls = get_message(type_map[gt_topic])

    poses: List[PoseSample] = []
    gts: List[PoseSample] = []

    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        t = float(t_ns) * 1e-9
        if topic == pose_topic:
            msg = deserialize_message(data, pose_cls)
            p = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z], dtype=float)
            q = np.array([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w], dtype=float)
            poses.append(PoseSample(t=t, p=p, q=q))
        elif topic == gt_topic:
            msg = deserialize_message(data, gt_cls)
            if hasattr(msg, "pose") and hasattr(msg.pose, "pose"):
                pm = msg.pose.pose
            elif hasattr(msg, "pose"):
                pm = msg.pose
            else:
                continue
            p = np.array([pm.position.x, pm.position.y, pm.position.z], dtype=float)
            q = np.array([pm.orientation.x, pm.orientation.y, pm.orientation.z, pm.orientation.w], dtype=float)
            gts.append(PoseSample(t=t, p=p, q=q))

    poses.sort(key=lambda s: s.t)
    gts.sort(key=lambda s: s.t)
    return poses, gts


def interpolate_gt(gts: List[PoseSample], t: float) -> Optional[PoseSample]:
    if not gts:
        return None
    if t <= gts[0].t:
        return gts[0]
    if t >= gts[-1].t:
        return gts[-1]
    lo = 0
    hi = len(gts) - 2
    while lo <= hi:
        mid = (lo + hi) // 2
        if gts[mid].t <= t < gts[mid + 1].t:
            i = mid
            break
        if t < gts[mid].t:
            hi = mid - 1
        else:
            lo = mid + 1
    else:
        i = max(0, min(len(gts) - 2, lo))

    a = gts[i]
    b = gts[i + 1]
    u = 0.0 if (b.t - a.t) < 1e-9 else (t - a.t) / (b.t - a.t)
    p = a.p * (1.0 - u) + b.p * u
    q = a.q * (1.0 - u) + b.q * u
    q = q / (np.linalg.norm(q) + 1e-12)
    return PoseSample(t=t, p=p, q=q)


def compute_errors(poses: List[PoseSample], gts: List[PoseSample]) -> Tuple[np.ndarray, np.ndarray]:
    pos_err = []
    yaw_err = []
    for s in poses:
        gt = interpolate_gt(gts, s.t)
        if gt is None:
            continue
        pos_err.append(float(np.linalg.norm(s.p - gt.p)))
        dyaw = quat_to_yaw(s.q) - quat_to_yaw(gt.q)
        dyaw = (dyaw + np.pi) % (2.0 * np.pi) - np.pi
        yaw_err.append(abs(float(dyaw)))
    return np.array(pos_err, dtype=float), np.array(yaw_err, dtype=float)


def rmse(x: np.ndarray) -> float:
    if x.size == 0:
        return float("nan")
    return float(np.sqrt(np.mean(x * x)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True, help="ros2 bag directory")
    ap.add_argument("--pose_topic", default="/magnetic_pose")
    ap.add_argument("--gt_topic", default="/groundtruth_pose")
    ap.add_argument("--out_dir", required=True)
    args = ap.parse_args()

    bag = Path(args.bag)
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    poses, gts = read_bag(bag, args.pose_topic, args.gt_topic)
    pos_err_m, yaw_err_rad = compute_errors(poses, gts)

    stats = {
        "count": int(pos_err_m.size),
        "position_rmse_m": rmse(pos_err_m),
        "position_mean_m": float(np.mean(pos_err_m)) if pos_err_m.size else float("nan"),
        "position_max_m": float(np.max(pos_err_m)) if pos_err_m.size else float("nan"),
        "yaw_rmse_deg": rmse(yaw_err_rad) * 180.0 / np.pi,
        "yaw_mean_deg": float(np.mean(yaw_err_rad)) * 180.0 / np.pi if yaw_err_rad.size else float("nan"),
        "yaw_max_deg": float(np.max(yaw_err_rad)) * 180.0 / np.pi if yaw_err_rad.size else float("nan"),
    }

    (out_dir / "error_stats.yaml").write_text(yaml.safe_dump(stats, sort_keys=False), encoding="utf-8")

    try:
        import matplotlib.pyplot as plt

        p = np.array([s.p for s in poses], dtype=float) if poses else np.zeros((0, 3))
        g = np.array([s.p for s in gts], dtype=float) if gts else np.zeros((0, 3))

        fig = plt.figure(figsize=(8, 6))
        ax = fig.add_subplot(111)
        if g.shape[0] > 0:
            ax.plot(g[:, 0], g[:, 1], label="groundtruth")
        if p.shape[0] > 0:
            ax.plot(p[:, 0], p[:, 1], label="magnetic_pose")
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.grid(True)
        ax.legend()
        fig.tight_layout()
        fig.savefig(out_dir / "trajectory_plot.pdf")
        plt.close(fig)
    except Exception:
        pass


if __name__ == "__main__":
    main()

