#!/usr/bin/env python3
import argparse
import json
import os
import shutil
from pathlib import Path

import numpy as np
import pandas as pd


JOINT_INDICES = [0, 1, 2, 3, 4, 5, 7, 8, 9, 10, 11, 12]
GRIPPER_INDICES = [6, 13]


def vector_to_numpy(value: object) -> np.ndarray:
    return np.asarray(value, dtype=np.float32).reshape(-1)


def make_state_delta_actions(
    df: pd.DataFrame,
    horizon: int,
    joint_delta_clip: float | None,
) -> list[np.ndarray]:
    actions: list[np.ndarray] = [np.zeros(14, dtype=np.float32) for _ in range(len(df))]
    for _, episode_df in df.groupby("episode_index", sort=False):
        positions = list(episode_df.index)
        states = np.stack(
            [vector_to_numpy(value) for value in episode_df["observation.state"]],
            axis=0,
        )
        for local_idx, row_idx in enumerate(positions):
            target_idx = min(local_idx + horizon, len(positions) - 1)
            current = states[local_idx]
            target = states[target_idx]
            delta = np.zeros_like(current, dtype=np.float32)
            delta[JOINT_INDICES] = target[JOINT_INDICES] - current[JOINT_INDICES]
            delta[GRIPPER_INDICES] = target[GRIPPER_INDICES]
            actions[row_idx] = apply_action_limits(delta, joint_delta_clip)
    return actions


def apply_action_limits(action: np.ndarray, joint_delta_clip: float | None) -> np.ndarray:
    delta = action.copy()
    if joint_delta_clip is not None:
        delta[JOINT_INDICES] = np.clip(
            delta[JOINT_INDICES],
            -joint_delta_clip,
            joint_delta_clip,
        )
    return delta.astype(np.float32)


def stats_for(values: np.ndarray) -> dict:
    return {
        "min": values.min(axis=0).astype(float).tolist(),
        "max": values.max(axis=0).astype(float).tolist(),
        "mean": values.mean(axis=0).astype(float).tolist(),
        "std": values.std(axis=0).astype(float).tolist(),
        "count": [int(values.shape[0])],
        "q01": np.quantile(values, 0.01, axis=0).astype(float).tolist(),
        "q10": np.quantile(values, 0.10, axis=0).astype(float).tolist(),
        "q50": np.quantile(values, 0.50, axis=0).astype(float).tolist(),
        "q90": np.quantile(values, 0.90, axis=0).astype(float).tolist(),
        "q99": np.quantile(values, 0.99, axis=0).astype(float).tolist(),
    }


def update_info(info_path: Path) -> None:
    info = json.loads(info_path.read_text())
    names = info["features"]["action"]["names"]
    updated = []
    for idx, name in enumerate(names):
        if idx in JOINT_INDICES:
            updated.append(name.replace("joint", "delta_joint"))
        else:
            updated.append(name.replace("joint7", "gripper_target"))
    info["features"]["action"]["names"] = updated
    info["delta_q_action"] = {
        "joint_indices": JOINT_INDICES,
        "gripper_indices_absolute_target": GRIPPER_INDICES,
        "definition": "action[joint] = observation.state[t+horizon][joint] - observation.state[t][joint]; gripper action remains future absolute target in meters",
    }
    info_path.write_text(json.dumps(info, indent=2) + "\n")


def hardlink_or_copy(src: str, dst: str) -> str:
    try:
        os.link(src, dst)
    except OSError:
        shutil.copy2(src, dst)
    return dst


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--src", required=True, type=Path)
    parser.add_argument("--dst", required=True, type=Path)
    parser.add_argument("--overwrite", action="store_true")
    parser.add_argument("--joint-delta-clip", type=float, default=None)
    parser.add_argument("--horizon", type=int, default=3)
    args = parser.parse_args()
    if args.horizon < 1:
        raise ValueError("--horizon must be >= 1")

    src = args.src.expanduser().resolve()
    dst = args.dst.expanduser().resolve()
    if not src.exists():
        raise FileNotFoundError(src)
    if dst.exists():
        if not args.overwrite:
            raise FileExistsError(dst)
        shutil.rmtree(dst)

    shutil.copytree(src, dst, copy_function=hardlink_or_copy)

    all_actions = []
    data_files = sorted((dst / "data").glob("chunk-*/*.parquet"))
    if not data_files:
        raise RuntimeError(f"No data parquet files found under {dst / 'data'}")

    for path in data_files:
        df = pd.read_parquet(path).reset_index(drop=True)
        df["action"] = make_state_delta_actions(
            df,
            horizon=args.horizon,
            joint_delta_clip=args.joint_delta_clip,
        )
        all_actions.extend(vector_to_numpy(value) for value in df["action"])
        tmp_path = path.with_suffix(path.suffix + ".tmp")
        df.to_parquet(tmp_path, index=False)
        os.replace(tmp_path, path)
        print(f"wrote {path}")

    actions = np.stack(all_actions, axis=0)
    stats_path = dst / "meta" / "stats.json"
    stats = json.loads(stats_path.read_text())
    stats["action"] = stats_for(actions)
    stats_path.write_text(json.dumps(stats, indent=2) + "\n")
    update_info(dst / "meta" / "info.json")

    print(f"created delta-q dataset: {dst}")
    print(f"frames: {actions.shape[0]}")
    print(f"horizon: {args.horizon}")
    print(f"joint delta clip: {args.joint_delta_clip}")
    print(f"joint delta min/max: {actions[:, JOINT_INDICES].min():.6f} / {actions[:, JOINT_INDICES].max():.6f}")
    print(f"gripper target min/max: {actions[:, GRIPPER_INDICES].min():.6f} / {actions[:, GRIPPER_INDICES].max():.6f}")


if __name__ == "__main__":
    main()
