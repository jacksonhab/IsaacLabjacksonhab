# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause

"""Imitation-learning reward terms for the LiBR Hexapod.

Reward params must contain only OmegaConf-compatible primitives (str, float,
list[int], etc.) so that Isaac Lab's Hydra config serialization does not fail.
The MotionReference object is built lazily on the first reward call and cached
in a module-level dict keyed by the config primitives.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import torch

from isaaclab.managers import SceneEntityCfg

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv

# Module-level cache: built once per unique (csv_path, gait_period, col_order).
_MOTION_REF_CACHE: dict = {}


def _get_motion_ref(csv_path: str | None, gait_period: float, csv_col_order: list[int] | None):
    """Return a cached MotionReference, constructing it on first call."""
    from .hexapod_mimic_motion import MotionReference

    cache_key = (csv_path, gait_period, tuple(csv_col_order) if csv_col_order is not None else None)
    if cache_key not in _MOTION_REF_CACHE:
        _MOTION_REF_CACHE[cache_key] = MotionReference(
            csv_path=csv_path,
            gait_period=gait_period,
            csv_col_order=list(csv_col_order) if csv_col_order is not None else None,
        )
    return _MOTION_REF_CACHE[cache_key]


def joint_pos_imitation(
    env: ManagerBasedRLEnv,
    csv_path: str | None = None,
    gait_period: float = 0.5,
    csv_col_order: list[int] | None = None,
    joint_sigma: float = 0.25,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    """Reward joint positions for matching a reference gait trajectory.

    Uses a Gaussian kernel so the reward is 1.0 when all joints are at their
    reference angles and decays smoothly as error grows:

        r = exp( -||q - q_ref||^2 / sigma^2 )

    The reference is indexed by gait phase = (episode_elapsed_time % gait_period)
    / gait_period.

    Args:
        env:           The RL environment.
        csv_path:      Path to reference gait CSV, or None to use the built-in
                       sinusoidal tripod gait.
        gait_period:   Duration of one gait cycle in seconds.
        csv_col_order: Integer list to reorder CSV columns to Isaac Lab's
                       alphabetical joint order.  Pass SIM_DOF_TO_ALPHA if the
                       CSV was produced by play.py / playReal.py.
        joint_sigma:   Gaussian width in radians.  Smaller → stricter tracking.
        asset_cfg:     Scene entity config for the robot articulation.
    """
    motion_ref = _get_motion_ref(csv_path, gait_period, csv_col_order)

    asset = env.scene[asset_cfg.name]
    q_current = asset.data.joint_pos                              # [N, 8] alphabetical

    episode_time = env.episode_length_buf.float() * env.step_dt  # [N]
    phase = motion_ref.get_phase(episode_time)
    q_ref = motion_ref.get_reference(phase)                       # [N, 8] alphabetical

    # Per-joint Gaussian averaged across joints.  Summing squared errors across
    # all joints causes two high-amplitude spine joints to kill the gradient for
    # all six leg joints — per-joint mean keeps the signal alive independently.
    per_joint = torch.exp(-((q_current - q_ref) ** 2) / (joint_sigma ** 2))  # [N, 8]
    return per_joint.mean(dim=-1)                                              # [N]


def spine_pos_imitation(
    env: ManagerBasedRLEnv,
    csv_path: str | None = None,
    gait_period: float = 0.5,
    csv_col_order: list[int] | None = None,
    spine_sigma: float = 0.15,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    """Like joint_pos_imitation but restricted to the two spine joints only.

    Spine joint indices in alphabetical ordering: BackLink=1, FrontLink=4.
    """
    SPINE_COLS = [1, 4]

    motion_ref = _get_motion_ref(csv_path, gait_period, csv_col_order)

    asset = env.scene[asset_cfg.name]
    q_current = asset.data.joint_pos[:, SPINE_COLS]              # [N, 2]

    episode_time = env.episode_length_buf.float() * env.step_dt
    phase = motion_ref.get_phase(episode_time)
    q_ref = motion_ref.get_reference(phase)[:, SPINE_COLS]       # [N, 2]

    per_joint = torch.exp(-((q_current - q_ref) ** 2) / (spine_sigma ** 2))  # [N, 2]
    return per_joint.mean(dim=-1)
