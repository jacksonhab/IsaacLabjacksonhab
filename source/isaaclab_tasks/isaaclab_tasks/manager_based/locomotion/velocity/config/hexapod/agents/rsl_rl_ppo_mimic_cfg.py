# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause

"""PPO runner configuration for the hexapod mimic + RL task.

Total training is split into two logical phases controlled by the curriculum:
  Phase 1  (iterations 0 … MIMIC_ITERATIONS-1):   imitation reward active.
  Phase 2  (iterations MIMIC_ITERATIONS … end):    pure RL rewards only.

The network architecture and observation space are identical to the standard
flat env, so a checkpoint from this task can be resumed directly under
Isaac-Velocity-Flat-Hexapod-v0 for further RL fine-tuning.
"""

from isaaclab.utils import configclass

from isaaclab_rl.rsl_rl import RslRlPpoAlgorithmCfg, RslRlPpoActorCriticCfg

from .rsl_rl_ppo_cfg import HexapodFlatPPORunnerCfg


@configclass
class HexapodMimicPPORunnerCfg(HexapodFlatPPORunnerCfg):
    """Runner config for HexapodMimicEnvCfg.

    Extends the flat-env config with:
      - A longer max_iterations to cover both the imitation and RL phases.
        Default: 800 mimic + 2200 RL = 3000 total iterations.
      - A distinctive experiment_name so logs land in a separate directory.
    """

    def __post_init__(self) -> None:
        super().__post_init__()

        self.max_iterations = 3000
        self.experiment_name = "hexapod_mimic"
        self.save_interval = 50

        # Lower initial action noise so the policy is forced to learn actual joint
        # control rather than outputting huge random actions that saturate at limits.
        # High init_noise_std (1.0) allows the policy to get imitation reward "for free"
        # by saturating joints near the reference — this causes VF divergence.
        self.policy.init_noise_std = 0.25

        # Reduce entropy coefficient to allow action std to decrease during imitation.
        # Default 0.01 kept the policy at std=14+ for 600 iterations; 0.005 lets the
        # policy collapse to more deterministic behavior when the gradient supports it.
        self.algorithm.entropy_coef = 0.005
