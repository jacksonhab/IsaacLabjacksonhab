# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Isaac Lab (v2.3.2) is a GPU-accelerated robotics simulation framework built on NVIDIA Isaac Sim (4.5/5.0/5.1). This fork has been customized for hexapod locomotion research, adding a 6-legged robot (LiBR Hexapod) with flat-terrain RL training configurations.

**Key runtime requirements:**

- Isaac Sim 4.5+ installed and on PATH (or at `_isaac_sim` symlink)
- Python 3.11, PyTorch 2.7.0 + CUDA 12.8
- Hexapod USD model at `C:/Users/jrh6552/Hexapod/IsaacLab/Hexapod_Flattened.usd`
- Data output directory at `C:/Users/jrh6552/Hexapod/IsaacLab/Position Files/`

## Common Commands

All scripts must be run via Isaac Sim's bundled Python (not system Python). Use `isaaclab.bat` on Windows:

```bat
:: Train hexapod on flat terrain
isaaclab.bat -p scripts/reinforcement_learning/rsl_rl/train.py --task Isaac-Velocity-Flat-Hexapod-v0 --num_envs 4096

:: Resume training from checkpoint
isaaclab.bat -p scripts/reinforcement_learning/rsl_rl/train.py --task Isaac-Velocity-Flat-Hexapod-v0 --resume

:: Play/evaluate a checkpoint (logs joint positions to CSV)
isaaclab.bat -p scripts/reinforcement_learning/rsl_rl/play.py --task Isaac-Velocity-Flat-Hexapod-Play-v0 --num_envs 1

:: Play open-loop gait from CSV (compare against RL policy rewards)
isaaclab.bat -p scripts/reinforcement_learning/rsl_rl/playReal.py --task Isaac-Velocity-Flat-Hexapod-Play-v0 --num_envs 1 --gait_csv <path_to_csv> --gait_mode pos --gait_dt <seconds_per_row> --warmup_time 1.0

:: List all registered environments
isaaclab.bat -p scripts/environments/list_envs.py

:: Run with a specific checkpoint
isaaclab.bat -p scripts/reinforcement_learning/rsl_rl/play.py --task Isaac-Velocity-Flat-Hexapod-Play-v0 --checkpoint <path>
```

```bat
:: Code quality (run from repo root)
pre-commit run --all-files

:: Linting (ruff, line-length=120)
ruff check source/
ruff format source/
```

```bat
:: Run tests (requires Isaac Sim Python)
isaaclab.bat -p -m pytest source/ -m "not isaacsim_ci"
```

Training logs save to `logs/rsl_rl/<experiment_name>/<timestamp>/`.

## Repository Structure

```text
source/
  isaaclab/          # Core framework: env managers, sensors, controllers, terrain
  isaaclab_assets/   # Robot and sensor config dataclasses (ArticulationCfg)
  isaaclab_tasks/    # Task definitions: reward/obs/termination/event MDP terms
  isaaclab_rl/       # RL-specific wrappers (RslRlVecEnvWrapper, export utilities)
  isaaclab_mimic/    # Imitation learning support
scripts/
  reinforcement_learning/rsl_rl/  # train.py, play.py, playReal.py (primary scripts)
  environments/                   # Utility scripts: list_envs, random_agent, zero_agent
```

## Architecture: How a Task is Defined

Tasks use a layered config inheritance pattern. Everything is a Python dataclass decorated with `@configclass`:

```text
LocomotionVelocityRoughEnvCfg          # source/isaaclab_tasks/.../velocity_env_cfg.py
  └── HexapodRoughEnvCfg               # config/hexapod/rough_env_cfg.py
        └── HexapodFlatEnvCfg          # config/hexapod/flat_env_cfg.py
              └── HexapodFlatEnvCfg_PLAY
```

The environment config holds nested sub-configs for:

- `scene` — robot, terrain, sensors (height scanner, contact sensor)
- `observations` — `policy` and `critic` groups with individual term configs
- `rewards` — each reward term has `.weight` and `.params`
- `terminations` — episode ending conditions
- `events` — resets and domain randomization (mass, friction, push)
- `curriculum` — terrain level progression
- `commands` — velocity command sampling ranges

The gym environment is instantiated by `ManagerBasedRLEnv` using these configs. Each sub-config (`rewards`, `observations`, etc.) is handled by a matching Manager class that dynamically calls the referenced MDP functions.

## Hexapod-Specific Details

**Robot asset:** `source/isaaclab_assets/isaaclab_assets/robots/hexapod.py`

- USD at `C:/Users/jrh6552/Hexapod/IsaacLab/Hexapod_Flattened.usd`
- 8 joints total: `FrontLink`, `BackLink` (spine), + 6 leg joints (`MiddleLeft/Right`, `BackLeft/Right`, `FrontLeft/Right`)
- **Two actuator groups** (split because spine and legs have different loading profiles):
  - `body_joints` (FrontLink, BackLink): stiffness=40, damping=0.4, velocity_limit=15.0 rad/s, effort_limit=4.5 N·m
    - Higher velocity limit required: body sin wave needs fast tracking that 5.5 rad/s cap would prevent
    - Lower stiffness than legs: reduces peak torque demand from sustained gravity loading on body sections
  - `leg_joints` (all 6 legs): stiffness=80, damping=0.9, velocity_limit=6.0 rad/s, effort_limit=4.5 N·m
  - Physical spec: Dynamixel XL430-W250-T, stall torque 1.4 N·m at 12V, no-load speed 5.97 rad/s
  - `effort_limit_sim` is NOT a 1:1 analog of physical torque; it caps the PD output and needs headroom for damping term (`damping × velocity` can exceed physical stall torque)
- Init pose: spine joints at 0.0 rad, all leg joints at -0.47 rad

**Gym registration:** `source/isaaclab_tasks/isaaclab_tasks/manager_based/locomotion/velocity/config/hexapod/__init__.py`

- `Isaac-Velocity-Flat-Hexapod-v0` / `Isaac-Velocity-Flat-Hexapod-Play-v0`
- `Isaac-Velocity-Rough-Hexapod-v0` / `Isaac-Velocity-Rough-Hexapod-Play-v0`

**Flat env key settings** (`flat_env_cfg.py`):

- Velocity target: lin_vel_x=(0.1, 0.1) m/s, y=0, yaw=0 (forward-only gait)
- No height scanner, no terrain curriculum, flat plane terrain
- Friction: static=(0.5, 0.6), dynamic=(0.35, 0.45) — tuned for PLA on wood
- Asymmetric actor-critic observations: actor sees proprioceptive-only (hardware-available), critic adds ground-truth base_lin_vel during training
- `obs_groups = {"policy": ["policy"], "critic": ["critic"]}` routes groups to actor/critic in PPO runner
- Action scale effectively 0.5: `q = q_default + 0.5 * action`
- `q_default` for legs: -0.47 rad (matches init_state); spine joints: 0.0 rad

**Hexapod-specific files in config folder:**

- `hexapod_obs_cfg.py` — `HexapodFlatObservationsCfg` with separate `PolicyCfg` (no base_lin_vel) and `CriticCfg` (adds base_lin_vel) observation groups
- `hexapod_rewards.py` — custom reward functions:
  - `feet_air_time_per_leg`: compares each leg's air time to its own previous contact time (duty-cycle aware), replacing the fixed global threshold in the standard `feet_air_time`. Uses `target_ratio` (default 0.67) so legs with longer contact phases need proportionally longer air phases. Only activates after a leg has completed its first contact phase.

**playReal.py** (`scripts/reinforcement_learning/rsl_rl/playReal.py`):

- Extends play.py to support open-loop gait CSV playback for sim-to-real comparison
- Key args: `--gait_csv`, `--gait_mode pos`, `--gait_dt <sec/row>`, `--warmup_time`, `--run_time`
- In `pos` mode: CSV values are absolute joint positions (rad); converted to actions via `(pos - default) / scale` where default comes from `robot.data.default_joint_pos`
- `q_default_list` in script: `[0.0, 0.0, -0.47, -0.47, -0.47, -0.47, -0.47, -0.47]`
- Prints applied torques (N·m) and base velocity (body frame, vx/vy/vz + yaw_rate) every 20 steps
- Logs joint positions to CSV and displacement tracking per step
- Reward breakdown printed at end of episode

**MATLAB gait conversion** (external, not in repo):

- Real DOF order: `[flink, blink, FR, FL, MR, ML, BR, BL]`
- Sim DOF order: `[BackLink, FrontLink, MiddleLeft, MiddleRight, BackLeft, BackRight, FrontLeft, FrontRight]`
- Reorder: `newOrder = [2 1 6 5 8 7 4 3]`
- Leg correction: `position_legs_rad = position_legs_rad * -1 + pi`
- Body corrections: BackLink unchanged, FrontLink negated (`*-1`)
- Ground contact encoder → rad: `val1 = (4096/2 + 300) → 2348 → 3.601 rad → -0.459 rad` (matches init_state -0.47)

**play.py data logging:**

- Joint positions (rad) → `C:/Users/jrh6552/Hexapod/IsaacLab/Position Files/HexapodRL_Rad_*.csv`
- Displacement tracking → `sim_displacement_log_*.csv`
- Reward breakdown printed at end of episode

## Actuator Tuning Notes

The `ImplicitActuatorCfg` in Isaac Lab applies: `torque = clip(stiffness*(q_target - q) - damping*q_dot, -effort_limit, effort_limit)`

Key tuning insights from this project:

- `effort_limit_sim` must be large enough to accommodate both position error AND damping terms simultaneously: at max velocity, `damping × velocity_limit` alone can equal or exceed the effort limit
- The real Dynamixel XL430 runs internal PID at ~1 kHz with load awareness; the sim PD controller needs extra headroom to approximate this
- Body (spine) joints saturate much more easily than leg joints because they sustain gravity loading through the full sin wave cycle; splitting actuator groups allows independent tuning
- If effort_limit is raised but joint still saturates: check velocity_limit_sim — if the commanded trajectory requires higher joint velocity than the cap, position error accumulates and torque saturates regardless of effort headroom

## RSL-RL Compatibility

The repo uses rsl-rl < 4.0.0. `handle_deprecated_rsl_rl_cfg` in `source/isaaclab_rl/isaaclab_rl/rsl_rl/utils.py` strips parameters unsupported by the installed version:

- Removes `optimizer` field
- Removes `share_cnn_encoders` (added in rsl-rl >= 4.0.0)

`HexapodFlatPPORunnerCfg` in `agents/rsl_rl_ppo_cfg.py` sets `obs_groups = {"policy": ["policy"], "critic": ["critic"]}` to route asymmetric observation groups to actor and critic networks respectively.

## MDP Terms Location

All reward/observation/termination/event functions referenced by string in configs are defined in:

- `source/isaaclab_tasks/isaaclab_tasks/manager_based/locomotion/velocity/config/hexapod/hexapod_rewards.py` — hexapod-specific custom rewards
- `source/isaaclab_tasks/isaaclab_tasks/manager_based/locomotion/velocity/mdp/` — locomotion-specific terms
- `source/isaaclab/isaaclab/envs/mdp/` — general-purpose MDP terms (shared across tasks)

When a config references e.g. `mdp.feet_air_time`, look in the locomotion mdp directory first, then the core mdp directory.

## Code Style

- Line length: 120 characters (ruff enforced)
- Python 3.11 type annotations (pyright strict mode)
- Pre-commit hooks: ruff lint + ruff-format + trailing whitespace
- No mock databases in tests; integration tests use live Isaac Sim physics
