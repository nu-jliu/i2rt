# Zero Gravity Example

Initializes a YAM robot in zero-gravity (gravity compensation) mode and prints joint observations at 1 Hz.

In this mode the robot actively cancels gravity torques, allowing the arm to be freely backdrivable by hand — useful for kinesthetic teaching and teleoperation setup.

## Usage

```bash
# Simulation (no hardware required)
uv run python examples/zero_gravity/zero_gravity.py --sim

# Real hardware (default: yam arm + linear_4310 gripper on can0)
uv run python examples/zero_gravity/zero_gravity.py

# Specify arm, gripper, and CAN channel
uv run python examples/zero_gravity/zero_gravity.py --arm yam --gripper linear_4310 --channel can0
uv run python examples/zero_gravity/zero_gravity.py --arm big_yam --gripper crank_4310 --channel can0

# No gripper
uv run python examples/zero_gravity/zero_gravity.py --arm yam --gripper no_gripper --sim

# Custom per-joint gravity compensation factors (J1–J6)
uv run python examples/zero_gravity/zero_gravity.py --gravity-comp-factor 1.3 1.3 1.3 1.3 1.3 1.3
uv run python examples/zero_gravity/zero_gravity.py --sim --gravity-comp-factor 1.5 1.5 1.2 1.0 1.0 1.0
```

## Arguments

| Argument | Default | Description |
|---|---|---|
| `--arm` | `yam` | Arm type: `yam`, `yam_pro`, `yam_ultra`, `big_yam` |
| `--gripper` | `linear_4310` | Gripper type: `crank_4310`, `linear_3507`, `linear_4310`, `yam_teaching_handle`, `no_gripper` |
| `--sim` | off | Use MuJoCo simulation instead of real hardware |
| `--channel` | `can0` | CAN bus channel for real hardware |
| `--gravity-comp-factor J1..J6` | arm default | Six per-joint multipliers that scale gravity compensation torques |

## Gravity Compensation Tuning

The `--gravity-comp-factor` values scale the computed gravity torque for each joint. Values above 1.0 over-compensate (arm drifts upward), values below 1.0 under-compensate (arm is heavier). Start with the defaults and adjust per-joint until the arm feels weightless when backdriven by hand.
