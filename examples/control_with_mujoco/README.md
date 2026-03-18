# MuJoCo Control Interface

Interactive MuJoCo viewer for i2rt robots. Starts in gravity-comp visualisation mode
and can toggle into IK-based mocap control.

## Modes

### Default (mocap IK control)

| Mode | Description |
|------|-------------|
| **VIS** (default) | Mirrors the robot's live joint positions. Gravity-comp is active on real hardware. |
| **CONTROL** | Moves the robot by dragging a target marker in the viewer via inverse kinematics. |

Press **SPACE** in the viewer to toggle between modes.

### Slider visualization (`--viz`)

Interactive slider-based joint visualization with self-collision checking. Use the Control panel sliders (right side) to move joints. Commands are blocked when a self-collision is detected.

## Usage

### Simulation (no hardware required)

```bash
# YAM arm with linear_4310 gripper (default)
python examples/control_with_mujoco/control_with_mujoco.py --sim

# big_yam arm with linear_4310 gripper
python examples/control_with_mujoco/control_with_mujoco.py --arm big_yam --gripper linear_4310 --sim

# Arm-only (no gripper)
python examples/control_with_mujoco/control_with_mujoco.py --arm yam --gripper no_gripper --sim

# Slider visualization mode
python examples/control_with_mujoco/control_with_mujoco.py --sim --viz
python examples/control_with_mujoco/control_with_mujoco.py --arm yam --gripper crank_4310 --sim --viz
```

### Real hardware

```bash
python examples/control_with_mujoco/control_with_mujoco.py --channel can0
python examples/control_with_mujoco/control_with_mujoco.py --arm big_yam --gripper linear_4310 --channel can0
```

## Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `--arm` | `yam` | Arm type: `yam`, `yam_pro`, `yam_ultra`, `big_yam` |
| `--gripper` | `linear_4310` | Gripper type: `linear_4310`, `linear_3507`, `crank_4310`, `yam_teaching_handle`, `no_gripper` |
| `--channel` | `can0` | CAN interface name (real hardware only) |
| `--sim` | off | Use simulation instead of real hardware |
| `--dt` | `0.02` | Control loop timestep in seconds |
| `--site` | auto | EE site name (`grasp_site` for grippers, `tcp_site` for teaching handle) |
| `--viz` | off | Slider visualization mode instead of mocap IK control |

## Viewer Controls (CONTROL mode)

1. Press **SPACE** to enter CONTROL mode (marker turns red)
2. **Double-click** the target sphere to select it
3. **Ctrl + right-drag** — translate the target
4. **Ctrl + left-drag** — rotate the target
5. Press **SPACE** again to return to VIS mode
