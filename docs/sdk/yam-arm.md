# YAM Arm API

## Import

```python
from i2rt.robots.get_robot import get_yam_robot
```

## `get_yam_robot()`

Factory function — the recommended way to create a robot instance.

```python
robot = get_yam_robot(
    channel: str = "can0",
    arm_type: ArmType = ArmType.YAM,
    gripper_type: GripperType = GripperType.LINEAR_4310,
    zero_gravity_mode: bool = True,
    ee_mass: Optional[float] = None,
    ee_inertia: Optional[np.ndarray] = None,
    gravity_comp_factor: Optional[np.ndarray] = None,
    sim: bool = False,
)
```

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `channel` | `str` | `"can0"` | CAN interface name (e.g. `can0`, `can_follower_l`). Ignored in sim mode. |
| `arm_type` | `ArmType` | `ArmType.YAM` | Arm variant: `yam`, `yam_pro`, `yam_ultra`, `big_yam` |
| `gripper_type` | `GripperType` | `GripperType.LINEAR_4310` | See [Grippers](/sdk/grippers) |
| `zero_gravity_mode` | `bool` | `True` | Enable gravity compensation on init |
| `ee_mass` | `Optional[float]` | `None` | End-effector mass override in kg for gravity compensation |
| `ee_inertia` | `Optional[np.ndarray]` | `None` | 10-element inertia override `[ipos(3), quat(4), diaginertia(3)]` |
| `gravity_comp_factor` | `Optional[np.ndarray]` | `None` | Per-joint (6-element) scaling factor for gravity torques |
| `sim` | `bool` | `False` | Return a `SimRobot` instead of connecting to real hardware |

**Returns:** `Robot` instance (`MotorChainRobot` for real hardware, `SimRobot` when `sim=True`).

::: tip Zero-gravity vs PD mode
With `zero_gravity_mode=True` the arm floats — great for teleoperation. With `False`, the arm holds its current joint positions as PD targets. Use `False` when operating without the motor safety timeout.
:::

---

## `MotorChainRobot`

### `get_joint_pos() → np.ndarray`

Returns the current joint positions as a numpy array in **radians**. The shape is `(7,)` when a gripper is attached (6 arm + 1 gripper), or `(6,)` with `no_gripper` / `yam_teaching_handle`.

```python
q = robot.get_joint_pos()
# array([-0.335, 0.002, 0.008, -0.020, -0.411, -0.073, 0.0])
```

### `command_joint_pos(target: np.ndarray) → None`

Commands all joints to move to `target` (radians). The controller uses PD tracking.

```python
import numpy as np
robot.command_joint_pos(np.zeros(7))  # move to home (6 arm joints + 1 gripper)
```

::: warning Joint limits
Joint limits are defined in the URDF (`i2rt/robot_models/arm/yam/yam.urdf`). Exceeding limits can cause motor errors.
:::

### `get_gripper_pos() → float`

Returns the gripper position (normalized `0.0` = fully closed, `1.0` = fully open, for linear grippers).

### `command_gripper_pos(position: float) → None`

Commands the gripper to a target position.

---

## Command-line: Zero-gravity test

```bash
# Arm enters gravity-compensated floating mode
python i2rt/robots/motor_chain_robot.py \
  --channel can0 \
  --gripper_type linear_4310
```

---

## Motor Configuration Utilities

### Set timeout

```bash
# Disable the 400 ms safety timeout
python i2rt/motor_config_tool/set_timeout.py --channel can0
python i2rt/motor_config_tool/set_timeout.py --channel can0  # run twice

# Re-enable
python i2rt/motor_config_tool/set_timeout.py --channel can0 --timeout
```

### Zero motor offset

```bash
# Zero motor ID 1 (run for each motor 1–6 as needed)
python i2rt/motor_config_tool/set_zero.py --channel can0 --motor_id 1
```

---

## MuJoCo Visualization

The SDK uses MuJoCo for visualization and gravity computation. The model files are at:

```
i2rt/robot_models/arm/yam/
├── yam.urdf   — URDF for external tools (ROS, etc.)
└── yam.xml    — MuJoCo MJCF model
```

Launch the visualizer without hardware:

```bash
python examples/minimum_gello/minimum_gello.py --mode visualizer_local
```

---

## Leader–Follower Script

`examples/minimum_gello/minimum_gello.py` is the primary teleoperation entry point:

```
usage: minimum_gello.py [options]

  --gripper TYPE        Gripper type (default: linear_4310)
  --mode MODE           follower | leader | visualizer_local
  --can-channel CHAN    CAN interface (default: can0)
  --bilateral_kp FLOAT  Bilateral stiffness 0.1–0.3 (default: 0.2)
```

Higher `bilateral_kp` = leader arm feels heavier (more force feedback from follower load).

---

## See Also

- [Quick Start](/getting-started/quick-start)
- [Grippers](/sdk/grippers)
- [Bimanual Teleoperation Example](/examples/bimanual-teleoperation)
