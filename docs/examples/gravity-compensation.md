# Gravity Compensation

**Location:** `examples/gravity_compensation/`

Runs the robot in zero-gravity mode — only gravity-compensation torques are applied (no PD position control). The arm floats freely and can be moved by hand. Joint state is printed to the terminal in real time.

## Hardware Required

- None for simulation (`--sim` flag)
- 1x YAM / YAM Pro / YAM Ultra / BIG YAM arm + CANable adapter for real hardware

## Running

### Simulation

```bash
python examples/gravity_compensation/gravity_compensation.py --sim
```

### Real Hardware

```bash
python examples/gravity_compensation/gravity_compensation.py --channel can0
```

### With Torque Logging

```bash
python examples/gravity_compensation/gravity_compensation.py --channel can0 --log-torques
python examples/gravity_compensation/gravity_compensation.py --sim --log-torques
```

### Different Arm/Gripper Assemblies

```bash
python examples/gravity_compensation/gravity_compensation.py --arm big_yam --gripper no_gripper --sim
python examples/gravity_compensation/gravity_compensation.py --arm yam_pro --gripper crank_4310 --sim
```

## Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `--arm` | `yam` | Arm type: `yam`, `yam_pro`, `yam_ultra`, `big_yam` |
| `--gripper` | `linear_4310` | Gripper type: `linear_4310`, `linear_3507`, `crank_4310`, `flexible_4310`, `yam_teaching_handle`, `no_gripper` |
| `--channel` | `can0` | CAN interface name (ignored in sim mode) |
| `--sim` | off | Use simulated robot instead of real hardware |
| `--log-torques` | off | Display gravity compensation torques per joint |

## Features

- Real-time Unicode table display of joint positions, velocities, and efforts
- Optional gravity-compensation torque logging (`--log-torques`)
- Support for all arm types and gripper types
- Loop frequency display
