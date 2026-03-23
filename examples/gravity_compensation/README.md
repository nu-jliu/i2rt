# Gravity Compensation

Runs the robot in zero-gravity mode — only gravity-compensation torques are applied (no PD position control). The arm floats freely and can be moved by hand. Joint state is printed to the terminal in real time.

## Usage

### Simulation

```bash
uv run python examples/gravity_compensation/gravity_compensation.py --sim
```

### Real hardware

```bash
uv run python examples/gravity_compensation/gravity_compensation.py --channel can0
```

### With torque logging

```bash
uv run python examples/gravity_compensation/gravity_compensation.py --channel can0 --log-torques
uv run python examples/gravity_compensation/gravity_compensation.py --sim --log-torques
```

### Different arm/gripper assemblies

```bash
uv run python examples/gravity_compensation/gravity_compensation.py --arm big_yam --gripper no_gripper --sim
uv run python examples/gravity_compensation/gravity_compensation.py --arm yam_pro --gripper crank_4310 --sim
```

## Arguments

| Argument         | Default        | Description                                      |
| ---------------- | -------------- | ------------------------------------------------ |
| `--arm`          | `yam`          | Arm type (`yam`, `yam_pro`, `yam_ultra`, `big_yam`) |
| `--gripper`      | `linear_4310`  | Gripper type (`linear_4310`, `linear_3507`, `crank_4310`, `flexible_4310`, `yam_teaching_handle`, `no_gripper`) |
| `--channel`      | `can0`         | CAN channel (ignored in sim mode)                |
| `--sim`          | off            | Use simulated robot                              |
| `--log-torques`  | off            | Display gravity compensation torques per joint   |
