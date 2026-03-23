"""Gravity compensation example for i2rt robots.

Starts the robot in zero-gravity mode so that only gravity-compensation
torques are applied (no PD position control).  The arm floats freely and
can be moved by hand.  Joint state is printed to the terminal in real time.

Usage:
    uv run python examples/gravity_compensation/gravity_compensation.py --sim
    uv run python examples/gravity_compensation/gravity_compensation.py --sim --log-torques
    uv run python examples/gravity_compensation/gravity_compensation.py --arm big_yam --gripper no_gripper --sim
    uv run python examples/gravity_compensation/gravity_compensation.py --channel can0 --log-torques
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import argparse
import logging
import time

import numpy as np

from i2rt.robots.get_robot import get_yam_robot
from i2rt.robots.utils import ArmType, GripperType

logging.basicConfig(level=logging.INFO)


def _format_state_table(
    joint_pos: np.ndarray,
    joint_vel: np.ndarray,
    joint_eff: np.ndarray,
    torques: np.ndarray | None,
    gripper_pos: float | None,
    loop_freq: float | None,
) -> str:
    """Build a Unicode box-drawing table for joint state (and optional torques)."""
    has_torques = torques is not None
    lw = 7  # label column width
    cw = 16  # data column width

    cols = ["Pos (rad)", "Vel (rad/s)", "Effort (Nm)"]
    if has_torques:
        cols.append("Torque (Nm)")

    top = f"┌{'─' * lw}" + "".join(f"┬{'─' * cw}" for _ in cols) + "┐"
    hdr = f"│{'Joint':^{lw}}" + "".join(f"│{c:^{cw}}" for c in cols) + "│"
    sep = f"├{'─' * lw}" + "".join(f"┼{'─' * cw}" for _ in cols) + "┤"
    btm = f"└{'─' * lw}" + "".join(f"┴{'─' * cw}" for _ in cols) + "┘"

    freq_str = f"  [{loop_freq:.1f} Hz]" if loop_freq is not None else ""
    rows = [f" Gravity Compensation{freq_str}", top, hdr, sep]
    for i in range(len(joint_pos)):
        line = f"│{f'j{i+1}':^{lw}}"
        line += f"│{joint_pos[i]:>+12.4f}    "
        line += f"│{joint_vel[i]:>+12.4f}    "
        line += f"│{joint_eff[i]:>+12.4f}    "
        if has_torques and i < len(torques):
            line += f"│{torques[i]:>+12.4f}    │"
        elif has_torques:
            line += f"│{'n/a':>16}│"
        else:
            line += "│"
        rows.append(line)

    if gripper_pos is not None:
        line = f"│{'grip':^{lw}}"
        line += f"│{gripper_pos:>12.4f}    "
        line += f"│{'':>16}"
        line += f"│{'':>16}"
        if has_torques:
            line += f"│{'':>16}│"
        else:
            line += "│"
        rows.append(line)

    rows.append(btm)
    return "\n".join(rows)


if __name__ == "__main__":
    arm_choices = [a.value for a in ArmType]
    gripper_choices = [g.value for g in GripperType]

    parser = argparse.ArgumentParser(
        description="Run a robot in gravity-compensation (zero-gravity) mode",
    )
    parser.add_argument("--arm", type=str, default="yam", choices=arm_choices)
    parser.add_argument("--gripper", type=str, default="linear_4310", choices=gripper_choices)
    parser.add_argument("--channel", type=str, default="can0", help="CAN channel")
    parser.add_argument("--sim", action="store_true", help="Use SimRobot")
    parser.add_argument("--log-torques", action="store_true", help="Log gravity compensation torques each iteration")
    args = parser.parse_args()

    arm = ArmType.from_string_name(args.arm)
    gripper = GripperType.from_string_name(args.gripper)

    robot = get_yam_robot(
        channel=args.channel,
        arm_type=arm,
        gripper_type=gripper,
        zero_gravity_mode=True,
        sim=args.sim,
    )

    # For sim torque computation when --log-torques is used
    kdl = None
    if args.log_torques and args.sim:
        from i2rt.utils.mujoco_utils import MuJoCoKDL

        kdl = MuJoCoKDL(robot.xml_path)

    print("Gravity compensation active. Press Ctrl+C to stop.")

    loop_freq = None
    last_time = time.monotonic()

    try:
        while True:
            now = time.monotonic()
            dt_actual = now - last_time
            last_time = now
            if dt_actual > 0:
                loop_freq = 1.0 / dt_actual

            obs = robot.get_observations()
            joint_pos = obs["joint_pos"]
            joint_vel = obs["joint_vel"]
            joint_eff = obs["joint_eff"]
            gripper_pos = obs.get("gripper_pos")
            if gripper_pos is not None:
                gripper_pos = float(gripper_pos[0])

            torques = None
            if args.log_torques:
                if args.sim and kdl is not None:
                    torques = kdl.compute_inverse_dynamics(
                        joint_pos, np.zeros_like(joint_pos), np.zeros_like(joint_pos)
                    )
                elif hasattr(robot, "get_motor_torques"):
                    torques = robot.get_motor_torques()

            table = _format_state_table(joint_pos, joint_vel, joint_eff, torques, gripper_pos, loop_freq)
            print("\033[2J\033[H" + table, flush=True)

    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        robot.close()
