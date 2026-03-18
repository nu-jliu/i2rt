"""Initialize a YAM robot in zero-gravity (gravity compensation) mode and print joint observations.

Usage:
    python examples/zero_gravity/zero_gravity.py
    python examples/zero_gravity/zero_gravity.py --sim
    python examples/zero_gravity/zero_gravity.py --arm yam --gripper linear_4310 --channel can0
    python examples/zero_gravity/zero_gravity.py --arm big_yam --gripper crank_4310 --channel can0
    python examples/zero_gravity/zero_gravity.py --arm yam --gripper no_gripper --sim
    python examples/zero_gravity/zero_gravity.py --gravity-comp-factor 1.3 1.3 1.3 1.3 1.3 1.3
    python examples/zero_gravity/zero_gravity.py --sim --gravity-comp-factor 1.5 1.5 1.2 1.0 1.0 1.0
"""

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import argparse
import logging

import numpy as np

from i2rt.robots.get_robot import get_yam_robot
from i2rt.robots.utils import ArmType, GripperType

if __name__ == "__main__":
    arm_choices = [a.value for a in ArmType]
    gripper_choices = [g.value for g in GripperType]

    parser = argparse.ArgumentParser(description="Initialize a YAM robot in zero-gravity mode")
    parser.add_argument("--arm", type=str, default="yam", choices=arm_choices)
    parser.add_argument("--gripper", type=str, default="linear_4310", choices=gripper_choices)
    parser.add_argument("--sim", action="store_true", help="Use sim mode instead of real hardware")
    parser.add_argument("--channel", type=str, default="can0")
    parser.add_argument(
        "--gravity-comp-factor",
        type=float,
        nargs=6,
        metavar=("J1", "J2", "J3", "J4", "J5", "J6"),
        help="Per-joint gravity compensation factors for the 6 arm joints (overrides arm-type default)",
    )
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO)

    arm_type = ArmType.from_string_name(args.arm)
    gripper_type = GripperType.from_string_name(args.gripper)
    gravity_comp_factor = np.array(args.gravity_comp_factor) if args.gravity_comp_factor is not None else None

    robot = get_yam_robot(
        channel=args.channel,
        arm_type=arm_type,
        gripper_type=gripper_type,
        sim=args.sim,
        gravity_comp_factor=gravity_comp_factor,
    )
    print(f"Robot initialized: arm={args.arm}, gripper={args.gripper}, sim={args.sim}")

    while True:
        print(robot.get_observations())
        print(f"motor_torques: {robot.get_motor_torques()}")
        time.sleep(0.1)
