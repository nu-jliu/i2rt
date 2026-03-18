"""MuJoCo control interface for i2rt robots.

Starts in VIS mode, mirroring the robot's joint state (gravity-comp active
on real hardware). Press SPACE to toggle into CONTROL mode, then double-click
the target marker, ctrl+right-drag to translate, ctrl+left-drag to rotate.
Commands are blocked when a self-collision is detected.

Usage:
    python examples/control_with_mujoco/control_with_mujoco.py --sim
    python examples/control_with_mujoco/control_with_mujoco.py --arm big_yam --gripper linear_4310 --sim
    python examples/control_with_mujoco/control_with_mujoco.py --channel can0
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import argparse

from i2rt.robots.get_robot import get_yam_robot
from i2rt.robots.utils import ArmType, GripperType
from i2rt.utils.mujoco_control_interface import MujocoControlInterface

if __name__ == "__main__":
    arm_choices = [a.value for a in ArmType]
    gripper_choices = [g.value for g in GripperType]

    parser = argparse.ArgumentParser(
        description="MuJoCo control interface for i2rt robots",
    )
    parser.add_argument("--arm", type=str, default="yam", choices=arm_choices)
    parser.add_argument("--gripper", type=str, default="linear_4310", choices=gripper_choices)
    parser.add_argument("--channel", type=str, default="can0", help="CAN channel")
    parser.add_argument("--sim", action="store_true", help="Use SimRobot")
    parser.add_argument("--dt", type=float, default=0.02, help="Loop timestep (s)")
    parser.add_argument("--site", type=str, default=None, help="EE site name (auto-detected if omitted)")
    args = parser.parse_args()

    arm = ArmType.from_string_name(args.arm)
    gripper = GripperType.from_string_name(args.gripper)

    robot = get_yam_robot(
        channel=args.channel,
        arm_type=arm,
        gripper_type=gripper,
        sim=args.sim,
    )

    # Teaching handle only has tcp_site; all grippers with fingers have grasp_site.
    if args.site is not None:
        site = args.site
    elif gripper == GripperType.YAM_TEACHING_HANDLE:
        site = "tcp_site"
    else:
        site = "grasp_site"

    iface = MujocoControlInterface.from_robot(robot, ee_site=site, dt=args.dt)
    iface.run()
