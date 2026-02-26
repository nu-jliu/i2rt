"""Real-time MuJoCo visualizer for i2rt robots.

Usage:
    robot = get_yam_robot(...)
    viz = MujocoVisualizer.from_robot(robot)
    viz.run()
"""

import time

import mujoco
import mujoco.viewer
import numpy as np

from i2rt.robots.robot import Robot
from i2rt.robots.utils import ArmType, GripperType
from i2rt.robots.motor_chain_robot import MotorChainRobot


class MujocoVisualizer:
    """MuJoCo viewer that mirrors an i2rt robot's joint positions."""

    def __init__(self, model: mujoco.MjModel, robot: Robot):
        self._model = model
        self._data = mujoco.MjData(model)
        self._robot = robot

    # ---- factory constructors ------------------------------------------------

    @classmethod
    def from_xml(cls, xml_path: str, robot: Robot) -> "MujocoVisualizer":
        model = mujoco.MjModel.from_xml_path(xml_path)
        return cls(model, robot)

    @classmethod
    def from_robot(cls, robot: "MotorChainRobot") -> "MujocoVisualizer":  # noqa: F821
        return cls.from_xml(robot.xml_path, robot)

    # ---- public API ----------------------------------------------------------

    def update(self, joint_pos: np.ndarray) -> None:
        """Set joint positions and forward-compute kinematics."""
        n = min(len(joint_pos), self._model.nq)
        self._data.qpos[:n] = joint_pos[:n]
        mujoco.mj_forward(self._model, self._data)

    def run(self, dt: float = 0.02) -> None:
        """Open the viewer and mirror the robot's joint positions until closed.

        Joint positions are polled from ``robot.get_joint_pos()`` each
        iteration.
        """
        nq = self._model.nq
        print("Mirror mode – reading joint positions from robot")
        with mujoco.viewer.launch_passive(
            self._model,
            self._data,
            show_left_ui=False,
            show_right_ui=False,
        ) as viewer:
            try:
                while viewer.is_running():
                    qpos = self._robot.get_joint_pos()
                    n = min(len(qpos), nq)
                    self._data.qpos[:n] = qpos[:n]
                    mujoco.mj_forward(self._model, self._data)
                    viewer.sync()
                    time.sleep(dt)
            except KeyboardInterrupt:
                pass


if __name__ == "__main__":
    import argparse

    from scipy.spatial.transform import Rotation
    from i2rt.robots.kinematics import Kinematics

    arm_choices = [a.value for a in ArmType]
    gripper_choices = [g.value for g in GripperType]

    parser = argparse.ArgumentParser(
        description="Visualize an i2rt robot model in MuJoCo"
    )
    parser.add_argument(
        "--arm",
        type=str,
        default="yam",
        choices=arm_choices,
        help="Arm type",
    )
    parser.add_argument(
        "--gripper",
        type=str,
        default="crank_4310",
        choices=gripper_choices,
        help="Gripper type",
    )
    parser.add_argument(
        "--channel",
        type=str,
        default="can0",
        help="CAN channel for robot comm",
    )
    parser.add_argument(
        "--sim",
        action="store_true",
        help="Use a SimRobot instead of real hardware",
    )
    parser.add_argument(
        "--dt",
        type=float,
        default=0.02,
        help="Loop timestep in seconds",
    )
    parser.add_argument(
        "--move-interval",
        type=float,
        default=2.0,
        help="Seconds between random EE moves",
    )
    parser.add_argument(
        "--site",
        type=str,
        default="grasp_site",
        help="MuJoCo site name for EE",
    )
    parser.add_argument(
        "--pos-range",
        type=float,
        default=0.05,
        help="Max random position offset (m)",
    )
    parser.add_argument(
        "--ori-range",
        type=float,
        default=0.3,
        help="Max random orientation offset (rad)",
    )
    args = parser.parse_args()

    from i2rt.robots.get_robot import get_yam_robot

    arm = ArmType.from_string_name(args.arm)
    gripper = GripperType.from_string_name(args.gripper)
    robot = get_yam_robot(
        channel=args.channel, arm_type=arm, gripper_type=gripper, sim=args.sim
    )

    viz = MujocoVisualizer.from_robot(robot)
    model = viz._model
    nq = model.nq

    kin = Kinematics(robot.xml_path, args.site)

    full_q = robot.get_joint_pos().copy()
    start_arm_q = full_q[:nq]
    target_arm_q = start_arm_q.copy()

    steps_per_move = max(1, int(args.move_interval / args.dt))
    step = steps_per_move  # trigger a new random target immediately

    print(
        f"Random EE pose mode – moving to a new random EE pose every {args.move_interval}s"
    )
    with mujoco.viewer.launch_passive(
        model, viz._data, show_left_ui=False, show_right_ui=False
    ) as viewer:
        try:
            while viewer.is_running():
                if step >= steps_per_move:
                    current_q = robot.get_joint_pos().copy()
                    arm_q = current_q[:nq]
                    current_pose = kin.fk(arm_q)  # 4x4 homogeneous matrix

                    # Random position offset
                    pos_offset = np.random.uniform(
                        -args.pos_range, args.pos_range, size=3
                    )
                    # Random orientation offset (small rotation)
                    rot_offset = Rotation.from_rotvec(
                        np.random.uniform(-args.ori_range, args.ori_range, size=3)
                    ).as_matrix()

                    target_pose = current_pose.copy()
                    target_pose[:3, 3] += pos_offset
                    target_pose[:3, :3] = rot_offset @ target_pose[:3, :3]

                    success, ik_arm_q = kin.ik(
                        target_pose, args.site, init_q=arm_q, verbose=True
                    )

                    if not success:
                        print("  IK failed, retrying with a new random pose...")
                        continue

                    start_arm_q = arm_q
                    target_arm_q = ik_arm_q
                    step = 0
                    print(f"  EE target pos: {target_pose[:3, 3]}")

                alpha = step / steps_per_move
                interp_arm_q = (1 - alpha) * start_arm_q + alpha * target_arm_q
                interp_q = np.concatenate([interp_arm_q, robot.get_joint_pos()[nq:]])
                robot.command_joint_pos(interp_q)

                qpos = robot.get_joint_pos()
                n = min(len(qpos), nq)
                viz._data.qpos[:n] = qpos[:n]
                mujoco.mj_forward(model, viz._data)
                viewer.sync()

                time.sleep(args.dt)
                step += 1
        except KeyboardInterrupt:
            pass
