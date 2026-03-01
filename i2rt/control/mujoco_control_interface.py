"""MuJoCo control interface for i2rt robots.

Starts in gravity-comp (vis-only) mode, mirroring the robot's joint state.
Press SPACE in the viewer to toggle mocap control mode, then double-click
the target marker, ctrl+right-drag to translate, ctrl+left-drag to rotate.

Usage:
    python i2rt/control/mujoco_control_interface.py --sim
    python i2rt/control/mujoco_control_interface.py --channel can0
"""

import os
import tempfile
import time
import xml.etree.ElementTree as ET
from enum import Enum, auto

import mujoco
import mujoco.viewer
import numpy as np

from i2rt.robots.kinematics import Kinematics
from i2rt.robots.motor_chain_robot import MotorChainRobot
from i2rt.robots.robot import Robot


class Mode(Enum):
    VIS = auto()
    CONTROL = auto()


class MujocoControlInterface:
    """MuJoCo viewer with gravity-comp visualisation and mocap IK control."""

    MOCAP_BODY_NAME = "mocap_target"
    _VIS_RGBA = np.array([0.2, 0.8, 0.2, 0.3])
    _CTRL_RGBA = np.array([0.9, 0.2, 0.2, 0.6])

    def __init__(
        self,
        robot: Robot,
        xml_path: str,
        ee_site: str = "grasp_site",
        dt: float = 0.02,
    ):
        self._robot = robot
        self._ee_site = ee_site
        self._dt = dt
        self._mode = Mode.VIS

        self._model = self._build_model(xml_path)
        self._data = mujoco.MjData(self._model)
        self._kin = Kinematics(xml_path, ee_site)

        self._nq = self._model.nq
        self._n_arm = sum(1 for j in range(self._model.njnt) if self._model.jnt_type[j] == mujoco.mjtJoint.mjJNT_HINGE)
        self._ee_site_id = mujoco.mj_name2id(
            self._model,
            mujoco.mjtObj.mjOBJ_SITE,
            ee_site,
        )
        self._mocap_id = self._model.body(self.MOCAP_BODY_NAME).mocapid[0]
        self._mocap_geom_id = mujoco.mj_name2id(
            self._model,
            mujoco.mjtObj.mjOBJ_GEOM,
            "mocap_target_geom",
        )

    @classmethod
    def from_robot(
        cls,
        robot: MotorChainRobot,
        ee_site: str = "grasp_site",
        dt: float = 0.02,
    ) -> "MujocoControlInterface":
        return cls(robot, robot.xml_path, ee_site, dt)

    # ---- model construction ---------------------------------------------------

    @classmethod
    def _build_model(cls, xml_path: str) -> mujoco.MjModel:
        """Load robot XML and inject a mocap target body."""
        tree = ET.parse(xml_path)
        root = tree.getroot()
        worldbody = root.find("worldbody")

        mocap = ET.SubElement(worldbody, "body")
        mocap.set("name", cls.MOCAP_BODY_NAME)
        mocap.set("mocap", "true")
        mocap.set("pos", "0 0 0.3")

        geom = ET.SubElement(mocap, "geom")
        geom.set("name", "mocap_target_geom")
        geom.set("type", "sphere")
        geom.set("size", "0.02")
        geom.set("rgba", " ".join(f"{v:.2f}" for v in cls._VIS_RGBA))
        geom.set("contype", "0")
        geom.set("conaffinity", "0")

        dir_path = os.path.dirname(os.path.abspath(xml_path))
        fd, tmp_path = tempfile.mkstemp(suffix=".xml", dir=dir_path)
        os.close(fd)
        try:
            tree.write(tmp_path, xml_declaration=True)
            model = mujoco.MjModel.from_xml_path(tmp_path)
        finally:
            try:
                os.unlink(tmp_path)
            except OSError:
                pass
        return model

    # ---- helpers --------------------------------------------------------------

    def _sync_mocap_to_ee(self) -> None:
        """Snap the mocap marker to the current end-effector pose."""
        site = self._data.site(self._ee_site_id)
        self._data.mocap_pos[self._mocap_id] = site.xpos.copy()
        quat = np.empty(4)
        mujoco.mju_mat2Quat(quat, site.xmat.flatten())
        self._data.mocap_quat[self._mocap_id] = quat

    def _mocap_pose_4x4(self) -> np.ndarray:
        """Return the mocap target pose as a 4x4 homogeneous matrix."""
        pos = self._data.mocap_pos[self._mocap_id]
        quat = self._data.mocap_quat[self._mocap_id]
        mat = np.eye(4)
        mat[:3, 3] = pos
        rot = np.empty(9)
        mujoco.mju_quat2Mat(rot, quat)
        mat[:3, :3] = rot.reshape(3, 3)
        return mat

    def _mirror_robot(self) -> None:
        """Copy robot joint positions into the MuJoCo model and forward-compute."""
        qpos = self._robot.get_joint_pos()
        n = min(len(qpos), self._nq)
        self._data.qpos[:n] = qpos[:n]
        self._denormalize_slide_joints(n)
        self._enforce_eq_constraints()
        mujoco.mj_forward(self._model, self._data)

    def _denormalize_slide_joints(self, n_set: int) -> None:
        """Scale normalized [0,1] slide joint values to physical range (meters)."""
        for j in range(self._model.njnt):
            adr = self._model.jnt_qposadr[j]
            if adr >= n_set:
                continue
            if self._model.jnt_type[j] == mujoco.mjtJoint.mjJNT_SLIDE:
                lo, hi = self._model.jnt_range[j]
                self._data.qpos[adr] = lo + self._data.qpos[adr] * (hi - lo)

    def _enforce_eq_constraints(self) -> None:
        """Project qpos to satisfy joint equality constraints (e.g. coupled fingers)."""
        for i in range(self._model.neq):
            if self._model.eq_type[i] != mujoco.mjtEq.mjEQ_JOINT:
                continue
            adr1 = self._model.jnt_qposadr[self._model.eq_obj1id[i]]
            adr2 = self._model.jnt_qposadr[self._model.eq_obj2id[i]]
            coef = self._model.eq_data[i, :5]
            q1 = self._data.qpos[adr1]
            self._data.qpos[adr2] = np.polyval(coef[::-1], q1)

    def _set_marker_color(self, rgba: np.ndarray) -> None:
        self._model.geom_rgba[self._mocap_geom_id] = rgba

    # ---- key callback ---------------------------------------------------------

    def _on_key(self, key: int) -> None:
        if key != 32:  # SPACE
            return
        if self._mode is Mode.VIS:
            self._mode = Mode.CONTROL
            self._sync_mocap_to_ee()
            self._set_marker_color(self._CTRL_RGBA)
            print(
                "[control] CONTROL mode — double-click target, ctrl+right-drag to translate, ctrl+left-drag to rotate"
            )
        else:
            self._mode = Mode.VIS
            self._set_marker_color(self._VIS_RGBA)
            if isinstance(self._robot, MotorChainRobot):
                n = self._robot.num_dofs()
                self._robot.update_kp_kd(np.zeros(n), np.zeros(n))
            print("[control] VIS mode — gravity comp, mirroring robot")

    # ---- main loop ------------------------------------------------------------

    def run(self) -> None:
        """Open the viewer and run the vis / control loop."""
        print("[control] Starting in VIS (gravity comp) mode")
        print("[control] Press SPACE to toggle CONTROL mode")

        with mujoco.viewer.launch_passive(
            self._model,
            self._data,
            key_callback=self._on_key,
            show_left_ui=False,
            show_right_ui=False,
        ) as viewer:
            try:
                while viewer.is_running():
                    self._mirror_robot()

                    if self._mode is Mode.VIS:
                        self._sync_mocap_to_ee()
                    else:
                        target = self._mocap_pose_4x4()
                        init_q = self._data.qpos[: self._nq].copy()
                        ok, ik_q = self._kin.ik(
                            target,
                            self._ee_site,
                            init_q=init_q,
                        )
                        if ok:
                            cmd = self._robot.get_joint_pos().copy()
                            cmd[: self._n_arm] = ik_q[: self._n_arm]
                            self._robot.command_joint_pos(cmd)

                    viewer.sync()
                    time.sleep(self._dt)
            except KeyboardInterrupt:
                pass

        print("[control] Viewer closed")


if __name__ == "__main__":
    import argparse
    import sys
    from pathlib import Path

    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

    from i2rt.robots.get_robot import get_yam_robot
    from i2rt.robots.utils import ArmType, GripperType

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
    parser.add_argument("--site", type=str, default="grasp_site", help="EE site name")
    args = parser.parse_args()

    arm = ArmType.from_string_name(args.arm)
    gripper = GripperType.from_string_name(args.gripper)
    robot = get_yam_robot(
        channel=args.channel,
        arm_type=arm,
        gripper_type=gripper,
        sim=args.sim,
    )

    iface = MujocoControlInterface.from_robot(robot, ee_site=args.site, dt=args.dt)
    iface.run()
