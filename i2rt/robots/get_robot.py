import logging
import time, os
from functools import partial
import xml.etree.ElementTree as ET
from copy import deepcopy
import tempfile
from typing import Optional

import numpy as np

from i2rt.motor_drivers.dm_driver import (
    CanInterface,
    DMChainCanInterface,
    EncoderChain,
    PassiveEncoderReader,
    ReceiveMode,
)
from i2rt.robots.motor_chain_robot import MotorChainRobot
from i2rt.robots.utils import GripperType, ArmType

def combine_arm_and_gripper_xml(arm_path, gripper_path, ee_mass=None, ee_inertia=None) -> str:
    """Combine arm and gripper XML files into a single XML string.

    Replaces the <body name="link_6"> subtree in the arm XML with the one from the
    gripper XML (if present). If ee_mass or ee_inertia are provided, update the
    inertial properties of the resulting link_6. Returns path to combined XML in /tmp/.
    """
    arm_tree = ET.parse(arm_path)
    arm_root = arm_tree.getroot()

    # Resolve arm mesh paths to absolute
    arm_dir = os.path.dirname(os.path.abspath(arm_path))
    arm_compiler = arm_root.find("compiler")
    arm_meshdir = arm_compiler.get("meshdir", "") if arm_compiler is not None else ""
    arm_asset = arm_root.find("asset")
    if arm_asset is not None:
        for child in arm_asset:
            if child.get("file") and not os.path.isabs(child.get("file")):
                abs_file = os.path.join(arm_dir, arm_meshdir, child.get("file"))
                child.set("file", os.path.abspath(abs_file))

    # Remove meshdir from compiler (all paths now absolute)
    if arm_compiler is not None and arm_compiler.get("meshdir"):
        del arm_compiler.attrib["meshdir"]

    # attempt to load gripper and replace link_6 if available
    if gripper_path:
        try:
            grip_tree = ET.parse(gripper_path)
            grip_root = grip_tree.getroot()
            grip_body = grip_root.find(".//body[@name='link_6']")
            if grip_body is None:
                grip_body = grip_root.find(".//body[@name='link6']")
        except Exception:
            grip_root = None
            grip_body = None

        # merge assets (avoid duplicates), resolving gripper mesh paths to absolute
        if grip_root is not None:
            grip_dir = os.path.dirname(os.path.abspath(gripper_path))
            grip_compiler = grip_root.find("compiler")
            grip_meshdir = grip_compiler.get("meshdir", "") if grip_compiler is not None else ""

            grip_asset = grip_root.find("asset")
            if grip_asset is not None:
                if arm_asset is None:
                    arm_asset = ET.Element("asset")
                    worldbody = arm_root.find("worldbody")
                    if worldbody is not None:
                        arm_root.insert(list(arm_root).index(worldbody), arm_asset)
                    else:
                        arm_root.append(arm_asset)
                existing = {(c.tag, c.get("name")) for c in arm_asset}
                for child in grip_asset:
                    key = (child.tag, child.get("name"))
                    if key not in existing:
                        elem = deepcopy(child)
                        if elem.get("file") and not os.path.isabs(elem.get("file")):
                            abs_file = os.path.join(grip_dir, grip_meshdir, elem.get("file"))
                            elem.set("file", os.path.abspath(abs_file))
                        arm_asset.append(elem)
                        existing.add(key)

        # replace arm's link_6 with gripper's if found
        if grip_body is not None:
            replaced = False
            for parent in arm_root.iter():
                children = list(parent)
                for idx, child in enumerate(children):
                    if child.tag == "body" and child.get("name") in ("link_6", "link6"):
                        parent.remove(child)
                        parent.insert(idx, deepcopy(grip_body))
                        replaced = True
                        break
                if replaced:
                    break

    # find resulting link_6 and apply end-effector overrides (mass/inertia)
    if ee_mass is not None or ee_inertia is not None:
        res_body = arm_root.find(".//body[@name='link_6']")
        if res_body is None:
            res_body = arm_root.find(".//body[@name='link6']")
        if res_body is not None:
            inertial = res_body.find("inertial")
            if inertial is None:
                inertial = ET.SubElement(res_body, "inertial")

            if ee_mass is not None:
                inertial.set("mass", str(float(ee_mass)))

            if ee_inertia is not None:
                arr = np.asarray(ee_inertia).ravel()
                ipos = " ".join(str(float(x)) for x in arr[:3])
                inertial.set("ipos", ipos)
                quat = " ".join(str(float(x)) for x in arr[3:7])
                inertial.set("quat", quat)
                diagin = " ".join(str(float(x)) for x in arr[-3:])
                inertial.set("diaginertia", diagin)

    # write combined xml to /tmp/ and return filepath
    out_path = tempfile.NamedTemporaryFile(suffix=".xml", prefix="i2rt_combined_", delete=False, dir="/tmp").name
    arm_tree.write(out_path, encoding="utf-8", xml_declaration=True)
    return out_path

def get_encoder_chain(can_interface: CanInterface) -> EncoderChain:
    passive_encoder_reader = PassiveEncoderReader(can_interface)
    return EncoderChain([0x50E], passive_encoder_reader)


def get_yam_robot(
    channel: str = "can0",
    arm_type: ArmType = ArmType.YAM,
    gripper_type: GripperType = GripperType.CRANK_4310,
    zero_gravity_mode: bool = True,
    ee_mass: Optional[float] = None, # scalar
    ee_inertia: Optional[np.ndarray] = None, # 10-dim array: ipos(3) + quat(4) + diaginertia(3) or full inertia(6)
    sim: bool = False,
) -> "MotorChainRobot":
    with_gripper = True
    with_teaching_handle = False
    if gripper_type == GripperType.YAM_TEACHING_HANDLE:
        with_gripper = False
        with_teaching_handle = True
    if gripper_type == GripperType.NO_GRIPPER:
        with_gripper = False
        with_teaching_handle = False

    gripper_path = gripper_type.get_xml_path()
    arm_path = arm_type.get_xml_path()
    model_path = combine_arm_and_gripper_xml(arm_path, gripper_path, ee_mass, ee_inertia)

    motor_list = [
        [0x01, "DM4340"],
        [0x02, "DM4340"],
        [0x03, "DM4340"],
        [0x04, "DM4310"],
        [0x05, "DM4310"],
        [0x06, "DM4310"],
    ]
    motor_offsets = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    joint_limits = np.array([[-2.617, 3.13], [0, 3.65], [0.0, 3.13], [-1.57, 1.57], [-1.57, 1.57], [-2.09, 2.09]])
    joint_limits[:, 0] += -0.15  # add some buffer to the joint limits
    joint_limits[:, 1] += 0.15

    motor_directions = [1, 1, 1, 1, 1, 1]
    kp = np.array([80, 80, 80, 40, 10, 10])
    kd = np.array([5, 5, 5, 1.5, 1.5, 1.5])
    if with_gripper:
        motor_type = gripper_type.get_motor_type()
        gripper_kp, gripper_kd = gripper_type.get_motor_kp_kd()
        assert motor_type != ""
        logging.info(
            f"adding gripper motor with type: {motor_type}, gripper_kp: {gripper_kp}, gripper_kd: {gripper_kd}"
        )
        motor_list.append([0x07, motor_type])
        motor_offsets.append(0.0)
        motor_directions.append(1)
        kp = np.concatenate([kp, np.array([gripper_kp])])
        kd = np.concatenate([kd, np.array([gripper_kd])])

    if sim:
        from i2rt.robots.sim_robot import SimRobot

        n_dofs = len(motor_list)
        gripper_index = 6 if with_gripper else None
        gripper_lims = gripper_type.get_gripper_limits() if with_gripper else None
        return SimRobot(
            xml_path=model_path,
            n_dofs=n_dofs,
            joint_limits=joint_limits,
            gripper_index=gripper_index,
            gripper_limits=gripper_lims,
        )

    motor_chain = DMChainCanInterface(
        motor_list,
        motor_offsets,
        motor_directions,
        channel,
        motor_chain_name="yam_real",
        receive_mode=ReceiveMode.p16,
        start_thread=False,
    )
    motor_states = motor_chain.read_states()
    print(f"motor_states: {motor_states}")
    motor_chain.close()

    current_pos = [m.pos for m in motor_states]
    logging.info(f"current_pos: {current_pos}")

    for idx, motor_state in enumerate(motor_states):
        motor_position = motor_state.pos
        # if not within -pi to pi, set to the nearest equivalent position
        if motor_position < -np.pi:
            logging.info(f"motor {idx} is at {motor_position}, adding {2 * np.pi}")
            extra_offset = -2 * np.pi
        elif motor_position > np.pi:
            logging.info(f"motor {idx} is at {motor_position}, subtracting {2 * np.pi}")
            extra_offset = +2 * np.pi
        else:
            extra_offset = 0.0
        motor_offsets[idx] += extra_offset

    time.sleep(0.5)
    logging.info(f"adjusted motor_offsets: {motor_offsets}")

    motor_chain = DMChainCanInterface(
        motor_list,
        motor_offsets,
        motor_directions,
        channel,
        motor_chain_name="yam_real",
        receive_mode=ReceiveMode.p16,
        get_same_bus_device_driver=get_encoder_chain if with_teaching_handle else None,
        use_buffered_reader=False,
    )
    motor_states = motor_chain.read_states()
    logging.info(f"YAM initial motor_states: {motor_states}")
    get_robot = partial(
        MotorChainRobot,
        motor_chain=motor_chain,
        xml_path=model_path,
        use_gravity_comp=True,
        gravity_comp_factor=1.3,
        joint_limits=joint_limits,
        kp=kp,
        kd=kd,
        zero_gravity_mode=zero_gravity_mode,
    )

    if with_gripper:
        return get_robot(
            gripper_index=6,
            gripper_limits=gripper_type.get_gripper_limits(),
            enable_gripper_calibration=gripper_type.get_gripper_needs_calibration(),
            gripper_type=gripper_type,
            limit_gripper_force=50.0,
        )
    else:
        return get_robot()
