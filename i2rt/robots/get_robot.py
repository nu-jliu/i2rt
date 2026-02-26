import logging
import time
from functools import partial
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
from i2rt.robots.utils import GripperType, ArmType, combine_arm_and_gripper_xml

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
    


if __name__ == "__main__":
    import argparse

    arm_choices = [a.value for a in ArmType]
    gripper_choices = [g.value for g in GripperType]

    parser = argparse.ArgumentParser(description="Initialize a YAM robot")
    parser.add_argument("--arm", type=str, default="yam", choices=arm_choices, help="Arm type")
    parser.add_argument("--gripper", type=str, default="crank_4310", choices=gripper_choices, help="Gripper type")
    parser.add_argument("--sim", action="store_true", help="Use sim mode instead of real hardware")
    parser.add_argument("--channel", type=str, default="can0", help="CAN channel (default: can0)")
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO)
    
    arm = args.arm
    gripper = args.gripper
    sim = args.sim
    channel = args.channel

    arm_type = ArmType.from_string_name(arm)
    gripper_type = GripperType.from_string_name(gripper)

    robot = get_yam_robot(
        channel=channel,
        arm_type=arm_type,
        gripper_type=gripper_type,
        sim=sim,
    )
    print(f"Robot initialized: arm={arm}, gripper={gripper}, sim={sim}, channel={channel}")
    
    while True:
        print(robot.get_observations())
        time.sleep(1)
