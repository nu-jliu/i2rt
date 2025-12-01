import logging
import sys
import threading
import time
from dataclasses import dataclass
from typing import Any, Dict

import numpy as np

from i2rt.motor_drivers.dm_driver import DMChainCanInterface

# configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
    handlers=[logging.StreamHandler(sys.stdout)],
)
logger = logging.getLogger("LinearRailController")

# GPIO Pins Definition
BRAKE_CONTROL_GPIO = 12  # Brake control GPIO
UPPER_LIMIT_GPIO = 5  # Upper limit GPIO
LOWER_LIMIT_GPIO = 6  # Lower limit GPIO
HOMING_SPEED_RATIO = 0.5


class SingleMotorControlInterface:
    """Single motor control interface for motor chains"""

    def __init__(self, motor_chain: DMChainCanInterface, target_motor_idx: int = -1):
        """Initialize single motor control interface"""
        if len(motor_chain) == 0:
            raise ValueError(f"Motor chain must contain at least 1 motor, got {len(motor_chain)}")

        self.motor_chain = motor_chain
        self.target_motor_idx = target_motor_idx

        if target_motor_idx < 0 or target_motor_idx >= len(motor_chain):
            raise ValueError(f"Motor index {target_motor_idx} out of range [0, {len(motor_chain)})")

        self.motor_id = motor_chain.motor_list[target_motor_idx][0]

    def set_velocity(self, vel: float) -> None:
        """Set motor velocity"""
        num_motors = len(self.motor_chain)

        velocities = np.zeros(num_motors)
        velocities[self.target_motor_idx] = vel

        torques = np.zeros(num_motors)
        self.motor_chain.set_commands(torques=torques, vel=velocities, pos=None, kp=None, kd=None, get_state=False)

    def get_state(self) -> Dict[str, Any]:
        """Get motor state"""
        return self.motor_chain.read_states()[self.target_motor_idx]

    @classmethod
    def from_multi_motor_chain(cls, motor_chain: DMChainCanInterface, motor_id: int) -> "SingleMotorControlInterface":
        """Create SingleMotorControlInterface from an existing multi-motor chain, here motor_id is the can ID not the index"""
        for target_motor_idx, (m_id, _) in enumerate(motor_chain.motor_list):
            if m_id == motor_id:
                return cls(motor_chain, target_motor_idx=target_motor_idx)

        raise ValueError(f"Motor with ID {motor_id} not found in motor chain")


@dataclass
class LinearRailController:
    rail_length: float
    motor_total_range: float

    def _post_init(self, single_motor_control_interface: SingleMotorControlInterface):
        """Initialize linear rail controller"""
        self.single_motor_control_interface = single_motor_control_interface
        self.initialized = False
        self.brake_on = True

        self._lock = threading.Lock()
        self.current_position = None  # in meter
        self.current_velocity = None
        self.sentinel_thread = threading.Thread(target=self._sentinel_process)
        self.sentinel_thread.start()

    def _rail_motor_radius_to_meter(self, radius: float) -> float:
        return radius / self.motor_total_range * self.rail_length

    def _sentinel_process(self) -> None:
        # TODO: check if the linear rail is out of bounds, if so, stop the motor and set the brake
        while True:
            rail_position = self.single_motor_control_interface.get_state()["position"]
            with self._lock:
                self.current_position = self._rail_motor_radius_to_meter(rail_position)
                if rail_position < 0.0 or rail_position > self.rail_length:
                    self.current_velocity = 0.0
                    self.single_motor_control_interface.set_velocity(0.0)
            time.sleep(0.03)
        raise NotImplementedError("Linear rail sentinel process not implemented")

    def _initialize_linear_rail(self) -> None:
        # TODO: turn on motor, disable the brake, slowly move the linear rail to lower limit, then home the linear rail
        self.current_position = 0.0
        self.current_velocity = 0.0
        raise NotImplementedError("Linear rail initialization not implemented")

    def get_state(self) -> Dict[str, Any]:
        # TODO: get the current position and speed of the linear rail, unit in meter, also get all IO and brake states
        return

    def set_velocity(self, vel: float) -> None:
        assert self.initialized, "Linear rail must be initialized before setting velocity"
        assert not self.brake_on, "Brake must be released before setting velocity"
        # TODO: if position out bound, do nothing and print warning
        # TODO: set the velocity of the linear rail, unit in meter/s
        raise NotImplementedError("Linear rail velocity setting not implemented")

    def set_brake(self, brake: bool) -> None:
        # TODO: if brake: set brake, otherwise release the brake,
        raise NotImplementedError("Linear rail brake setting not implemented")
