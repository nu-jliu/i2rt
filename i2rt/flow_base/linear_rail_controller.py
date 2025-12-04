import logging
import sys
import threading
import time
from typing import Any, Dict

import numpy as np
from RPi import GPIO

from i2rt.motor_drivers.dm_driver import DMChainCanInterface
from i2rt.motor_drivers.utils import MotorInfo

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
HOMING_TIMEOUT = 30.0  # Timeout for homing procedure in seconds


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

        # Preserve velocities of other motors (e.g., base motors) by reading current commands
        with self.motor_chain.command_lock:
            current_commands = self.motor_chain.commands
            if current_commands and len(current_commands) == num_motors:
                # Preserve velocities of other motors
                for idx in range(num_motors):
                    if idx != self.target_motor_idx:
                        velocities[idx] = current_commands[idx].vel

        torques = np.zeros(num_motors)
        self.motor_chain.set_commands(torques=torques, vel=velocities, pos=None, kp=None, kd=None, get_state=False)

    def get_state(self) -> MotorInfo:
        """Get motor state"""
        return self.motor_chain.read_states()[self.target_motor_idx]

    @classmethod
    def from_multi_motor_chain(cls, motor_chain: DMChainCanInterface, motor_id: int) -> "SingleMotorControlInterface":
        """Create SingleMotorControlInterface from an existing multi-motor chain, here motor_id is the can ID not the index"""
        for target_motor_idx, (m_id, _) in enumerate(motor_chain.motor_list):
            if m_id == motor_id:
                return cls(motor_chain, target_motor_idx=target_motor_idx)

        raise ValueError(f"Motor with ID {motor_id} not found in motor chain")


class LinearRailController:
    def __init__(
        self,
        single_motor_control_interface: SingleMotorControlInterface,
        rail_speed: float = 14.0,
        auto_home: bool = True,  # Automatically start homing after initialization
        homing_timeout: float = HOMING_TIMEOUT,  # Timeout for homing procedure in seconds
    ):
        """Initialize linear rail controller
        
        Args:
            single_motor_control_interface: Motor control interface (required)
            rail_speed: Maximum rail speed in rad/s
            auto_home: Whether to automatically home after initialization
            homing_timeout: Timeout for homing procedure in seconds
        """
        self.single_motor_control_interface = single_motor_control_interface
        self.rail_speed = rail_speed
        self.auto_home = auto_home
        self.homing_timeout = homing_timeout
        
        self.initialized = False
        self.brake_on = True

        self._lock = threading.Lock()
        self.upper_limit_triggered = False
        self.lower_limit_triggered = False
        self._gpio_mode_set = False
        self._homing_event = threading.Event()
        self._homing_start_time = None
        self.homing_speed_ratio = HOMING_SPEED_RATIO

        self._initialize_gpio()

        if self.auto_home:
            self._initialize_linear_rail()
        else:
            # If auto_home is False, just release brake and mark as initialized
            self.set_brake(engaged=False)
            with self._lock:
                self.initialized = True
            logger.info("Linear rail initialized without auto-homing")

    def _ensure_gpio_mode(self) -> None:
        """Ensure GPIO mode is set"""
        if not self._gpio_mode_set:
            try:
                GPIO.setmode(GPIO.BCM)
                self._gpio_mode_set = True
            except RuntimeError:
                # GPIO mode already set
                self._gpio_mode_set = True

    def _initialize_gpio(self) -> None:
        """Initialize GPIO pins for limit switches and brake control with event callbacks"""
        try:
            self._ensure_gpio_mode()
            GPIO.setup(BRAKE_CONTROL_GPIO, GPIO.OUT)
            GPIO.setup(UPPER_LIMIT_GPIO, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.setup(LOWER_LIMIT_GPIO, GPIO.IN, pull_up_down=GPIO.PUD_UP)

            # Read initial limit switch states
            initial_upper_limit = GPIO.input(UPPER_LIMIT_GPIO) == GPIO.HIGH
            initial_lower_limit = GPIO.input(LOWER_LIMIT_GPIO) == GPIO.HIGH

            with self._lock:
                self.upper_limit_triggered = initial_upper_limit
                self.lower_limit_triggered = initial_lower_limit

                if initial_lower_limit:
                    logger.info("Linear rail is at lower limit")
                elif initial_upper_limit:
                    logger.info("Linear rail is at upper limit")
                else:
                    logger.info("Linear rail initialized - not at any limit")

            GPIO.add_event_detect(
                UPPER_LIMIT_GPIO,
                GPIO.BOTH,  # Detect both rising and falling edges
                callback=self._upper_limit_callback,
                bouncetime=50,  # Debounce time in milliseconds
            )
            GPIO.add_event_detect(
                LOWER_LIMIT_GPIO,
                GPIO.BOTH,  # Detect both rising and falling edges
                callback=self._lower_limit_callback,
                bouncetime=50,  # Debounce time in milliseconds
            )

            logger.info("GPIO initialized successfully with event callbacks for limit switches")
        except Exception as e:
            logger.error(f"GPIO initialization failed: {e}")

    def _limit_switch_callback(self, channel: int, is_upper: bool) -> None:
        """Generic callback function for limit switch GPIO events - only updates limit state

        Args:
            channel: GPIO channel number
            is_upper: True for upper limit, False for lower limit
        """
        try:
            gpio_pin = UPPER_LIMIT_GPIO if is_upper else LOWER_LIMIT_GPIO
            limit_state = GPIO.input(gpio_pin) == GPIO.HIGH
            limit_name = "upper" if is_upper else "lower"

            with self._lock:
                if is_upper:
                    self.upper_limit_triggered = limit_state
                else:
                    self.lower_limit_triggered = limit_state

                if limit_state:
                    logger.warning(f"{limit_name.capitalize()} limit switch triggered!")
                else:
                    logger.info(f"{limit_name.capitalize()} limit switch released")
        except Exception as e:
            limit_name = "upper" if is_upper else "lower"
            logger.error(f"Error in {limit_name} limit callback: {e}")

    def _upper_limit_callback(self, channel: int) -> None:
        """Callback wrapper for upper limit switch"""
        self._limit_switch_callback(channel, is_upper=True)

    def _lower_limit_callback(self, channel: int) -> None:
        """Callback wrapper for lower limit switch"""
        self._limit_switch_callback(channel, is_upper=False)

    def set_brake(self, engaged: bool) -> None:
        """Set brake state (engaged=True: brake on, engaged=False: brake off)"""
        try:
            self._ensure_gpio_mode()
            GPIO.output(BRAKE_CONTROL_GPIO, GPIO.LOW if engaged else GPIO.HIGH)
            with self._lock:
                self.brake_on = engaged
            action = "engaged" if engaged else "released"
            logger.info(f"Brake {action}")
        except Exception as e:
            action = "engage" if engaged else "release"
            logger.error(f"Failed to {action} brake: {e}")

    def _initialize_linear_rail(self) -> None:
        """Initialize linear rail: release brake and home to lower limit if needed"""
        try:
            # Release brake first
            self.set_brake(engaged=False)

            # Check if already at lower limit
            with self._lock:
                if self.lower_limit_triggered:
                    logger.info("Linear rail is already at lower limit - ready for operation")
                    self.initialized = True
                    return

            # If not at lower limit, start homing
            # Set homing state
            with self._lock:
                self._homing_event.set()
                self._homing_start_time = time.time()

            # Move backward at homing speed (half of normal speed, like linear_motor.py)
            motor_velocity = -self.rail_speed * HOMING_SPEED_RATIO  # Negative = move backward
            self.single_motor_control_interface.set_velocity(motor_velocity)
            logger.info(f"Homing started with velocity {motor_velocity:.3f} rad/s (moving backward)")

            # Wait for homing to complete or timeout
            start_time = time.time()
            while time.time() - start_time < self.homing_timeout:
                with self._lock:
                    if self.lower_limit_triggered:
                        # Reached lower limit, stop motor
                        self.single_motor_control_interface.set_velocity(0.0)
                        elapsed_time = time.time() - start_time
                        logger.info(f"Homing success! Zero position found in {elapsed_time:.1f}s")
                        self._homing_event.clear()
                        self._homing_start_time = None
                        self.initialized = True
                        return

                time.sleep(0.1)  # Check every 100ms

            # Timeout
            self.single_motor_control_interface.set_velocity(0.0)
            with self._lock:
                self._homing_event.clear()
                self._homing_start_time = None
            raise RuntimeError(f"Homing procedure timed out after {self.homing_timeout} seconds")

        except Exception as e:
            logger.error(f"Linear rail initialization failed: {e}")
            self.initialized = False
            self.single_motor_control_interface.set_velocity(0.0)
            with self._lock:
                self._homing_event.clear()
            raise  # Re-raise the exception (timeout RuntimeError or other errors)

    def _stop_homing(self) -> None:
        """Stop homing procedure and reset state (assumes lock is held)"""
        self.single_motor_control_interface.set_velocity(0.0)
        self._homing_event.clear()
        self._homing_start_time = None

    def get_state(self) -> Dict[str, Any]:
        """Get the current state of the linear rail"""
        motor_state = self.single_motor_control_interface.get_state()

        with self._lock:
            brake_on = self.brake_on
            initialized = self.initialized
            upper_limit = self.upper_limit_triggered
            lower_limit = self.lower_limit_triggered

        return {
            "position": motor_state.pos,  # Use motor position directly
            "velocity": motor_state.vel,  # Use motor velocity directly
            "brake_on": brake_on,
            "initialized": initialized,
            "upper_limit_triggered": upper_limit,
            "lower_limit_triggered": lower_limit,
            "motor_state": motor_state,
        }

    def set_velocity(self, vel: float) -> None:
        """Set the velocity of the linear rail, unit in rad/s (motor velocity)"""
        assert self.initialized, "Linear rail must be initialized before setting velocity"
        assert not self.brake_on, "Brake must be released before setting velocity"

        with self._lock:
            if vel > 0.0 and self.upper_limit_triggered:
                logger.warning("Upper limit triggered, cannot move forward")
                self.single_motor_control_interface.set_velocity(0.0)
                return
            if vel < 0.0 and self.lower_limit_triggered:
                logger.warning("Lower limit triggered, cannot move backward")
                self.single_motor_control_interface.set_velocity(0.0)
                if self._homing_event.is_set():
                    elapsed_time = time.time() - self._homing_start_time if self._homing_start_time else 0.0
                    logger.info(f"Homing success! Zero position found in {elapsed_time:.1f}s")
                    self._stop_homing()
                return

            try:
                self.single_motor_control_interface.set_velocity(vel)
                if vel != 0.0:
                    logger.info(f"Linear rail velocity set to {vel:.3f} rad/s")
            except Exception as e:
                logger.error(f"Failed to set linear rail velocity: {e}")
                raise

    def cleanup(self) -> None:
        """Clean up resources"""
        try:
            self.single_motor_control_interface.set_velocity(0.0)
            if self.initialized:
                self.set_brake(engaged=True)
                GPIO.remove_event_detect(UPPER_LIMIT_GPIO)
                GPIO.remove_event_detect(LOWER_LIMIT_GPIO)
                GPIO.cleanup()
            logger.info("Linear rail controller cleaned up successfully")
        except Exception as e:
            logger.error(f"Cleanup error: {e}")
