#!/usr/bin/env python

# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import logging
import time
import numpy as np

from lerobot.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.dynamixel import (
    DriveMode,
    DynamixelMotorsBus,
    OperatingMode,
)

from ..teleoperator import Teleoperator
from .config_ultrahand import UltrahandConfig

logger = logging.getLogger(__name__)


class Ultrahand(Teleoperator):
    """
    - [Ultrahand](https://github.com/Ultrahand-AI/Ultrahand)
    """

    config_class = UltrahandConfig
    name = "ultrahand"

    def __init__(self, config: UltrahandConfig):
        super().__init__(config)
        self.config = config
        self.bus = DynamixelMotorsBus(
            port=self.config.port,
            motors={
                "shoulder_pan": Motor(1, "xc330-m288", MotorNormMode.RANGE_M100_100),
                "shoulder_lift": Motor(2, "xc330-m288", MotorNormMode.RANGE_M100_100),
                "arm": Motor(3, "xl330-m288", MotorNormMode.RANGE_M100_100),
                "elbow": Motor(4, "xl330-m288", MotorNormMode.RANGE_M100_100),
                "forearm": Motor(5, "xl330-m288", MotorNormMode.RANGE_M100_100),
                "wrist": Motor(6, "xl330-m288", MotorNormMode.RANGE_M100_100),
                "gripper": Motor(7, "xl330-m288", MotorNormMode.RANGE_M100_100),
            },
            calibration=self.calibration,
        )

        self.zero_offset = ["shoulder_lift", "arm", "forearm"]
        self.half_offset = ["shoulder_pan", "elbow", "wrist", "gripper"]
        self.min_pos = {
            "shoulder_pan": 0,
            "shoulder_lift": 0,
            "arm": 0,
            "elbow": 1024,
            "forearm": 0,
            "wrist": 1024,
            "gripper": 0,
        }
        self.max_pos = {
            "shoulder_pan": 4095,
            "shoulder_lift": 2047,
            "arm": 4095,
            "elbow": 4095,
            "forearm": 4095,
            "wrist": 3072,
            "gripper": 4095,
        }

    @property
    def action_features(self) -> dict[str, type]:
        return {f"{motor}.pos": float for motor in self.bus.motors}

    @property
    def feedback_features(self) -> dict[str, type]:
        return {}

    @property
    def is_connected(self) -> bool:
        return self.bus.is_connected

    def connect(self, calibrate: bool = True) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        self.bus.connect()
        if not self.is_calibrated and calibrate:
            logger.info(
                "Mismatch between calibration values in the motor and the calibration file or no calibration file found"
            )
            self.calibrate()

        self.configure()
        logger.info(f"{self} connected.")

    @property
    def is_calibrated(self) -> bool:
        return self.bus.is_calibrated

    def calibrate(self) -> None:
        if self.calibration:
            # Calibration file exists, ask user whether to use it or run new calibration
            user_input = input(
                f"Press ENTER to use provided calibration file associated with the id {self.id}, or type 'c' and press ENTER to run calibration: "
            )
            if user_input.strip().lower() != "c":
                logger.info(
                    f"Writing calibration file associated with the id {self.id} to the motors"
                )
                self.bus.write_calibration(self.calibration)
                return
        logger.info(f"\nRunning calibration of {self}")
        self.bus.disable_torque()
        for motor in self.bus.motors:
            self.bus.write(
                "Operating_Mode", motor, OperatingMode.EXTENDED_POSITION.value
            )

        drive_modes = {motor: 0 for motor in self.bus.motors}

        input(f"Move {self} to the middle of its range of motion and press ENTER....")
        half_homing_offsets = self.bus.set_half_turn_homings(self.half_offset)
        zero_homing_offsets = self.bus.set_zero_turn_homings(self.zero_offset)
        homing_offsets = {**half_homing_offsets, **zero_homing_offsets}

        full_turn_motors = ["shoulder_pan", "gripper"]
        unknown_range_motors = [
            motor for motor in self.bus.motors if motor not in full_turn_motors
        ]
        print(
            f"Move all joints except {full_turn_motors} sequentially through their "
            "entire ranges of motion.\nRecording positions. Press ENTER to stop..."
        )
        # range_mins, range_maxes = self.bus.record_ranges_of_motion(unknown_range_motors)
        # for motor in full_turn_motors:
        #     range_mins[motor] = 0
        #     range_maxes[motor] = 4095
        range_mins = self.min_pos
        range_maxes = self.max_pos

        self.calibration = {}
        for motor, m in self.bus.motors.items():
            self.calibration[motor] = MotorCalibration(
                id=m.id,
                drive_mode=drive_modes[motor],
                homing_offset=homing_offsets[motor],
                range_min=range_mins[motor],
                range_max=range_maxes[motor],
            )

        self.bus.write_calibration(self.calibration)
        self._save_calibration()
        logger.info(f"Calibration saved to {self.calibration_fpath}")

    def configure(self) -> None:
        self.bus.disable_torque()
        self.bus.configure_motors()
        for motor in self.bus.motors:
            if motor != "gripper":
                # Use 'extended position mode' for all motors except gripper, because in joint mode the servos
                # can't rotate more than 360 degrees (from 0 to 4095) And some mistake can happen while
                # assembling the arm, you could end up with a servo with a position 0 or 4095 at a crucial
                # point
                self.bus.write(
                    "Operating_Mode", motor, OperatingMode.EXTENDED_POSITION.value
                )

        # Use 'position control current based' for gripper to be limited by the limit of the current.
        # For the follower gripper, it means it can grasp an object without forcing too much even tho,
        # its goal position is a complete grasp (both gripper fingers are ordered to join and reach a touch).
        # For the leader gripper, it means we can use it as a physical trigger, since we can force with our finger
        # to make it move, and it will move back to its original target position when we release the force.
        self.bus.write(
            "Operating_Mode", "gripper", OperatingMode.CURRENT_POSITION.value
        )
        # Set gripper's goal pos in current position mode so that we can use it as a trigger.
        self.bus.enable_torque("gripper")
        if self.is_calibrated:
            self.bus.write("Goal_Position", "gripper", self.config.gripper_open_pos)

    def setup_motors(self) -> None:
        for motor in reversed(self.bus.motors):
            input(
                f"Connect the controller board to the '{motor}' motor only and press enter."
            )
            self.bus.setup_motor(motor)
            print(f"'{motor}' motor id set to {self.bus.motors[motor].id}")

    def get_action(self) -> dict[str, float]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        start = time.perf_counter()
        action = self.bus.sync_read("Present_Position")
        action = {f"{motor}.pos": val for motor, val in action.items()}
        dt_ms = (time.perf_counter() - start) * 1e3
        logger.debug(f"{self} read action: {dt_ms:.1f}ms")
        return action

    def send_feedback(self, feedback: dict[str, float]) -> None:
        # TODO(rcadene, aliberts): Implement force feedback

        raise NotImplementedError

    def disconnect(self) -> None:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        self.bus.disconnect()
        logger.info(f"{self} disconnected.")

    def get_motors_position(self):
        positions = np.zeros(len(self.bus.motors))

        # 使用bus的sync_read方法获取位置
        positions_raw = self.bus.sync_read("Present_Position", normalize=False)
        for motor, m in self.bus.motors.items():
            if motor in self.zero_offset:
                positions[m.id - 1] = (positions_raw[motor] - 0) / 2048 * np.pi
            else:
                positions[m.id - 1] = (positions_raw[motor] - 2048) / 2048 * np.pi

        return positions
