import copy
import math

import navx
import ntcore
import profiler
import rev
import wpilib
from phoenix5.led import CANdle
from phoenix6.hardware import CANcoder
from timing import TimeData


class RobotHALBuffer():
    def __init__(self) -> None:
        pass

    def resetEncoders(self) -> None:
        pass

    def stopMotors(self) -> None:
        pass

    def publish(self, table: ntcore.NetworkTable) -> None:
        pass

class RobotHAL():
    def __init__(self) -> None:
        self.prev = RobotHALBuffer()

    def resetGyroToAngle(self, ang: float) -> None:
        self.gyro.reset()
        self.gyro.setAngleAdjustment(-math.degrees(ang))

    def update(self, buf: RobotHALBuffer, time: TimeData) -> None:
        prev = self.prev
        self.prev = copy.deepcopy(buf)