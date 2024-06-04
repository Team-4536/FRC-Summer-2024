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
        self.leftDriveVolts: list[float] = [0, 0]
        self.rightDriveVolts: list[float] = [0, 0]
        self.leftDrivePositions: list[float] = [0, 0]
        self.rightDrivePositions: list[float] = [0, 0]
        self.leftDriveSpeedMeasured: list[float] = [0, 0]
        self.rightDriveSpeedMeasured: list[float] = [0, 0]

    def resetEncoders(self) -> None:
        for i in range(2):
            self.leftDrivePositions[i] = 0
            self.rightDrivePositions[i] = 0

    def stopMotors(self) -> None:
        self.leftDriveVolts = [0, 0]
        self.rightDriveVolts = [0, 0]

    def publish(self, table: ntcore.NetworkTable) -> None:
        pass

class RobotHAL():
    def __init__(self) -> None:
        self.prev: RobotHALBuffer = RobotHALBuffer()

        self.leftDriveMotors: list[rev.CANSparkMax] = [rev.CANSparkMax(0, rev.CANSparkMax.MotorType.kBrushless),
                                rev.CANSparkMax(1, rev.CANSparkMax.MotorType.kBrushless)]
        self.rightDriveMotors: list[rev.CANSparkMax] = [rev.CANSparkMax(2, rev.CANSparkMax.MotorType.kBrushless),
                                 rev.CANSparkMax(3, rev.CANSparkMax.MotorType.kBrushless)]

        self.leftDriveEncoders: list[rev.SparkRelativeEncoder] = [x.getEncoder() for x in self.leftDriveMotors]
        self.rightDriveEncoders: list[rev.SparkRelativeEncoder] = [x.getEncoder() for x in self.rightDriveMotors]

        self.driveGearing: float = 1
        self.wheelRadius: float = 1

    def update(self, buf: RobotHALBuffer, time: TimeData) -> None:
        prev = self.prev
        
        self.prev = copy.deepcopy(buf)

        for m, s in zip(self.leftDriveMotors, buf.leftDriveVolts):
            m.set(s)

        for m, s in zip(self.rightDriveMotors, buf.rightDriveVolts):
            m.set(s)

        for i in range(2):
            e = self.leftDriveEncoders[i]
            buf.leftDrivePositions[i] = math.radians((e.getPosition() / self.driveGearing) * 360) * self.wheelRadius
            buf.leftDriveSpeedMeasured[i] = math.radians((e.getVelocity() / self.driveGearing) * 360) * self.wheelRadius / 60

        for i in range(2):
            e = self.rightDriveEncoders[i]
            buf.rightDrivePositions[i] = math.radians((e.getPosition() / self.driveGearing) * 360) * self.wheelRadius
            buf.rightDriveSpeedMeasured[i] = math.radians((e.getVelocity() / self.driveGearing) * 360) * self.wheelRadius / 60
