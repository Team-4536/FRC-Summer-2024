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


class RobotHALBuffer:
    def __init__(self) -> None:
        pass

    def resetEncoders(self) -> None:
        pass

    def stopMotors(self) -> None:
        pass

    def publish(self, table: ntcore.NetworkTable) -> None:
        pass


class RobotHAL:
    def __init__(self) -> None:
        self.prev: RobotHALBuffer = RobotHALBuffer()

        self.leftDriveMotors: list[rev.CANSparkMax] = [
            rev.CANSparkMax(0, rev.CANSparkMax.MotorType.kBrushless),
            rev.CANSparkMax(1, rev.CANSparkMax.MotorType.kBrushless),
        ]
        self.rightDriveMotors: list[rev.CANSparkMax] = [
            rev.CANSparkMax(2, rev.CANSparkMax.MotorType.kBrushless),
            rev.CANSparkMax(3, rev.CANSparkMax.MotorType.kBrushless),
        ]

        self.leftDriveEncoders: list[rev.SparkRelativeEncoder] = [
            x.getEncoder() for x in self.leftDriveMotors
        ]
        self.rightDriveEncoders: list[rev.SparkRelativeEncoder] = [
            x.getEncoder() for x in self.rightDriveMotors
        ]

        self.intakePivot: rev.CANSparkMax = rev.CANSparkMax(
            4, rev.CANSparkMax.MotorType.kBrushless
        )
        self.intakePivotEncoder: rev.SparkRelativeEncoder = (
            self.intakePivot.getEncoder()
        )

        self.intakeFeed: rev.CANSparkMax = rev.CANSparkMax(
            5, rev.CANSparkMax.MotorType.kBrushless
        )
        self.intakeFeedEncoder: rev.SparkRelativeEncoder = self.intakeFeed.getEncoder()

        self.shooterFeed: rev.CANSparkMax = rev.CANSparkMax(
            6, rev.CANSparkMax.MotorType.kBrushless
        )
        self.shooterFeedEncoder: rev.SparkRelativeEncoder = (
            self.shooterFeed.getEncoder()
        )

        self.shooterAim: rev.CANSparkMax = rev.CANSparkMax(
            7, rev.CANSparkMax.MotorType.kBrushless
        )
        self.shooterAimEncoder: rev.SparkRelativeEncoder = self.shooterAim.getEncoder()

        self.shooterTopMotor: rev.CANSparkMax = rev.CANSparkMax(
            8, rev.CANSparkMax.MotorType.kBrushless
        )
        self.shooterTopMotorEncoder: rev.SparkRelativeEncoder = (
            self.shooterTopMotor.getEncoder()
        )

        self.shooterBottomMotor: rev.CANSparkMax = rev.CANSparkMax(
            9, rev.CANSparkMax.MotorType.kBrushless
        )
        self.shooterBottomMotorEncoder: rev.SparkRelativeEncoder = (
            self.shooterBottomMotor.getEncoder()
        )

    def update(self, buf: RobotHALBuffer, time: TimeData) -> None:
        prev = self.prev
        self.prev = copy.deepcopy(buf)
