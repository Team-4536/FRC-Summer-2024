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
        self.leftDrivePercent: float = 0
        self.rightDrivePercent: float = 0
        self.leftDrivePositions: float = 0
        self.rightDrivePositions: float = 0
        self.leftDriveSpeedMeasured: float = 0
        self.rightDriveSpeedMeasured: float = 0

        self.intakePivotVolts: float = 0
        self.intakePivotAngle: float = 0

        self.intakeFeedVolts: float = 0
        self.intakeFeedAngle: float = 0

        self.shooterFeedVolts: float = 0
        self.shooterFeedAngle: float = 0

        self.shooterAimVolts: float = 0
        self.shooterAimAngle: float = 0

        self.shooterTopMotorVolts: float = 0
        self.shooterTopMotorAngle: float = 0

        self.shooterBottomMotorVolts: float = 0
        self.shooterBottomMotorAngle: float = 0

        self.yaw: float = 0

    def stopMotors(self) -> None:
        self.leftDrivePercent = 0
        self.rightDrivePercent = 0

        self.intakePivotVolts = 0
        self.intakeFeedVolts = 0
        self.shooterFeedVolts = 0
        self.shooterAimVolts = 0
        self.shooterTopMotorVolts = 0
        self.shooterBottomMotorVolts = 0

    def publish(self, table: ntcore.NetworkTable) -> None:
        pass


class RobotHAL:
    # constant values that are determined at compile time, not run time, put that stuff in __init__
    DRIVE_GEARING: float = 1
    WHEEL_RADIUS: float = 1

    INTAKE_PIVOT_GEARING: int = 1
    INTAKE_FEED_GEARING: int = 1
    SHOOTER_FEED_GEARING: int = 1
    SHOOTER_AIM_GEARING: int = 1
    SHOOTER_TOP_MOTOR_GEARING: int = 1
    SHOOTER_BOTTOM_MOTOR_GEARING: int = 1

    def __init__(self) -> None:
        self.prev: RobotHALBuffer = RobotHALBuffer()

        # create motors
         # the motor controllers are on follower mode, so 2 will follow 1, and 4 will follow 3
        self.leftDriveMotor: rev.CANSparkMax = rev.CANSparkMax(1, rev.CANSparkMax.MotorType.kBrushless)
        self.rightDriveMotor: rev.CANSparkMax = rev.CANSparkMax(3, rev.CANSparkMax.MotorType.kBrushless)

        # create drive encoders and reset positions
        self.leftDriveEncoder: rev.SparkRelativeEncoder = self.leftDriveMotor.getEncoder()
        self.rightDriveEncoder: rev.SparkRelativeEncoder = self.rightDriveMotor.getEncoder()
        self.leftDriveEncoder.setPosition(0)
        self.rightDriveEncoder.setPosition(0)

        # mechanism stuff
        self.intakePivot: rev.CANSparkMax = rev.CANSparkMax(
            4, rev.CANSparkMax.MotorType.kBrushless
        )
        self.intakePivotEncoder: rev.SparkRelativeEncoder = (
            self.intakePivot.getEncoder()
        )
        self.intakePivotEncoder.setPosition(0)

        self.intakeFeed: rev.CANSparkMax = rev.CANSparkMax(
            5, rev.CANSparkMax.MotorType.kBrushless
        )
        self.intakeFeedEncoder: rev.SparkRelativeEncoder = self.intakeFeed.getEncoder()
        self.intakeFeedEncoder.setPosition(0)

        self.shooterFeed: rev.CANSparkMax = rev.CANSparkMax(
            6, rev.CANSparkMax.MotorType.kBrushless
        )
        self.shooterFeedEncoder: rev.SparkRelativeEncoder = (
            self.shooterFeed.getEncoder()
        )
        self.shooterFeedEncoder.setPosition(0)

        self.shooterAim: rev.CANSparkMax = rev.CANSparkMax(
            7, rev.CANSparkMax.MotorType.kBrushless
        )
        self.shooterAimEncoder: rev.SparkRelativeEncoder = self.shooterAim.getEncoder()
        self.shooterAimEncoder.setPosition(0)

        self.shooterTopMotor: rev.CANSparkMax = rev.CANSparkMax(
            8, rev.CANSparkMax.MotorType.kBrushless
        )
        self.shooterTopMotorEncoder: rev.SparkRelativeEncoder = (
            self.shooterTopMotor.getEncoder()
        )
        self.shooterTopMotorEncoder.setPosition(0)

        self.shooterBottomMotor: rev.CANSparkMax = rev.CANSparkMax(
            9, rev.CANSparkMax.MotorType.kBrushless
        )
        self.shooterBottomMotorEncoder: rev.SparkRelativeEncoder = (
            self.shooterBottomMotor.getEncoder()
        )
        self.shooterBottomMotorEncoder.setPosition(0)

        self.gyro: navx.AHRS = navx.AHRS(wpilib.SerialPort.Port.kUSB1)

    # angle expected in CCW radians
    def resetGyroToAngle(self, angleRads: float) -> None:
        self.gyro.reset()
        self.gyro.setAngleAdjustment(-math.degrees(angleRads))

    def update(self, buf: RobotHALBuffer, time: TimeData) -> None:
        prev = self.prev
        self.prev = copy.deepcopy(buf)

        self.leftDriveMotor.set(buf.leftDrivePercent)
        self.rightDriveMotor.set(buf.rightDrivePercent)

        # left drive encoder
        buf.leftDrivePositions = (math.radians((self.leftDriveEncoder.getPosition() / self.DRIVE_GEARING) * 360)* self.WHEEL_RADIUS)
        buf.leftDriveSpeedMeasured = (math.radians((self.leftDriveEncoder.getVelocity() / self.DRIVE_GEARING) * 360)* self.WHEEL_RADIUS/ 60)

        # right drive encoder
        buf.rightDrivePositions = (math.radians((self.rightDriveEncoder.getPosition() / self.DRIVE_GEARING) * 360)* self.WHEEL_RADIUS)
        buf.rightDriveSpeedMeasured = (math.radians((self.rightDriveEncoder.getVelocity() / self.DRIVE_GEARING) * 360)* self.WHEEL_RADIUS/ 60)

        # mechanism stuff
        self.intakePivot.setVoltage(buf.intakePivotVolts)
        buf.intakePivotAngle = (
            self.intakePivotEncoder.getPosition()
            * math.pi
            * 2
            / self.INTAKE_PIVOT_GEARING
        )

        self.intakeFeed.setVoltage(buf.intakeFeedVolts)
        buf.intakeFeedAngle = (
            self.intakeFeedEncoder.getPosition()
            * math.pi
            * 2
            / self.INTAKE_FEED_GEARING
        )

        self.shooterFeed.setVoltage(buf.shooterFeedVolts)
        buf.shooterFeedAngle = (
            self.shooterFeedEncoder.getPosition()
            * math.pi
            * 2
            / self.SHOOTER_FEED_GEARING
        )

        self.shooterAim.setVoltage(buf.shooterAimVolts)
        buf.shooterAimAngle = (
            self.shooterAimEncoder.getPosition()
            * math.pi
            * 2
            / self.SHOOTER_AIM_GEARING
        )

        self.shooterTopMotor.setVoltage(buf.shooterTopMotorVolts)
        self.shooterBottomMotor.setVoltage(buf.shooterBottomMotorVolts)
        buf.shooterTopMotorAngle = (
            self.shooterTopMotorEncoder.getPosition()
            * math.pi
            * 2
            / self.SHOOTER_TOP_MOTOR_GEARING
        )
        buf.shooterBottomMotorAngle = (
            self.shooterBottomMotorEncoder.getPosition()
            * math.pi
            * 2
            / self.SHOOTER_BOTTOM_MOTOR_GEARING
        )

        buf.yaw = math.radians(-self.gyro.getAngle())
