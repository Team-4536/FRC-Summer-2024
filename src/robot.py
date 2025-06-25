import math

import profiler
import robotHAL
import wpilib
from ntcore import NetworkTableInstance
from pathplannerlib.controller import PIDConstants, PPHolonomicDriveController
from PIDController import PIDController, PIDControllerForArm, updatePIDsInNT
from real import angleWrap, lerp
from simHAL import RobotSimHAL
from timing import TimeData
from utils import CircularScalar, Scalar
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import ChassisSpeeds, SwerveModulePosition


class RobotInputs:
    def __init__(self) -> None:
        self.driveCtrlr = wpilib.XboxController(0)
        self.armCtrlr = wpilib.XboxController(1)
        self.buttonPanel = wpilib.Joystick(2)

        self.driveScalar = CircularScalar(0.06, 1)
        self.turningScalar = CircularScalar(0.1, 1)
        self.manualAimScalar = Scalar(deadZone=0.1)

        self.driveLeft: float = 0.0
        self.driveRight: float = 0.0
        self.speedCtrl: float = 0.0
        self.gyroReset: bool = False

    def update(self) -> None:

        self.driveDeadzone = 0.07

        # left
        if abs(self.driveCtrlr.getLeftY()) > self.driveDeadzone:
            self.driveLeft = -self.driveCtrlr.getLeftY()
        else:
            self.driveLeft = 0

        # right
        if abs(self.driveCtrlr.getRightY()) > self.driveDeadzone:
            self.driveRight = -self.driveCtrlr.getRightY()
        else:
            self.driveRight = 0

        self.gyroReset = self.driveCtrlr.getStartButtonPressed()


class Robot(wpilib.TimedRobot):
    def robotInit(self) -> None:
        self.time = TimeData(None)
        self.hal = robotHAL.RobotHALBuffer()
        self.hardware: robotHAL.RobotHAL | RobotSimHAL
        if self.isSimulation():
            self.hardware = RobotSimHAL()
        else:
            self.hardware = robotHAL.RobotHAL()
        self.hardware.update(self.hal, self.time)

        self.table = NetworkTableInstance.getDefault().getTable("telemetry")

        self.input = RobotInputs()

        self.drive = None

        self.driveGyroYawOffset = (
            0.0  # the last angle that drivers reset the field oriented drive to zero at
        )

        self.autoSideChooser = wpilib.SendableChooser()
        wpilib.SmartDashboard.putData("auto side chooser", self.autoSideChooser)

        self.odomField = wpilib.Field2d()
        wpilib.SmartDashboard.putData("odom", self.odomField)

        # kp can be 4 if wanted
        self.turnPID = PIDController("turnPID", 3, 0, 0)

        self.ang = 0

        self.frontLimelightTable = NetworkTableInstance.getDefault().getTable(
            "limelight-front"
        )
        self.robotPoseTable = NetworkTableInstance.getDefault().getTable("robot pose")

    def robotPeriodic(self) -> None:
        profiler.start()

        self.time = TimeData(self.time)

        self.table.putNumber("DriveLeftInput", self.input.driveCtrlr.getLeftY())
        self.table.putNumber("DriveRightInput", self.input.driveCtrlr.getRightY())

        self.table.putNumber("DriveLeftPercent", self.input.driveLeft)
        self.table.putNumber("DriveRightPercent", self.input.driveRight)

        self.hal.publish(self.table)
        self.input.update()

        updatePIDsInNT()

    def teleopInit(self) -> None:
        pass

    def teleopPeriodic(self) -> None:
        self.input.update()
        self.hal.stopMotors()

        # constant (change in code for now)
        driveScaler = 1

        self.hal.leftDrivePercent = self.input.driveLeft * driveScaler
        self.hal.rightDrivePercent = self.input.driveRight * driveScaler

        self.hardware.update(self.hal, self.time)

    def autonomousInit(self) -> None:
        # when simulating, initalize sim to have a preloaded ring
        # if isinstance(self.hardware, RobotSimHAL):

        pass

    def autonomousPeriodic(self) -> None:
        self.hal.stopMotors()
        # self.hardware.update(self.hal, self.time)
        pass

    def disabledInit(self) -> None:
        self.disabledPeriodic()

    def disabledPeriodic(self) -> None:
        self.hal.stopMotors()

        self.hardware.update(self.hal, self.time)
        pass
