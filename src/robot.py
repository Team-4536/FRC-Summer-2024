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



    def __init__(self) -> None:
        self.driveCtrlr = wpilib.XboxController(0)
        self.armCtrlr = wpilib.XboxController(1)
        self.buttonPanel = wpilib.Joystick(4)

        self.driveScalar = CircularScalar(0.06, 1)
        self.turningScalar = CircularScalar(0.1, 1)
        self.manualAimScalar = Scalar(deadZone=0.1)

        self.driveX: float = 0.0
        self.driveY: float = 0.0
        self.turningX: float = 0.0
        self.turningY: float = 0.0
        self.speedCtrl: float = 0.0
        self.gyroReset: bool = False
        self.absToggle: bool = False

    def update(self) -> None:
        pass


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

        self.driveGyroYawOffset = 0.0 # the last angle that drivers reset the field oriented drive to zero at
        
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

        self.hal.publish(self.table)


        updatePIDsInNT()

    def teleopInit(self) -> None:
        pass

    def teleopPeriodic(self) -> None:
        self.input.update()
        self.hal.stopMotors()

        speedControlEdited = lerp(1, 5.0, self.input.speedCtrl)
        turnScalar = 6

        driveVector = Translation2d(self.input.driveX * speedControlEdited, self.input.driveY * speedControlEdited)
        turnVector = Translation2d(self.input.turningY, self.input.turningX) #for pid only


        self.hardware.update(self.hal, self.time)

    def autonomousInit(self) -> None:
        # when simulating, initalize sim to have a preloaded ring
        if isinstance(self.hardware, RobotSimHAL):

            pass        

    def autonomousPeriodic(self) -> None:
        self.hal.stopMotors()
        self.hardware.update(self.hal, self.time)

    def disabledInit(self) -> None:
        self.disabledPeriodic()

    def disabledPeriodic(self) -> None:
        self.hal.stopMotors()

        self.hardware.update(self.hal, self.time)
