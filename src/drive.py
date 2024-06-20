from turtle import right
from ntcore import NetworkTableInstance
from PIDController import PIDController
from robotHAL import RobotHALBuffer
from wpimath.kinematics import DifferentialDriveKinematics

class Drive:
    def __init__(self):
        self.leftDrivePID = PIDController("LeftDrive", 0, 0, 0, 0)
        self.rightDrivePID = PIDController("RightDrive", 0, 0, 0, 0)

        self.tankDriveKinematics = DifferentialDriveKinematics(
            trackWidth=1
        )  # temp track width value

    def resetOdom(self):
        pass

    def update(self, dt: float, hal: RobotHALBuffer, leftSpeed: float, rightSpeed: float):
        hal.leftDriveVolt = self.leftDrivePID.tick(leftSpeed, hal.leftDriveSpeedMeasured, dt)
        hal.rightDriveVolt = self.rightDrivePID.tick(rightSpeed, hal.rightDriveSpeedMeasured, dt)

    def updateOdom(self, hal):
        pass
