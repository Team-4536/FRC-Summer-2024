from PIDController import PIDController
from robotHAL import RobotHALBuffer
from wpimath.kinematics import (
    ChassisSpeeds,
    DifferentialDriveKinematics
)

class Drive:
    def __init__(self):
        self.drivePIDs = [PIDController("LeftDrive", 0, 0, 0, 0), # idk what these values should be  ¯\(ツ)/¯
                          PIDController("RightDrive", 0, 0, 0, 0)]
        self.tankDriveKinematics = DifferentialDriveKinematics(trackWidth=1) # temp track width value

    def resetOdom(self):
        pass

    def update(self, dt: float, hal: RobotHALBuffer, speed: ChassisSpeeds):
        self.wheel_speeds = self.tankDriveKinematics.toWheelSpeeds(speed)
        
        left_drive_voltage: list[posit] = [self.drivePIDs[0].tick(self.wheel_speeds.left, hal.leftDriveSpeedMeasured, dt)]
        right_drive_voltage: float = self.drivePIDs[1].tick(self.wheel_speeds.right, hal.rightDriveSpeedMeasured, dt)

        hal.leftDriveVolts = left_drive_voltage * 3
        hal.rightDriveVolts = right_drive_voltage


    def updateOdom(self, hal):
        pass
