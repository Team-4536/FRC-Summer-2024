from PIDController import PIDController
from robotHAL import RobotHALBuffer
from wpimath.kinematics import ChassisSpeeds, DifferentialDriveKinematics


class Drive:
    def __init__(self):
        self.drivePIDs = [
            PIDController(
                "LeftDrive", 0, 0, 0, 0
            ),  # idk what these values should be  ¯\(ツ)/¯
            PIDController("RightDrive", 0, 0, 0, 0),
        ]
        self.tankDriveKinematics = DifferentialDriveKinematics(
            trackWidth=1
        )  # temp track width value

    def resetOdom(self):
        pass

    def update(self, dt: float, hal: RobotHALBuffer, speed: ChassisSpeeds):
        self.wheelSpeeds = self.tankDriveKinematics.toWheelSpeeds(speed)

        leftDriveVoltage = [
            self.drivePIDs[0].tick(self.wheelSpeeds.left, x, dt)
            for x in hal.leftDriveSpeedMeasured
        ]
        rightDriveVoltage = [
            self.drivePIDs[0].tick(self.wheelSpeeds.left, x, dt)
            for x in hal.rightDriveSpeedMeasured
        ]

        hal.leftDriveVolts = leftDriveVoltage
        hal.rightDriveVolts = rightDriveVoltage

    def updateOdom(self, hal):
        pass
