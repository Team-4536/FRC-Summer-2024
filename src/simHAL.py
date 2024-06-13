import copy
import math

from ntcore import NetworkTableInstance
from real import angleWrap, lerp
from robotHAL import RobotHALBuffer
from timing import TimeData
from wpimath.geometry import Rotation2d, Translation2d


class RobotSimHAL:
    def __init__(self):
        self.prev = RobotHALBuffer()
        self.drivePositions = [0.0, 0.0, 0.0, 0.0]
        self.driveVels = [0.0, 0.0, 0.0, 0.0]
        self.steerEncoderPositions = [0.0, 0.0, 0.0, 0.0]
        self.steerVels = [0.0, 0.0, 0.0, 0.0]
        self.yaw = 0.0

        self.shooterAngVel = 0.0
        self.shooterAimVel = 0.0
        self.shooterAimPos = 0.0

        self.table = NetworkTableInstance.getDefault().getTable("sim")

        self.ringPos = 0
        self.ringTransitionStart = -1

    def update(self, buf: RobotHALBuffer, time: TimeData) -> None:
        # prev = self.prev
        self.prev = copy.deepcopy(buf)
