import copy
import math

from ntcore import NetworkTableInstance
from real import angleWrap, lerp
from robotHAL import RobotHALBuffer
from timing import TimeData
from wpimath.geometry import Rotation2d, Translation2d


class RobotSimHAL():
    def __init__(self):
        self.prev = RobotHALBuffer()
        self.drivePositions = [0.0, 0.0, 0.0, 0.0]
        self.driveVels = [0.0, 0.0, 0.0, 0.0]
        self.steerEncoderPositions = [0.0, 0.0, 0.0, 0.0]
        self.steerVels: list[float] = [0.0, 0.0, 0.0, 0.0]
        self.yaw: float = 0.0
        self.driveDistq: list[float] = [0, 0, 0, 0]


        self.table = NetworkTableInstance.getDefault().getTable("sim")

        

    def update(self, buf: RobotHALBuffer, time: TimeData) -> None:
        # prev = self.prev
        self.prev = copy.deepcopy(buf)

        prefs = ["FL", "FR", "BL", "BR"]
        for i in range(2):
            self.driveVels[i] = lerp(self.driveVels[i], buf.driveVolts[i] * 1/0.2, 0.2)
            self.table.putNumber(prefs[i] + "SimSteerVel", self.steerVels[i])
        
        angleDeltaSum = 0.0
        for i in range(2):#need these variables in robot hal buffer please            buf.driveSpeedMeasured[i] = self.driveVels[i]
            
            self.driveDistq[i] = self.driveVels[i] * time.dt
            buf.drivePositions[i] += self.driveDistq[i]
            self.finalDriveDist = [(self.driveDistq[1] + self.driveDistq[2])/2, (self.driveDistq[3] + self.driveDistq[4])/2]

            new = Translation2d(self.finalDriveDist, 0) 
            x = TankDrive.modulePositions[i].x # idk how this will work
            y = 
            delta = angleWrap(math.atan2(new.y, new.x) - math.atan2(old.y, old.x))
            angleDeltaSum += delta
        self.yaw += atan2(y, x)
        buf.yaw = self.yaw
