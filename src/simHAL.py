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
        self.rightDriveVels = [0.0, 0.0]
        self.leftDriveVels = [0.0, 0.0]
        self.steerEncoderPositions = [0.0, 0.0, 0.0, 0.0]
        self.steerVels: list[float] = [0.0, 0.0, 0.0, 0.0]
        self.yaw: float = 0.0
        self.driveDistq: list[float] = [0, 0, 0, 0]
        self.width = 0.612775

        self.table = NetworkTableInstance.getDefault().getTable("sim")

        

    def update(self, buf: RobotHALBuffer, time: TimeData) -> None:
        # prev = self.prev
        self.prev = copy.deepcopy(buf)

        prefs = ["FL", "FR", "BL", "BR"]
        for i in range(2):
            self.rightDriveVels[i] = lerp(self.rightDriveVels[i], buf.rightDriveVolts[i] * 1/0.2, 0.2)
            self.leftDriveVels[i] = lerp(self.leftDriveVels[i], buf.leftDriveVolts[i] * 1/0.2, 0.2)
            #self.table.putNumber(prefs[i] + "SimSteerVel", self.steerVels[i])
        
        angleDeltaSum = 0.0
        self.driveVels = [self.rightDriveVels[0], self.rightDriveVels[1], self.leftDriveVels[0], self.leftDriveVels[1]]

        for i in range(2):#need these variables in robot hal buffer please            buf.driveSpeedMeasured[i] = self.driveVels[i]
            
            self.driveDistq[i] = self.driveVels[i] * time.dt
            #buf.drivePositions[i] += self.driveDistq[i]
            self.finalDriveDist = [(self.driveDistq[0] + self.driveDistq[1])/2, (self.driveDistq[2] + self.driveDistq[3])/2]

            #new = Translation2d(self.finalDriveDist, 0)  # idk how this will work
            y = abs(self.finalDriveDist[0] - self.finalDriveDist[1])
            #delta = angleWrap(math.atan2(new.y, new.x) - math.atan2(old.y, old.x))
            #angleDeltaSum += delta
        self.yaw += math.atan2(y, self.width)
        buf.yaw = self.yaw

       # NetworkTableInstance.getDefault().getTable("sim").putNumber("yaw", self.yaw)
    

#py -m robot.py --main src sim