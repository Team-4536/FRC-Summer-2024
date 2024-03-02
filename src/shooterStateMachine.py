import math
from enum import Enum

from ntcore import NetworkTableInstance
from PIDController import PIDController, PIDControllerForArm
from robotHAL import RobotHALBuffer


class ShooterTarget(Enum):
    NONE = 0
    AMP = 1
    PODIUM = 2
    SUBWOOFER = 3

class StateMachine():
    READY_FOR_RING = 0
    FEEDING = 1
    STORED_IN_SHOOTER = 2
    AIMING = 3
    SHOOTING = 4

    SPEED_SMOOTH_SCALAR = 0.1
    AIM_SMOOTH_SCALAR = 0.05

    # 0 is target aim, 1 is target speeds, 2 is cam
    ampSetpoint = (1.7, 100, 0)
    podiumSetpoint = (0, 0, 3)
    subwooferSetpoint = (0, 250, 0)

    def __init__(self):
        self.table = NetworkTableInstance.getDefault().getTable("ShooterStateMachineSettings")
        self.table.putNumber("kff", 0.00181)
        self.table.putNumber("kp", 0.0008)

        self.aimPID = PIDControllerForArm("aim", 0.8, 0, 0, 0, 0.02, 0.1)
        self.camPID = PIDController("cam", 0.06)
        self.shooterPID = PIDController("shooter", 0.0008, 0, 0, 0.00181)


        self.table.putNumber("podiumAim", math.radians(18))
        self.table.putNumber("podiumSpeed", 250)
        self.table.putNumber("podiumCam", 0.0)

        self.aimSetpoint = 0
        self.speedSetpoint = 0
        self.camSetpoint = 0
        self.PIDspeedSetpoint = 0
        self.PIDaimSetpoint = 0
        self.PIDcamSetpoint = 0
        self.state = self.READY_FOR_RING

        self.onTarget: bool = False # variable for autos to check in on this, as well as for not firing to early

        self.inputAim: ShooterTarget = ShooterTarget.NONE
        self.inputRev: bool = False
        self.inputShoot: bool = False
        self.inputFeed: bool = False

        self.inputProfile: float = 0.0

    # none will not change currently targeted pos
    def aim(self, target: ShooterTarget):
        self.table.putNumber("aimed", target.value)
        self.inputAim = target
    def rev(self, rev: bool):
        self.table.putBoolean("revved", rev)
        self.inputRev = rev
    def shoot(self, shoot: bool):
        self.table.putBoolean("shooting", shoot)
        self.inputShoot = shoot
    def feed(self, feed: bool):
        self.table.putBoolean("feeding (shooter)", feed) #untested
        self.inputFeed = feed

    def publishInfo(self):
        self.table.putNumber("state", self.state)
        self.table.putNumber("targetSpeed", self.speedSetpoint)
        self.table.putNumber("targetSpeedSmoothed", self.PIDspeedSetpoint)
        self.table.putNumber("targetAim", self.aimSetpoint)
        self.table.putNumber("targetAimSmoothed", self.PIDaimSetpoint)
        self.table.putNumber("targetCam", self.camSetpoint)
        self.table.putBoolean("onTarget", self.onTarget)


    # To send commands to the state machine, use aim(), rev(), and shoot() before calling this
    # Note that calling aim from READY_FOR_RING will feed and then aim
    def update(self, hal: RobotHALBuffer, time: float, dt: float):
        self.table.putNumber("inputRev", float(self.inputRev))
        self.table.putNumber("inputAim", self.inputAim.value)

        self.podiumSetpoint = (self.table.getNumber("podiumAim", 0.0), self.table.getNumber("podiumSpeed", 0.0), self.table.getNumber("podiumCam", 0.0))

        if(self.inputAim != ShooterTarget.NONE):
            if(self.inputAim == ShooterTarget.AMP):
                self.aimSetpoint = self.ampSetpoint[0]
                self.speedSetpoint = self.ampSetpoint[1]
                self.camSetpoint = self.ampSetpoint[2]
            elif(self.inputAim == ShooterTarget.PODIUM):
                self.aimSetpoint = self.podiumSetpoint[0]
                self.speedSetpoint = self.podiumSetpoint[1]
                self.camSetpoint = self.podiumSetpoint[2]
            elif(self.inputAim == ShooterTarget.SUBWOOFER):
                self.aimSetpoint = self.subwooferSetpoint[0]
                self.speedSetpoint = self.subwooferSetpoint[1]
                self.camSetpoint = self.subwooferSetpoint[2]

        self.onTarget = False
        if self.state == self.AIMING or self.state == self.SHOOTING:
            self.onTarget = abs(hal.shooterAimPos - self.aimSetpoint) < 0.1 and abs(hal.shooterAngVelocityMeasured - self.speedSetpoint) < 10

        if(self.state == self.READY_FOR_RING):
            aimTarget = 0
            speedTarget = 0
            camTarget = self.camSetpoint

            if(self.inputFeed and hal.intakeSensor):
                self.state = self.FEEDING

        elif(self.state == self.FEEDING):
            aimTarget = 0
            speedTarget = 0
            camTarget = 0
            hal.shooterIntakeSpeed = 0.1
            hal.intakeSpeeds[1] = 0.1
            camTarget = self.camSetpoint
            if hal.shooterSensor:
                self.state = self.STORED_IN_SHOOTER

        elif(self.state == self.STORED_IN_SHOOTER):
            hal.shooterIntakeSpeed = 0
            hal.intakeSpeeds[1] = 0
            aimTarget = 0
            speedTarget = 0
            camTarget = self.camSetpoint
            if self.inputAim != ShooterTarget.NONE:
                self.state = self.AIMING

        elif(self.state == self.AIMING):
            aimTarget = self.aimSetpoint
            speedTarget = 0
            camTarget = self.camSetpoint
            if self.inputRev:
                speedTarget = self.speedSetpoint
            if self.inputShoot:
                if self.onTarget:
                    self.state = self.SHOOTING
                    self.time = time

        elif(self.state == self.SHOOTING):
            aimTarget = self.aimSetpoint
            speedTarget = self.speedSetpoint
            camTarget = self.camSetpoint
            hal.shooterIntakeSpeed = 0.4
            hal.intakeSpeeds[1] = 0.4
            if(time - self.time > 1.0):
                self.state = self.READY_FOR_RING

        else:
            aimTarget = 0
            speedTarget = 0
            camTarget = self.camSetpoint

        self.PIDaimSetpoint = (aimTarget - self.PIDaimSetpoint) * self.AIM_SMOOTH_SCALAR + self.PIDaimSetpoint
        hal.shooterAimSpeed = self.aimPID.tick(self.PIDaimSetpoint, hal.shooterAimPos, dt)
        # TODO: profile to improve perf

        self.PIDspeedSetpoint = (speedTarget - self.PIDspeedSetpoint) * self.SPEED_SMOOTH_SCALAR + self.PIDspeedSetpoint
        hal.shooterSpeed = self.shooterPID.tick(self.PIDspeedSetpoint, hal.shooterAngVelocityMeasured, dt)

        hal.camSpeed = self.camPID.tick(camTarget, hal.camPos, dt)

        self.inputAim = ShooterTarget.NONE
        self.inputRev = False
        self.inputShoot = False
        self.inputFeed = False

        return self.state
