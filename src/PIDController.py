import math

from ntcore import NetworkTable, NetworkTableInstance
from real import signum
from wpimath.controller import ProfiledPIDController, ArmFeedforward
from wpimath.trajectory import TrapezoidProfile

createdPIDControllers: list["PIDController"] = []
pidTable: NetworkTable = NetworkTableInstance.getDefault().getTable("pid")
createdArmFeedForwardControllers: list["ArmFeedForward"] = []
armFeedForwardTable: NetworkTable = NetworkTableInstance.getDefault().getTable(
    "armFeedForward"
)


def updatePIDsInNT():
    for c in createdPIDControllers:
        c._publish()


def updateArmFeedForwardInNT():
    for c in createdArmFeedForwardControllers:
        c._publish()


class PIDController:
    def __init__(
        self,
        name: str,
        kp: float = 0,
        ki: float = 0,
        kd: float = 0,
        kff: float = 0,
        maxVel: float = 100,
        maxAcc: float = 100,
    ) -> None:
        self.name = name
        self.PID: ProfiledPIDController = ProfiledPIDController(
            kp, ki, kd, TrapezoidProfile.Constraints(maxVel, maxAcc)
        )
        createdPIDControllers.append(self)

    # function returns the recommended force towards the target
    def tick(self, target: float, position: float, dt: float) -> float:
        return self.PID.calculate(position, target)

    def reset(self, currentPos: float) -> None:
        self.PID.reset(currentPos)

    def _publish(self) -> None:
        t = pidTable.getSubTable(self.name)
        if t.getNumber("Kp", None) is None:
            t.putNumber("Kp", self.PID.getP())
            t.putNumber("Ki", self.PID.getI())
            t.putNumber("Kd", self.PID.getI())
            t.putNumber("integralZone", self.PID.getIZone())
        else:
            self.PID.setP(t.getNumber("Kp", 0))
            self.PID.setI(t.getNumber("Ki", 0))
            self.PID.setD(t.getNumber("Kd", 0))
            self.PID.setIZone(t.getNumber("integralZone", 0))


class ArmFeedForward:
    def __init__(
        self, name: str, ks: float = 0, kg: float = 0, kv: float = 0, ka: float = 0
    ) -> None:
        self.name = name
        self.feedForward: ArmFeedforward = ArmFeedforward(ks, kg, kv, ka)
        createdArmFeedForwardControllers.append(self)

    def tick(self, angle: float, velocity: float, acceleration: float):
        return self.feedForward.calculate(angle, velocity, acceleration)

    def _publish(self) -> None:
        t = armFeedForwardTable.getSubTable(self.name)
        if t.getNumber("Ks", None) is None:
            t.putNumber("Ks", self.feedForward.kS)
            t.putNumber("Kg", self.feedForward.kG)
            t.putNumber("Kv", self.feedForward.kV)
            t.putNumber("Ka", self.feedForward.kA)
