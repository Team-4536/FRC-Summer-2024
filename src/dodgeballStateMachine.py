from enum import Enum

from ntcore import NetworkTable, NetworkTableInstance
from PIDController import PIDControllerForArm, PIDController
from robotHAL import RobotHALBuffer


class StateEnum(Enum):
    IDLE = 1
    INTAKEING = 2
    AIMING = 3
    SHOOTING = 4


class ShooterTarget(Enum):
    NONE = 0
    LOW = 1
    MEDIUM = 2
    HIGH = 3


class DodgeballStateMachine:

    LOW_SETPOINT = 0
    MEDIUM_SETPOINT = 0
    HIGH_SETPOINT = 0
    REV_TARGET = 0

    def __init__(self) -> None:
        self.table: NetworkTable = NetworkTableInstance.getDefault().getTable(
            "dodgeballStateMachine"
        )
        self.state: StateEnum = StateEnum.IDLE

        self.aimPID: PIDControllerForArm = PIDControllerForArm("aim", 0, 0, 0, 0, 0, 0)
        self.topShooterPID: PIDController = PIDController("topShooter", 0, 0, 0, 0)
        self.bottomShooterPID: PIDController = PIDController(
            "bottomShooter", 0, 0, 0, 0
        )

        self.inputIntake: bool = False
        self.inputAim: ShooterTarget = ShooterTarget.NONE
        self.inputRevving: bool = False
        self.inputShoot: bool = False

        self.time: float = 0

    def intake(self, intake: bool) -> None:
        self.table.putBoolean("intake", intake)
        self.inputIntake = intake

    def aiming(self, target: ShooterTarget) -> None:
        self.table.putNumber("aimed", target.value)
        self.inputAim = target

    def rev(self, rev: bool) -> None:
        self.table.putBoolean("rev", rev)
        self.inputRevving = rev

    def shoot(self, shoot: bool) -> None:
        self.table.putBoolean("shoot", shoot)
        self.inputShoot = shoot

    def update(self, hal: RobotHALBuffer, time: float, dt: float) -> None:

        if self.state == StateEnum.IDLE:
            hal.intakeFeedVolts = 0
            hal.intakePivotVolts = 0
            hal.shooterFeedVolts = 0
            hal.shooterAimVolts = 0
            hal.shooterTopMotorVolts = 0
            hal.shooterBottomMotorVolts = 0

            if self.inputIntake:
                self.state = StateEnum.INTAKEING

        elif self.state == StateEnum.INTAKEING:
            hal.intakeFeedVolts = 0
            hal.intakePivotVolts = 0
            hal.shooterFeedVolts = 0
            hal.shooterAimVolts = 0
            hal.shooterTopMotorVolts = 0
            hal.shooterBottomMotorVolts = 0

            if hal.intakeSensor:
                self.state = StateEnum.AIMING
            elif not self.inputIntake:
                self.state = StateEnum.IDLE

        elif self.state == StateEnum.AIMING:
            hal.intakeFeedVolts = 0
            hal.intakePivotVolts = 0
            hal.shooterFeedVolts = 0

            if self.rev:
                hal.shooterTopMotorVolts = self.topShooterPID.tick(
                    self.REV_TARGET, hal.shooterTopMotorAngle, dt
                )
                hal.shooterBottomMotorVolts = self.bottomShooterPID.tick(
                    self.REV_TARGET, hal.shooterBottomMotorAngle, dt
                )

            if self.inputAim == ShooterTarget.LOW:
                hal.shooterAimVolts = self.aimPID.tick(
                    self.LOW_SETPOINT, hal.shooterAimAngle, dt
                )
            elif self.inputAim == ShooterTarget.MEDIUM:
                hal.shooterAimVolts = self.aimPID.tick(
                    self.MEDIUM_SETPOINT, hal.shooterAimAngle, dt
                )
            elif self.inputAim == ShooterTarget.HIGH:
                hal.shooterAimVolts = self.aimPID.tick(
                    self.HIGH_SETPOINT, hal.shooterAimAngle, dt
                )

            if self.inputShoot:
                self.state = StateEnum.SHOOTING
                self.time = time

        elif self.state == StateEnum.SHOOTING:
            hal.intakeFeedVolts = 0
            hal.intakePivotVolts = 0
            hal.shooterFeedVolts = 0
            hal.shooterAimVolts = 0  # will need to add just anti gravity
            hal.shooterTopMotorVolts = self.topShooterPID.tick(
                self.REV_TARGET, hal.shooterTopMotorAngle, dt
            )
            hal.shooterBottomMotorVolts = self.bottomShooterPID.tick(
                self.REV_TARGET, hal.shooterBottomMotorAngle, dt
            )

            if (time - self.time) > 1:
                self.state = StateEnum.IDLE

        self.inputIntake = False
        self.inputAim = ShooterTarget.NONE
        self.inputRevving = False
        self.inputShoot = False

    def publish(self) -> None:
        self.table.putString("State", self.state.name)
