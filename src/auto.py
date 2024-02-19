
from collections.abc import Callable
from typing import TYPE_CHECKING

from ntcore import NetworkTableInstance

if TYPE_CHECKING:
    from robot import Robot

StageFunc = Callable[['Robot'], bool | None]
class Stage:
    def __init__(self, f: StageFunc, name: str) -> None:
        self.func = f
        self.name = name
        self.nextStage: 'Stage | None' = None
        self.abortStage: 'Stage | None' = None

    def add(self, s: 'Stage') -> tuple['Stage', 'Stage']:
        self.nextStage = s
        return self, s

class Auto():
    def __init__(self, time: float) -> None:
        self.currentStage: Stage | None = None
        self.stageStart = time
        self.table = NetworkTableInstance.getDefault().getTable("autos")

    def update(self, r: "Robot") -> None:
        self.table.putNumber("stageTime", self.stageStart)
        if self.currentStage is not None:
            self.table.putString("stage", f"{self.currentStage.name}")
            done = self.currentStage.func(r)
            if done is True:
                self.stagestart = self.currentStage.nextStage
                self.stagestart = r.time.timeSinceInit
            elif done is None:
                self.stagestart = self.currentStage.abortStage
                self.stagestart = r.time.timeSinceInit
