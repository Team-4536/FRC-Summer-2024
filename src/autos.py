import math

import autoUtils
import robot
from wpimath.geometry import Pose2d, Rotation2d

"""
DOCS FOR HOW AUTOS WORK
    (info for this file as well as autoUtils.py)
    (as of may 30 2024, by rob)

- This file should contain only autos that should go on the dash, in the form of a function, because autoUtils.py scrapes this file for what to put there
    - don't put utility functions or any other state at top level (util functions belong, you guessed it, in autoUtils.py)
    - Every function should have the signiture:
        def fun(r: 'robot.Robot'):
    - Every function should always yield a string that documents what condition the auto is currently waiting on (the yielded string is published to 'autoMessage', for debugging purposes.)
    - There is no return type marked because including a yield statement inside of a function messes with it, but every auto function should have at least one yield
    - Having more/less/different args or a non-string-generator return type will break things.

- Autos are defined as functions
    - as of making this, generators/coroutines appear to be a much, much cleaner/easier way to create autos, compared to things like
        commands or the god-awful staging system (check out the 23-24 repo to see that).
    - it is possible to create auto code as if it were just a high level script that controlled the robot, for example:

        while not r.HasGamePiece():
            r.collectGamePiece()
            yield "waiting to collect game piece"

        This snippet is written as if it were 'calling the robot', even though the robot is actually calling the auto function every frame.
        Generators act as 'resumable functions' where every time yield is called, they exit, but the next time they are called, they resume from the last place they yielded.
        Check out examples from this repo as well as the 23-24 repo (in the yield branch as of writing this) for how to write autos (or ask rob).

- 'yield from' will yield the value from the inner function, only when that function yields. If it doesn't, then control just goes to the next line

"""

def testingAuto(r: 'robot.Robot'):
    r.resetGyroAndOdomToPose(Pose2d())
    yield from autoUtils.wait(2)
    r.resetGyroAndOdomToPose(Pose2d(1, 0, Rotation2d(0)))
    yield from autoUtils.wait(3)
    r.resetGyroAndOdomToPose(Pose2d(2, 0, Rotation2d(math.pi/2)))
    yield from autoUtils.wait(4)
    r.resetGyroAndOdomToPose(Pose2d(3, 0, Rotation2d(-math.pi/2)))