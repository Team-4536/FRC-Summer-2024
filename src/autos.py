import robot
import wpilib
from ntcore import NetworkTableInstance
import autoUtils

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

def shootStartingRing(r: 'robot.Robot'):
    yield from autoUtils.intakeUntilRingGot(r)
    while not autoUtils.prepShooter(r, ShooterTarget.SUBWOOFER, True):
        yield "waiting on shooter prep"
    yield from autoUtils.fireShooterUntilDone(r)

def shootThenIntakeCenterRing(r: 'robot.Robot'):
    middleTraj = autoUtils.loadTrajectory("middle", r.onRedSide)
    returnTraj = autoUtils.loadTrajectory("middleBack", r.onRedSide)
    r.resetGyroAndOdomToPose(middleTraj.getInitialTargetHolonomicPose())

    autoUtils.tryResetOdomWithLimelight(r, 0)
    yield from RobotAutos.shootStartingRing(r)
    yield from autoUtils.scoreRing(r, middleTraj, returnTraj)

def troll(r: 'robot.Robot'):
    traj = autoUtils.loadTrajectory("troll", r.onRedSide)
    r.resetGyroAndOdomToPose(traj.getInitialTargetHolonomicPose())

    autoUtils.tryResetOdomWithLimelight(r, 0)
    yield from RobotAutos.shootStartingRing(r)
    yield from autoUtils.runPathUntilDone(r, traj)

def getAllMidUpLow(r: 'robot.Robot'):
    middleOut = autoUtils.loadTrajectory("middle", r.onRedSide)
    middleBack = autoUtils.loadTrajectory("middleBack", r.onRedSide)
    upperOut = autoUtils.loadTrajectory("upper", r.onRedSide)
    upperBack = autoUtils.loadTrajectory("upperBack", r.onRedSide)
    lowerOut = autoUtils.loadTrajectory("lower", r.onRedSide)
    lowerBack = autoUtils.loadTrajectory("lowerBack", r.onRedSide)
    r.resetGyroAndOdomToPose(middleOut.getInitialTargetHolonomicPose())

    autoUtils.tryResetOdomWithLimelight(r, 0)
    yield from RobotAutos.shootStartingRing(r)
    yield from autoUtils.scoreRing(r, middleOut, middleBack)
    autoUtils.tryResetOdomWithLimelight(r, 0)
    yield from autoUtils.scoreRing(r, upperOut, upperBack)
    autoUtils.tryResetOdomWithLimelight(r, 0)
    yield from autoUtils.scoreRing(r, lowerOut, lowerBack)

def getAllMidLowUp(r: 'robot.Robot'):
    middleOut = autoUtils.loadTrajectory("middle", r.onRedSide)
    middleBack = autoUtils.loadTrajectory("middleBack", r.onRedSide)
    upperOut = autoUtils.loadTrajectory("upper", r.onRedSide)
    upperBack = autoUtils.loadTrajectory("upperBack", r.onRedSide)
    lowerOut = autoUtils.loadTrajectory("lower", r.onRedSide)
    lowerBack = autoUtils.loadTrajectory("lowerBack", r.onRedSide)

    r.resetGyroAndOdomToPose(middleOut.getInitialTargetHolonomicPose())

    autoUtils.tryResetOdomWithLimelight(r, 0)
    yield from RobotAutos.shootStartingRing(r)
    yield from autoUtils.scoreRing(r, middleOut, middleBack)
    autoUtils.tryResetOdomWithLimelight(r, 0)
    yield from autoUtils.scoreRing(r, lowerOut, lowerBack)
    autoUtils.tryResetOdomWithLimelight(r, 0)
    yield from autoUtils.scoreRing(r, upperOut, upperBack)

def exitBackwards(r: 'robot.Robot'):
    traj = autoUtils.loadTrajectory("exit", r.onRedSide)
    r.resetGyroAndOdomToPose(traj.getInitialTargetHolonomicPose())
    yield from autoUtils.runPathUntilDone(r, traj)

def shootAndGetFarMiddle(r: 'robot.Robot'):
    outTraj = autoUtils.loadTrajectory("far-middle", r.onRedSide)
    returnTraj = autoUtils.loadTrajectory("far-middle", r.onRedSide)
    r.resetGyroAndOdomToPose(outTraj.getInitialTargetHolonomicPose())

    autoUtils.tryResetOdomWithLimelight(r, 0)
    yield from RobotAutos.shootStartingRing(r)
    yield from autoUtils.scoreRing(r, outTraj, returnTraj)

@staticmethod
def shootFromUpperSpeakerAndScoreUpperNote(r: 'robot.Robot'):
    outTraj = autoUtils.loadTrajectory("side-upper", r.onRedSide)
    returnTraj = autoUtils.loadTrajectory("side-upper-back", r.onRedSide)
    r.resetGyroAndOdomToPose(outTraj.getInitialTargetHolonomicPose())

    autoUtils.tryResetOdomWithLimelight(r, 0)
    yield from RobotAutos.shootStartingRing(r)
    yield from autoUtils.scoreRing(r, outTraj, returnTraj)

@staticmethod
def shootFromUpperSpeakerAndScoreTwo(r: 'robot.Robot'):
    outTraj = autoUtils.loadTrajectory("side-upper", r.onRedSide)
    returnTraj = autoUtils.loadTrajectory("side-upper-back", r.onRedSide)
    farOutTraj = autoUtils.loadTrajectory("sideFar-upper-v02", r.onRedSide)
    farReturnTraj = autoUtils.loadTrajectory("sideFar-upper-back-v02", r.onRedSide)
    r.resetGyroAndOdomToPose(outTraj.getInitialTargetHolonomicPose())

    autoUtils.tryResetOdomWithLimelight(r, 0)
    yield from RobotAutos.shootStartingRing(r)
    yield from autoUtils.scoreRing(r, outTraj, returnTraj)
    autoUtils.tryResetOdomWithLimelight(r, 0)
    yield from autoUtils.scoreRing(r, farOutTraj, farReturnTraj)

@staticmethod
def shootFromLowerSpeakerAndScoreLowerNote(r: 'robot.Robot'):
    outTraj = autoUtils.loadTrajectory("side-lower", r.onRedSide)
    returnTraj = autoUtils.loadTrajectory("side-lower-back", r.onRedSide)
    r.resetGyroAndOdomToPose(outTraj.getInitialTargetHolonomicPose())

    autoUtils.tryResetOdomWithLimelight(r, 0)
    yield from RobotAutos.shootStartingRing(r)
    yield from autoUtils.scoreRing(r, outTraj, returnTraj)

