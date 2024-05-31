import math
from inspect import signature

import autos
import robot
import wpilib
from ntcore import NetworkTableInstance
from pathplannerlib.path import PathPlannerPath
from pathplannerlib.trajectory import PathPlannerTrajectory
from wpimath.geometry import Pose2d
from wpimath.kinematics import ChassisSpeeds

"""
- see autos.py for an in-depth explaination of the entire auto system, as well as should be in this file and autos.py
- also contains documentation for usage
- these two are split for reasons listed in there
"""

class AutoManager():
    # default initializes class, publishes auto chooser with options filled out
    # all options are generated from declarations in autos.py, check there for specifics
    # each option has the name and value of a function/generator to run, except doNothing which is added as the default and has value None
    def __init__(self) -> None:
        self.autoChooser = wpilib.SendableChooser()
        self.generator = None
        # NOTE: where auto strings are published to is documented as being autoMessage in autos.py, so if you're gonna change this also update that 
        self.outputPub = NetworkTableInstance.getDefault().getStringTopic("autoMessage").publish()

        self.autoChooser.setDefaultOption('doNothing', None)
        for key in autos.__dict__:
            fn = autos.__dict__[key]
            if callable(fn) and (len(signature(fn).parameters) == 1):
                self.autoChooser.addOption(key, key)
        wpilib.SmartDashboard.putData('auto chooser', self.autoChooser)

    # takes what auto has been selected on the dash and creates the generator for it
    # if the key is invalid or none, self.auto = None (which will do nothing when runAuto is called)
    # the reference to the robot, r, is not able to be updated after this call (because of generator wierdness)
    # The auto should call robot functions to reset pose when needed
    def chooseAuto(self, r: 'robot.Robot'):
        self.generator = None

        key = self.autoChooser.getSelected()
        if key is not None:
            if autos.__dict__.__contains__(key):
                self.generator = autos.__dict__[key]()
        # default case leaves it at none
        # TODO: worning message somewhere

    # runs one iteration of the last selected auto (from chooseAuto)
    # publishes the reason the auto yielded this frame
    def runAuto(self):
        if(self.generator is not None):
            try:
                self.outputPub.set(self.generator.__next__())
            except StopIteration:
                self.outputPub.set("[finished executing]")

# NOTE: filename is *just* the title of the file, with no extension and no path
# filename is directly passed to pathplanner.loadPath
def loadTrajectory(fileName: str, flipped: bool) -> PathPlannerTrajectory:
    p = PathPlannerPath.fromPathFile(fileName)
    if flipped:
        p = p.flipPath()
    t = p.getTrajectory(ChassisSpeeds(), p.getPreviewStartingHolonomicPose().rotation())
    return t

def runPath(r: 'robot.Robot', t: PathPlannerTrajectory, timeIntoTraj: float):
    goal = t.sample(timeIntoTraj)

    table = NetworkTableInstance.getDefault().getTable("autos")
    table.putNumber("pathGoalX", goal.getTargetHolonomicPose().X())
    table.putNumber("pathGoalY", goal.getTargetHolonomicPose().Y())
    table.putNumber("pathGoalR", goal.getTargetHolonomicPose().rotation().radians())

    table.putNumber("odomR", r.drive.odometry.getPose().rotation().radians())
    adjustedSpeeds = r.holonomicController.calculateRobotRelativeSpeeds(r.drive.odometry.getPose(), goal)
    table.putNumber("pathVelX", adjustedSpeeds.vx)
    table.putNumber("pathVelY", adjustedSpeeds.vy)
    table.putNumber("pathVelR", adjustedSpeeds.omega)

    r.drive.update(r.time.dt, r.hal, adjustedSpeeds)

def runPathUntilDone(r: 'robot.Robot', t: PathPlannerTrajectory):
    timer = wpilib.Timer()
    timer.start()
    while(timer.get() < t.getTotalTimeSeconds()):
        runPath(r, t, timer.get())
        yield "running path until finished"

def wait(duration: float):
    t = wpilib.Timer()
    t.start()
    while(t.get() < duration):
        yield f"waiting for {duration}s"

def tryResetOdomWithLimelight(r: 'robot.Robot', pipeline: int) -> bool:
    limelightTable = r.frontLimelightTable
    robotPoseTable = r.robotPoseTable

    if(limelightTable.getNumber("getPipe", -1) != pipeline):
        limelightTable.putNumber("pipeline", pipeline)

    #gets the pos from limelight
    visionPose = limelightTable.getNumberArray("botpose_wpiblue", [0,0,0,0,0,0,0])
    #debug values
    robotPoseTable.putNumber("limeXPos", visionPose[0])
    robotPoseTable.putNumber("limeYPos", visionPose[1])
    robotPoseTable.putNumber("limeYaw", visionPose[5])
    if (not (visionPose[0] == 0 and visionPose[1] == 0 and visionPose[5] == 0)):
        visionPose2D:Pose2d = Pose2d(visionPose[0], visionPose[1], math.radians(visionPose[5]))

        #X, Y, & Yaw are updated correctly
        r.drive.resetOdometry(visionPose2D, r.hal)
        return True
    return False
