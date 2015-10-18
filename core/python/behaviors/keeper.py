import core, memory
import pose, commands, cfgstiff, cfgpose
import mem_objects
from task import Task
from state_machine import *
import random, core
from memory import joint_commands
from geometry import *

class BlockLeft(Node):
  def run(self):
    UTdebug.log(15, "Blocking left")
#    print "Block Left"
#    joint_commands.setJointCommandDeg(core.LShoulderRoll, 90.0)
    return pose.ToPose(cfgpose.blockleft,2.0,100.0)
class BlockRight(Node):
  def run(self):
#    print "Block Right"
    UTdebug.log(15, "Blocking right")
#    joint_commands.setJointCommandDeg(core.RShoulderRoll, -90.0)
    return pose.ToPose(cfgpose.blockright,2.0,100.0)

class BlockCenter(Node):
  def run(self):
    UTdebug.log(15, "Blocking right")
#    print "Block Center"
#    joint_commands.setJointCommand(core.RShoulderPitch,0.0)
#    joint_commands.setJointCommand(core.LShoulderPitch,0.0)
    return pose.ToPose(cfgpose.blockcenter,2.0,100.0)

class Blocker(Node):
 
  def __init__(self):
    Node.__init__(self)
    self.lastSeenCt = 0

  def run(self):
    commands.setStiffness()
#    commands.stand()
    ball = mem_objects.world_objects[core.WO_BALL]
    selfRobot = mem_objects.world_objects[core.WO_TEAM5]
    relBall = ball.loc.globalToRelative(selfRobot.loc, selfRobot.orientation)

    if ball.seen:
      commands.setHeadPan(ball.bearing, 0.1)
      self.lastSeenCt = 0
    else:
      self.lastSeenCt += 1

    if self.lastSeenCt > 120:
      commands.setHeadPan(0,0.1)

    eta = float('inf')
    print "Ball: {0}, {1} Velocity: {2}, {3} Vision: {4} {5}".format(relBall.x, relBall.y, ball.absVel.x, ball.absVel.y, ball.visionDistance, ball.visionBearing*core.RAD_T_DEG)
    if ball.absVel.x < 0:
#      eta = -1.0 * (ball.loc.x + 1000) / ball.absVel.x
      eta = -1.0 * relBall.x / ball.absVel.x
    #if abs(ball.loc.x / ball.relVel.x) < 3.0 and ball.relVel.x < 0: # Ball will reach us in 3 seconds
    if eta < 10 and relBall.x < 1000 and eta > 3:
#      intercept = ball.loc.y + (ball.absVel.y * eta)
      intercept = relBall.y + ball.absVel.y * eta
      print eta
      print ball.loc
      print relBall
      print ball.absVel
#      print ball.relVel
      print intercept
      if intercept < 500 and intercept > -500:
        UTdebug.log(15, "Ball is close, blocking!")
        if intercept > 120:
          choice = "left"
        elif intercept < -120:
          choice = "right"
        else:
          choice = "center"
        self.postSignal(choice)

class Reset(Node):
    def run(self):
      return pose.ToPose(cfgpose.sittingPoseV3,2.0)

class Playing(LoopingStateMachine):
  def setup(self):
    blocker = Blocker()
    blocks = {
      "left": BlockLeft(),
      "right": BlockRight(),
      "center": BlockCenter()
    }
    reset = Reset()
    for name in blocks:
      b = blocks[name]
      self.trans(blocker, S(name), b, T(2), reset, T(2),blocker)
