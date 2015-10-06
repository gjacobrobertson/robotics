import core, memory
import pose, commands, cfgstiff
import mem_objects
from task import Task
from state_machine import *
import random, core
from memory import joint_commands

class BlockLeft(Node):
  def run(self):
    UTdebug.log(15, "Blocking left")
    joint_commands.setJointCommand(core.LShoulderRoll, 90.0 * core.DEG_T_RAD)

class BlockRight(Node):
  def run(self):
    UTdebug.log(15, "Blocking right")
    joint_commands.setJointCommand(core.RShoulderRoll, -90.0 * core.DEG_T_RAD)


class BlockCenter(Node):
  def run(self):
    UTdebug.log(15, "Blocking right")
    joint_commands.setJointCommand(core.RShoulderPitch,0.0)
    joint_commands.setJointCommand(core.LShoulderPitch,0.0)

class Blocker(Node):
  def run(self):
    ball = mem_objects.world_objects[core.WO_BALL]
    selfRobot = mem_objects.world_objects[core.WO_SELF]
    commands.setHeadPan(ball.bearing, 0.1)

    relBall = ball.loc.globalToRelative(selfRobot.loc, selfRobot.orientation)
    shouldBlock = False
    if abs(relBall.loc.x / ball.relVel.x) < 3.0 and ball.relVel.x < 0: # Ball will reach us in 3 seconds
      shouldBlock = True

    if shouldBlock: #if ball.distance < 500 and ball.relVel.x < 0:
      UTdebug.log(15, "Ball is close, blocking!")
      if ball.bearing > 30 * core.DEG_T_RAD:
        choice = "left"
      elif ball.bearing < -30 * core.DEG_T_RAD:
        choice = "right"
      else:
        choice = "center"
      self.postSignal(choice)

class Playing(LoopingStateMachine):
  def setup(self):
    blocker = Blocker()
    blocks = {
      "left": BlockLeft(),
      "right": BlockRight(),
      "center": BlockCenter()
    }
    for name in blocks:
      b = blocks[name]
      self.trans(blocker, S(name), b, T(5), blocker)
