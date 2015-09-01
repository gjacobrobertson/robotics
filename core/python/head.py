#!/usr/bin/env python
from task import Task
import commands
import core
import cfgstiff
from memory import *

DEG_T_RAD = core.DEG_T_RAD
RAD_T_DEG = core.RAD_T_DEG

class MoveHead(Task):
  def __init__(self, tilt = None, pan = 0.0, time = 2.0):
    Task.__init__(self)
    self.tilt = tilt
    self.pan = pan
    self.time = time

  def run(self):
    commands.setHeadPan(self.pan, self.time)
    if self.tilt is not None:
      commands.setHeadTilt(self.tilt)

    if self.getTime() > self.time: self.finish()

class TrackBall(Task):
  def __init__(self):
    Task.__init__(self)
    
  def run(self):
    ball = world_objects.getObjPtr(core.WO_BALL)
    angle = ball.bearing
    commands.stand()
    commands.setHeadPan(angle, 0.2)

class ScanForBall(Task):
  def __init__(self):
    Task.__init__(self)
    self.seenBallCount = 0
    self.ballCountThreshold = 0;
    self.sign = 1
    print "Init Scan"

  def run(self):
    print "Scan!"    
    if core.joint_values[core.HeadYaw] >= 40 * DEG_T_RAD:
      self.sign *= -1
    elif core.joint_values[core.HeadYaw] <= -40 * DEG_T_RAD:
      self.sign *= -1
    commands.stand()
    print "Angle: {0}, Sign: {1}".format(core.joint_values[core.HeadYaw] * RAD_T_DEG,self.sign)
    commands.setStiffness()
    commands.setHeadPan(self.sign * 45 * DEG_T_RAD, 2.0)
