#!/usr/bin/env python

import commands, head, cfgstiff, pose
from task import Task
from state_machine import *
import core,util
from memory import *

DEG_T_RAD = core.DEG_T_RAD
RAD_T_DEG = core.RAD_T_DEG

class Playing(Task):
  def run(self):
    commands.setStiffness()
    ball = world_objects.getObjPtr(core.WO_BALL)
    currentTilt = core.joint_values[core.HeadTilt] * RAD_T_DEG
    trackingSpeed = 10
    if ball.seen:
      aboveCenter = ball.imageCenterY < 100
      belowCenter = ball.imageCenterY > 140
      print "Current Tilt: {0}".format(currentTilt)
      print "AboveCenter: {}".format(aboveCenter)
      print "BelowCenter: {0}".format(belowCenter)
      if aboveCenter and currentTilt < 30:
        commands.setHeadTilt(currentTilt + trackingSpeed)
      elif belowCenter and currentTilt > -30:
        commands.setHeadTilt(currentTilt - trackingSpeed)
      else:
        commands.setHeadTilt(currentTilt)
    else:
        commands.setHeadTilt(currentTilt)
