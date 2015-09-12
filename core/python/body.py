#!/usr/bin/env python
from task import Task
import commands
import core
import cfgstiff
from memory import *

class RotateOnSpot(Task):
  def __init__(self, tilt = None, pan = 0.0, time = 2.0):
    Task.__init__(self)
    self.tilt = tilt
    self.speed = 0.4
    self.rotateLeft = True
    self.time_limit = 120.0

  def run(self):
    tVel = self.speed * (1.0 if self.rotateLeft else -1.0)
    commands.setWalkVelocity(0.0,0.0,tVel)

    if self.getTime() > self.time_limit: self.finish()


