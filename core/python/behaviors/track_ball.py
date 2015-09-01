#!/usr/bin/env python

import commands, head, cfgstiff, pose
from task import Task
from state_machine import *
import core
from memory import *

DEG_T_RAD = core.DEG_T_RAD

class Ready(Task):
  def run(self):
    commands.standStraight()
    if self.getTime() > 5.0:
      memory.speech.say("ready to play")
      self.finish()

class Set(Task):
  def run(self):
    commands.standStraight()

class Playing(StateMachine):
  class Stand(Node):
    def run(self):
      commands.stand()
      if self.getTime() > 3.0:
        self.finish()


  class BallLostEvent(Event):
    def __init__(self):
      super(self.__class__, self).__init__()
      self.lostCount = 0;      

    def ready(self):
      ball = world_objects.getObjPtr(core.WO_BALL)
      if not ball.seen:
        self.lostCount += 1
      else:
        self.lostCount -= 1
      if self.lostCount > 5:
        return True
      else:
        return False

  class BallFoundEvent(Event):
    def __init__(self):
      super(self.__class__,self).__init__()
      self.foundCount = 0

    def ready(self):
      ball = world_objects.getObjPtr(core.WO_BALL)
      if ball.seen:
        self.foundCount += 1
      else:
        self.foundCount -= 1
      if self.foundCount > 5:
        return True
      else:
        return False

  class TrackBall(Node):
    def run(self):
      return head.TrackBall()  
  
  class BallSearch(TaskNode):
    def __init__(self, task):
      self.task = task
      self.task._parent = self
      super(TaskNode, self).__init__() 

  def setup(self):
    stand = self.Stand()
    lost = self.BallLostEvent()
    found = self.BallFoundEvent()
    track = self.TrackBall()
    search = self.BallSearch(head.ScanForBall())

#    self.trans(stand, T(20.0), stand)
    self.trans(search, found, track)
    self.trans(track, lost, search)
    
