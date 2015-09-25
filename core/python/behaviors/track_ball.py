#!/usr/bin/env python

import commands, head, cfgstiff, pose
from task import Task
from state_machine import *
import core,util
from memory import *

DEG_T_RAD = core.DEG_T_RAD
RAD_T_DEG = core.RAD_T_DEG

class Playing(StateMachine):

  class Stand(Node):
    def run(self):
      print "Stand"
      commands.stand()
      
#      if self.getTime() > 2.0:
#        memory.speech.say("playing stand complete")
#        self.finish()

  class SearchBallNode(Task):

    def __init__(self):
      Task.__init__(self)
      self.direction = 1
      self.seenCount = 0

#    def reset(self):
#      super.reset()
#      self.seenCount = 0

    def run(self):
      print "Search: {0}, {1}".format(self.direction, self.seenCount)
      commands.setStiffness(cfgstiff.One)
      commands.stand()
      current_angle = core.joint_values[core.HeadYaw]
      commands.setHeadPan(self.direction * 55 * DEG_T_RAD, 1.0)
      ball = world_objects.getObjPtr(core.WO_BALL)
      if ball.seen:
        self.seenCount = self.seenCount + 1
      else:
        self.seenCount = max(0,self.seenCount - 1)
      if self.direction == 1 and current_angle > 45 * DEG_T_RAD:
        self.direction = -1
      if self.direction == -1 and current_angle < -45 * DEG_T_RAD:
        self.direction = 1
      if self.seenCount > 3:
        self.seenCount = 0
        self.finish()
        print self.seenCount


  class TrackBallNode(Task):
    def __init__(self):
      Task.__init__(self)
      self.lostCount = 0

#    def reset(self):
#      super(TrackBallNode, self).reset()
#      super.reset()
#      self.lostCount = 0

    def run(self):
      print "Track {0}".format(self.lostCount)
      commands.stand()
      ball = world_objects.getObjPtr(core.WO_BALL)
      angle = core.joint_values[core.HeadYaw]
      if ball.seen:
        self.lostCount = max(0,self.lostCount - 1)
        angle = ball.visionBearing
        print angle
        commands.setHeadPan(angle, 0.2)
      else:
        self.lostCount = self.lostCount + 1
      if self.lostCount > 7:
        self.lostCount = 0
        self.finish()

  def setup(self):
    search = self.SearchBallNode()
    track = self.TrackBallNode()
    stand = self.Stand()
    self.trans(search,C,track)
    self.trans(track,C,search)
    self.setFinish(None)


#  def run(self):
#    commands.setStiffness()
#    ball = world_objects.getObjPtr(core.WO_BALL)
#    currentTilt = core.joint_values[core.HeadTilt] * RAD_T_DEG
#    trackingSpeed = 10
#    if ball.seen:
#      aboveCenter = ball.imageCenterY < 100
#      belowCenter = ball.imageCenterY > 140
#      print "Current Tilt: {0}".format(currentTilt)
#      print "AboveCenter: {}".format(aboveCenter)
#      print "BelowCenter: {0}".format(belowCenter)
#      if aboveCenter and currentTilt < 30:
#        commands.setHeadTilt(currentTilt + trackingSpeed)
#      elif belowCenter and currentTilt > -30:
#        commands.setHeadTilt(currentTilt - trackingSpeed)
#      else:
#        commands.setHeadTilt(currentTilt)
#    else:
#        commands.setHeadTilt(currentTilt)
