#!/usr/bin/env python

import commands, head, cfgstiff, pose
from task import Task
from state_machine import *
import core,util
from memory import *

DEG_T_RAD = core.DEG_T_RAD
RAD_T_DEG = core.RAD_T_DEG

class Playing(StateMachine):

  def run(self):
      print "Stand"
      commands.stand()

      beacon_b_y = world_objects.getObjPtr(core.WO_BEACON_BLUE_YELLOW)
      beacon_y_b = world_objects.getObjPtr(core.WO_BEACON_YELLOW_BLUE)
      beacon_b_p = world_objects.getObjPtr(core.WO_BEACON_BLUE_PINK)
      beacon_p_b = world_objects.getObjPtr(core.WO_BEACON_PINK_BLUE)
      beacon_p_y = world_objects.getObjPtr(core.WO_BEACON_PINK_YELLOW)
      beacon_y_p = world_objects.getObjPtr(core.WO_BEACON_YELLOW_PINK)

      if beacon_b_y.seen:
        print "BLUE YELLOW DISTANCE: {0}".format(beacon_b_y.visionDistance)
      if beacon_y_b.seen:
        print "YELLOW BLUE DISTANCE: {0}".format(beacon_y_b.visionDistance)
      if beacon_b_p.seen:
        print "BLUE PINK DISTANCE: {0}".format(beacon_b_p.visionDistance)
      if beacon_p_b.seen:
        print "PINK BLUE DISTANCE: {0}".format(beacon_p_b.visionDistance)
      if beacon_p_y.seen:
        print "PINK YELLOW DISTANCE: {0}".format(beacon_p_y.visionDistance)
      if beacon_y_p.seen:
        print "YELLOW PINK DISTANCE: {0}".format(beacon_y_p.visionDistance)

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
