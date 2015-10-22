import memory, pose, commands, cfgstiff, core, util
import math
from memory import *
from task import Task
from state_machine import *
#from geometry import *
import geometry
#import robot_state

class Testing(StateMachine):

  class Walk(Node):
    def run(self):
      commands.setWalkVelocity(0.5,0.0,0.0)

  def run(self):
    return self.Walk()
    

class Playing(StateMachine):

  class Stand(Node):
    def run(self):
      commands.stand()
      if self.getTime() > 4.0:
        self.finish()

  class WalkToCenter(Node):

    def __init__(self):
      Node.__init__(self)
      self.finishCt = 0
      self.lastObsCount = 0
      self.beacons = [core.WO_BEACON_BLUE_YELLOW,
			core.WO_BEACON_YELLOW_BLUE,
			core.WO_BEACON_YELLOW_PINK,
			core.WO_BEACON_PINK_YELLOW,
			core.WO_BEACON_BLUE_PINK,
			core.WO_BEACON_PINK_BLUE]
      self.names = ["BY","YB","YP","PY","BP","PB"]
      self.direction = 1.0
      self.tilt = -18
      self.prevX = 0
      self.prevY = 0
      self.prevT = 0
      self.omniWalk = False
      self.changeCt = 0
      self.doneCt = 0

    def run(self):

      # Localization information
      robot = world_objects.getObjPtr(robot_state.WO_SELF)
      center = core.Point2D(0,0)
      angleToCenter = robot.loc.getBearingTo(center,robot.orientation)
      distanceToCenter = robot.loc.getDistanceTo(center)
      
      # Walk Commands to reach center
      xVel = 0.0
      yVel = 0.0
      tVel = 0.0

      if not self.omniWalk:

        if abs(angleToCenter) > 15 * core.DEG_T_RAD:
          tVel = 0.5 * (angleToCenter / abs(angleToCenter))
#        else:
#          yVel = 0.2 * (angleToCenter / abs(angleToCenter))
        if abs(angleToCenter) > 30 * core.DEG_T_RAD:
          xVel = 0
        elif distanceToCenter > 100:
          xVel = max(min((distanceToCenter + 100) / 300.0, 0.5),-0.5)
        if distanceToCenter < 600 and tVel > 0.0:
          xVel = 0.0
        elif distanceToCenter < 200:
          relCenter = center.globalToRelative(robot.loc,robot.orientation)
          xVel = max(min((relCenter.x/300.0),0.5),-0.5)
          tVel = 0.0
        elif distanceToCenter < 100:
          xVel = 0
          tVel = 0
                   
      else:
        relCenter = center.globalToRelative(robot.loc,robot.orientation)
        xVel = max(min((relCenter.x + 0)/ 300.0, 0.5),-0.5)
        yVel = max(min((relCenter.y)/200.0, 0.5),-0.5)

      if self.omniWalk and distanceToCenter > 800:
        self.changeCt += 1
      elif self.omniWalk:
        self.changeCt = max(self.changeCt - 1,0)
      if not self.omniWalk and distanceToCenter < 600:
        self.changeCt += 1
      elif not self.omniWalk:
        self.changeCt = max(self.changeCt - 1,0)

      xVel = 0.6*xVel + self.prevX * 0.4
      yVel = 0.3*yVel + self.prevY * 0.7
      tVel = 0.3*tVel + self.prevT * 0.7      
      commands.setWalkVelocity(xVel,yVel,tVel)
      self.prevX = xVel
      self.prevY = yVel
      self.prevT = tVel
      print "Position {0}, {1}, {2} and velocity: {3}, {4}, {5}".format(robot.loc.x,robot.loc.y,robot.orientation,xVel,yVel,tVel)      
      # Update how long its been since we saw a beacon
      seen = False
      for b in self.beacons:
        beacon = world_objects.getObjPtr(b)
        if beacon.seen:
          seen = True
          print self.names[self.beacons.index(b)] + " {0} and {1}".format(beacon.visionDistance,beacon.visionBearing)
          break

      # If we haven't seen a beacon in a while we should stop and rotate
      if seen:
        self.lastObsCount = max(self.lastObsCount - 2, 0)
      else:
        self.lastObsCount += 1
      if self.lastObsCount > 150: # 5 seconds without seeing a beacon
        self.lastObsCount = 0
        self.finish()

      # move head
      commands.setHeadPan(self.direction * 50 * core.DEG_T_RAD, 0.5)
      if abs(core.joint_values[core.HeadYaw]) > 45 * core.DEG_T_RAD:
        if core.joint_values[core.HeadYaw] > 0:
          self.direction = -1
        else:
          self.direction = 1

      if self.changeCt > 10:
        self.changeCt = 0
        self.prevX = 0.0
        self.prevY = 0.0
        self.prevT = 0.0

      if distanceToCenter < 100:
        self.doneCt +=1
      else:
        self.doneCt = max(self.doneCt - 1,0)

      if self.doneCt > 5:
        self.finish()

  class Rotate(Node):
    def __init__(self):
      Node.__init__(self)
      self.seenCt = 0
      self.beacons = [core.WO_BEACON_BLUE_YELLOW,
                        core.WO_BEACON_YELLOW_BLUE,
                        core.WO_BEACON_YELLOW_PINK,
                        core.WO_BEACON_PINK_YELLOW,
                        core.WO_BEACON_BLUE_PINK,
                        core.WO_BEACON_PINK_BLUE]

    def run(self):

      seen = False
      for b in self.beacons:
        beacon = world_objects.getObjPtr(b)
        if beacon.seen:
          seen = True
          break

      if seen:
        self.seenCt += 1
      else:
        self.seenCt = max(self.seenCt - 1, 0)

      if self.seenCt > 5:
        self.seenCt = 0
        self.finish()

      commands.setWalkVelocity(0.0,0.0,0.5)
#      commands.setHeadTilt(-18)
      commands.setHeadPan(0.0,2.0)

  class Search(Node):
  
    def __init__(self):
      Node.__init__(self)
      self.seenCt = 0
      self.direction = 1
      self.beacons = [core.WO_BEACON_BLUE_YELLOW,
                        core.WO_BEACON_YELLOW_BLUE,
                        core.WO_BEACON_YELLOW_PINK,
                        core.WO_BEACON_PINK_YELLOW,
                        core.WO_BEACON_BLUE_PINK,
                        core.WO_BEACON_PINK_BLUE]


    def run(self):

      commands.stand()
      commands.setHeadPan(self.direction * 50 * core.DEG_T_RAD, 0.5)
      if abs(core.joint_values[core.HeadYaw]) > 45 * core.DEG_T_RAD:
        if core.joint_values[core.HeadYaw] > 0:
          self.direction = -1
        else:
          self.direction = 1

      ct = 0
      seen = False
      for b in self.beacons:
        beacon = world_objects.getObjPtr(b)
        if beacon.seen:
          seen = True
          ct += 1

      if ct >= 2:
        self.finish()

      if seen:
        self.seenCt += 1
      else:
        self.seenCt = max(self.seenCt - 1, 0)

      if self.seenCt > 5:
        self.seenCt = 0
        self.finish()


  def setup(self):

    stand = self.Stand()
    walkToCenter = self.WalkToCenter()
    rotate = self.Rotate()
    search = self.Search()

    self.trans(stand, C, search)
    self.trans(search, T(5.0), rotate)
    self.trans(rotate,T(10.0),search)
    self.trans(rotate,C,walkToCenter)
    self.trans(search, C, walkToCenter)
    self.trans(walkToCenter, C, search)
    self.trans(stand, T(10.0), search)    
#    self.trans(walkToCenter, C, rotate, C, walkToCenter)
    self.setFinish(None)
