import memory, pose, commands, cfgstiff, core, util
import math
from memory import *
from task import Task
from state_machine import *
from pid import PID


class Playing(StateMachine):
  class Stand(Node):
    def run(self):
      commands.stand()
      if self.getTime() > 4.0:
        self.finish()

  class Kick(Node):
    def run(self):
      if self.getFrames() <= 3:
        memory.walk_request.noWalk()
        memory.kick_request.setFwdKick()
      if self.getFrames() > 10 and not memory.kick_request.kick_running_:
        self.finish()

  class Approach(Node):
    targetDistance = 300
    def __init__(self):
      Node.__init__(self)
      self.x_pid = PID(0.5, 0.0001, 0.001, 300)
      self.t_pid = PID(0.5, 0.005, 0.0, 0.87)
      self.target_pos = None
      self.last_seen = 0
      self.kick_offset = 50 
      self.finishCt = 0

    def run(self):
      ball = world_objects.getObjPtr(core.WO_BALL)
      if ball.seen:
        print "Ball Distance: ", ball.visionDistance
        self.last_seen = 0
        ball_x, ball_y = self.get_object_position(ball)        
        bearing = ball.visionBearing;
        if ball_x < 300 and abs(bearing) < 10 * core.DEG_T_RAD:
          self.finishCt += 1
        else:
          self.finishCt = max(self.finishCt - 1, 0)

        xVel = self.x_pid.update(ball_x)
        yVel = 0 #self.y.update(ball_y - self.kick_offset) / 200.0
	tVel = self.t_pid.update(bearing)

        xVel = max(min(xVel,0.5),-0.5)
        yVel = max(min(yVel,1.0),-1.0)
        tVel = max(min(tVel,1.0),-1.0)

        print "Approach: {0}, {1} --> Walk Velocity: {2}, {3}".format(ball_x, bearing, xVel, tVel)
        commands.setWalkVelocity(xVel, yVel, tVel)
      else:
        self.last_seen += 1
      if self.last_seen > 10:
        print "Search!"
        commands.setWalkVelocity(0, 0, 0.4)
        self.finishCt = max(self.finishCt - 1, 0)
      if self.finishCt > 3:
        self.x_pid.reset()
        self.t_pid.reset()
        self.finish()

    def get_object_position(self, obj):
      obj_x = math.cos(obj.visionBearing) * obj.visionDistance
      obj_y = math.sin(obj.visionBearing) * obj.visionDistance
      return (obj_x, obj_y)
      
  class Rotate(Node):
    def __init__(self):
      Node.__init__(self)
      self.finishCt = 0

    def run(self):
      ball = world_objects.getObjPtr(core.WO_BALL)
      goal = world_objects.getObjPtr(core.WO_UNKNOWN_GOAL)
      if ball.seen and goal.seen and abs(goal.visionBearing) < 10 * core.DEG_T_RAD:
        self.finishCt += 1
      else:
        self.finishCt = max(0,self.finishCt - 1)
      tVel = max(min(1.0,ball.visionBearing / 0.87),-1.0)
      xVel = max(min(1.0, 0.4 * ball.visionDistance / 300), -1.0)
      print "Ball: {0} {1}".format(ball.visionDistance,ball.visionBearing * core.RAD_T_DEG)
      print "Rotate Velocity: {0} {1}".format(xVel,tVel)
      commands.setWalkVelocity(xVel,-0.3,tVel)
      if self.finishCt > 5:
        self.finish() 
      
  class FinalApproach(Node):
    targetDistance = 100
    def __init__(self):
      Node.__init__(self)
      self.x = PID(0.5, 0.001, 0.001, 300)
      self.y = PID(0.5, 0.001, 0.0, 180) # was 0.0005
      self.target_pos = None
      self.last_seen = 0
      self.kick_offset = 50
      self.finishCt = 0
      b = world_objects.getObjPtr(core.WO_BALL)
      self.initialTheta = b.visionBearing 

    def run(self):
      ball = world_objects.getObjPtr(core.WO_BALL)
      if ball.seen:
        print "Ball Distance: ", ball.visionDistance
        self.last_seen = 0
        ball_x, ball_y = self.get_object_position(ball)
        if ball_x < 160 and abs(ball_y - self.kick_offset) < 30:
          self.finishCt += 1
        else:
          self.finishCt = max(self.finishCt - 1, 0)

        xVel = self.x.update(ball_x)
        yVel = self.y.update(ball_y - self.kick_offset)
        tVel = (ball.visionBearing - self.initialTheta) / 0.87

        xVel = max(min(xVel,0.5),-0.5)
        yVel = max(min(yVel,1.0),-1.0)
        tVel = 0 #max(min(tVel,1.0),-1.0)

        if tVel * yVel > 0:
          tVel = tVel * -1

        print "Final: {0}, {1}, {2} --> Walk Velocity: {3}, {4}, {5}".format(ball_x, ball_y - self.kick_offset, ball.visionBearing - self.initialTheta, xVel, yVel, tVel)
        commands.setWalkVelocity(xVel, yVel, tVel)
      else:
        self.last_seen += 1
      if self.last_seen > 10:
        commands.setWalkVelocity(0, 0, 0)
        self.finishCt = max(self.finishCt - 1, 0)
      if self.finishCt > 3:
        self.x.reset()
        self.y.reset()
        self.finish()

    def get_object_position(self, obj):
      obj_x = math.cos(obj.visionBearing) * obj.visionDistance
      obj_y = math.sin(obj.visionBearing) * obj.visionDistance
      return (obj_x, obj_y)

  class Off(Node):
    def run(self):
      commands.setStiffness(cfgstiff.Zero)
      if self.getTime() > 2.0:
        memory.speech.say("turned off stiffness")
        self.finish()

  class LinedUp(Event):
    def ready(self):
      ball = world_objects.getObjPtr(core.WO_BALL)
      if ball.visionDistance < 160:
        return True
      else:
        return False

  def setup(self):
#    self.trans(self.Stand(), C, self.FinalApproach(), C, self.Stand(), C, self.Kick(), C, self.Stand(), C, pose.Sit(), C, self.Off())
    #self.trans(self.stand(), C, self.Kick(), C, self.Stand(), C, pose.Sit(), C, self.Off())
    stand = self.Stand()
    approach = self.Approach()
    rotate = self.Rotate()
    final = self.FinalApproach()
    kick = self.Kick()
    close = self.LinedUp()
    #self.trans(self.Stand(), C, self.Approach(), C, self.Stand(), C, self.Rotate(), C, self.Stand(), C, self.FinalApproach(), C, self.Stand(), C, self.Kick(), C, pose.Sit(), C, self.Off())
    self.trans(self.Stand(), C, approach)
    self.trans(approach, C, self.Stand(), C, rotate)
    self.trans(rotate, C, self.Stand(), C, final)
    self.trans(final, C, self.Stand(), C, kick)
    self.trans(kick, C, self.Stand(), C, approach)
    self.setFinish(None)
