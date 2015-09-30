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
    target_x = 150
    target_y = 50
    target_d = math.hypot(target_x, target_y)
    deflection_t = math.asin(target_y / target_x)

    def __init__(self):
      Node.__init__(self)
      self.x = PID(0.3, 0.001, 0.0, 300)
      self.y = PID(0.4, 0.001, 0.0, 200)
      self.t = PID(0.1, 0.001, 0.0, 0.87 )
      self.controller = (self.x, self.y, self.t)
      self.target_pos = None
      self.last_seen = 0

    def run(self):
      goal = world_objects.getObjPtr(core.WO_UNKNOWN_GOAL)
      ball = world_objects.getObjPtr(core.WO_BALL)
      if ball.seen and goal.seen:
        print "Ball Distance: ", ball.visionDistance 
        self.last_seen = 0
        self.target_pos = self.get_target_position(goal, ball)
        print "Target Position: ", self.target_pos
        (x, y, t) = self.target_pos
        if (x < 50 and abs(y) < 10 and abs(math.degrees(t)) < 10):
          self.finish()
        update = lambda e, pid: pid.update(e)
        control = map(lambda x: update(*x), zip(self.target_pos, self.controller))
        print "Setting Walk Velocity: ", control
        commands.setWalkVelocity(*control)
      else:
        self.last_seen += 1
      if self.last_seen > 10:
        commands.setWalkVelocity(0, 0, 0)
        
      
    def get_target_position(self, goal, ball):
      ball_x, ball_y = self.get_object_position(ball)
      goal_x, goal_y = self.get_object_position(goal)
      dy = goal_y - ball_y
      dx = goal_x - ball_x
      try:
        r = dy / dx
      except ZeroDivisionError:
        r = float('inf') * dy
      target_t = math.atan(r)
      target_deflection = target_t + self.deflection_t
      target_x = ball_x - (math.cos(target_deflection) * self.target_d)
      target_y = ball_y - (math.sin(target_deflection) * self.target_d)
      return (target_x, target_y, target_t)

    def get_object_position(self, obj):
      obj_x = math.cos(obj.visionBearing) * obj.visionDistance
      obj_y = math.sin(obj.visionBearing) * obj.visionDistance
      return (obj_x, obj_y)

  class FinalApproach(Node):
    targetDistance = 100
    def __init__(self):
      Node.__init__(self)
      self.x = PID(0.5, 0.0001, 0.001)
      self.y = PID(0.5, 0.005, 0.0)
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
        if ball_x < 150 and abs(ball_y - self.kick_offset) < 40:
          self.finishCt += 1
        else:
          self.finishCt = max(self.finishCt - 1, 0)

        xVel = self.x.update(ball_x) / 300.0 
        yVel = self.y.update(ball_y - self.kick_offset) / 200.0

        xVel = max(min(xVel,0.5),-0.5)
        yVel = max(min(yVel,1.0),-1.0)

        print "Error: {0}, {1} --> Walk Velocity: {2}, {3}".format(ball_x, ball_y - self.kick_offset, xVel, yVel)
        commands.setWalkVelocity(xVel, yVel, 0.0)
      else:
        self.last_seen += 1
      if self.last_seen > 10:
        commands.setWalkVelocity(0, 0, 0)
        self.finishCt = max(self.finishCt - 1, 0)
      if self.finishCt > 3:
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

  def setup(self):
#    self.trans(self.Stand(), C, self.FinalApproach(), C, self.Stand(), C, self.Kick(), C, self.Stand(), C, pose.Sit(), C, self.Off())
    #self.trans(self.stand(), C, self.Kick(), C, self.Stand(), C, pose.Sit(), C, self.Off())
    self.trans(self.Stand(), C, self.Approach(), C, self.Stand(), C,pose.Sit(), C, self.Off())
