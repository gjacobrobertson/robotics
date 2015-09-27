import memory, pose, commands, cfgstiff
from task import Task
from state_machine import *

class Playing(StateMachine):
  class Stand(Node):
    def run(self):
      commands.stand()
      if self.getTime() > 3.0:
        self.finish()

  class Kick(Node):
    def run(self):
      if self.getFrames() <= 3:
        memory.walk_request.noWalk()
        memory.kick_request.setFwdKick()
      if self.getFrames() > 10 and not memory.kick_request.kick_running_:
        self.finish()

  class Approach(Node):
    targetDistance = 250
    def __init__(self):
      self.x = PID(1.0 / 1500, 0, 0)
      self.y = PID(1.0 / 1500, 0, 0)
      self.t = PID(1.0 / (math.PI / 2), 0, 0)
      self.controller = (x, y, t)

    def run(self):
      goal = world_objects.getObjPtr(core.WO_GOAL)
      ball = world_objects.getObjPtr(core.WO_BALL)
      target_pos = get_target_position(goal, ball)
      (x, y, t) = target_pos
      if (x < 50 and y < 50 and math.degrees(t) < 5):
        self.finish()
      update = lambda(e, pid): pid.update(e)
      control = map(lambda x: update(*x), zip(target_pos, self.controller))
      commands.setWalkVelocity(*control)
      
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
      target_x = ball_x - (math.cos(target_t) * self.targetDistance)
      target_y = ball_y - (math.sin(target_t) * self.targetDistance)
      return (target_x, target_y, target_t)

    def get_object_position(self, obj):
      obj_x = math.cos(obj.visionBearing) * obj.visionDistance
      obj_y = math.sin(obj.visionBearing) * obj.visionDistance

  class Off(Node):
    def run(self):
      commands.setStiffness(cfgstiff.Zero)
      if self.getTime() > 2.0:
        memory.speech.say("turned off stiffness")
        self.finish()

  def setup(self):
    self.trans(self.Stand(), C, self.Approach(), C, self.Stand(), C, self.Kick(), C, self.Stand(), C, pose.Sit(), C, self.Off())
