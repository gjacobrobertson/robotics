import memory, pose, commands, cfgstiff
from state_machine import *
from pid import PID

class Playing(StateMachine):
  class Stand(Node):
    def run(self):
      commands.stand()
      if self.getTime() > 5.0:
        self.finish()

  class Approach(Node):
    targetDistance = 250
    def __init__(self):
      x = PID(1, 0, 0)
      y = PID(1, 0, 0)
      t = PID(1, 0, 0)
      controller = (x, y, t)

    def run(self):
      goal = world_objects.getObjPtr(core.WO_GOAL)
      ball = world_objects.getObjPtr(core.WO_BALL)
      target_pos = get_target_position(goal, ball)
      update = lambda(e, pid): pid.update(e)
      control = map(lambda x: update(*x), zip(target_pos, controller))
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
