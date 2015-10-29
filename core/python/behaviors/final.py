import memory, pose, commands, cfgstiff, core, util, cfgpose
import math
from memory import *
from task import Task
from state_machine import *
from pid import PID
import mem_objects
from state_machine import *
import random
from memory import joint_commands
from geometry import *

class BlockLeft(Node):
  def run(self):
    UTdebug.log(15, "Blocking left")
    print "Block Left"
#    joint_commands.setJointCommandDeg(core.LShoulderRoll, 90.0)
    return pose.BlockLeft(2.0) 
#    return pose.ToPose(cfgpose.blockleft,2.0,100.0)
class BlockRight(Node):
  def run(self):
    print "Block Right"
    UTdebug.log(15, "Blocking right")
#    joint_commands.setJointCommandDeg(core.RShoulderRoll, -90.0)
    return pose.BlockRight(2.0) #
#    return pose.ToPose(cfgpose.blockright,2.0,100.0)

class BlockCenter(Node):
  def run(self):
    UTdebug.log(15, "Blocking right")
    print "Block Center"
#    joint_commands.setJointCommand(core.RShoulderPitch,0.0)
#    joint_commands.setJointCommand(core.LShoulderPitch,0.0)
    return pose.Squat() 
#    return pose.ToPose(cfgpose.blockcenter,2.0,100.0)

class InitBlocker(Node):

  def __init__(self):
    Node.__init__(self)
    self.direction = 1

  def run(self):

    commands.stand()
    commands.setHeadPan(self.direction * 50 * core.DEG_T_RAD, 0.5)
    if abs(core.joint_values[core.HeadYaw]) > 45 * core.DEG_T_RAD:
      if core.joint_values[core.HeadYaw] > 0:
        self.direction = -1
      else:
        self.direction = 1
     

class Blocker(Node):

  def __init__(self):
    Node.__init__(self)
    self.lastSeenCt = 0
    self.blockCt = 0
    self.time0 = self.getTime()
    self.direction = 1
    self.started = False
    self.delay = 5.0

  def run(self):
    if self.getTime() > self.delay:
      self.started = True

#    commands.setStiffness()
    commands.stand()
    ball = mem_objects.world_objects[core.WO_BALL]
    selfRobot = mem_objects.world_objects[core.WO_TEAM5]
    relBall = ball.loc.globalToRelative(selfRobot.loc, selfRobot.orientation)

    if ball.seen:
#      commands.setHeadPan(ball.bearing, 0.1)
      self.lastSeenCt = max(0,self.lastSeenCt - 1)
    else:
      self.lastSeenCt += 1

#    if self.lastSeenCt > 120:
    commands.setHeadPan(0,0.1)

    eta = float('inf')
#    print "Ball {6}: {0}, {1} Velocity: {2}, {3} Vision: {4} {5}".format(relBall.x, relBall.y, ball.absVel.x, ball.absVel.y, ball.visionDistance, ball.visionBearing*core.RAD_T_DEG, ball.seen)
    if ball.absVel.x < 0:
      eta = -1.0 * relBall.x / ball.relVel.x
    #if abs(ball.loc.x / ball.relVel.x) < 3.0 and ball.relVel.x < 0: # Ball will reach us in 3 seconds
    if eta < 10 and relBall.x < 1500 and eta > 3 and (self.getTime() - self.time0 <= 5.0):
#      intercept = ball.loc.y + (ball.absVel.y * eta)
      intercept = relBall.y + ball.absVel.y * eta
      print eta
      print ball.loc
      print relBall
      print ball.absVel
#      print ball.relVel
      print intercept
      if intercept < 500 and intercept > -500:
        self.blockCt += 1
        if self.blockCt > 0:
          UTdebug.log(15, "Ball is close, blocking!")
          if intercept > 120:
            choice = "left"
          elif intercept < 0:
            choice = "right"
          else:
            choice = "center"
          self.postSignal(choice)
      else:
        self.blockCt = max(self.blockCt - 1, 0)
    elif self.getTime() - self.time0 <= 5.0 and (self.getTime() > self.delay or self.started):
      target_pos = self.get_target_position(ball)
      self.move_to_position(*target_pos)

    if relBall.x < 1000.0:
      self.time0 = self.getTime()

    if self.getTime() - self.time0 > 5.0:
      commands.setHeadPan(50 * core.DEG_T_RAD,0.2)
      if self.getTime() - self.time0 > 6.5:
        self.time0 = self.getTime()
    elif ball.seen:
      commands.setHeadPan(ball.visionBearing,0.2)
    else:
      commands.setHeadPan(self.direction * 30 * core.DEG_T_RAD,0.5)
      if core.joint_values[core.HeadYaw] > 25 * core.DEG_T_RAD:
        self.direction = -1
      elif core.joint_values[core.HeadYaw] < -25 * core.DEG_T_RAD:
        self.direction = 1

  # Point in goalie box on intercept  between goal and ball.
  # Given in absolute coordinates
  def get_target_position(self, ball):
      goal = world_objects.getObjPtr(core.WO_OWN_GOAL)
      target_x = -425
      y_bound = 500
      m = (ball.loc.y - goal.loc.y) / (ball.loc.x + goal.loc.x)
      b = goal.loc.y - m* goal.loc.x
     
      target_y = max(min(ball.loc.y,400),-400) #m * target_x + b
#      target_y = min(target_y, y_bound)
#      target_y = max(target_y, y_bound * -1.0)

      return (target_x, target_y)

  def move_to_position(self, target_x, target_y):
    robot = world_objects.getObjPtr(robot_state.WO_SELF);
    ball = world_objects.getObjPtr(core.WO_BALL)
    xVel = 0.0
    yVel = 0.0
    tVel = 0.0

    #Adjust angle
    angleToTarget = ball.visionBearing#robot.orientation# * -1.0
    if abs(angleToTarget) > 10 * core.DEG_T_RAD and abs(angleToTarget) < 90.0 * core.DEG_T_RAD:
      tVel = 0.5 * max(min(angleToTarget / 0.87,0.3),-0.3) #0.25 * (angleToTarget / abs(angleToTarget))
      
    xToTarget = target_x - robot.loc.x
    xVel = 0.2
#    if abs(xToTarget) > 100:
#      xVel = max(0.25 * (xToTarget / abs(xToTarget)),0.1)

    yToTarget = target_y - robot.loc.y
    if abs(yToTarget) > 100:
      yVel = min(max(yToTarget / 200.0, -0.4),0.4)
      tVel = 0.0
    else:
      xVel = 0.0
      tVel = 0.0    
    print "Goalie Position: {0} {1} {2} Velocity {3} {4} {5}".format(robot.loc.x,robot.loc.y,core.RAD_T_DEG * robot.orientation,xVel,yVel,tVel)
    commands.setWalkVelocity(xVel, yVel, tVel)

class Reset(Node):
    def run(self):
      commands.stand()
#      return pose.ToPose(cfgpose.sittingPoseV3,2.0)

class Set(LoopingStateMachine):
  def setup(self):
    init = InitBlocker()
    blocker = Blocker()
    blocks = {
      "left": BlockLeft(),
      "right": BlockRight(),
      "center": pose.ToPose(cfgpose.sittingPoseV3,0.5) #Squat()#BlockCenter()
    }
    reset = Reset()
    self.trans(init,T(20),blocker)
    for name in blocks:
      b = blocks[name]
      self.trans(blocker, S(name), b, T(5.0), reset, T(2),blocker)


class Playing(StateMachine):
  class Stand(Node):
    def run(self):
      commands.stand()
      if self.getTime() > 2.0:
        self.finish()

  class Kick(Node):
    def run(self):
      if self.getFrames() <= 3:
        memory.walk_request.noWalk()
        memory.kick_request.setFwdKick()
      if self.getFrames() > 10 and not memory.kick_request.kick_running_:
        self.finish()

  class Dribble(Node):
    def run(self):
      commands.setWalkVelocity(1.0,0.0,0.0)
      if self.getTime() > 2.0:
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
      commands.setHeadPan(0.0,0.2)
      ball = world_objects.getObjPtr(core.WO_BALL)
      if ball.seen and ball.visionDistance < 3000:
        print "Ball Distance: ", ball.visionDistance
        self.last_seen = 0
        ball_x, ball_y = self.get_object_position(ball)        
        bearing = ball.visionBearing;
        if ball_x < 500 and abs(bearing) < 10 * core.DEG_T_RAD:
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
      selfRobot = world_objects.getObjPtr(robot_state.WO_SELF)
      self.direction = -1
      if selfRobot.loc.y > 0:
        self.direction = 1


    def run(self):
      ball = world_objects.getObjPtr(core.WO_BALL)
      goal = world_objects.getObjPtr(core.WO_OWN_GOAL)
      selfRobot = world_objects.getObjPtr(robot_state.WO_SELF)
      if ball.seen and goal.seen and abs(goal.visionBearing) < 10 * core.DEG_T_RAD:
        self.finishCt += 1
      else:
        self.finishCt = max(0,self.finishCt - 1)
      tVel = max(min(1.0,ball.visionBearing / 0.87),-1.0)
      xVel = max(min(1.0, 0.4 * ball.visionDistance / 300), -1.0)
      if ball.visionDistance > 2000:
        xVel = 0
      print "Ball: {0} {1}".format(ball.visionDistance,ball.visionBearing * core.RAD_T_DEG)
      print "Rotate Velocity: {0} {1} Finish: {2}".format(xVel,tVel,self.finishCt)

      commands.setWalkVelocity(xVel,self.direction * 0.3,tVel)
      if self.finishCt > 3:
        if selfRobot.loc.y > 0:
          self.direction = -1
        else:
          self.direction = 1
        self.finish() 
      
      if abs(selfRobot.orientation) > 165 * core.DEG_T_RAD:
        self.finishCt += 1

  class FinalApproach(Node):
    targetDistance = 100
    def __init__(self):
      Node.__init__(self)
      self.x = PID(0.6, 0.003, 0.001, 300)
      self.y = PID(0.6, 0.003, 0.0, 180) # was 0.0005
      self.target_pos = None
      self.last_seen = 0
      self.kick_offset = 50
      self.finishCt = 0
      b = world_objects.getObjPtr(core.WO_BALL)
      self.initialTheta = b.visionBearing 

    def run(self):
      ball = world_objects.getObjPtr(core.WO_BALL)
      if ball.seen and ball.visionDistance < 1000:
        print "Ball Distance: ", ball.visionDistance
        self.last_seen = 0
        ball_x, ball_y = self.get_object_position(ball)
        if ball_x < 100 and abs(ball_y - self.kick_offset) < 30:
          self.finishCt += 1
        else:
          self.finishCt = max(self.finishCt - 1, 0)

        xVel = 0.0
        if ball_x >= 100:
          xVel = self.x.update(ball_x)
        yVel = 0.0
        if abs(ball_y - self.kick_offset) >= 30:
          yVel = self.y.update(ball_y - self.kick_offset)
        tVel = (ball.visionBearing - self.initialTheta) / 0.87

        xVel = max(min(xVel,0.5),-0.5)
        yVel = max(min(yVel,1.0),-1.0)
        tVel = 0#max(min(yVel,0.1),-0.1)

        if tVel * yVel < 0:
          tVel = tVel * -1

        print "Final: {0}, {1}, {2} --> Walk Velocity: {3}, {4}, {5}".format(ball_x, ball_y - self.kick_offset, ball.visionBearing - self.initialTheta, xVel, yVel, tVel)
        commands.setWalkVelocity(xVel, yVel, tVel)
#        memory.walk_request.setLineUp(0.0,True)
      else:
        self.last_seen += 1
      if self.last_seen > 10:
        commands.setWalkVelocity(0, 0, 0)
        self.finishCt = max(self.finishCt - 1, 0)
      if self.finishCt > 3:
        selfRobot = world_objects.getObjPtr(robot_state.WO_SELF);
        self.x.reset()
        self.y.reset()
        if selfRobot.loc.x < 5000:
          self.postSignal("kick")
        else:
          self.postSignal("dribble")
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
    dribble = self.Dribble()
    #self.trans(self.Stand(), C, self.Approach(), C, self.Stand(), C, self.Rotate(), C, self.Stand(), C, self.FinalApproach(), C, self.Stand(), C, self.Kick(), C, pose.Sit(), C, self.Off())
    self.trans(self.Stand(), C, approach)
    self.trans(approach, C, rotate)
    self.trans(rotate, C, final)
    self.trans(final, S("kick"), self.Stand(), C, kick)
    self.trans(final, S("dribble"), dribble, C, approach)#self.Stand())
#    self.trans(dribble, C, approach)
    self.trans(kick, C, self.Stand(), C, approach)
    self.setFinish(None)
