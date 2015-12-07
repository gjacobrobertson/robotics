import memory, pose, commands, cfgstiff
from task import Task
from state_machine import *

class Playing(StateMachine):
  class Stand(Node):
    def run(self):
      commands.setStiffness()
      commands.stand()
      if self.getTime() > 2.0:
        self.finish()

  class Forward(Node):
    def run(self):
      commands.setWalkVelocity(0.5,0,0)

  class Backward(Node):
    def run(self):
      commands.setWalkVelocity(-0.5,0,0)

  class Left(Node):
    def run(self):
      commands.setWalkVelocity(0,0.5,0)

  class Right(Node):
    def run(self):
      commands.setWalkVelocity(0,-0.5,0)

  class TurnLeft(Node):
    def run(self):
      commands.setWalkVelocity(0,0,0.5)

  class TurnRight(Node):
    def run(self):
      commands.setWalkVelocity(0,0,-0.5)

  class Off(Node):
    def run(self):
      commands.setStiffness(cfgstiff.Zero)
      if self.getTime() > 2.0:
        self.finish()

  def setup(self):
    stand = self.Stand
    F = self.Forward
    B = self.Backward
    L = self.Left
    R = self.Right
    TL = self.TurnLeft
    RT = self.TurnRight
    self.trans(stand(), C, F(), T(5.0), L(), T(5.0), B(), T(5.0), R(), T(5.0), pose.Sit(), C, self.Off())


