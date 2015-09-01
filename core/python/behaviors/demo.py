import memory, pose, commands, cfgstiff
from task import Task
from state_machine import *

class Playing(StateMachine):
  class Stand(Node):
    def run(self):
      commands.stand()
      if self.getTime() > 5.0:
        print "standing up"
        self.finish()

  class Walk(Node):
    def run(self):
      commands.setWalkVelocity(0.5,0,0)

  class Stop(Node):
    def run(self):
      commands.setWalkVelocity(0,0,0)

  class Turn(Node):
    def run(self):
      commands.setWalkVelocity(0,0,0.5)

  class Curve(Node):
    def run(self):
      commands.setWalkVelocity(0.5,0,0.5)

  class Off(Node):
    def run(self):
      commands.setStiffness(cfgstiff.Zero)
      if self.getTime() > 2.0:
        print "stiffness off"
        self.finish()

  def setup(self):
    stand = self.Stand()
    walk = self.Walk()
    sit = pose.Sit()
    off = self.Off()
    turn = self.Turn()
    curve = self.Curve()
    self.trans(stand, C, walk, T(5.0), turn, T(5.0), curve, T(8.0), sit, C, off)
