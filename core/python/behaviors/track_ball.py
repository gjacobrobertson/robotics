import memory, commands, head
from task import Task
from state_machine import *

class Ready(Task):
  def run(self):
    commands.standStraight()
    if self.getTime() > 5.0:
      memory.speech.say("ready to play")
      self.finish()

class Set(Task):
  def run(self):
    commands.standStraight()

class Playing(StateMachine):
  class Stand(Node):
    def run(self):
      commands.stand()

  class BallLostEvent(Event):
    def __init__(self, event, name=None):
      super(BallLostEvent, self).__init__()
      self.lostCount = 0;      

    def ready(self):
      ball = core.world_objects.getObjPtr(core.WO_BALL)
      if not ball.seen:
        self.lostCount += 1
      else:
        self.lostCount -= 1
      if self.lostCount > 5:
        return True
      else:
        return False

  class BallFoundEvent(Event):
    def __init__(self):
      super(BallFoundEvent, self).__init__()
      self.foundCount = 0

    def ready(self):
      ball = core.world_objects.getObjPtr(core.WO_BALL)
      if ball.seen:
        self.foundCount += 1
      else:
        self.foundCount -= 1
      if self.foundCount > 5:
        return True
      else:
        return False

  class TrackBall(Node):
    def run(self):
      return head.TrackBall()  
  
  class BallSearch(Node):
    def run(self):
      return head.ScanForBall()

  def setup(self):
    stand = self.Stand()
    lost = self.BallLostEvent()
    found = self.BallFoundEvent()
    track = self.TrackBall()
    search = self.BallSearch()

    self.trans(stand, C, search)
    self.trans(search, found, track)
    self.trans(track, lost, search)
    
