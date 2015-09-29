class PID(object):
  def __init__(self, kp, ki, kd):
    self.kp = kp
    self.ki = ki
    self.kd = kd
    self.last_error = 0
    self.cum_error = 0

  def update(self, e):
    p = self.kp * e

    #Update integral
    self.cum_error += e
    i = self.ki * self.cum_error

    #Update Derivative
    d = self.kd * (e - self.last_error)
    self.last_error = e
 
    return p + i + d
