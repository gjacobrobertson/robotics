class PID(object):
  def __init__(self, kp, ki, kd, scale=1.0, alpha=0.5):
    self.kp = kp
    self.ki = ki
    self.kd = kd
    self.scale = scale
    self.alpha = alpha
    self.last_error = 0
    self.cum_error = 0

  def reset(self):
    self.cum_error = 0
    self.last_error = 0

  def update(self, e):
    e = (self.alpha * e) + ((1.0 - self.alpha) * self.last_error)
    p = self.kp * e

    #Update integral
    self.cum_error += e
    i = self.ki * self.cum_error

    #Update Derivative
    d = self.kd * (e - self.last_error)
    self.last_error = e
 
    return (p + i + d) / self.scale
