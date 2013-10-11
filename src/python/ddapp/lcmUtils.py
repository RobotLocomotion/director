import lcm

class GlobalLCM(object):

  _handle = None

  @classmethod
  def get(cls):
      if cls._handle == None:
          cls._handle = lcm.LCM()
      return cls._handle
