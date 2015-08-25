class Point:
  def __init__(self, *args, **kwds):
    self.x=0.0;
    self.y=0.0;
    self.z=0.0;

class Quaternion:
  def __init__(self, *args, **kwds):
    self.x=0.0;
    self.y=0.0;
    self.z=0.0;
    self.w=0.0;

class Pose:
  def __init__(self, *args, **kwds):
    self.position = Point()
    self.orientation = Quaternion()
