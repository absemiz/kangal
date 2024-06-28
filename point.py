from vector3 import Vector3
from geometry_msgs.msg import Point as ROSPoint

class Point(Vector3):
	def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0):
		super().__init__(x, y, z)
		
	@classmethod
	def from_ros_message(cls, ros_point: ROSPoint) -> 'Point':
		return cls(ros_point.x, ros_point.y, ros_point.z)
		
	def to_ros_message(self) -> ROSPoint:
		return ROSPoint(self.x, self.y, self.z)