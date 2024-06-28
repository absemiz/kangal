from vector3 import Vector3
from sensor_msgs.msg import LaserScan as ROSLaserScan
from math import pi

class LaserScan:
	def __init__(self, ros_laser_scan: ROSLaserScan):
		self.ros_laser_scan: ROSLaserScan = ros_laser_scan
		
	def angle_to_index(self, angle: float) -> int:
		return int(
			(angle - self.ros_laser_scan.angle_min) / self.ros_laser_scan.angle_increment
		)
			
	def sample_at(self, angle: float) -> float:
		index: int = self.angle_to_index(angle)
		return self.ros_laser_scan.ranges[index]
	
	def sample_as_vector_at(self, angle: float) -> Vector3:
		magnitude: float = self.sample_at(angle)
		if magnitude == float('inf'):
			magnitude = self.ros_laser_scan.range_max
		return Vector3.from_yaw_pitch_length(yaw=angle, length=magnitude)
		
