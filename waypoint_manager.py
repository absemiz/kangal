#!/usr/bin/env python

from sensor_msgs.msg import LaserScan as ROSLaserScan
from geometry_msgs.msg import Point as ROSPoint, Quaternion as ROSQuaternion, Pose as ROSPose
from nav_msgs.msg import Odometry as ROSOdometry
from laser_scan import LaserScan
from vector3 import Vector3
from quaternion import Quaternion
from math import pi, degrees
from math_utility import signum
from ros_message_utility import decompose_odometry, compose_pose
from typing import Tuple, Callable
from point import Point
from rospy import init_node, is_shutdown, Subscriber, Publisher

class WaypointManager:
	ODOMETRY_TOPIC: str = '/r2d2_diff_drive_controller/odom'
	LASER_SCAN_TOPIC: str = '/laser/scan'
	WAYPOINT_TOPIC: str = '/waypoint/next'
	NODE_NAME: str = 'waypoint_manager'
	def __init__(self):
		Subscriber(WaypointManager.LASER_SCAN_TOPIC, ROSLaserScan, self.on_laser_scan_message_received)
		Subscriber(WaypointManager.ODOMETRY_TOPIC, ROSOdometry, self.on_odometry_message_received)
		self.waypoint_publisher: Publisher = Publisher(WaypointManager.WAYPOINT_TOPIC, ROSPoint, queue_size=1)
		
		self.last_laser_scan: ROSLaserScan = ROSLaserScan()
		self.last_odometry: ROSOdometry = ROSOdometry()

		self.laser_scan_available: bool = False
		self.odometry_available: bool = False
		
		
	def on_laser_scan_message_received(self, ros_laser_scan: ROSLaserScan):
		self.last_laser_scan = ros_laser_scan
		self.laser_scan_available = True
		
	def on_odometry_message_received(self, odometry: ROSOdometry):
		self.last_odometry = odometry
		self.odometry_available = True

	def available(self) -> bool:
		return self.laser_scan_available and self.odometry_available
		
	def evaluate_next_waypoint(self, on_next_point_evaluated: Callable[[Vector3], None]):
		laser_scan: LaserScan = LaserScan(self.last_laser_scan)
		
		scan_start_angle: float = -pi / 2
		scan_end_angle: float = pi / 2
		
		angle_iterator: float = scan_start_angle
		angle_increment: float = self.last_laser_scan.angle_increment
		
		repelling_radius: float = self.last_laser_scan.range_max * 0.1
		
		waypoint_magnitude_constant: float = 1.0
		next_waypoint: Vector3 = Vector3()
		
		while angle_iterator <= scan_end_angle:
			sample_vector: Vector3 = laser_scan.sample_as_vector_at(angle_iterator)
			
			waypoint_component_magnitude: float = abs(sample_vector.length() - repelling_radius) * (self.last_laser_scan.range_max / repelling_radius)
			waypoint_component_angle: float = sample_vector.yaw() if sample_vector.length() > repelling_radius else sample_vector.yaw() - signum(sample_vector.yaw()) * (pi / 2)
			waypoint_component: Vector3 = Vector3.from_yaw_pitch_length(yaw=waypoint_component_angle, length=waypoint_component_magnitude)

			next_waypoint += waypoint_component
			
			angle_iterator += angle_increment
		next_waypoint = 2.0 * next_waypoint.unit()
		on_next_point_evaluated(next_waypoint)
		
	def globalize(self, local_vector: Vector3) -> Tuple[ROSPoint, ROSQuaternion]:
		ros_position, ros_orientation = decompose_odometry(self.last_odometry)
		
		orientation: Quaternion = Quaternion.from_ros_message(ros_orientation)
		orientation = orientation.projection2D()
		
		angle: float = local_vector.yaw()
		rotation_quaternion: Quaternion = Quaternion.from_yaw(angle)
		target_quaternion: Quaternion = rotation_quaternion * orientation

		current_position: Point = Point.from_ros_message(ros_position)
		target_position: Vector3 = current_position + local_vector
		target_point: Point = Point(target_position.x, target_position.y, target_position.z)
		
		return (target_point.to_ros_message(), target_quaternion.to_ros_message())
		
	def publish_waypoint(self, waypoint: Vector3):
		target_waypoint: Point = Point(waypoint.x, waypoint.y, waypoint.z)
		self.waypoint_publisher(target_waypoint.to_ros_message())
		
	@staticmethod
	def operate():
		init_node(WaypointManager.NODE_NAME)
		waypoint_manager: WaypointManager = WaypointManager()
		while not is_shutdown():
			if waypoint_manager.available():
				waypoint_manager.evaluate_next_waypoint(waypoint_manager.publish_waypoint)
			
if __name__ == '__main__':
	WaypointManager.operate()
		
