#!/usr/bin/env python
from vector3 import Vector3
from quaternion import Quaternion
from point import Point
from nav_msgs.msg import Odometry as ROSOdometry
from geometry_msgs.msg import Quaternion as ROSQuaternion, Vector3 as ROSVector3, Point as ROSPoint, Pose as ROSPose, Twist as ROSTwist
from math_utility import float_equals, signum
from ros_message_utility import decompose_odometry, compose_pose, decompose_pose, compose_twist
from math import pi, degrees
from typing import Tuple, Callable

from rospy import init_node, is_shutdown, Subscriber, Publisher

class MovementManager:
	MOVEMENT_CONTROLLER_TOPIC: str = '/r2d2_diff_drive_controller/cmd_vel'
	ODOMETRY_TOPIC: str = '/r2d2_diff_drive_controller/odom'
	WAYPOINT_TOPIC: str = '/waypoint/next'
	NODE_NAME: str = 'movement_manager'
	def __init__(self):
		Subscriber(MovementManager.ODOMETRY_TOPIC, ROSOdometry, self.on_odometry_message_received)
		Subscriber(MovementManager.WAYPOINT_TOPIC, ROSPoint, self.on_waypoint_message_received)

		self.movement_controller: Publisher = Publisher(MovementManager.MOVEMENT_CONTROLLER_TOPIC, ROSTwist, queue_size=1)

		self.linear_speed_constant: float = 1.0#m/s
		self.angular_speed_constant: float = 4.0#rad/s
		self.angle_tolerance: float = pi / 120#rad
		self.stop_angle: float = pi / 6
		self.position_tolerance: float = 0.5#m

		self.current_position: ROSPoint = ROSPoint()
		self.current_orientation: ROSQuaternion = ROSQuaternion()

		self.target_position: ROSPoint = ROSPoint(5, 0, 0)	            #Initial state
		self.target_orientation: ROSQuaternion = self.current_orientation   #Initial state

	def on_odometry_message_received(self, odometry: ROSOdometry):
		self.current_position, self.current_orientation = decompose_odometry(odometry)

	def on_waypoint_message_received(self, waypoint: ROSPoint):
		if self.waypoint_reached():
			self.target_position = waypoint

	def stop(self):
		self.publish_movement(Vector3.zero(), Vector3.zero())

	def waypoint_reached(self) -> bool:
		current: Point = Point.from_ros_message(self.current_position)
		target: Point = Point.from_ros_message(self.target_position)
		# DEBUG START
		print(f'Current Position: {current}')
		print(f'Target Position: {target}')
		print(f'Waypoint: {target - current}, Length: {(target - current).length()}')
		# DEBUG END
		return (target - current).length() < self.position_tolerance

	def publish_movement(self, linear: Vector3, angular: Vector3) -> None:
		movement_twist: ROSTwist = compose_twist(linear.to_ros_message(), angular.to_ros_message())
		self.movement_controller.publish(movement_twist)

	def evaluate_movement_vectors2(self, on_movement_vectors_evaluated: Callable[[Vector3, Vector3], None]) -> None:
		if self.waypoint_reached():
			self.stop()
			return
		current_position: Point = Point.from_ros_message(self.current_position)
		target_position: Point = Point.from_ros_message(self.target_position)
		waypoint = target_position - current_position

		current_orientation: Quaternion = Quaternion.from_ros_message(self.current_orientation)
		heading_vector: Vector3 = current_orientation.rotation_operator(Vector3(x = 1.0)).unit()

		turn_sign: float = signum(current_position.cross(target_position).z)

		if not float_equals(waypoint.unit().dot(heading_vector), 1.0, epsilon = 0.05):
			linear_velocity: Vector3 = Vector3.zero()
			angular_velocity: Vector3 = Vector3(z = turn_sign * 2.0)
			on_movement_vectors_evaluated(linear_velocity, angular_velocity)
		else:
			linear_velocity: Vector3 = Vector3(x = 1.0)
			angular_velocity: Vector3 = Vector3.zero()
			on_movement_vectors_evaluated(linear_velocity, angular_velocity)


	def evaluate_movement_vectors(
		self,
		on_movement_vectors_evaluated: Callable[[Vector3, Vector3], None]
	) -> Tuple[Vector3, Vector3]:
		current: Quaternion = Quaternion.from_ros_message(self.current_orientation).projection2D()
		target: Quaternion = Quaternion.from_ros_message(self.target_orientation)
		# DEBUG START
		current_angle_message: str = f'Current Angle: {degrees(current.rotation_angle()):.2f}'
		target_angle_message: str = f'Target Angle: {degrees(target.rotation_angle()):.2f}'
		print(current_angle_message)
		print(target_angle_message)
		# DEBUG END
		if float_equals(current.rotation_angle(), target.rotation_angle(), epsilon = self.angle_tolerance):
			linear: Vector3 =  Vector3(self.linear_speed_constant, 0.0, 0.0)
			angular: Vector3 = Vector3.zero()
			on_movement_vectors_evaluated(linear, angular)
		else:
			linear: Vector3 = Vector3.zero()
			angular: Vector3 = Vector3(0.0, 0.0, signum(target.rotation_angle() - current.rotation_angle()) * self.angular_speed_constant)
			on_movement_vectors_evaluated(linear, angular)

	@staticmethod
	def operate():
		init_node(MovementManager.NODE_NAME)
		movement_manager: MovementManager = MovementManager()
		while not is_shutdown():
			movement_manager.evaluate_movement_vectors2(movement_manager.publish_movement)

if __name__ == '__main__':
	MovementManager.operate()
