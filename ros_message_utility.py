from geometry_msgs.msg import Pose as ROSPose, Point as ROSPoint, Quaternion as ROSQuaternion, Vector3 as ROSVector3, Twist as ROSTwist
from nav_msgs.msg import Odometry as ROSOdometry
from typing import Tuple

def decompose_odometry(odometry: ROSOdometry) -> Tuple[ROSPoint, ROSQuaternion]:
	orientation: ROSQuaternion = odometry.pose.pose.orientation
	position: ROSPoint = odometry.pose.pose.position
	return (position, orientation)
	
def compose_pose(decomposed_pose: Tuple[ROSPoint, ROSQuaternion]) -> ROSPose:
	position, orientation = decomposed_pose
	return ROSPose(position, orientation)
	
def decompose_pose(pose: ROSPose) -> Tuple[ROSPoint, ROSQuaternion]:
	return (pose.position, pose.orientation)
	
def compose_twist(linear: ROSVector3, angular: ROSVector3) -> ROSTwist:
	return ROSTwist(linear, angular)
