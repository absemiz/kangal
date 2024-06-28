from geometry_msgs.msg import Quaternion as ROSQuaternion
from vector3 import Vector3
from math import acos, cos, sin, atan2, sqrt

class Quaternion:
	def __init__(self, w: float = 1.0, x: float = 0.0, y: float = 0.0, z: float = 0.0):
		self.scalar: float = w
		self.vector: Vector3 = Vector3(x, y, z)
		
	def __add__(self, other: 'Quaternion') -> 'Quaternion':
		scalar: float = self.scalar + other.scalar
		vector: Vector3 = self.vector + other.vector
		return Quaternion(scalar, vector.x, vector.y, vector.z)
		
	def __mul__(self, other: 'Quaternion') -> 'Quaternion':
		scalar: float = self.scalar * other.scalar - self.vector.dot(other.vector)
		vector: Vector3 = self.scalar * other.vector + other.scalar * self.vector + self.vector.cross(other.vector)
		return Quaternion(scalar, vector.x, vector.y, vector.z)

	def __rmul__(self, other: 'Quaternion') -> 'Quaternion':
		scalar: float = other.scalar * self.scalar - other.vector.dot(self.vector)
		vector: Vector3 = other.scalar * self.vector + self.scalar * other.vector + other.vector.cross(self.vector)
		return Quaternion(scalar, vector.x, vector.y, vector.z)
	
	def scale(self, s: float) -> 'Quaternion':
		scalar: float = s * self.scalar
		vector: Vector3 = s * self.vector
		return Quaternion(scalar, vector.x, vector.y, vector.z)
	
	def conjugate(self) -> 'Quaternion':
		vector: Vector3 = -self.vector
		return Quaternion(self.scalar, vector.x, vector.y, vector.z)
	
	def length(self) -> float:
		return sqrt(self.scalar ** 2 + self.vector.length() ** 2)
		
	def unit(self) -> 'Quaternion':
		if self.length() == 0.0:
			return Quaternion()
		return self.scale((1.0 / self.length()))
		
	def rotation_angle(self) -> float:
		unit: Quaternion = self.unit()
		return 2.0 * atan2(unit.vector.length(), unit.scalar)
		
	def rotation_operator(self, v: Vector3) -> Vector3:
		unit: Quaternion = self.unit()
		conjugate: Quaternion = unit.conjugate()
		vector_as_quaternion: Quaternion = Quaternion(0.0, v.x, v.y, v.z)
		result: Quaternion = unit * vector_as_quaternion * conjugate
		return Vector3(result.vector.x, result.vector.y, result.vector.z)
		
	def projection2D(self) -> 'Quaternion':
		unit_quaternion: 'Quaternion' = self.unit()
		return Quaternion(unit_quaternion.scalar, z=unit_quaternion.vector.z)
		
	@classmethod
	def from_yaw(cls, yaw: float) -> 'Quaternion':
		scalar: float = cos(yaw / 2.0)
		vector: Vector3 = sin(yaw / 2.0) * Vector3(0.0, 0.0, 1.0)
		return Quaternion(scalar, vector.x, vector.y, vector.z)
	
	@classmethod
	def from_ros_message(cls, q: ROSQuaternion) -> 'Quaternion':
		return cls(q.w, q.x, q.y, q.z)

	def to_ros_message(self) -> ROSQuaternion:
		return ROSQuaternion(self.vector.x, self.vector.y, self.vector.z, self.scalar)
