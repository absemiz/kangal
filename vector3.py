from math import atan2, sqrt, sin, cos, acos, pi
from math_utility import float_equals
from geometry_msgs.msg import Vector3 as ROSVector3

class Vector3:
	def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0) -> None:
		self.x = x
		self.y = y
		self.z = z

	def __add__(self, other: 'Vector3') -> 'Vector3':
		return Vector3(
			self.x + other.x,
			self.y + other.y,
			self.z + other.z
		)

	def __str__(self) -> str:
		as_string: str = f'<{self.x:.2f}, {self.y:.2f}, {self.z:.2f}>'
		return as_string

	def __sub__(self, other: 'Vector3') -> 'Vector3':
		return self + (-other)

	def __eq__(self, other: 'Vector3') -> bool:
		return self.equals(other)

	def __neg__(self) -> 'Vector3':
		return self * (-1.0)

	def __mul__(self, scalar: float) -> 'Vector3':
		return Vector3(
			self.x * scalar,
			self.y * scalar,
			self.z * scalar
		)

	def __rmul__(self, scalar: float) -> 'Vector3':
		return self.__mul__(scalar)

	def __truediv__(self, scalar: float) -> 'Vector3':
		if scalar == 0.0:
			raise ZeroDivisionError
		return self * (1 / scalar)

	def dot(self, other: 'Vector3') -> float:
		return (
				self.x * other.x +
				self.y * other.y +
				self.z * other.z
		)

	def cross(self, other: 'Vector3') -> 'Vector3':
		return Vector3(
			self.y * other.z - self.z * other.y,
			self.z * other.x - self.x * other.z,
			self.x * other.y - self.y * other.x
		)

	def length(self) -> float:
		return sqrt(self.dot(self))

	def unit(self) -> 'Vector3':
		if self.length() == 0.0:
			return Vector3()
		return self / self.length()

	def pitch(self) -> float:
		return atan2(self.z, sqrt(self.x ** 2 + self.y ** 2))

	def yaw(self) -> float:
		return atan2(self.y, self.x)

	def angle_between(self, other: 'Vector3') -> float:
		if self.length() * other.length() == 0.0:
			return 0.0
		return acos(self.dot(other) / (self.length() * other.length()))

	def equals(self, other: 'Vector3', tolerance: float = 1E-1):
		return (
				float_equals(self.x, other.x, tolerance) and
				float_equals(self.y, other.y, tolerance) and
				float_equals(self.z, other.z, tolerance)
		)
	
	def to_ros_message(self) -> ROSVector3:
		return ROSVector3(self.x, self.y, self.z)

	@classmethod
	def from_yaw_pitch_length(
			cls,
			yaw: float = 0.0,
			pitch: float = 0.0,
			length: float = 1.0
	) -> 'Vector3':
		x: float = length * cos(pitch) * cos(yaw)
		y: float = length * cos(pitch) * sin(yaw)
		z: float = length * sin(pitch)
		return cls(x, y, z)
		
	@classmethod
	def zero(cls) -> 'Vector3':
		return cls(0.0, 0.0, 0.0)
