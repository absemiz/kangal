
def float_equals(a: float, b: float, epsilon: float = 1E-1):
	return abs(a - b) < epsilon

def signum(f: float) -> float:
	if f == 0.0: 
		return 0.0
	return f / abs(f)