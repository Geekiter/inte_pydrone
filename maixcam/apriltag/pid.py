class PID:
    def __init__(self, p, i, d, v_max, v_min, target):
        self.p = p
        self.i = i
        self.d = d
        self.last_error = 0
        self.integral = 0
        self.v_max = v_max
        self.v_min = v_min
        self.target = target

    def update(self, current_value):
        error = self.target - current_value
        self.integral += error
        derivative = error - self.last_error
        self.last_error = error
        output = self.p * error + self.i * self.integral + self.d * derivative
        output = max(min(output, self.v_max), self.v_min)
        return output
