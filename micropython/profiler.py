class Profiler:
    def __init__(
            self,
            front_right_shoulder=None, front_left_shoulder=None,
            front_right_leg=None, front_left_leg=None,
            front_right_foot=None, front_left_foot=None,
            rear_right_shoulder=None, rear_left_shoulder=None,
            rear_right_leg=None, rear_left_leg=None,
            rear_right_foot=None, rear_left_foot=None,
    ):
        self.front_right_shoulder = Axis(front_right_shoulder)
        self.front_left_shoulder = Axis(front_left_shoulder)
        self.front_right_leg = Axis(front_right_leg)
        self.front_left_leg = Axis(front_left_leg)
        self.front_right_foot = Axis(front_right_foot)
        self.front_left_foot = Axis(front_left_foot)
        self.rear_right_shoulder = Axis(rear_right_shoulder)
        self.rear_left_shoulder = Axis(rear_left_shoulder)
        self.rear_right_leg = Axis(rear_right_leg)
        self.rear_left_leg = Axis(rear_left_leg)
        self.rear_right_foot = Axis(rear_right_foot)
        self.rear_left_foot = Axis(rear_left_foot)

        self.axes = [
            self.front_right_shoulder,
            self.front_left_shoulder,
            self.front_right_leg,
            self.front_left_leg,
            self.front_right_foot,
            self.front_left_foot,
            self.rear_right_shoulder,
            self.rear_left_shoulder,
            self.rear_right_leg,
            self.rear_left_leg,
            self.rear_right_foot,
            self.rear_left_foot,
        ]
        self.position_target_queue = []

    def update_position_targets(
            self,
            front_right_shoulder=None, front_left_shoulder=None,
            front_right_leg=None, front_left_leg=None,
            front_right_foot=None, front_left_foot=None,
            rear_right_shoulder=None, rear_left_shoulder=None,
            rear_right_leg=None, rear_left_leg=None,
            rear_right_foot=None, rear_left_foot=None,
    ):
        if front_right_shoulder is not None:
            self.front_right_shoulder.target_deg = front_right_shoulder
        if front_left_shoulder is not None:
            self.front_left_shoulder.target_deg = front_left_shoulder
        if front_right_leg is not None:
            self.front_right_leg.target_deg = front_right_leg
        if front_left_leg is not None:
            self.front_left_leg.target_deg = front_left_leg
        if front_right_foot is not None:
            self.front_right_foot.target_deg = front_right_foot
        if front_left_foot is not None:
            self.front_left_foot.target_deg = front_left_foot
        if rear_right_shoulder is not None:
            self.rear_right_shoulder.target_deg = rear_right_shoulder
        if rear_left_shoulder is not None:
            self.rear_left_shoulder.target_deg = rear_left_shoulder
        if rear_right_leg is not None:
            self.rear_right_leg.target_deg = rear_right_leg
        if rear_left_leg is not None:
            self.rear_left_leg.target_deg = rear_left_leg
        if rear_right_foot is not None:
            self.rear_right_foot.target_deg = rear_right_foot
        if rear_left_foot is not None:
            self.rear_left_foot.target_deg = rear_left_foot

    def add_position_target(
        self,
        front_right_shoulder=None, front_left_shoulder=None,
        front_right_leg=None, front_left_leg=None,
        front_right_foot=None, front_left_foot=None,
        rear_right_shoulder=None, rear_left_shoulder=None,
        rear_right_leg=None, rear_left_leg=None,
        rear_right_foot=None, rear_left_foot=None,
    ):
        self.position_target_queue.append((
          front_right_shoulder,
          front_left_shoulder,
          front_right_leg,
          front_left_leg,
          front_right_foot,
          front_left_foot,
          rear_right_shoulder,
          rear_left_shoulder,
          rear_right_leg,
          rear_left_leg,
          rear_right_foot,
          rear_left_foot,
        ))

    def get_position_commands(self):
        return [axis.command_deg for axis in self.axes]

    def tick(self):
        for axis in self.axes:
            axis.tick()

        if self.get_all_are_in_position() and self.position_target_queue:
            self.update_position_targets(*self.position_target_queue.pop(0))

    def get_all_are_in_position(self):
        all_in_position = True
        for axis in self.axes:
            if not axis.get_is_in_position():
                all_in_position = False
                break
        return all_in_position

    def get_motion_complete(self):
        return self.get_all_are_in_position() and (len(self.position_target_queue) == 0)


class Axis:
    def __init__(self, target_deg):
        self.target_deg = target_deg
        self.command_deg = target_deg
        self.command_deg_float = self.command_deg
        self.in_position = True

        self._velocity = 0.0
        self._velocity_max = 3.0
        self._acceleration = 0.005

    def tick(self):
        if (self.target_deg is not None) and (self.command_deg is None):
            # this is the first time this servo is being used (i.e. target is no longer None)
            self.command_deg = self.target_deg
            self.command_deg_float = self.target_deg
        elif self.target_deg is None:
            # this servo is not used yet
            self.command_deg = None
            self.command_deg_float = None

        if (self.target_deg is not None) and (self.command_deg is not None):  # only work on channel that are used
            remaining_distance = int(self.target_deg - self.command_deg_float)

            stop_distance = abs((self._velocity * self._velocity) / (2 * self._acceleration))

            if remaining_distance > 0:
                self.in_position = False
                if remaining_distance < stop_distance:  # need to slow down
                    self._velocity -= self._acceleration
                else:  # speed up towards target
                    self._velocity += self._acceleration
                if self._velocity > self._velocity_max:
                    self._velocity = self._velocity_max
                self.command_deg_float = self.command_deg_float + self._velocity
            elif remaining_distance < 0:
                self.in_position = False
                if abs(remaining_distance) < stop_distance:  # need to slow down
                    self._velocity += self._acceleration
                else:  # speed up towards target
                    self._velocity -= self._acceleration
                if self._velocity < -self._velocity_max:
                    self._velocity = -self._velocity_max
                self.command_deg_float = self.command_deg_float + self._velocity
            else:
                self.in_position = True
                self._velocity = 0.0
                self.command_deg_float = self.target_deg

            self.command_deg = int(self.command_deg_float)

    def get_is_in_position(self):
        return self.in_position
