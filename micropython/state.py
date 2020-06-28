
DEBUG = False

try:
    # noinspection PyUnresolvedReferences
    import utime
    # noinspection PyUnresolvedReferences
    ticks_diff = utime.ticks_diff
except ImportError:
    def ticks_diff(ticks1, ticks2):
        """ Overwrite MicroPython ticks diff function with one that will work on the PC target. """
        return ticks1 - ticks2


class StateMachine:
    INIT = 0
    TURNING_SERVOS_ON = 1
    MOVING_DOWN = 3
    DOWN = 4
    MOVING_UP = 5
    UP = 6

    def __init__(self):
        self.sub_state = 0
        self.state = self.INIT
        self.last_state_transition_us = None

    def update(self, profiler, rc, last_loop_ms):
        if self.state == self.INIT:
            transition = False
            if rc is not None:
                if rc[6] > 500:
                    transition = True
            if transition or DEBUG:
                self.state = self.TURNING_SERVOS_ON
                self.last_state_transition_us = last_loop_ms

        if self.state == self.TURNING_SERVOS_ON:
            if self.sub_state == 0:
                profiler.servos_init = True
                self.sub_state = 1
            elif (self.sub_state == 1) and (ticks_diff(last_loop_ms, self.last_state_transition_us) > 2e6):
                self.state = self.DOWN
                self.last_state_transition_us = last_loop_ms

        if self.state == self.DOWN:
            transition = False
            if rc is not None:
                if rc[7] > 500:
                    transition = True
            if transition or DEBUG:
                profiler.add_position_target(
                    front_right_x=0, front_right_y=-30, front_right_z=100,
                    front_left_x=0, front_left_y=-30, front_left_z=100,
                    rear_right_x=0, rear_right_y=-30, rear_right_z=100,
                    rear_left_x=0, rear_left_y=-30, rear_left_z=100,
                )
                profiler.add_position_target(
                    front_right_x=0, front_right_y=-30, front_right_z=140,
                    front_left_x=0, front_left_y=-30, front_left_z=140,
                    rear_right_x=0, rear_right_y=-30, rear_right_z=140,
                    rear_left_x=0, rear_left_y=-30, rear_left_z=140,
                )
                self.state = self.MOVING_UP
                self.last_state_transition_us = last_loop_ms

        if self.state == self.MOVING_UP:
            if profiler.get_motion_complete():
                self.state = self.UP
                self.sub_state = 0
                self.last_state_transition_us = last_loop_ms

        if self.state == self.UP:
            transition = False
            if rc is not None:
                if rc[7] < 500:
                    transition = True
                else:
                    x_command = ((rc[1] - 980) / 800) * -50
                    if abs(x_command) < 5:
                        x_command = 0
                    y_command = ((rc[0] - 980) / 800) * -40
                    if abs(y_command) < 5:
                        y_command = 0
                    z_command = ((rc[2] - 980) / 800) * 15
                    if abs(z_command) < 5:
                        z_command = 0

                    if rc[7] > 1500:  # walk mode
                        self.walk(profiler, y_command)
                    else:  # translational axes
                        if profiler.get_motion_complete():
                            profiler.add_position_target(  # sphinx
                                front_right_x=x_command, front_right_y=y_command - 30, front_right_z=140 + z_command,
                                front_left_x=x_command, front_left_y=y_command - 30, front_left_z=140 + z_command,
                                rear_right_x=x_command, rear_right_y=y_command - 30, rear_right_z=140 + z_command,
                                rear_left_x=x_command, rear_left_y=y_command - 30, rear_left_z=140 + z_command,
                            )
            if transition or DEBUG:
                profiler.add_position_target(
                    front_right_x=0, front_right_y=-30, front_right_z=100,
                    front_left_x=0, front_left_y=-30, front_left_z=100,
                    rear_right_x=0, rear_right_y=-30, rear_right_z=100,
                    rear_left_x=0, rear_left_y=-30, rear_left_z=100,
                )
                profiler.add_position_target(
                    front_right_x=profiler.start_pose_x, front_right_y=profiler.start_pose_y, front_right_z=profiler.start_pose_z,
                    front_left_x=-profiler.start_pose_x, front_left_y=profiler.start_pose_y, front_left_z=profiler.start_pose_z,
                    rear_right_x=profiler.start_pose_x, rear_right_y=profiler.start_pose_y, rear_right_z=profiler.start_pose_z,
                    rear_left_x=-profiler.start_pose_x, rear_left_y=profiler.start_pose_y, rear_left_z=profiler.start_pose_z,
                )
                self.state = self.MOVING_DOWN
                self.last_state_transition_us = last_loop_ms

        if self.state == self.MOVING_DOWN:
            if profiler.get_motion_complete():
                self.state = self.DOWN
                self.last_state_transition_us = last_loop_ms

    def walk(self, profiler, walk_velocity: float):
        if (walk_velocity < 0) or (self.sub_state != 0):
            if profiler.get_motion_complete():

                leg_y_step = 110
                body_y_step = 54
                leg_z = 155
                leg_z_step = 35

                if self.sub_state == 0:  # shift body weight backwards while moving front leg
                    profiler.set_velocity_and_acceleration()
                    leg_y = 0 * body_y_step
                    profiler.add_position_target(
                        front_right_x=0, front_right_y=leg_y, front_right_z=leg_z,
                        front_left_x=0, front_left_y=leg_y, front_left_z=leg_z,
                        rear_right_x=0, rear_right_y=leg_y, rear_right_z=leg_z,
                        rear_left_x=0, rear_left_y=leg_y, rear_left_z=leg_z,
                    )
                    self.sub_state += 1
                elif self.sub_state == 1:  # front left lift
                    profiler.set_velocity_and_acceleration(40, 2)
                    leg_y = 0 * body_y_step
                    profiler.add_position_target(
                        front_right_x=0, front_right_y=leg_y, front_right_z=leg_z,
                        front_left_x=0, front_left_y=leg_y, front_left_z=leg_z - leg_z_step,
                        rear_right_x=0, rear_right_y=leg_y, rear_right_z=leg_z,
                        rear_left_x=0, rear_left_y=leg_y, rear_left_z=leg_z,
                    )
                    self.sub_state += 1
                elif self.sub_state == 2:  # front left forward
                    leg_y = 0 * body_y_step
                    profiler.add_position_target(
                        front_right_x=0, front_right_y=leg_y, front_right_z=leg_z,
                        front_left_x=0, front_left_y=leg_y + leg_y_step, front_left_z=leg_z - leg_z_step,
                        rear_right_x=0, rear_right_y=leg_y, rear_right_z=leg_z,
                        rear_left_x=0, rear_left_y=leg_y, rear_left_z=leg_z,
                    )
                    self.sub_state += 1
                elif self.sub_state == 3:  # front left lower
                    leg_y = 0 * body_y_step
                    profiler.add_position_target(
                        front_right_x=0, front_right_y=leg_y, front_right_z=leg_z,
                        front_left_x=0, front_left_y=leg_y + leg_y_step, front_left_z=leg_z,
                        rear_right_x=0, rear_right_y=leg_y, rear_right_z=leg_z,
                        rear_left_x=0, rear_left_y=leg_y, rear_left_z=leg_z,
                    )
                    self.sub_state += 1

                elif self.sub_state == 4:  # shift body weight forward while moving rear leg
                    profiler.set_velocity_and_acceleration()
                    leg_y = -1 * body_y_step
                    profiler.add_position_target(
                        front_right_x=0, front_right_y=leg_y, front_right_z=leg_z,
                        front_left_x=0, front_left_y=leg_y + leg_y_step, front_left_z=leg_z,
                        rear_right_x=0, rear_right_y=leg_y, rear_right_z=leg_z,
                        rear_left_x=0, rear_left_y=leg_y, rear_left_z=leg_z,
                    )
                    self.sub_state += 1
                elif self.sub_state == 5:  # rear right lift
                    profiler.set_velocity_and_acceleration(40, 2)
                    leg_y = -1 * body_y_step
                    profiler.add_position_target(
                        front_right_x=0, front_right_y=leg_y, front_right_z=leg_z,
                        front_left_x=0, front_left_y=leg_y + leg_y_step, front_left_z=leg_z,
                        rear_right_x=0, rear_right_y=leg_y, rear_right_z=leg_z - leg_z_step,
                        rear_left_x=0, rear_left_y=leg_y, rear_left_z=leg_z,
                    )
                    self.sub_state += 1
                elif self.sub_state == 6:  # rear right forward
                    leg_y = -1 * body_y_step
                    profiler.add_position_target(
                        front_right_x=0, front_right_y=leg_y, front_right_z=leg_z,
                        front_left_x=0, front_left_y=leg_y + leg_y_step, front_left_z=leg_z,
                        rear_right_x=0, rear_right_y=leg_y + leg_y_step, rear_right_z=leg_z - leg_z_step,
                        rear_left_x=0, rear_left_y=leg_y, rear_left_z=leg_z,
                    )
                    self.sub_state += 1
                elif self.sub_state == 7:  # rear right lower
                    leg_y = -1 * body_y_step
                    profiler.add_position_target(
                        front_right_x=0, front_right_y=leg_y, front_right_z=leg_z,
                        front_left_x=0, front_left_y=leg_y + leg_y_step, front_left_z=leg_z,
                        rear_right_x=0, rear_right_y=leg_y + leg_y_step, rear_right_z=leg_z,
                        rear_left_x=0, rear_left_y=leg_y, rear_left_z=leg_z,
                    )
                    self.sub_state += 1

                elif self.sub_state == 8:  # shift body weight backward while moving front leg
                    profiler.set_velocity_and_acceleration()
                    leg_y = -2 * body_y_step
                    profiler.add_position_target(
                        front_right_x=0, front_right_y=leg_y, front_right_z=leg_z,
                        front_left_x=0, front_left_y=leg_y + leg_y_step, front_left_z=leg_z,
                        rear_right_x=0, rear_right_y=leg_y + leg_y_step, rear_right_z=leg_z,
                        rear_left_x=0, rear_left_y=leg_y, rear_left_z=leg_z,
                    )
                    self.sub_state += 1
                elif self.sub_state == 9:  # front right lift
                    profiler.set_velocity_and_acceleration(40, 2)
                    leg_y = -2 * body_y_step
                    profiler.add_position_target(
                        front_right_x=0, front_right_y=leg_y, front_right_z=leg_z - leg_z_step,
                        front_left_x=0, front_left_y=leg_y + leg_y_step, front_left_z=leg_z,
                        rear_right_x=0, rear_right_y=leg_y + leg_y_step, rear_right_z=leg_z,
                        rear_left_x=0, rear_left_y=leg_y, rear_left_z=leg_z,
                    )
                    self.sub_state += 1
                elif self.sub_state == 10:  # front right forward
                    leg_y = -2 * body_y_step
                    profiler.add_position_target(
                        front_right_x=0, front_right_y=leg_y + leg_y_step, front_right_z=leg_z - leg_z_step,
                        front_left_x=0, front_left_y=leg_y + leg_y_step, front_left_z=leg_z,
                        rear_right_x=0, rear_right_y=leg_y + leg_y_step, rear_right_z=leg_z,
                        rear_left_x=0, rear_left_y=leg_y, rear_left_z=leg_z,
                    )
                    self.sub_state += 1
                elif self.sub_state == 11:  # front right lower
                    leg_y = -2 * body_y_step
                    profiler.add_position_target(
                        front_right_x=0, front_right_y=leg_y + leg_y_step, front_right_z=leg_z,
                        front_left_x=0, front_left_y=leg_y + leg_y_step, front_left_z=leg_z,
                        rear_right_x=0, rear_right_y=leg_y + leg_y_step, rear_right_z=leg_z,
                        rear_left_x=0, rear_left_y=leg_y, rear_left_z=leg_z,
                    )
                    self.sub_state += 1

                elif self.sub_state == 12:  # shift body weight forward while moving rear leg
                    profiler.set_velocity_and_acceleration()
                    leg_y = -3 * body_y_step
                    profiler.add_position_target(
                        front_right_x=0, front_right_y=leg_y + leg_y_step, front_right_z=leg_z,
                        front_left_x=0, front_left_y=leg_y + leg_y_step, front_left_z=leg_z,
                        rear_right_x=0, rear_right_y=leg_y + leg_y_step, rear_right_z=leg_z,
                        rear_left_x=0, rear_left_y=leg_y, rear_left_z=leg_z,
                    )
                    self.sub_state += 1
                elif self.sub_state == 13:  # rear left lift
                    profiler.set_velocity_and_acceleration(40, 2)
                    leg_y = -3 * body_y_step
                    profiler.add_position_target(
                        front_right_x=0, front_right_y=leg_y + leg_y_step, front_right_z=leg_z,
                        front_left_x=0, front_left_y=leg_y + leg_y_step, front_left_z=leg_z,
                        rear_right_x=0, rear_right_y=leg_y + leg_y_step, rear_right_z=leg_z,
                        rear_left_x=0, rear_left_y=leg_y, rear_left_z=leg_z - leg_z_step,
                    )
                    self.sub_state += 1
                elif self.sub_state == 14:  # rear left forward
                    leg_y = -3 * body_y_step
                    profiler.add_position_target(
                        front_right_x=0, front_right_y=leg_y + leg_y_step, front_right_z=leg_z,
                        front_left_x=0, front_left_y=leg_y + leg_y_step, front_left_z=leg_z,
                        rear_right_x=0, rear_right_y=leg_y + leg_y_step, rear_right_z=leg_z,
                        rear_left_x=0, rear_left_y=leg_y + leg_y_step, rear_left_z=leg_z - leg_z_step,
                    )
                    self.sub_state += 1
                elif self.sub_state == 15:  # rear left lower
                    leg_y = -3 * body_y_step
                    profiler.add_position_target(
                        front_right_x=0, front_right_y=leg_y + leg_y_step, front_right_z=leg_z,
                        front_left_x=0, front_left_y=leg_y + leg_y_step, front_left_z=leg_z,
                        rear_right_x=0, rear_right_y=leg_y + leg_y_step, rear_right_z=leg_z,
                        rear_left_x=0, rear_left_y=leg_y + leg_y_step, rear_left_z=leg_z,
                    )
                    self.sub_state += 1

                elif self.sub_state == 16:  # back to starting pose
                    profiler.set_velocity_and_acceleration()
                    profiler.add_position_target(
                        front_right_x=0, front_right_y=-30, front_right_z=leg_z,
                        front_left_x=0, front_left_y=-30, front_left_z=leg_z,
                        rear_right_x=0, rear_right_y=-30, rear_right_z=leg_z,
                        rear_left_x=0, rear_left_y=-30, rear_left_z=leg_z,
                    )
                    self.sub_state = 0
                    for leg in [profiler.front_left, profiler.front_right, profiler.rear_left, profiler.rear_right]:
                        leg.velocity_max = leg.velocity_max_default
                        leg.acceleration = leg.acceleration_default

        # pose for trimming servos
        # profiler.add_position_target(
        #     front_right_x=0, front_right_y=115, front_right_z=120,
        #     front_left_x=0, front_left_y=115, front_left_z=120,
        #     rear_right_x=0, rear_right_y=115, rear_right_z=120,
        #     rear_left_x=0, rear_left_y=115, rear_left_z=120,
        # )
