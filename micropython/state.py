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

    def update(self, profiler, rc, next_loop_ms):

        if self.state == self.INIT:
            if rc is not None:
                if rc[6] > 1500:
                    self.state = self.TURNING_SERVOS_ON
                    self.last_state_transition_us = next_loop_ms

        if self.state == self.TURNING_SERVOS_ON:
            if self.sub_state == 0:
                profiler.add_position_target(
                    front_right_shoulder=40,
                    front_left_shoulder=40,
                )
                self.sub_state = 1
            elif (self.sub_state == 1) and ((next_loop_ms - self.last_state_transition_us) > 2e6):
                profiler.add_position_target(
                    front_right_leg=-60,
                    front_left_leg=-60,
                )
                self.sub_state = 2
            elif (self.sub_state == 2) and ((next_loop_ms - self.last_state_transition_us) > 4e6):
                profiler.add_position_target(
                    front_right_foot=140,
                    front_left_foot=140
                )
                self.sub_state = 3
            elif (self.sub_state == 3) and ((next_loop_ms - self.last_state_transition_us) > 6e6):
                self.state = self.DOWN
                self.last_state_transition_us = next_loop_ms

        if self.state == self.DOWN:
            if rc is not None:
                if rc[7] > 500:
                    profiler.add_position_target(  # chicken straight shoulder
                        front_right_shoulder=40,
                        front_left_shoulder=40,
                        front_right_leg=-90,
                        front_left_leg=-90,
                        front_right_foot=140,
                        front_left_foot=140
                    )
                    profiler.add_position_target(  # sphinx
                        front_right_shoulder=0,
                        front_left_shoulder=0,
                        front_right_leg=-50,
                        front_left_leg=-50,
                        front_right_foot=110,
                        front_left_foot=110
                    )
                    self.state = self.MOVING_UP
                    self.last_state_transition_us = next_loop_ms

        if self.state == self.MOVING_UP:
            if len(profiler.position_target_queue) == 0:
                self.state = self.UP
                self.last_state_transition_us = next_loop_ms

        if self.state == self.UP:
            if rc is not None:
                if rc[7] < 500:
                    profiler.add_position_target(  # chicken straight shoulder
                        front_right_shoulder=40,
                        front_left_shoulder=40,
                        front_right_leg=-90,
                        front_left_leg=-90,
                        front_right_foot=140,
                        front_left_foot=140
                    )
                    profiler.add_position_target(
                        front_right_shoulder=40,
                        front_left_shoulder=40,
                        front_right_leg=-60,
                        front_left_leg=-60,
                        front_right_foot=140,
                        front_left_foot=140
                    )
                    self.state = self.MOVING_DOWN
                    self.last_state_transition_us = next_loop_ms

        if self.state == self.MOVING_DOWN:
            if len(profiler.position_target_queue) == 0:
                self.state = self.DOWN
                self.last_state_transition_us = next_loop_ms
