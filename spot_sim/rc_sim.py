import logging
import os
import threading
import time

import PySimpleGUI as sg


class RCSimThread(threading.Thread):
    def __init__(self):
        super().__init__()
        self.running = True
        self.log = logging.getLogger(__name__)
        self.window = None

    def run(self) -> None:
        sg.theme('DarkAmber')  # Add a touch of color
        # All the stuff inside your window.
        text_width = 5
        center_pwm = 980
        half_pwm_range = 800
        layout = [
            [
                sg.Text('Arm', size=(text_width, 0)),
                sg.Checkbox('Armed', key="arm"),
            ],
            [
                sg.Text('Mode', size=(text_width, 0)),
                sg.Slider(
                    range=(0, 2),
                    default_value=0,
                    size=(20, 15),
                    orientation='horizontal',
                    font=('Helvetica', 12),
                    key="mode"
                )

            ],
            [
                sg.Text('Pitch', size=(text_width, 0)),
                sg.Slider(
                    range=(center_pwm - half_pwm_range, center_pwm +  half_pwm_range),
                    default_value=center_pwm,
                    size=(20, 15),
                    orientation='horizontal',
                    font=('Helvetica', 12),
                    key="pitch"
                )

            ],
            [
                sg.Text('Roll', size=(text_width, 0)),
                sg.Slider(
                    range=(center_pwm - half_pwm_range, center_pwm +  half_pwm_range),
                    default_value=center_pwm,
                    size=(20, 15),
                    orientation='horizontal',
                    font=('Helvetica', 12),
                    key="roll"
                )

            ],
            [
                sg.Text('Throttle', size=(text_width, 0)),
                sg.Slider(
                    range=(center_pwm - half_pwm_range, center_pwm +  half_pwm_range),
                    default_value=center_pwm,
                    size=(20, 15),
                    orientation='horizontal',
                    font=('Helvetica', 12),
                    key="throttle"
                )

            ]
        ]

        file_path = os.path.join(os.path.dirname(__file__), "spot_sim_data.txt")

        # Create the Window
        self.window = sg.Window(
            'RC Controller Simulator',
            layout,
            relative_location=(660, -300)
        )
        # Event Loop to process "events" and get the "values" of the inputs

        # Truncate the rc input file
        with open(file_path, "w") as rc_input_fp:
            rc_input_fp.write("")

        while self.running:
            event, values = self.window.read(timeout=1000)

            if event is sg.WIN_CLOSED:  # if user closes window or clicks cancel
                self.log.debug('RC Simulator Window Closed')
                self.running = False

            if isinstance(values, dict):
                self.log.debug('RC Simulator Simulating: {}'.format(values))

                arm_pwm = "501" if values["arm"] else "500"  # RC 6
                if values["mode"] == 0.0:  # RC 7
                    mode_pwm = "499"
                elif values["mode"] == 1.0:
                    mode_pwm = "501"
                elif values["mode"] == 2.0:
                    mode_pwm = "1501"
                else:
                    raise KeyError('Unknown mode state')

                with open(file_path, "a") as rc_input_fp:
                    rc_input_fp.write("\x02{}\x03\r\n".format("\t".join([
                        "2",  # packet type 2 is RC input
                        str(int(values["pitch"])),  # 0th RC channel Y command
                        str(int(values["roll"])),  # X command
                        str(int(values["throttle"])),  # Z command
                        "3",
                        "4",
                        "5",
                        arm_pwm,  # Arm spot
                        mode_pwm,  # Mode Control
                        "8",
                        "9",
                        "10",
                        "11",
                        "12",
                        "13",
                        "14",
                        "15",
                        "16",
                        "17",
                        "18",
                    ])))
        self.log.debug('RC Simulator Terminated')

    def kill(self):
        self.running = False


if __name__ == '__main__':
    logging.basicConfig(
        level=logging.DEBUG,
        format='%(asctime)s.%(msecs)03d %(name)s %(levelname)s: %(message)s',
        datefmt='%Y-%m-%d,%H:%M:%S'
    )
    rc_sim_thread = RCSimThread()
    rc_sim_thread.start()

    try:
        while rc_sim_thread.running:
            time.sleep(0.1)
    except KeyboardInterrupt as e:
        rc_sim_thread.kill()
        rc_sim_thread.join()
