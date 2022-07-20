import enum
import logging
import os
import math
import sys
import threading
import time

import pybullet
import pybullet_data
import rc_sim

sys.path.append(os.path.join(os.path.dirname(__file__), "..", "micropython"))

import state
from profiler import Profiler

logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s.%(msecs)03d %(name)s %(levelname)s: %(message)s',
    datefmt='%Y-%m-%d,%H:%M:%S'
)


class ReadSerialFileThread(threading.Thread):
    class SerialStates(enum.Enum):
        LOOKING_FOR_START = enum.auto()
        LOOKING_FOR_STOP = enum.auto()

    def __init__(self):
        super().__init__()
        self.running = True

        self.targets = [None] * 12
        self.rcs = None

        self.targets_lock = threading.Lock()
        self.rcs_lock = threading.Lock()
        self.log = logging.getLogger(__name__)

    def run(self) -> None:
        file_path = os.path.join(os.path.dirname(__file__), "spot_sim_data.txt")
        stat = os.stat(file_path)
        size = stat[6]
        file_check_read_count = 0
        with open(file_path, "rb") as file_pointer:
            file_pointer.seek(size)
            serial_state = self.SerialStates.LOOKING_FOR_START
            serial_buff = bytearray()
            while self.running:
                new_byte = file_pointer.read(1)
                if new_byte:
                    if serial_state == self.SerialStates.LOOKING_FOR_START:
                        if new_byte == b"\x02":
                            serial_buff = bytearray()
                            serial_state = self.SerialStates.LOOKING_FOR_STOP
                    elif serial_state == self.SerialStates.LOOKING_FOR_STOP:
                        if new_byte == b"\x02":  # shouldn't see start byte mid packet
                            serial_state = self.SerialStates.LOOKING_FOR_START
                        elif new_byte == b"\x03":
                            serial_state = self.SerialStates.LOOKING_FOR_START
                            packet = serial_buff.decode()
                            self.handle_packet(packet)
                        serial_buff += new_byte
                else:
                    time.sleep(0.01)

                if file_check_read_count > 100:
                    file_check_read_count = 0
                    new_stat = os.stat(file_path)
                    current_file_size = new_stat[6]
                    position_in_file = file_pointer.tell()
                    file_position_diff = current_file_size - position_in_file
                    if abs(file_position_diff) > 10000:
                        how_far_from_end_to_seek_bytes = 200
                        if current_file_size < 200:
                            how_far_from_end_to_seek_bytes = 0  # handle empty file
                        file_pointer.seek(current_file_size - how_far_from_end_to_seek_bytes)
                        self.log.warning("Current file size minus position in file was {}".format(file_position_diff))
                        serial_state = self.SerialStates.LOOKING_FOR_START

                file_check_read_count += 1

    def handle_packet(self, packet):
        self.log.debug("Handled RC packet: {}".format(packet))
        packet_type = int(packet.split("\t")[0])
        packet_split = packet.split("\t")[1:]
        if packet_type == 0x1:
            try:
                targets = []
                for target in packet_split[:12]:
                    if "None" in target:
                        targets.append(None)
                    else:
                        targets.append(int(target))
                self.set_targets(targets)
            except:
                self.log.warning("Failed to parse targets packet:\n{}\n".format(packet))
        elif packet_type == 0x2:
            try:
                rcs = []
                for rc in packet_split[:16]:
                    rcs.append(int(rc))
                for rc in packet_split[16:20]:
                    rcs.append(bool(rc))
                self.set_rcs(rcs)
            except:
                self.log.warning("Failed to parse rcs packet\n{}\n".format(packet))

    def set_targets(self, targets):
        with self.targets_lock:
            self.targets = targets

    def get_targets(self):
        with self.targets_lock:
            return self.targets

    def set_rcs(self, rcs):
        with self.rcs_lock:
            self.rcs = rcs

    def get_rcs(self):
        with self.rcs_lock:
            return self.rcs

    def kill(self):
        self.running = False


class SpotSim:
    def __init__(self, size=5):
        self.log = logging.getLogger(__name__)

        # for RC simulator
        self.rc_thread = rc_sim.RCSimThread()
        self.rc_thread.start()

        # startup pybullet
        self.physicsClient = pybullet.connect(pybullet.GUI)
        pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())  # used by loadURDF
        pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_GUI, 0)

        # for kinematics demo
        self.movement_state = 0
        self.forward_target = 0.0
        self.forward_target_going_up = True
        self.height_target = 0.180
        self.height_target_going_up = True
        self.side_target = 0.0
        self.side_target_going_up = True
        self.roll_target = 0.0
        self.roll_target_going_up = True
        self.pitch_target = 0.0
        self.pitch_target_going_up = True
        self.yaw_target = 0.0
        self.yaw_target_going_up = True

        # for software sim
        self.profiler = Profiler()
        self.state_machine = state.StateMachine()

        # for hardware and software sims
        self.serial_thread = ReadSerialFileThread()
        self.serial_thread.start()
        self.next_loop_s = 0  # updated by main loop in sim

        self._env_step_counter = 0

    def reset(self):
        self._env_step_counter = 0
        pybullet.resetSimulation()
        pybullet.setGravity(0, 0, -9.8)
        pybullet.setTimeStep(0.01)
        plane_id = pybullet.loadURDF("plane.urdf")

        pybullet.resetDebugVisualizerCamera(
            cameraDistance=0.75,
            cameraYaw=60,
            cameraPitch=-30,
            cameraTargetPosition=[0, 0, 0]
        )

        urdf_xml_path = os.path.join(
            os.path.dirname(__file__), "..", "nvidia-jetson-nano", "urdf", "spotmicroai_gen.urdf.xml"
        )

        self.bot_id = pybullet.loadURDF(
            urdf_xml_path,
            [0, 0, 0.255],
            pybullet.getQuaternionFromEuler([0, 0, math.pi]),
            flags=pybullet.URDF_USE_SELF_COLLISION | pybullet.URDF_INITIALIZE_SAT_FEATURES,
        )

        num_joints = pybullet.getNumJoints(self.bot_id)
        joint_info = [pybullet.getJointInfo(self.bot_id, i) for i in range(num_joints)]
        for joint in joint_info:
            # modify toe physics
            if joint[1][-3:] == b"toe":
                pybullet.changeDynamics(self.bot_id, joint[0], lateralFriction=0.2)
                # pybullet.changeDynamics(self.bot_id, joint[0], contactStiffness=0, contactDamping=0.01)

    def _compute_observation(self):
        motor_positions = []
        joint_motors = [
            b"front_left_leg", b"front_right_leg", b"front_left_foot", b"front_right_foot",
            b"rear_left_leg", b"rear_right_leg", b"rear_left_foot", b"rear_right_foot"]
        num_joints = pybullet.getNumJoints(self.bot_id)
        joint_info = [pybullet.getJointInfo(self.bot_id, i) for i in range(num_joints)]
        for joint in joint_info:
            if joint[1] in joint_motors:
                motor_positions.append(pybullet.getJointState(self.bot_id, joint[0])[0])

        link_world_position, link_quaternion_orientation = pybullet.getLinkState(self.bot_id, 0)[:2]
        link_euler_orientation = pybullet.getEulerFromQuaternion(link_quaternion_orientation)

    def step(self):
        # self._kinematics_demo()
        # self._hardware_sim()
        self._software_sim()
        
        pybullet.stepSimulation()
        self._env_step_counter += 1

    def _kinematics_demo(self):
        targets = self.kinematics_demo()
        self.goto_deg(*targets)

    def _hardware_sim(self):
        targets = self.serial_thread.get_targets()
        if targets:
            self.goto_deg(*targets)

    def _software_sim(self, rc_input=True):
        if not rc_input:
            state.DEBUG = True
        self.profiler.tick()
        pos_cmds = self.profiler.get_position_commands()
        rc_command = self.serial_thread.get_rcs()
        self.state_machine.update(self.profiler, rc_command, self.next_loop_s * 1e6)
        self.goto_deg(*pos_cmds)

    def goto_deg(
        self,
        front_right_shoulder, front_left_shoulder,
        front_right_leg, front_left_leg,
        front_right_foot, front_left_foot,
        rear_right_shoulder, rear_left_shoulder,
        rear_right_leg, rear_left_leg,
        rear_right_foot, rear_left_foot,
    ):
        joint_pairs = [
            [b"front_right_shoulder", b"front_left_shoulder"],
            [b"front_right_leg", b"front_left_leg"],
            [b"front_right_foot", b"front_left_foot"],
            [b"rear_right_shoulder", b"rear_left_shoulder"],
            [b"rear_right_leg", b"rear_left_leg"],
            [b"rear_right_foot", b"rear_left_foot"],
        ]
        print_angles = False
        controlled_joints = []
        for joint_pair in joint_pairs:
            controlled_joints += [joint_pair[0], joint_pair[1]]

        num_joints = pybullet.getNumJoints(self.bot_id)
        joint_info = [pybullet.getJointInfo(self.bot_id, i) for i in range(num_joints)]
        param_dict_with_b_keys = {key.encode(): val for key, val in locals().items()}
        for joint in joint_info:
            if joint[1] in param_dict_with_b_keys.keys():
                if param_dict_with_b_keys[joint[1]] is not None:
                    target_position = math.radians(param_dict_with_b_keys[joint[1]])
                    if b"left_shoulder" in joint[1]:
                        target_position = -target_position
                    pybullet.setJointMotorControl2(
                        bodyUniqueId=self.bot_id,
                        jointIndex=joint[0],
                        controlMode=pybullet.POSITION_CONTROL,
                        targetPosition=target_position,
                        maxVelocity=math.radians(1 / (0.11 / 60)),  # radians / sec : servo is 0.11s/60deg
                        force=35,
                    )
            if print_angles:
                link_world_position, link_quaternion_orientation = pybullet.getLinkState(self.bot_id, 0)[:2]
                link_euler_orientation = pybullet.getEulerFromQuaternion(link_quaternion_orientation)
                print("{}: {}".format(joint[1], link_euler_orientation))

    def kinematics_demo(self):
        N57 = 0.1150000058  # URDF Dimensions
        N58 = 0.1204159355  # URDF Dimensions
        # N57 = 0.108454  # CAD Measured Y Delta  76.157mm, Z Delta 77.217mm = 108.454mm
        # N58 = 0.135558  # CAD Measured Y Delta 102.793mm, Z Delta 88.372mm = 135.558mm

        if self.movement_state == 0:
            if self.height_target_going_up:
                self.height_target += 0.001
            else:
                self.height_target -= 0.001

        if self.movement_state == 1:
            if self.forward_target_going_up:
                self.forward_target += 0.001
            else:
                self.forward_target -= 0.001

        if self.movement_state == 2:
            if self.side_target_going_up:
                self.side_target += 0.001
            else:
                self.side_target -= 0.001

        if self.movement_state == 3:
            if self.roll_target_going_up:
                self.roll_target += 0.2
            else:
                self.roll_target -= 0.2

        if self.height_target > 0.2 and self.height_target_going_up:
            self.height_target_going_up = False
            self.movement_state += 1
        if self.height_target < 0.12 and not self.height_target_going_up:
            self.height_target_going_up = True

        if self.forward_target > 0.05 and self.forward_target_going_up:
            self.forward_target_going_up = False
            self.movement_state += 1
        if self.forward_target < -0.03 and not self.forward_target_going_up:
            self.forward_target_going_up = True

        if self.side_target > 0.05 and self.side_target_going_up:
            self.side_target_going_up = False
            self.movement_state += 1
        if self.side_target < -0.05 and not self.side_target_going_up:
            self.side_target_going_up = True

        if self.roll_target > 20 and self.roll_target_going_up:
            self.roll_target_going_up = False
            self.movement_state += 1
        if self.roll_target < -20 and not self.roll_target_going_up:
            self.roll_target_going_up = True

        if self.movement_state == 3:
            self.movement_state = 0

        forward_target = self.forward_target
        height_target = self.height_target
        side_target = self.side_target
        shoulder_length = 0.052
        body_width = 0.072

        r_height_target = height_target + ((math.tan(math.radians(self.roll_target)) * (body_width / 2)))
        l_height_target = height_target - ((math.tan(math.radians(self.roll_target)) * (body_width / 2)))

        r_diag_hyp = math.sqrt(r_height_target**2 + (shoulder_length - side_target)**2)
        r_leg_length = math.sqrt(r_diag_hyp**2 - shoulder_length**2)
        right_shoulder_angle = math.asin((shoulder_length - side_target) / r_leg_length) + math.asin(r_leg_length / r_diag_hyp) - (math.pi / 2)
        r_leg_length = math.sqrt(r_leg_length**2 + forward_target**2)
        r_A_due_to_forward_target = math.sin(forward_target / r_height_target)
        right_leg_angle = -(math.acos((N58**2 + r_leg_length**2 - N57**2) / (2 * N58 * r_leg_length)) + r_A_due_to_forward_target)
        right_foot_angle = math.pi - math.acos((N57**2 + N58**2 - r_leg_length**2) / (2 * N57 * N58))

        l_diag_hyp = math.sqrt(l_height_target**2 + (shoulder_length + side_target)**2)
        l_leg_length = math.sqrt(l_diag_hyp**2 - shoulder_length**2)
        left_shoulder_angle = math.asin((shoulder_length + side_target) / l_leg_length) + math.asin(l_leg_length / l_diag_hyp) - (math.pi / 2)
        l_leg_length = math.sqrt(l_leg_length**2 + forward_target**2)
        l_A_due_to_forward_target = math.sin(forward_target / l_height_target)
        left_leg_angle = -(math.acos((N58**2 + l_leg_length**2 - N57**2) / (2 * N58 * l_leg_length)) + l_A_due_to_forward_target)
        left_foot_angle = math.pi - math.acos((N57**2 + N58**2 - l_leg_length**2) / (2 * N57 * N58))

        return [math.degrees(x) for x in [
            right_shoulder_angle,
            left_shoulder_angle,
            right_leg_angle,
            left_leg_angle,
            right_foot_angle,
            left_foot_angle,
            right_shoulder_angle,
            left_shoulder_angle,
            right_leg_angle,
            left_leg_angle,
            right_foot_angle,
            left_foot_angle,
        ]]
