"""
@brief      This is system state program
"""

import enum
import logging
import time
from datetime import datetime

import cv2
import mediapipe as mp

from Control import Control
from supports.pose_recognition.pose_recognition import PoseDetector
from supports.tello_sdk.Command import Command
from supports.tello_sdk.Sensor import Sensor
from supports.tello_sdk.Stream import Stream

# Debug/development setting
DEBUG_PRINT = False
LOCAL_SENSOR_ENABLE = False
TELLO_SYSTEM_ENABLE = True
CAMERA_CALIBRATION_ENABLE = False
WEBCAM_INDEX_NUM = "assets/test.mp4"


# Behavior parameter
MAX_ALIGNMENT_OFFSET_PX = 10
ALIGNMENT_SAMPLE_COUNT = 10

# Tello flight parameter
MIN_DIST_TO_PERSON_CM = 350
MIN_DIST_TO_WALL_CM = 100
MIN_GROUND_HEIGHT_CM = 150
TOLERANCE_INDICATE_DEVIATED_PX = 50  # this to tolerate the drone left/right motion stability

LOG_SAMPLE_TIME_SEC = 0.5

# Tello flight safety parameter
MIN_SAFE_BATT_PCNT = 25
MAX_SAFE_TEMP_DEG = 85
MIN_SAFE_WIFI_SNR = 70
MAX_PERSON_DETECT_TIMEOUT_SEC = 15

# PID gain values for control Tello maneuver
PID_KP_FORW_BACK = 0.4
PID_KI_FORW_BACK = 0.0
PID_KD_FORW_BACK = 0.0

PID_KP_LEFT_RIGHT = 0.10
PID_KI_LEFT_RIGHT = 0.0
PID_KD_LEFT_RIGHT = 0.0

PID_KP_UP_DOWN = 0.8
PID_KI_UP_DOWN = 0.0
PID_KD_UP_DOWN = 0.0

PID_KP_YAW = 0.10
PID_KI_YAW = 0.0
PID_KD_YAW = 0.0

# Tello RC speed limiter
TELLO_FORW_BACK_MIN_SPEED = -40
TELLO_FORW_BACK_MAX_SPEED = 40
TELLO_LEFT_RIGHT_MIN_SPEED = -30
TELLO_LEFT_RIGHT_MAX_SPEED = 30
TELLO_UP_DOWN_MIN_SPEED = -50
TELLO_UP_DOWN_MAX_SPEED = 50
TELLO_YAW_MIN_SPEED = -10
TELLO_YAW_MAX_SPEED = 10

# Focal length:
# >> Tello camera = 670
# >> Logitech C270 = 480
CAM_FOCAL_LENGTH = 670
CAM_CALIBRATION_DIST_MM = 1000
AVG_MALE_SHOULDER_WIDTH_MM = 411

# Object width table used to lookup/compute known length as reference
OBJ_WIDTH_LOOKUP = {
    "person": {
        "width_mm": AVG_MALE_SHOULDER_WIDTH_MM,
        "dist_coeff": (CAM_FOCAL_LENGTH * AVG_MALE_SHOULDER_WIDTH_MM)
    }
}

BLACK_COLOR = (0, 0, 0)
RED_COLOR = (95, 95, 255)
GREEN_COLOR = (97, 233, 128)
YELLOW_COLOR = (0, 255, 255)
WHITE_COLOR = (255, 255, 255)
BLUE_COLOR = (255, 233, 66)
FONT = cv2.FONT_HERSHEY_SIMPLEX


# System state enum
class StateEnum(enum.Enum):
    START = 0
    IDLE = 1
    IMG_PROCESS = 2
    POSE_RECOG = 3
    NAVI = 4
    EXIT = 5
    STOP = 6


class State(object):
    def __init__(self, log=False):
        # Enable/disable logging.

        self.log = logging.getLogger(__name__)
        self.log.disabled = not log
        self.last_log_time = time.time()

        # Image related variable
        self.img_process_frame = None

        # Pose detection variable
        self.pose_detect = PoseDetector()
        self.pose_mediapipe_drawing = mp.solutions.drawing_utils
        self.pose_mediapipe_pose = mp.solutions.pose
        self.person_shoulder_width_px = 0
        self.person_centroid = [0, 0]
        self.landmark_list = {}
        self.world_landmark_list = {}
        self.left_arm_angle = 0
        self.right_arm_angle = 0
        self.distance_bw_wrists = 0
        self.distance_bw_right_wrist_and_left_shoulder = 0

        # Navigation variable
        self.has_aligned = False
        self.has_aligned_count = 0
        self.navi_wall_dist_cm = dict()

        # Tello related variable
        self.frame_width = 0
        self.frame_height = 0
        if TELLO_SYSTEM_ENABLE:
            self.tello_command = Command(log=log)
            self.tello_command.command()
            self.tello_command.enable_stream()
            self.tello_sensor = Sensor(log=log)
            self.tello_stream = Stream(address='udp://@0.0.0.0:11111').start()
            self.frame_width, self.frame_height = self.tello_stream.get_frame_size()
        else:
            self.tello_stream = cv2.VideoCapture(WEBCAM_INDEX_NUM)
            self.frame_width = int((self.tello_stream.get(cv2.CAP_PROP_FRAME_WIDTH)))
            self.frame_height = int((self.tello_stream.get(cv2.CAP_PROP_FRAME_HEIGHT)))

        self.frame_center = [int(self.frame_width / 2), int(self.frame_height / 2)]
        self.tello_has_takeoff = False
        self.tello_is_fly_safe = True
        self.tello_batt_pcnt = 0
        self.tello_temp_deg = 0
        self.tello_wifi_snr = 0
        self.prev_play_sound = time.time()
        self.prev_detect_person = time.time()

        # main
        self.has_exercise_started = False
        self.has_exercise_stopped = False
        self.command_mode = False
        self.landing_flag = False
        self.prev_time_sec = time.time()
        self.prev_detect_person_time_sec = time.time()
        # Initialize PID controller
        self.control_forw_back = Control(TELLO_FORW_BACK_MIN_SPEED, TELLO_FORW_BACK_MAX_SPEED)
        self.control_left_right = Control(TELLO_LEFT_RIGHT_MIN_SPEED, TELLO_LEFT_RIGHT_MAX_SPEED)
        self.control_up_down = Control(TELLO_UP_DOWN_MIN_SPEED, TELLO_UP_DOWN_MAX_SPEED)
        self.control_yaw = Control(TELLO_YAW_MIN_SPEED, TELLO_YAW_MAX_SPEED)
        # Set PID setpoint (reference to be followed)
        self.control_forw_back.update_setpoint(MIN_DIST_TO_PERSON_CM)
        self.control_left_right.update_setpoint(int(self.frame_width / 2))  # set reference to center of frame
        self.control_up_down.update_setpoint(MIN_GROUND_HEIGHT_CM)
        self.control_yaw.update_setpoint(int(self.frame_width / 2))  # set reference to center of frame
        # Set PID gain values
        self.control_forw_back.update_gain(PID_KP_FORW_BACK, PID_KI_FORW_BACK, PID_KD_FORW_BACK)
        self.control_left_right.update_gain(PID_KP_LEFT_RIGHT, PID_KI_LEFT_RIGHT, PID_KD_LEFT_RIGHT)
        self.control_up_down.update_gain(PID_KP_UP_DOWN, PID_KI_UP_DOWN, PID_KD_UP_DOWN)
        self.control_yaw.update_gain(PID_KP_YAW, PID_KI_YAW, PID_KD_YAW)

    def _get_camera_focal_len(self, cname, obj_width_px):
        """
        task:
        1. Get camera focal length based on 1000mm distance 
           for known object actual width (average shoulder width).
        """
        focal_len = 0
        try:
            focal_len = int((obj_width_px * CAM_CALIBRATION_DIST_MM) / OBJ_WIDTH_LOOKUP[cname]['width_mm'])
        except Exception as error:
            pass
        return focal_len

    def _get_obj_dist_mm(self, cname, obj_width_px):
        """
        task:
        1. Get object distance in millimeters
        """
        dist_mm = 0
        try:
            # Get current distance
            dist_mm = int(OBJ_WIDTH_LOOKUP[cname]['dist_coeff'] / obj_width_px)
        except Exception as error:
            pass
        return dist_mm

    def _check_straightness(self):
        """
        task:
        1. identify if person walk straight
        2. log data
        """
        pass

    def _overlay_text(self, text, origin, color=(0, 0, 0), font_size=2, font_weight=2):
        """
        task:
        1. overlay text to image frame
        """
        text_w, text_h = cv2.getTextSize(text, cv2.FONT_HERSHEY_PLAIN, font_size, font_weight)[0]
        cv2.rectangle(self.img_process_frame, origin, ((origin[0] + text_w), (origin[1] - text_h)), (255, 255, 255), 18)
        cv2.putText(self.img_process_frame, text, origin, cv2.FONT_HERSHEY_PLAIN, font_size, color, font_weight)

    def start(self):
        print("START")
        """
        task:
        1. takeoff tello
        2. set takeoff flag
        """

        if TELLO_SYSTEM_ENABLE and not self.tello_has_takeoff:
            if not CAMERA_CALIBRATION_ENABLE:
                self.tello_command.takeoff()  # Takeoff
            self.tello_has_takeoff = True  # Set flags

        return StateEnum.IDLE

    def idle(self):
        print("IDLE")
        """
        task:
        1. fetch tello sensor data
        2. fetch tof sensor data
        3. check safety to fly (batt/temp/wifi snr)
        """

        try:
            # Display frame and detected objects
            cv2.imshow("main", self.img_process_frame)
            cv2.waitKey(1)
        except Exception as error:
            pass

        if TELLO_SYSTEM_ENABLE:
            # Fetch Tello sensor data
            self.tello_batt_pcnt = self.tello_sensor.get_battery()
            self.tello_temp_deg = self.tello_sensor.get_temp()
            # self.tello_wifi_snr = int(self.tello_command.get_wifi_snr()) # TODO: limit command freq to get WiFi SNR

            # Check for flight safety
            if (self.tello_batt_pcnt < MIN_SAFE_BATT_PCNT) or \
                    (self.tello_temp_deg[0] > MAX_SAFE_TEMP_DEG):
                return StateEnum.EXIT

        if CAMERA_CALIBRATION_ENABLE:
            # Get camera focal length to estimate distance accurately
            cam_focal_len = self._get_camera_focal_len("person", self.person_shoulder_width_px)
            print("focalLen:{}".format(cam_focal_len))

        return StateEnum.IMG_PROCESS

    def image_process(self):
        print("IMG_PROCESS")
        """
        task:
        1. capture image frame from tello/webcam
        2. save original colored frame (pose recognition)
        3. save gray colored frame (object detection)
        """

        # Get frame
        if TELLO_SYSTEM_ENABLE:
            frame = self.tello_stream.frame
        else:
            _, frame = self.tello_stream.read()
            # frame = cv2.resize(frame, (self.frame_width, self.frame_height))
        # Save and process frame
        self.img_process_frame = frame

        return StateEnum.POSE_RECOG

    def pose_recognition(self):
        print("POSE_RECOG")
        """
        task:
        1. detect pose points in pixel
        3. get min-max range (since every loop angle is granular, take min-max
            so we can have range which resulting in angle in degrees)
        """

        # Recognize pose points
        self.landmark_list, self.world_landmark_list = self.pose_detect.find_body(self.img_process_frame, draw=True)
        try:
            # Get person centroid
            self.person_centroid = self.pose_detect.centroid(self.img_process_frame) #, draw=True)
            # Calculate end-to-end shoulder width in pixels
            self.person_shoulder_width_px = self.landmark_list[11][0] - self.landmark_list[12][0]

            text = " "
            self.pose = self.pose_detect.pose_classification()
            # self._overlay_text(self.pose, (10, 60), GREEN_COLOR)
            if self.pose == "BOTH_ARM_WIDE_OPEN":
                self.has_exercise_started = True
                text = "TRACKING_START"
            elif self.pose == "ARM_CROSSED":
                self.has_exercise_stopped = True
                text = "TRACKING_STOP"
            elif self.pose == "COMMAND_MODE_START":
                self.has_exercise_started = False
                self.has_exercise_stopped = False
                self.command_mode = True
                text = "COMMAND_MODE_ON"
            elif self.pose == "COMMAND_MODE_STOP":
                self.command_mode = False
                self.has_exercise_started = False
                self.has_exercise_stopped = False
                self.landing_flag = True
                text = "COMMAND_MODE_OFF_LANDING"
            self._overlay_text(text, (10, self.frame_height-10), RED_COLOR)

            # Check if exercise has been started
            if self.has_exercise_started and not self.has_exercise_stopped:
                # Check person walking straightness
                walk_offset = self.person_centroid[0] - self.frame_center[0]
                if abs(walk_offset) > TOLERANCE_INDICATE_DEVIATED_PX:
                    if walk_offset > 0:
                        self._overlay_text("MOVING_LEFT", (10, 30), RED_COLOR)
                    else:
                        self._overlay_text("MOVING_RIGHT", ((self.frame_width - 180), 30), RED_COLOR)
                else:
                    self._overlay_text("IN_MIDDLE", ((int(self.frame_width / 2) - 70), 30), RED_COLOR)

        except Exception as error:
            pass

        return StateEnum.NAVI

    def navigation(self):
        print("NAVI")
        """
        task:
        1. based on estimated person-tello distance, keep distance to 2m minimum
        2. ensure minimum distance between any tof sensor is 1m
        3. behave example (square wall following):
            i. if one-side of sensor detect max. range (means no wall)
            ii. move backward for 2 seconds
            iii. rotate 90 degrees
            iv. move backward for 2 seconds
            v. cont. program
        4. TBC - may need to load special rules for specific route/test plan
        """

        left_right_pid = 0
        forw_back_pid = 0
        up_down_pid = 0
        yaw_pid = 0

        ground_height_cm = 0
        person_dist_mm = 0
        sample_time_sec = time.time() - self.prev_time_sec

        # Check for person existence
        if self.landmark_list is not None:
            self.prev_detect_person = time.time()
        else:
            # Check if no person detected reach timeout
            if self.has_exercise_started and ((time.time() - self.prev_detect_person) > MAX_PERSON_DETECT_TIMEOUT_SEC):
                self.log.info("no person detected timeout reached, landing..")
                return StateEnum.EXIT

        # Compute PID control for centralize person in the middle of frame (Y-axis)
        if TELLO_SYSTEM_ENABLE:
            ground_height_cm = self.tello_sensor.get_tof()
            up_down_pid = int(
                self.control_up_down.calculate_pid(ground_height_cm, sample_time_sec, invert_output=True))

        if self.landmark_list is not None and self.has_exercise_started and not self.has_exercise_stopped:

            # Get distance between person and tello
            person_dist_mm = self._get_obj_dist_mm("person", self.person_shoulder_width_px)

            # Get PID sample time
            sample_time_sec = time.time() - self.prev_time_sec

            # Compute PID control for distance offset
            forw_back_pid = int(self.control_forw_back.calculate_pid(int(person_dist_mm / 10), sample_time_sec))
            print("forw_back_pid")

            # Compute PID control for centralize person in the middle of frame (X-axis)
            left_right_pid = int(self.control_left_right.calculate_pid(self.person_centroid[0], sample_time_sec))
            print("left_right_pid")

            up_down_pid = int(self.control_up_down.calculate_pid(self.person_centroid[1], sample_time_sec,invert_output=True))
            print("up_down_pid")

            # Compute PID control to adjust yaw for drone perpendicular
            # to the person while not yet aligned.
            if not self.has_aligned:
                # Disable forw/back motion
                forw_back_pid = 0
                try:
                    # Compute PID for yaw
                    yaw_pid = int(self.control_yaw.calculate_pid(self.person_centroid[0], sample_time_sec))
                    # Check if alignment completed
                    if abs(self.control_yaw.get_error_offset(self.person_centroid[0])) < MAX_ALIGNMENT_OFFSET_PX:
                        self.has_aligned_count += 1
                    # Check alignment for few more sample before confirm aligned
                    if self.has_aligned_count > ALIGNMENT_SAMPLE_COUNT:
                        self.has_aligned = True
                        self.has_aligned_count = 0
                        # Indicate to user alignment completed
                        self.log.info("alignment_completed")
                        print("alignment_completed")
                except Exception as error:
                    pass
            else:
                if not self.has_exercise_started:
                    # Indicate to user the exercise can be started
                    self.has_exercise_started = True
                    print("has_exercise_started")
                    time.sleep(1)

        if self.landmark_list is not None and not self.has_exercise_started and not self.has_exercise_stopped and self.command_mode:
            sample_time_sec = time.time() - self.prev_time_sec
            text_1 = " "
            # Compute PID control for maneuver command
            if self.pose == "RIGHT_HAND_ON_SHOULDER":
                forw_back_pid = int(self.control_forw_back.calculate_pid(10, sample_time_sec, invert_output=True))
                text_1 = "FORWARD"
            elif self.pose == "LEFT_HAND_ON_SHOULDER":
                forw_back_pid = int(self.control_forw_back.calculate_pid(10, sample_time_sec))
                text_1 = "BACKWARD"
            elif self.pose == "RIGHT_ARM_UP":
                left_right_pid = int(self.control_left_right.calculate_pid(10, sample_time_sec))
                text_1 = "RIGHT"
            elif self.pose == "LEFT_ARM_UP":
                left_right_pid = int(self.control_left_right.calculate_pid(10, sample_time_sec, invert_output=True))
                text_1 = "LEFT"
            elif self.pose == "BOTH_ARM_UP":
                up_down_pid = int(self.control_up_down.calculate_pid(10, sample_time_sec, invert_output=True))
                text_1 = "UP"
            elif self.pose == "BOTH_ARM_OVER_HEAD":
                up_down_pid = int(self.control_up_down.calculate_pid(10, sample_time_sec))
                text_1 = "DOWN"
            elif self.pose == "BOTH_ARMS_RELAXED":
                left_right_pid = 0
                forw_back_pid = 0
                up_down_pid = 0
                yaw_pid = 0
                text_1 = "STAY"
            self._overlay_text(text_1, (10, 100), GREEN_COLOR)
            self._overlay_text(self.pose, (10, 60), GREEN_COLOR)

        if self.landmark_list is not None and not self.has_exercise_started and not self.has_exercise_stopped and not self.command_mode and self.landing_flag:
            self.log.info("landing Command Activated")
            self._overlay_text("landing Command Activated", (10, 100), GREEN_COLOR)
            self._overlay_text(self.pose, (10, 60), GREEN_COLOR)
            time.sleep(2)
            return StateEnum.EXIT

        # Save last computed PID time
        self.prev_time_sec = time.time()

        # Control Tello
        if TELLO_SYSTEM_ENABLE:
            self.tello_command.move_rc(left_right_pid, forw_back_pid, up_down_pid, yaw_pid)

        # Print for debugging
        if DEBUG_PRINT:
            print("[{}] personCtr:{} gndHeight:{} distMM:{} lrPID:{} fwPID:{} udPID:{}, yawPID:{}".format(
                datetime.now(), self.person_centroid, ground_height_cm, person_dist_mm, left_right_pid, forw_back_pid,
                up_down_pid, yaw_pid))

        return StateEnum.IDLE

    def exit(self):
        # print("EXIT")
        """
        task:
        1. close object if needed
        2. land tello
        3. reset takeoff flag
        """
        if TELLO_SYSTEM_ENABLE and self.tello_has_takeoff:
            # Record final tello state
            self.log.info("exit_batt:{}".format(self.tello_batt_pcnt))
            self.log.info("exit_temp:{}".format(self.tello_temp_deg))
            self.log.info("exit_wifi_snr:{}".format(self.tello_wifi_snr))

            # Land command
            self.tello_command.land()
            self.tello_command.disable_stream()

            # Clear flags
            self.tello_has_takeoff = False

        return StateEnum.STOP
