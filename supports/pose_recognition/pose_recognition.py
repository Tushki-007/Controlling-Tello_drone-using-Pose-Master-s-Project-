import math

import cv2
import mediapipe as mp


# noinspection PyBroadException
class PoseDetector:
    try:
        def __init__(self,
                     static_image_mode=False,
                     model_complexity=1,
                     smooth_landmarks=True,
                     enable_segmentation=False,
                     smooth_segmentation=True,
                     min_detection_confidence=0.5,
                     min_tracking_confidence=0.7):
            """Initializes a MediaPipe Pose object.
            Args:
            static_image_mode: Whether to treat the input images as a batch of static and possibly unrelated images, or a video stream.
            model_complexity: Complexity of the pose landmark model: 0, 1 or 2. smooth_landmarks: Whether to filter landmarks across different input images to reduce jitter.
            enable_segmentation: Whether to predict segmentation mask.
            smooth_segmentation: Whether to filter segmentation across different input images to reduce jitter.
            min_detection_confidence: Minimum confidence value ([0.0, 1.0]) for person detection to be considered successful.
            min_tracking_confidence: Minimum confidence value ([0.0, 1.0]) for the pose landmarks to be considered tracked successfully.
            """
            # Initializing a PoseDetector object
            self.static_image_mode = static_image_mode
            self.model_complexity = model_complexity
            self.smooth_landmarks = smooth_landmarks
            self.enable_segmentation = enable_segmentation
            self.smooth_segmentation = smooth_segmentation
            self.min_detection_confidence = min_detection_confidence
            self.min_tracking_confidence = min_tracking_confidence

            self.landmark_list = {}
            self.world_landmarks_list = {}
            self.results = None

            self.mp_pose = mp.solutions.pose
            self.mp_draw = mp.solutions.drawing_utils
            self.mp_drawing_styles = mp.solutions.drawing_styles

            # Pose variables
            self.pose_type = {}
            self.left_arm_angle = 0
            self.right_arm_angle = 0
            self.left_leg_angle = 0
            self.right_leg_angle = 0
            self.right_arm_shoulder_angle = 0
            self.left_arm_shoulder_angle = 0
            self.distance_bw_shoulders = 0
            self.distance_bw_wrists = 0
            self.distance_bw_right_wrist_and_left_shoulder = 0
            self.distance_bw_left_wrist_and_right_shoulder = 0

            self.body = self.mp_pose.Pose(self.static_image_mode,
                                          self.model_complexity,
                                          self.smooth_landmarks,
                                          self.enable_segmentation,
                                          self.smooth_segmentation,
                                          self.min_detection_confidence,
                                          self.min_tracking_confidence)

        # noinspection PyBroadException
        def find_body(self, img, draw=False):
            """
            :param img: Input Image
            :param draw: True/False To draw the landmarks
            :return: list of Land marks which contain Landmark ID, X, Y, Z coordinates (image coordinates)
            :return: list of Land marks which contain real world Landmark ID_w, X_w, Y_w, Z_w coordinates (in meters)
            """

            # Convert the BGR image to RGB before processing
            image = cv2.cvtColor(img, cv2.COLOR_XYZ2RGB)
            # To improve performance, mark the image as not writeable to pass by reference
            image.flags.writeable = False
            self.results = self.body.process(image)
            if draw:
                image.flags.writeable = True
                self.mp_draw.draw_landmarks(img, self.results.pose_landmarks, self.mp_pose.POSE_CONNECTIONS, landmark_drawing_spec=self.mp_drawing_styles.get_default_pose_landmarks_style())
            try:
                for ID, lm in enumerate(self.results.pose_landmarks.landmark):
                    h, w, c = img.shape
                    # converting pixel coordinates in image coordinates
                    cx, cy = int(lm.x * w), int(lm.y * h)
                    # cz Represents the landmark depth with the depth at the midpoint of hips being the origin,
                    # and the smaller the value the closer the landmark is to the camera. The magnitude of cz uses
                    # roughly the same scale as cx.
                    cz = round(float(lm.z), 2)
                    self.landmark_list[ID] = [cx, cy, cz]
                # Multiplying by 100 to convert meter into centimeters
                for ID_w, lm_w in enumerate(self.results.pose_world_landmarks.landmark):
                    x_w = round(float(lm_w.x * 100), 2)
                    y_w = round(float(lm_w.y * 100), 2)
                    z_w = round(float(lm_w.z * 100), 2)
                    self.world_landmarks_list[ID_w] = [x_w, y_w, z_w]

            except Exception as error:
                pass
            return self.landmark_list, self.world_landmarks_list

        # noinspection PyBroadException
        def find_angle(self, starting_point, middle_point, end_point):
            """
            Gives the input of three points for which you need to find the angle.
            The return value is measured according the position of the person
            if person standing in front facing towards the camera the it takes
            the outer angle clockwise otherwise takes the inner angle.

            :return: angle
            """
            try:
                # Get the landmarks
                x1, y1, _ = self.world_landmarks_list[starting_point]
                x2, y2, _ = self.world_landmarks_list[middle_point]
                x3, y3, _ = self.world_landmarks_list[end_point]

                # Calculate the Angle
                angle = math.degrees(math.atan2(y3 - y2, x3 - x2) -
                                     math.atan2(y1 - y2, x1 - x2))
                if angle < 0:
                    angle += 360

                return angle
            except Exception as error:
                pass

        # noinspection PyBroadException
        def centroid(self, img, draw=False):
            """
                It will calculates the centroid for 4 pre define points of the body,
                which are left_shoulder, right_shoulder, left_hip & right_hip.

            :return: Center point of the above mentioned points
            """
            result = [0, 0]
            try:
                xx1, yy1, _ = self.landmark_list[11]  # left_shoulder
                xx2, yy2, _ = self.landmark_list[12]  # right_shoulder
                xx3, yy3, _ = self.landmark_list[23]  # left_hip
                xx4, yy4, _ = self.landmark_list[24]  # right_hip
                # Calculate the centroid
                result = (xx1 + xx2 + xx3 + xx4) // 4, (yy1 + yy2 + yy3 + yy4) // 4
                # Drawing the Centroid
                if draw:
                    cv2.circle(img, result, 10, (255, 255, 255), cv2.FILLED)
            except Exception as error:
                pass
            return result

        # noinspection PyBroadException
        def dist2p(self, starting_point, end_point):
            """
            :return: distance between the two points
            """
            try:
                x1, y1, _ = self.world_landmarks_list[starting_point]
                x2, y2, _ = self.world_landmarks_list[end_point]
                return math.hypot(x2 - x1, y2 - y1)
            except Exception as error:
                pass

        # noinspection PyBroadException
        def angle_bw_2_points(self, starting_point, end_point, landmark=False):
            """
                Calculate the angle between segment(A,B) and vertical axe
            """
            angle = 0
            try:
                if landmark:
                    x1, y1, _ = self.landmark_list[starting_point]
                    x2, y2, _ = self.landmark_list[end_point]
                else:
                    x1, y1 = starting_point[0], starting_point[1]
                    x2, y2 = end_point[0], end_point[1]

                angle = math.degrees(math.atan2(y2 - y1, x2 - x1) - math.pi / 2)
            except Exception as error:
                pass
            return angle

        # noinspection PyBroadException
        def pose_classification(self):
            """
              LEFT_SHOULDER = 11
              RIGHT_SHOULDER = 12
              LEFT_ELBOW = 13
              RIGHT_ELBOW = 14
              LEFT_WRIST = 15
              RIGHT_WRIST = 16
              LEFT_HIP = 23
              RIGHT_HIP = 24
              LEFT_KNEE = 25
              RIGHT_KNEE = 26
              LEFT_ANKLE = 27
              RIGHT_ANKLE = 28

            :return: Pose
            """
            try:
                if self.landmark_list:
                    # Angles
                    self.left_arm_angle = self.find_angle(11, 13, 15)
                    self.right_arm_angle = self.find_angle(12, 14, 16)
                    self.left_leg_angle = self.find_angle(23, 25, 27)
                    self.right_leg_angle = self.find_angle(24, 26, 28)
                    self.right_arm_shoulder_angle = self.find_angle(11, 12, 14)
                    self.left_arm_shoulder_angle = self.find_angle(12, 11, 13)

                    # Distance
                    self.distance_bw_shoulders = self.dist2p(11, 12)
                    self.distance_bw_wrists = self.dist2p(15, 16)
                    self.distance_bw_right_wrist_and_left_shoulder = self.dist2p(11, 16)
                    self.distance_bw_left_wrist_and_right_shoulder = self.dist2p(12, 15)

                    if (250 <= self.right_arm_angle <= 290) and \
                        (70 <= self.left_arm_angle <= 110) and \
                        (170 <= self.right_arm_shoulder_angle <= 200) and \
                        (160 <= self.left_arm_shoulder_angle <= 190) and \
                        (60 <= self.distance_bw_wrists <= 90):
                        self.pose_type = "BOTH_ARM_UP"

                    elif (260 <= self.right_arm_angle <= 280) and \
                        (80 <= self.left_arm_angle <= 110) and \
                        (200 <= self.right_arm_shoulder_angle) and \
                        (self.left_arm_shoulder_angle <= 160) and \
                        (20 <= self.distance_bw_wrists <= 30):
                        self.pose_type = "BOTH_ARM_OVER_HEAD"

                    elif (170 <= self.right_arm_angle <= 190) and \
                        (170 <= self.left_arm_angle <= 190) and \
                        (120 <= self.distance_bw_wrists <= 130):
                        self.pose_type = "BOTH_ARM_WIDE_OPEN"

                    elif (250 <= self.right_arm_angle <= 290) and \
                        (170 <= self.left_arm_angle <= 190) and \
                        (80 <= self.distance_bw_wrists <= 90):
                        self.pose_type = "RIGHT_ARM_UP"

                    elif (160 <= self.right_arm_angle <= 180) and \
                        (70 <= self.left_arm_angle <= 110) and \
                        (80 <= self.distance_bw_wrists <= 90):
                        self.pose_type = "LEFT_ARM_UP"

                    elif (60 <= self.right_arm_angle <= 80) and \
                        (160 <= self.left_arm_angle <= 180) and \
                        (15 <= self.distance_bw_right_wrist_and_left_shoulder <= 30):
                        self.pose_type = "RIGHT_HAND_ON_SHOULDER"

                    elif (160 <= self.right_arm_angle <= 180) and \
                        (280 <= self.left_arm_angle <= 300) and \
                        (15 <= self.distance_bw_left_wrist_and_right_shoulder <= 30):
                        self.pose_type = "LEFT_HAND_ON_SHOULDER"

                    elif (10 <= self.distance_bw_left_wrist_and_right_shoulder <= 18) and \
                        (10 <= self.distance_bw_right_wrist_and_left_shoulder <= 18) and \
                        (5 <= self.distance_bw_wrists <= 10):
                        self.pose_type = "ARM_CROSSED"

                    elif (160 <= self.right_arm_angle <= 180) and \
                        (170 <= self.left_arm_angle <= 190) and \
                        (40 <= self.distance_bw_wrists <= 50):
                        self.pose_type = "BOTH_ARMS_RELAXED"

                    elif (110 <= self.right_arm_angle <= 120) and \
                        (240 <= self.left_arm_angle <= 250) and \
                        (50 <= self.distance_bw_right_wrist_and_left_shoulder) and \
                        (50 <= self.distance_bw_left_wrist_and_right_shoulder):
                        self.pose_type = "COMMAND_MODE_START"

                    elif (self.left_arm_angle <= 50) and \
                        (310 <= self.right_arm_angle) and \
                        (self.right_arm_shoulder_angle <= 120) and \
                        (240 <= self.left_arm_shoulder_angle):
                        self.pose_type = "COMMAND_MODE_STOP"

                return self.pose_type
            except Exception as error:
                pass
    except Exception as class_error:
        pass
