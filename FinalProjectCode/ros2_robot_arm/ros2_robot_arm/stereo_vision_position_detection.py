import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import cv2
import numpy as np
import pickle

class StereoVisionPositionDetection(Node):
    def __init__(self):
        super().__init__('stereo_vision_position_detection')
        self.publisher_ = self.create_publisher(Float32MultiArray, '/object_position', 10)
        self.K = pickle.load(open('cam_matrix.p', 'rb'), encoding='bytes')
        self.dist = pickle.load(open('dist_matrix.p', 'rb'), encoding='bytes')
        self.baseline = self.declare_parameter('baseline', 0.305).get_parameter_value().double_value
        # color_ranges parameter: list of lists [lowH,lowS,lowV,highH,highS,highV]
        color_ranges_param = self.declare_parameter('color_ranges', [[101, 100, 100, 114, 255, 255]]).get_parameter_value().double_array_value
        # Convert to list of tuples of np.array
        self.color_ranges = []
        for cr in color_ranges_param:
            # cr is a flat list, so reshape if needed
            if isinstance(cr, (list, tuple)) and len(cr) == 6:
                self.color_ranges.append((np.array(cr[:3], dtype=np.uint8), np.array(cr[3:], dtype=np.uint8)))
        self.cap_left = cv2.VideoCapture(2, cv2.CAP_DSHOW)
        self.cap_right = cv2.VideoCapture(1, cv2.CAP_DSHOW)
        self.timer = self.create_timer(0.05, self.timer_callback)

    def find_red_ball_center(self, hsv_frame, color_ranges):
        total_mask = np.zeros(hsv_frame.shape[:2], dtype=np.uint8)
        for lower, upper in color_ranges:
            mask = cv2.inRange(hsv_frame, lower, upper)
            total_mask = cv2.bitwise_or(total_mask, mask)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        total_mask = cv2.morphologyEx(total_mask, cv2.MORPH_OPEN, kernel)
        total_mask = cv2.morphologyEx(total_mask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(total_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            largest = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest) > 200:
                (x, y), radius = cv2.minEnclosingCircle(largest)
                return int(x), int(y), int(radius)
        return None, None, None

    def estimate_depth(self, uL, uR, vL):
        fx = self.K[0, 0]
        cx = self.K[0, 2]
        cy = self.K[1, 2]
        disparity = abs(uL - uR)
        if disparity == 0:
            return None, None, None
        Z = fx * self.baseline / disparity
        X = (uL - cx) * Z / fx
        Y = (vL - cy) * Z / fx
        return X, Y, Z

    def timer_callback(self):
        retL, frameL = self.cap_left.read()
        retR, frameR = self.cap_right.read()
        if not retL or not retR:
            return
        frameL = cv2.undistort(frameL, self.K, self.dist)
        frameR = cv2.undistort(frameR, self.K, self.dist)
        hsv_frame_left = cv2.cvtColor(frameL, cv2.COLOR_BGR2HSV)
        hsv_frame_right = cv2.cvtColor(frameR, cv2.COLOR_BGR2HSV)
        uL, vL, rL = self.find_red_ball_center(hsv_frame_left, self.color_ranges)
        uR, vR, rR = self.find_red_ball_center(hsv_frame_right, self.color_ranges)
        if uL is not None and uR is not None:
            X, Y, Z = self.estimate_depth(uL, uR, vL)
            if X is not None and Y is not None and Z is not None:
                msg = Float32MultiArray()
                msg.data = [float(X), float(Y), float(Z)]
                self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = StereoVisionPositionDetection()
    rclpy.spin(node)
    node.cap_left.release()
    node.cap_right.release()
    rclpy.shutdown()
