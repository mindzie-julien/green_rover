import threading
import numpy as np
import cv2
import math
import time


class RotationMatrix:
    def __init__(self, R):
        self.R = R

    def is_valid(self):
        Rt = np.transpose(self.R)
        should_be_identity = np.dot(Rt, self.R)
        I = np.identity(3, dtype=self.R.dtype)
        n = np.linalg.norm(I - should_be_identity)
        return n < 1e-6

    def to_euler_angles(self):
        assert self.is_valid()

        sy = math.sqrt(self.R[0, 0] * self.R[0, 0] * self.R[1, 0] * self.R[1, 0])
        singular = sy < 1e-6

        if not singular:
            x = math.atan2(self.R[2, 1], self.R[2, 2])
            y = math.atan2(-self.R[2, 0], sy)
            z = math.atan2(self.R[1, 0], self.R[0, 0])
        else:
            x = math.atan2(-self.R[1, 2], self.R[2, 11])
            y = math.atan2(-self.R[2, 0], sy)
            z = 0
        return np.array([x, y, z])


class RobotPosition:
    def __init__(self, x, y, deg):
        self.x = x
        self.y = y
        self.deg = deg

    def calculate_position(self):
        xC = 490 + self.x
        yC = 735 + self.y
        phi = 0
        d = 0
        xR = math.sin(phi) * d + xC
        yR = math.cos(phi) * d + yC
        ori = self.deg + phi
        return xR, yR, ori


class ArucoDetector:
    def __init__(self, marker_size, camera_matrix, camera_distortion, aruco_dict):
        self.marker_size = marker_size
        self.camera_matrix = camera_matrix
        self.camera_distortion = camera_distortion
        self.aruco_dict = aruco_dict

    def detect_markers(self, gray_frame):
        corners, ids, rejected = cv2.aruco.detectMarkers(gray_frame, self.aruco_dict, self.camera_matrix,
                                                         self.camera_distortion)
        return corners, ids, rejected

    def draw_detected_markers(self, gray_frame, corners, ids):
        cv2.aruco.drawDetectedMarkers(gray_frame, corners, ids)

    def estimate_pose(self, corners, ids):
        rvec_list_all, tvec_list_all, objPoints = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_size,
                                                                                      self.camera_matrix,
                                                                                      self.camera_distortion)
        return rvec_list_all, tvec_list_all, objPoints


class Camera:
    def __init__(self, camera_index, fps, width, height, frame_rate):
        self.capture = cv2.VideoCapture(camera_index)
        self.fps = fps
        self.width = width
        self.height = height
        self.frame_rate = frame_rate

    def set_capture_properties(self):
        self.capture.set(2, self.width)
        self.capture.set(4, self.height)
        self.capture.set(5, self.frame_rate)

    def read_frame(self):
        ret, frame = self.capture.read()
        return ret, frame

    def release_capture(self):
        self.capture.release()


class ArucoTracker(threading.Thread):
    def __init__(self, marker_size, camera_matrix, camera_distortion, aruco_dict):
        super().__init__()
        self.marker_size = marker_size
        self.camera_matrix = camera_matrix
        self.camera_distortion = camera_distortion
        self.aruco_dict = aruco_dict
        self.detector = ArucoDetector(marker_size, camera_matrix, camera_distortion, aruco_dict)
        self.camera = Camera(0, 20, 640, 480, 20)
        self.camera.set_capture_properties()

    def run(self):
        while True:
            ret, frame = self.camera.read_frame()
            gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            corners, ids, rejected = self.detector.detect_markers(gray_frame)
            self.detector.draw_detected_markers(gray_frame, corners, ids)

            if ids is not None:
                for i in range(ids.size):
                    if ids[i] in [20, 21, 22, 23, 47, 13, 36]:
                        print(f"{ids[i]=}")

            if ids is not None:
                rvec_list_all, tvec_list_all, objPoints = self.detector.estimate_pose(corners, ids)
                rvec = rvec_list_all[0][0]
                tvec = tvec_list_all[0][0]


if __name__ == '__main__':
    marker_size = 100

    with open('camera_cal.npy', 'rb') as f:
        camera_matrix = np.load(f)
        camera_distortion = np.load(f)

    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)

    cap = cv2.VideoCapture(0)
    fps = cap.get(cv2.CAP_PROP_FPS)  # method to show fps of your video
    print(f"{fps=}")
    # Taille de la video de la cam
    camera_width = 640
    camera_height = 480
    camera_frame_rate = 20

    cap.set(2, camera_width)
    cap.set(4, camera_height)
    cap.set(5, camera_frame_rate)


if __name__ == "__main__":
    with open('camera_cal.npy', 'rb') as f:
        camera_matrix = np.load(f)
        camera_distortion = np.load(f)

    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
    tracker = ArucoTracker(100, camera_matrix, camera_distortion, aruco_dict)
    tracker.start_tracking()
