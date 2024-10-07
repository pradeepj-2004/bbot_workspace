# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
# import cv2 as cv
# import numpy as np
# import os
# import math

# class PoseEstimator(Node):
#     def __init__(self):
#         super().__init__("PoseEstimator")
#         self.create_subscription(Image, "/camera/image_raw",self.pose_estimation,qos_profile=10)
#         # self.timer_ = self.create_timer(1, self.pose_estimation)
#         self.bridge = CvBridge()
#         self.get_logger().info("Pose Estimator node has started")

#     def pose_estimation(self, msg):
#         dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_6X6_250)

#         id_to_find = 1
#         marker_size = 200 # cm

#         # realWorldEfficiency = .7
#         # aruco_dict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_ARUCO_ORIGINAL)
#         parameters = cv.aruco.DetectorParameters()

#         self.cameraMatrix = np.array([[381.671779, 0, 319.284384],
#                                       [0, 381.728233, 239.272848],
#                                       [0, 0, 1]], dtype="double")
        
#         self.cameraDistortion = np.array([-0.000395, 0.000979, -0.000220, 0.000152, 0.0000], dtype="double")

#         # Specify the calibration directory
#         # calib_path = "~/cameracalib"

#         # # Expand the user directory
#         # calib_path = os.path.expanduser(calib_path)

#         # # Load the camera calibration matrices
#         # cameraMatrix = np.loadtxt(os.path.join(calib_path, 'cameraMatrix.txt'), delimiter=',')
#         # cameraDistortion = np.loadtxt(os.path.join(calib_path, 'cameraDistortion.txt'), delimiter=',')

#         frame_converted = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
#         # cv.imshow("Converted Frame", frame_converted)
#         # cv.waitKey(1)

#         frame_np= np.array(frame_converted)

#         gray_img = cv.cvtColor(frame_converted, cv.COLOR_BGR2GRAY)
#         ids = ''
#         detector = cv.aruco.ArucoDetector(dictionary, parameters)
#         corners, ids, rejected = cv.aruco.detectMarkers(gray_img, dictionary)
                
                 
#         if ids is not None:
#             print("Found these IDs in the frame:")
#             print(ids)
#         if ids is not None and ids[0] == id_to_find:
#             ret = cv.aruco.estimatePoseSingleMarkers(corners, marker_size, self.cameraMatrix, self.cameraDistortion)
#             rvec, tvec = ret[0][0, 0, :], ret[1][0, 0, :]
#             rotation_matrix, _ = cv.Rodrigues(rvec)
#             rvec_degrees = np.degrees(rvec)
#             x = "{:.2f}".format(tvec[0])
#             y = "{:.2f}".format(tvec[1])
#             z = "{:.2f}".format(tvec[2])
#             print(x,y,z)
#             marker_position = "MARKER POSITION: x=" + x + " y=" + y + " z=" + z
#             print(marker_position)
#             print("")

#         if corners:
#             rvec, tvec, _ = cv.aruco.estimatePoseSingleMarkers(corners, marker_size, self.cameraMatrix, self.cameraDistortion)
#             total_markers = range(0, ids.size)
#             for ids, corners,i in zip(ids, corners, total_markers):
#                 cv.polylines(frame_converted, [corners.astype(np.int32)], True, (90,255,255), 4, cv.LINE_AA)
#                 corners = corners.reshape(4,2)
#                 corners = corners.astype(int)
#                 top_right = corners[0].ravel()
#                 bottom_left = corners[1].ravel()
#                 # bottom_left = corners[1].ravel()
#                 # bottom_left = corners[1].ravel()
#                 dist=((tvec[i][0][0])**2+(tvec[i][0][1])**2+(tvec[i][0][2])**2)**(1/2)
#                 anglebtw_marker_x= math.acos(float(x)/round(dist,2))
#                 anglebtw_marker_y= math.acos(float(y)/round(dist,2))
#                 anglebtw_marker_z= math.acos(float(z)/round(dist,2))
#                 frame_converted = cv.drawFrameAxes(frame_converted, self.cameraMatrix, self.cameraDistortion, rvec[i], tvec[i], 3, 2)
#                 text_frame =cv.putText(frame_converted, f"id:{ids[0]} Dist: {round(dist,2)}",top_right ,cv.FONT_HERSHEY_PLAIN, 1.3, (0, 0, 255), 2, cv.LINE_AA)
#                 text_frame = cv.resize(text_frame, (800, 450))
#                 cv.imshow("text",text_frame)
#                 cv.waitKey(1)
#             # print(f"rvec",rvec)
#             # print(f"tvec", tvec)
#             # rvec1=rvec.ravel()
#             # print(f"rotation matrix", rotation_matrix)
#             # print(f"rvec_degrees", rvec_degrees)
#             # rotation_in_y_rad = rvec1[1]
#             # print(f"rotation_in_Y_RAD", rotation_in_y_rad)
#             # rotation_in_y_deg = rvec_degrees[1]
#             # print(f"rotation_in_Y_DEG", rotation_in_y_deg)
#             # print(anglebtw_marker_x,anglebtw_marker_y,anglebtw_marker_z)
            
#         else:
#             print("ARUCO " + str(id_to_find) + " NOT FOUND IN FRAME.")
#             print("")

        

# def main(args=None):
#     rclpy.init(args=args)
#     node = PoseEstimator()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == "__main__":
#     main()

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv
import numpy as np
from geometry_msgs.msg import TransformStamped
import tf2_ros
import tf_transformations
from scipy.spatial.transform import Rotation as R


class ArucoPoseEstimator(Node):
    def __init__(self):
        super().__init__("aruco_pose_estimator")
        self.subscription = self.create_subscription(Image, "/camera/image_raw", self.pose_estimation, 10)
        # self.subscription = self.create_subscription(Image, "/camera/image_raw", self.pose_estimation, 10)
        self.bridge = CvBridge()
        self.get_logger().info("Aruco Pose Estimator node has started")

    def pose_estimation(self, msg):
        # Load the ArUco dictionary and parameters
        dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_6X6_250)
        parameters = cv.aruco.DetectorParameters()

        # Camera matrix and distortion coefficients (should be calibrated)
        camera_matrix = np.array([[381.583424, 0, 319.420259],
                                  [0, 381.625150, 239.308262],
                                  [0, 0, 1.00]], dtype="double")
        distortion_coeffs = np.array([-0.000630, 0.000897,-0.000228, 0.000161, 0.0000], dtype="double")

        # Convert the ROS Image message to an OpenCV image
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        cv.imshow("frame", gray)
        cv.waitKey(1)
        # Detect ArUco markers
        detector = cv.aruco.ArucoDetector(dictionary, parameters)
        corners, ids, _ = cv.aruco.detectMarkers(gray, dictionary)

        # If markers are detected
        if ids is not None:
            # For each detected marker, estimate the pose
            for i in range(len(ids)):
                rvec, tvec, _ = cv.aruco.estimatePoseSingleMarkers(corners[i], 2, camera_matrix, distortion_coeffs)  # 0.2 is the marker size in meters

                # Print rotational and translational vectors
                self.get_logger().info(f"Marker ID: {ids[i][0]}")
                self.get_logger().info(f"Rotation Vector (rvec):\n{rvec}")
                self.get_logger().info(f"Translation Vector (tvec):\n{tvec}")

                self.get_logger().info(f"Marker ID: {ids[i][0]}")
                self.get_logger().info(f"Rotation Vector (rvec):\n{rvec}")
                self.get_logger().info(f"Translation Vector (tvec):\n{tvec}")

                # Create a TransformStamped message
                self.br = tf2_ros.TransformBroadcaster(self)

                transform = TransformStamped()

                # Set the timestamp and frame ID
                transform.header.stamp = self.get_clock().now().to_msg()
                transform.header.frame_id = "camera_optical_link"
                transform.child_frame_id = f"aruco_marker_{ids[i][0]}"

                # Set the translation
                transform.transform.translation.x = tvec[0][0][0]
                transform.transform.translation.y = tvec[0][0][1]
                transform.transform.translation.z = tvec[0][0][2]

                # Convert rvec to quaternion for rotation
                rotation_matrix, _ = cv.Rodrigues(rvec[0][0])
                print(rotation_matrix)
                r = R.from_matrix(rotation_matrix)
                quaternion = r.as_quat()  # Returns (x, y, z, w)

                # quaternion = tf_transformations.quaternion_from_matrix(rotation_matrix)

                transform.transform.rotation.x = quaternion[0]
                transform.transform.rotation.y = quaternion[1]
                transform.transform.rotation.z = quaternion[2]
                transform.transform.rotation.w = quaternion[3]

                # Broadcast the transform
                self.br.sendTransform(transform)
                self.get_logger().info("transform_published")
        else:
            self.get_logger().info("No ArUco marker detected.")

def main(args=None):
    rclpy.init(args=args)
    node = ArucoPoseEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()