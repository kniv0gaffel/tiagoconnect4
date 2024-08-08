#!/usr/bin/env python2

import rospy
import cv2
import cv2.aruco as aruco
import tf2_ros as tf2
from tf2_geometry_msgs import do_transform_pose
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import String
import numpy as np



class ArucoDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.tfBuffer = tf2.Buffer()
        self.image_sub = rospy.Subscriber("/xtion/rgb/image_raw", Image, self.image_callback)
        self.aruco_marker_pub = rospy.Publisher("/aruco/marker/pose", String, queue_size=10)
        self.aruco_piece_pub = rospy.Publisher("/aruco/piece/pose", String, queue_size=10)

        # Approximate camera calibration parameters
        # TODO: Calibrate the camera and use the actual parameters
        self.camera_matrix = np.array([[525.0, 0,     319.5],   # fx, 0, cx
                                       [0,     525.0, 239.5],   # 0, fy, cy
                                       [0,     0,     1    ]])          # 0, 0, 1

        self.dist_coeffs = np.array([0.1, -0.25, 0, 0, 0])   # [k1, k2, p1, p2, k3]

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("Could not convert from '{}' to 'bgr8'. Error: {}".format(data.encoding, e))
            return

    # Detect ArUco markers
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        parameters = aruco.DetectorParameters_create()

        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        if ids is not None and len(corners) > 0:
            # Ensure estimatePoseSingleMarkers returns the expected number of values
            estimate_result = aruco.estimatePoseSingleMarkers(corners, 0.05, self.camera_matrix, self.dist_coeffs)

            if len(estimate_result) == 2:
                rvecs, tvecs = estimate_result

                for i, marker_id in enumerate(ids):
                    # Draw bounding box around the marker
                    cv2.polylines(cv_image, [corners[i].astype(int)], True, (0, 255, 0), 2)
                    # Draw axis for the marker
                    aruco.drawAxis(cv_image, self.camera_matrix, self.dist_coeffs, rvecs[i], tvecs[i], 0.1)
                    # Log and publish the ID and pose of the marker
                    translation = tvecs[i].flatten()
                    rotation = rvecs[i].flatten()
                    rospy.loginfo("Detected ArUco ID: {} Translation: {} Rotation: {}".format(marker_id[0], translation, rotation))


                    aruco_pose = PoseStamped()
                    aruco_pose.pose.position.x = translation[0]
                    aruco_pose.pose.position.y = translation[1]
                    aruco_pose.pose.position.z = translation[2]
                    aruco_pose.pose.orientation = tf2.transformations.quaternion_from_euler(rotation[0], rotation[1], rotation[2])
                    aruco_pose.header.stamp = rospy.Time.now()
                    aruco_pose.header.frame_id = data.header.frame_id
                    aruco_pose.header.child_frame_id = "aruco_" + str(marker_id[0])
                    aruco_pose.header.frame_id = self.strip_leading_slash(aruco_pose.header.frame_id)
                    
                    rospy.loginfo("Marker in camera frame" + str(aruco_pose))
                    self.aruco_marker_pub.publish(aruco_pose)

                    rospy.loginfo("Transforming from frame: " +
                    aruco_pose.header.frame_id + " to 'base_footprint'")
                    ps = PoseStamped()
                    ps.pose.position = aruco_pose.pose.position
                    ps.header.stamp = self.tfBuffer.get_latest_common_time("base_footprint", aruco_pose.header.frame_id)
                    ps.header.frame_id = aruco_pose.header.frame_id
                    transform_ok = False
                    while not transform_ok and not rospy.is_shutdown():
                        try:
                            transform = self.tfBuffer.lookup_transform("base_footprint", 
                                                   ps.header.frame_id,
                                                   rospy.Time(0))
                            aruco_ps = do_transform_pose(ps, transform)
                            transform_ok = True
                        except tf2.ExtrapolationException as e:
                            rospy.logwarn(
                                "Exception on transforming point... trying again \n(" +
                                str(e) + ")")
                            rospy.sleep(0.01)
                            ps.header.stamp = self.tfBuffer.get_latest_common_time("base_footprint", aruco_pose.header.frame_id)
                        piece = PoseStamped()

                    # TODO set the pose of the object to be picked
                    # make an offset to the pose of the marker
                    rospy.loginfo("Setting chip pose based on ArUco detection")
                    piece.pose.position = aruco_ps.pose.position
                    piece.pose.position.y += 0.01
                    piece.pose.position.z += 0.01
                    piece.header.frame_id = 'base_footprint'
                    rospy.loginfo("Piece in base_footprint frame" + str(piece))
                    self.aruco_piece_pub.publish(piece)

                    # Optionally display the translation and rotation on the image
                    cv2.putText(cv_image, "ID: {} T: {} R: {}".format(marker_id[0], translation, rotation),
                                tuple(corners[i][0][0].astype(int)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2, cv2.LINE_AA)
            else:
                rospy.logwarn("estimatePoseSingleMarkers returned an unexpected number of values.")
        else:
            rospy.loginfo("No ArUco markers detected.")

        # Display the image with the detected markers and pose
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)

    def cleanup(self):
        cv2.destroyAllWindows()



if __name__ == '__main__':
    rospy.init_node('aruco_detector', anonymous=True)
    detector = ArucoDetector()
    rospy.on_shutdown(detector.cleanup)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down ArUco Detector Node")
