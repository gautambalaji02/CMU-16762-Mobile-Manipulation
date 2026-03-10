import cv2
import yaml
import rclpy
import os.path as osp
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO
import detection_utils
import message_filters
import numpy as np


# Don't forget to start the camera before starting this node!
# Part 1: using in-gripper camera
#    ros2 launch stretch_core d405_basic.launch.py
# Part 2: using head camera
#    ros2 launch stretch_core d435i_low_resolution.launch.py
#
# ros2 run rviz2 rviz2 -d `ros2 pkg prefix --share stretch_calibration`/rviz/stretch_simple_test.rviz


class YOLOEObjectDetector(Node):
    def __init__(self, obj_queries):
        super().__init__('yoloe_object_detector')
        self.visualize = True

        # ----------- Camera Streaming Setup -----------

        # subscribe to the robot's color and aligned depth camera image topics from the gripper camera
        # using message_filters, instead of self.create_subscription() to allow us
        #   to synchronize the two camera streams can use a single callback that triggers when both come in
        # TODO: ------------- start --------------
        # leave as is for part 1, 
        # change for part 2 to use the head camera
        self.color_sub = message_filters.Subscriber(self, Image, '/gripper_camera/color/image_rect_raw')
        self.depth_sub = message_filters.Subscriber(self, Image, '/gripper_camera/aligned_depth_to_color/image_raw')
        self.color_cam_info_sub = message_filters.Subscriber(self, CameraInfo, '/gripper_camera/color/camera_info')
        # TODO: -------------- end ---------------
        self.latest_color = None
        self.latest_depth = None
        self.latest_color_cam_info = None

        # Use ApproximateTimeSynchronizer and register a callback function that runs within some time tolerance of when both images are received
        self.synchronizer = message_filters.ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub, self.color_cam_info_sub],
            queue_size=10,
            slop=0.01  # 10ms tolerance
        )
        self.synchronizer.registerCallback(self.image_callback)

        # bridge to convert ROS2 image messages to OpenCV images
        self.bridge = CvBridge()

        # -----------------------------------------------------

        # ----------- YOLO-E Object Detection SetuP -----------

        # Load the YOLOE model, which should already saved to common models directory on the robot
        #   we use yolo-e-v26-small for its high performance and low latency on limited compute
        model_path = '/home/hello-robot/models'
        model_name = 'yoloe-26s-seg.pt'
        self.model = YOLO(osp.join(model_path, model_name))

        # pass prompt for the object/s you want to detect
        self.obj_queries = obj_queries
        self.model.set_classes(self.obj_queries)

        # Run the detector and goals at a fixed frequency to reduce latency introduced by the detector
        #   and give the robot time to move between poses
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.publish_goals_callback)
        self.goal_pub = self.create_publisher(PoseStamped, '/object_detector/goal_pose', 10)
        self.goal_pose_msg = None

        # -----------------------------------------------------

    def image_callback(self, color_msg, depth_msg, color_cam_info_msg):
        # TODO: ------------- start --------------
        try:
            # YOLO-E expects RGB; cv_bridge gives us RGB directly with 'rgb8'
            self.latest_color = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='rgb8')
            # depth is 16-bit single channel, use passthrough to preserve raw mm values
            self.latest_depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
            self.latest_color_cam_info = color_cam_info_msg
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')
        # TODO: -------------- end ---------------


    def publish_goals_callback(self):
        # TODO: ------------- start --------------
        if self.latest_color is None:
            return

        # YOLO-E expects RGB — latest_color is already RGB from image_callback
        results = self.model.predict(self.latest_color)
        detections = detection_utils.parse_results(results)
        # TODO: -------------- end ---------------

        # create visualizations from the detections
        if self.visualize:
            detection_utils.visualize_detections_masks(
                # TODO: minor - change the part= arg when you edit your code for part 2! 
                #   adjusts the color scaling of the depth image display to match the camera range
                part=1, detections=detections, rgb_image=self.latest_color, depth_image=self.latest_depth)

        # get the goal pose and publish it, if it exists
        self.get_goal_pose(detections)

        # BUG FIX: starter code had self.pose_msg which is not an attribute of this class;
        #          the correct attribute initialized in __init__ is self.goal_pose_msg
        if self.goal_pose_msg is None:
            print("OBJECT NOT DETECTED, no pose to publish")
            return
        else:
            self.goal_pub.publish(self.goal_pose_msg)
            print()
            print("---------- Published Goal Pose ----------")


    def get_goal_pose(self, detections, target_idx=0):
        if detections is None or len(detections) == 0:
            return None

        # TODO: ------------- start --------------
        detection = detections[target_idx]
        centroid = detection['centroid']  # (x, y) in pixel coords

        # depth array is indexed [row, col] = [y, x], centroid is (x, y)
        z_depth = self.latest_depth[centroid[1], centroid[0]]

        if z_depth == 0:
            self.get_logger().warn('Depth at centroid is zero, skipping')
            return None

        xyz = detection_utils.pixel_to_3d(centroid, z_depth, self.latest_color_cam_info)

        self.goal_pose_msg = detection_utils.get_pose_msg(
            self.get_clock().now().to_msg(),
            'gripper_camera_color_optical_frame',
            xyz
        )
        # TODO: -------------- end ---------------


if __name__ == '__main__':
    rclpy.init()

    # load in the full list of object queries from the yaml file, as well as a target (if specified)
    with open('object_queries.yaml', 'r') as file:
        config = yaml.safe_load(file)
        obj_queries = config['queries']

    yolo_object_detector = YOLOEObjectDetector(obj_queries)
    rclpy.spin(yolo_object_detector)
    yolo_object_detector.destroy_node()
    rclpy.shutdown()
