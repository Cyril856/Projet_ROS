import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import LaserScan
import numpy as np
import time
import math
import cv2
import threading
from geometry_msgs.msg import Twist

class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_node')
        # Subs
        self.camera_sub = self.create_subscription(
            CompressedImage,
            '/camera/image_raw/compressed',
            self.listener_callback,
            10
        )

        # Pub
        self.publisher = self.create_publisher(Twist,'/cmd_vel',10)
        self.twist = Twist()

        # Variables pour stocker les dernières données reçues
        self.latest_image = None
        self.image = None
        self.horizon = 130
        self.margin  = 160

        self.middle_screen =  None
        self.middle_point = None
        self.steerdir = None
        self.blue_centroid = None

    def listener_callback(self, msg):
        self.latest_image = msg
        self.affichage(self.latest_image)

    # fonctions caméra
    def affichage(self, latest_image):
        np_arr = np.frombuffer(latest_image.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if image is not None:
            self.image = image
            display = self.hsv_segmentation(image)
            display = cv2.cvtColor(display, cv2.COLOR_HSV2BGR) if display.shape[2] == 3 else display

            # Horizon line in white
            cv2.line(display, (0, self.horizon), (display.shape[1], self.horizon), (255, 255, 255), 1)                       

            # fifth screen dividers in grey
            third= display.shape[1] // 3
            cv2.line(display, (third, self.horizon), (third, display.shape[0]), (100, 100, 100), 1)
            cv2.line(display, (4 * third, self.horizon), (4 * third, display.shape[0]), (100, 100, 100), 1)

            cv2.imshow("Compressed Image", display)
            cv2.waitKey(1)

        else:
            self.get_logger().warn("Failed to decode compressed image")

    def hsv_segmentation(self,image):
            image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

            lower_blue = np.array([48, 17, 24])
            upper_blue = np.array([96, 255, 255])

            mask_blue = cv2.inRange(image_hsv, lower_blue, upper_blue)


            mask = cv2.bitwise_or(mask_blue)

            hsv_seg = cv2.bitwise_and(image, image, mask=mask)
            hsv_seg = cv2.cvtColor(hsv_seg, cv2.COLOR_BGR2HSV)
            return hsv_seg

    def steer(self, image_hsv):
        linear_scale = self.get_parameter('linear_scale').value
        angular_scale = self.get_parameter('angular_scale').value

        width = image_hsv.shape[1]
        height = image_hsv.shape[0]
        third_screen = width // 3

        horizon = min(self.horizon, height - 1)

        lower_roi = image_hsv[self.horizon:, :, :]
        h_lroi = lower_roi[:, :, 0]
        s_lroi = lower_roi[:, :, 1]
        v_lroi = lower_roi[:, :, 2]

        blue_mask = ((h_lroi >= 30) & (h_lroi <= 93) & (s_lroi >= 30) & (v_lroi >= 100)) ## à modifier

        # 2D binary images for moment calculation
        blue_binary = np.zeros(lower_roi.shape[:2], dtype=np.uint8)
        blue_binary[blue_mask] = 255

        blue_centroid = self.centroid(blue_binary)

        # Store for display (offset back to full image coordinates)
        self.blue_centroid = (blue_centroid[0], blue_centroid[1] + self.horizon) if blue_centroid else None

        blue_valid = (
            blue_centroid is not None and
            self.horizon <= blue_centroid[1] + horizon < height and  # in lower screen
            third_screen > blue_centroid[0] > 2 * third_screen      # in middle third
        )

        if blue_valid :
            self.get_logger().info("Ligne bleue détectée en face !!")

    def centroid(self, image_bin):
        M = cv2.moments(image_bin)

        # Avoid division by zero if mask is empty
        if M["m00"] == 0:
            return None

        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        return cX, cY


    def run(self):
        rate = self.create_rate(20)

        while rclpy.ok():
            if self.image is not None and not(self.roundabout_mode):
                image_hsv = self.hsv_segmentation(self.image)
                self.steer(image_hsv)
            rate.sleep()

def main(args=None):
    rclpy.init(args=args)

    node = ObstacleAvoidance()
    thread =  threading.Thread(target=rclpy.spin, args=(node,),daemon=True)
    thread.start()

    try:
        node.run()

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()