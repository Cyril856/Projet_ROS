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

class Goal(Node):
    def __init__(self):
        super().__init__('goal_node')
        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed', #/camera
            self.listener_callback,
            10
        )

        self.lidar_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )

        self.message = Twist()
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.image = None
        self.regions = {}

        self.ball_centroid = None
        self.left_pole_centroid = None
        self.right_pole_centroid = None
        self.goal_centroid = None

    def listener_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if image is not None:
            self.image = image.copy()

    def hsv_segmentation(self, image):
        image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        lower_yellow = np.array([20, 80, 20])
        upper_yellow = np.array([35, 255, 255])

        lower_red1 = np.array([0, 30, 20])
        upper_red1 = np.array([10, 255, 255])

        lower_red2 = np.array([160, 30, 140])
        upper_red2 = np.array([179, 255, 255])

        mask_yellow = cv2.inRange(image_hsv, lower_yellow, upper_yellow)
        mask_red1   = cv2.inRange(image_hsv, lower_red1,   upper_red1)
        mask_red2   = cv2.inRange(image_hsv, lower_red2,   upper_red2)
        mask_red    = cv2.bitwise_or(mask_red1, mask_red2)

        return image_hsv, mask_yellow, mask_red

    def safe_min(self, ranges_slice, default=float('inf')):
        filtered = [x for x in ranges_slice if not math.isinf(x) and not math.isnan(x) and x > 0.0]
        return min(filtered) if filtered else default

    def lidar_callback(self, msg):
        self.regions = {
            'front' : self.safe_min(msg.ranges[345:360] + msg.ranges[0:15]),
            'fleft' : self.safe_min(msg.ranges[16:75]),
            'left'  : self.safe_min(msg.ranges[76:120]),
            'right' : self.safe_min(msg.ranges[240:285]),
            'fright': self.safe_min(msg.ranges[286:345])
        }

    def goal_lidar(self):
        if not self.regions:
            return False

        front  = self.regions.get('front',  float('inf'))
        left  = self.regions.get('fleft',  float('inf'))
        right = self.regions.get('fright', float('inf'))

        pole_range = 1.5 

        poles_detected = (left < pole_range and right < pole_range)

        self.get_logger().info(
            f"Lidar | front: {front:.2f} | fleft: {left:.2f} | fright: {right:.2f} | "
            f"poles_detected: {poles_detected}"
        )

        return poles_detected

    def centroid(self, binary):
        M = cv2.moments(binary)
        if M["m00"] == 0:
            return None
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        return cX, cY

    def compute_centroids(self):
        image = self.image
        if image is None:
            return

        image_hsv, mask_yellow, mask_red = self.hsv_segmentation(image)

        self.ball_centroid = self.centroid(mask_yellow)

        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(
            mask_red, connectivity=8
        )

        min_area = 20
        pole_centroids = []
        for i in range(1, num_labels):
            if stats[i, cv2.CC_STAT_AREA] >= min_area:
                pole_centroids.append((int(centroids[i][0]), int(centroids[i][1])))

        if len(pole_centroids) >= 2:
            pole_centroids.sort(key=lambda c: c[0])
            self.left_pole_centroid  = pole_centroids[0]
            self.right_pole_centroid = pole_centroids[-1]
            self.goal_centroid = (
                (self.left_pole_centroid[0] + self.right_pole_centroid[0]) // 2,
                (self.left_pole_centroid[1] + self.right_pole_centroid[1]) // 2
            )
        elif len(pole_centroids) == 1:
            self.left_pole_centroid  = pole_centroids[0]
            self.right_pole_centroid = None
            self.goal_centroid       = None
        else:
            self.left_pole_centroid  = None
            self.right_pole_centroid = None
            self.goal_centroid       = None

        self.get_logger().info(
            f"ball: {self.ball_centroid} | "
            f"left pole: {self.left_pole_centroid} | "
            f"right pole: {self.right_pole_centroid} | "
            f"goal center: {self.goal_centroid}"
        )

    def display(self):
        image = self.image
        if image is None:
            return

        display = image.copy()

        if self.ball_centroid is not None:
            cv2.circle(display, self.ball_centroid, 7, (0, 255, 255), -1)
            cv2.putText(display, "Ball", (self.ball_centroid[0] + 8, self.ball_centroid[1]),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)

        if self.left_pole_centroid is not None:
            cv2.circle(display, self.left_pole_centroid, 7, (0, 0, 255), -1)
            cv2.putText(display, "L pole", (self.left_pole_centroid[0] + 8, self.left_pole_centroid[1]),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)

        if self.right_pole_centroid is not None:
            cv2.circle(display, self.right_pole_centroid, 7, (0, 0, 255), -1)
            cv2.putText(display, "R pole", (self.right_pole_centroid[0] + 8, self.right_pole_centroid[1]),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)

        if self.goal_centroid is not None:
            cv2.circle(display, self.goal_centroid, 5, (255, 0, 0), -1)
            if self.left_pole_centroid and self.right_pole_centroid:
                cv2.line(display, self.left_pole_centroid, self.right_pole_centroid, (255, 255, 255), 1)

        cv2.imshow("Goal View", display)
        cv2.waitKey(1)

    def run(self):
        self.get_logger().info("Goal node started")
        while rclpy.ok():
            if self.image is not None:
                self.compute_centroids()
                self.display()
                
                msg = Twist()
                
                if self.ball_centroid is not None and self.goal_centroid is not None:
                    ball_x = self.ball_centroid[0]
                    goal_x = self.goal_centroid[0]
                    
                    alignment_error = ball_x - goal_x
                    

                    if alignment_error < -20: 
                        msg.linear.x = -0.05   
                        msg.angular.z = 0.2    

                    elif alignment_error > 20:
                        msg.linear.x = -0.05   
                        msg.angular.z = -0.2   
                    else:
                        msg.linear.x = 0.15    
                        msg.angular.z = 0.0
                
                elif self.ball_centroid is not None:
                    img_center = self.image.shape[1] // 2
                    if self.ball_centroid[0] < img_center - 20:
                        msg.angular.z = 0.2
                    elif self.ball_centroid[0] > img_center + 20:
                        msg.angular.z = -0.2
                    else:
                        msg.linear.x = 0.05
                else:
                    
                    msg.angular.z = 0.3
                
                self.publisher.publish(msg)

            time.sleep(0.05)


def main(args=None):
    rclpy.init(args=args)
    node = Goal()

    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
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