import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import LaserScan
import numpy as np
import time
import math
import cv2
import threading
from std_srvs.srv import SetBool
from geometry_msgs.msg import Twist


class Goal(Node):
    def __init__(self):
        super().__init__('goal_node')

        self.subscription = self.create_subscription(
            CompressedImage,
            '/camera/image_raw/compressed', #/camera
            self.listener_callback,
            10
        )

        self.message = Twist()
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.image = None

        self.ball_centroid = None
        self.ball_detected = False
        self.left_pole_centroid = None
        self.right_pole_centroid = None
        self.goal_centroid = None

        # controle de node
        self.active = True
        self.srv = self.create_service(SetBool, '/activate_goal', self.handle_activation)

      # Callback du service
    def handle_activation(self, request, response):
        self.active = request.data
        response.success = True
        response.message = f"Goal node state: {self.active}"
        return response

    def listener_callback(self, msg):
        if not self.active:
            return  # Ignore les images si la node est inactive
        np_arr = np.frombuffer(msg.data, np.uint8)
        
        # Décoder l'image
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) 

        if image is not None:
            self.image = image.copy()

    def hsv_segmentation(self, image):
        image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        lower_yellow = np.array([20, 10, 40])
        upper_yellow = np.array([35, 255, 255])

        lower_red1 = np.array([0, 130, 30])
        upper_red1 = np.array([10, 255, 150])

        lower_red2 = np.array([160, 130, 30])
        upper_red2 = np.array([179, 255, 150])

        mask_yellow = cv2.inRange(image_hsv, lower_yellow, upper_yellow)
        mask_red1   = cv2.inRange(image_hsv, lower_red1,   upper_red1)
        mask_red2   = cv2.inRange(image_hsv, lower_red2,   upper_red2)
        mask_red    = cv2.bitwise_or(mask_red1, mask_red2)

        return image_hsv, mask_yellow, mask_red

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

        height = image.shape[0]
        upper_limit = height // 2  # only look at top half for poles

        image_hsv, mask_yellow, mask_red = self.hsv_segmentation(image)

        # Ball centroid from full image
        self.ball_centroid = self.centroid(mask_yellow)

        # Poles from upper region only
        mask_red_upper = mask_red[:upper_limit, :]

        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(
            mask_red_upper, connectivity=8
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
        last_angular = 0.0
        current_time = time.time()
        
        while rclpy.ok():
            if not self.active:
                time.sleep(0.1)
                continue

            self.get_logger().info("Goal node started")

            if self.image is not None:
                self.compute_centroids()
                self.display()
                
                msg = Twist()
                img_center = self.image.shape[1] // 2

                
                if self.ball_centroid is not None:
                    self.ball_detected=True
                    ball_x = self.ball_centroid[0]
                    
                    if self.goal_centroid is not None:
                        target_x = self.goal_centroid[0]
                        error = ball_x - target_x
                    else:
                        if self.left_pole_centroid is not None:
                            error = 1.2 * (ball_x - self.left_pole_centroid[0])+10
                        elif self.right_pole_centroid is not None:
                            error = 1.2 * (ball_x - self.right_pole_centroid[0])-10
                        else:
                            self.get_logger().info("Looking for pole and centering on ball")
                            error=ball_x - img_center
                            msg.angular.z = float(error) / img_center * 0.5 + 0.1

                    msg.angular.z = -float(error) / img_center * 0.5
                    msg.linear.x = 0.05
                    last_angular = msg.angular.z 

                else:
                    if not self.ball_detected:
                        self.get_logger().info("Looking for ball")
                        msg.angular.z=-0.5
                    else:
                        if self.left_pole_centroid is not None and self.left_pole_centroid[1]>100:
                            self.get_logger().info("Ball received and heading for goal")
                            msg.linear.x = 0.07
                            msg.angular.z = last_angular * 0.05 
                        elif self.left_pole_centroid is not None:
                            error = 1.2 * (ball_x - self.left_pole_centroid[0])+10
                            msg.angular.z = -float(error) / img_center * 0.5
                        elif self.right_pole_centroid is not None:
                            error = 1.2 * (ball_x - self.right_pole_centroid[0])-10
                            msg.angular.z = -float(error) / img_center * 0.5
                        else:
                            msg.angular.z=0.0
                            msg.linear.x=0.0

                        pole_recently_seen = (self.last_pole_seen_time is not None and 
                                         (current_time - self.last_pole_seen_time) < 3.0)

                        if pole_recently_seen:
                            self.get_logger().info("Poteau perdu mais on continue l'attaque (3s buffer)")
                            msg.linear.x = 0.09
                        else:
                            self.get_logger().info("3 secondes écoulées sans poteau : STOP")
                            msg.linear.x = 0.0
                            msg.angular.z = 0.0

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
        #rclpy.shutdown() # ne pas activer sinon risque de shutdown toutes les nodes
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()