import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import numpy as np
import time
import math
import cv2
import threading
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool
from sensor_msgs.msg import Image # pour gazebo
from cv_bridge import CvBridge # pour gazebo

class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_node')
        # Subs
        ## Gazebo
        self.bridge = CvBridge()
        self.camera_sub = self.create_subscription(
            #CompressedImage,
            Image,
            #'/camera/image_raw/compressed',
            '/image_raw', # en simu !
            self.listener_callback,
            10
        )

        # Client
        self.client_controle = self.create_client(SetBool, '/blueline_status')

        # Pub
        self.publisher = self.create_publisher(Twist,'/cmd_vel',10)
        self.twist = Twist()

        # Variables pour stocker les dernières données reçues
        self.latest_image = None
        self.image = None
        self.horizon = 250
        self.margin  = 160

        self.middle_screen =  None
        self.middle_point = None
        self.steerdir = None
        self.blue_centroid = None

        # controle de node
        self.active = True
        self.srv = self.create_service(SetBool, '/activate_blueline', self.handle_activation)

    # Callback du service
    def handle_activation(self, request, response):
        self.active = request.data
        response.success = True
        response.message = f"Blue Line node state: {self.active}"
        return response

    def call_service(self, client, state):
        if not client.wait_for_service(timeout_sec=1.0):
            return
        req = SetBool.Request()
        req.data = state
        client.call_async(req)

    def listener_callback(self, msg):
        if not self.active:
            return  # Ignore les données LiDAR si la node est inactive
        self.latest_image = msg
        self.affichage(self.latest_image)

    # fonctions caméra
    def affichage(self, latest_image):
        np_arr = np.frombuffer(latest_image.data, np.uint8)
        
        # Décoder l'image
        #image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) 
        image = self.bridge.imgmsg_to_cv2(latest_image, desired_encoding='bgr8') ## gazebo

        if image is not None:
            self.image = image
            display = self.hsv_segmentation(image)
            display = cv2.cvtColor(display, cv2.COLOR_HSV2BGR) if display.shape[2] == 3 else display

            # Horizon line in white
            cv2.line(display, (0, self.horizon), (display.shape[1], self.horizon), (255, 255, 255), 1)                       

            # fifth screen dividers in grey
            third= display.shape[1] // 3
            cv2.line(display, (third, self.horizon), (third, display.shape[0]), (100, 100, 100), 1)
            cv2.line(display, (2 * third, self.horizon), (2 * third, display.shape[0]), (100, 100, 100), 1)

            cv2.imshow("Compressed Image", display)
            cv2.waitKey(1)

        else:
            self.get_logger().warn("Failed to decode compressed image")

    def hsv_segmentation(self,image):
            image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

            lower_blue = np.array([95,140,150])
            upper_blue = np.array([130,255,255])

            mask = cv2.inRange(image_hsv, lower_blue, upper_blue)

            hsv_seg = cv2.bitwise_and(image, image, mask=mask)
            hsv_seg = cv2.cvtColor(hsv_seg, cv2.COLOR_BGR2HSV)
            return hsv_seg

    def steer(self, image_hsv):

        width = image_hsv.shape[1]
        height = image_hsv.shape[0]
        third_screen = width // 3

        horizon = min(self.horizon, height - 1)

        lower_roi = image_hsv[self.horizon:, :, :]
        h_lroi = lower_roi[:, :, 0]
        s_lroi = lower_roi[:, :, 1]
        v_lroi = lower_roi[:, :, 2]

        blue_mask = ((h_lroi >= 95) & (h_lroi <= 130) & (s_lroi >= 140) & (v_lroi >= 190))

        # 2D binary images for moment calculation
        blue_binary = np.zeros(lower_roi.shape[:2], dtype=np.uint8)
        blue_binary[blue_mask] = 255

        blue_centroid = self.centroid(blue_binary)

        # Store for display (offset back to full image coordinates)
        self.blue_centroid = (blue_centroid[0], blue_centroid[1] + self.horizon) if blue_centroid else None

        blue_valid = (
            blue_centroid is not None and
            self.horizon <= blue_centroid[1] + horizon < height and  # in lower screen
            third_screen < blue_centroid[0] < 2 * third_screen      # in middle third
        )

        if blue_valid :
            self.get_logger().info("Ligne bleue détectée en face !!")
            self.call_service(self.client_controle, True)

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
            if self.image is not None :
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
        #rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()