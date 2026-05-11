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



class LineFollowing(Node):
    def __init__(self):
        super().__init__('compressed_image_subscriber')
        
        # Gazebo
        #self.bridge = CvBridge()

        self.camera_sub = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed', #/camera
            self.listener_callback,
            10
        )

        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )
        #self.subscription  # to prevent unused variable warning

        self.message = Twist()
        self.publisher = self.create_publisher(Twist,'/cmd_vel',10)

        self.latest_image = None
        self.latest_scan = None
    
        self.image = None
        self.horizon = 275 # #130

        self.middle_screen =  None
        self.middle_point = None
        self.steerdir = None
        self.margin  = 350 # #135

        self.green_centroid = None
        self.red_centroid = None

        self.upper_green_centroid = None
        self.upper_red_centroid = None

        self.roundabout_mode= False
        self.roundabout_count=0

        self.declare_parameter('RAB_direction', 'R') 
        self.roundabout_dir = self.get_parameter('RAB_direction').value

        self.declare_parameter('linear_scale', 1.0)
        self.declare_parameter('angular_scale',1.0)

        # STOP
        self.declare_parameter('emergency_stop_dist', 0.2)
        self.emergency_stop_dist = self.get_parameter('emergency_stop_dist').value
        self.stop = False

        # controle de node
        self.active = False
        self.srv = self.create_service(SetBool, '/activate_linefollow', self.handle_activation)

      # Callback du service
    def handle_activation(self, request, response):
        self.active = request.data
        response.success = True
        response.message = f"Line Following node state: {self.active}"
        return response

    def safe_min(self, ranges_slice, default=float('inf')):
            filtered = [x for x in ranges_slice if not math.isinf(x) and not math.isnan(x) and x > 0.0]
            return min(filtered) if filtered else default

    def lidar_callback(self, msg):
        if not self.active:
            return  # Ignore les données LiDAR si la node est inactive
        self.latest_scan = msg
        front = self.safe_min(msg.ranges[345:360] + msg.ranges[0:15])
        if front < self.emergency_stop_dist:
            self.stop = True
            self.message.linear.x = 0.0
            self.message.angular.z = 0.0
            self.publisher.publish(self.message)
            self.get_logger().warn(f"EMERGENCY STOP — obstacle à {front:.2f} m")
        else : 
            self.stop = False
    
    def listener_callback(self, msg):
        if not self.active:
            return  # Ignore les images si la node est inactive
        self.latest_image = msg

        if self.stop:
            return
        # Convertir les données compressées en tableau numpy
        np_arr = np.frombuffer(msg.data, np.uint8)
        # Décoder l'image
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        #image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        if image is not None:
            self.image = image
            display = self.hsv_segmentation(image)
            display = cv2.cvtColor(display, cv2.COLOR_HSV2BGR) if display.shape[2] == 3 else display

            # Horizon line in white
            cv2.line(display, (0, self.horizon), (display.shape[1], self.horizon), (255, 255, 255), 1)

            # fifth screen dividers in grey
            fifth= display.shape[1] // 5
            cv2.line(display, (fifth, self.horizon), (fifth, display.shape[0]), (100, 100, 100), 1)
            cv2.line(display, (4 * fifth, self.horizon), (4 * fifth, display.shape[0]), (100, 100, 100), 1)

            # Green centroid in green
            if self.green_centroid is not None:
                cv2.circle(display, self.green_centroid, 7, (0, 255, 0), -1)

            # Red centroid in red
            if self.red_centroid is not None:
                cv2.circle(display, self.red_centroid, 7, (0, 0, 255), -1)

            # Middle screen in blue
            if self.middle_screen is not None:
                cv2.circle(display, (self.middle_screen, self.horizon), 5, (255, 0, 0), -1)

            # Steering target (middle_point) in yellow
            if self.middle_point is not None:
                cv2.circle(display, (self.middle_point, self.horizon), 5, (0, 255, 255), -1)

            # Line between centroids if both visible
            if self.green_centroid is not None and self.red_centroid is not None:
                cv2.line(display, self.green_centroid, self.red_centroid, (255, 255, 255), 1)

            # Full image view with centroids
            full_display = image.copy()

            # Horizon line
            cv2.line(full_display, (0, self.horizon), (full_display.shape[1], self.horizon), (255, 255, 255), 1)

            # Third screen dividers
            third = full_display.shape[1] // 3
            cv2.line(full_display, (third, self.horizon), (third, full_display.shape[0]), (100, 100, 100), 1)
            cv2.line(full_display, (2 * third, self.horizon), (2 * third, full_display.shape[0]), (100, 100, 100), 1)


            if self.upper_green_centroid is not None:
                cv2.circle(full_display, self.upper_green_centroid, 7, (0, 255, 0), -1)
                cv2.putText(full_display, f"G({self.upper_green_centroid[0]},{self.upper_green_centroid[1]})",
                            (self.upper_green_centroid[0] + 10, self.upper_green_centroid[1]),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)

            if self.upper_red_centroid is not None:
                cv2.circle(full_display, self.upper_red_centroid, 7, (0, 0, 255), -1)
                cv2.putText(full_display, f"R({self.upper_red_centroid[0]},{self.upper_red_centroid[1]})",
                            (self.upper_red_centroid[0] + 10, self.upper_red_centroid[1]),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
                
            # Line between centroids
            if self.green_centroid is not None and self.red_centroid is not None:
                cv2.line(full_display, self.green_centroid, self.red_centroid, (255, 255, 255), 1)

            cv2.imshow("Full View", full_display)
                        

            cv2.imshow("Compressed Image", display)
            cv2.waitKey(1)

        else:
            self.get_logger().warn("Failed to decode compressed image")

    def hsv_segmentation(self,image):
            image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

            lower_red1 = np.array([0, 60, 60])
            upper_red1 = np.array([16, 255, 255])

            lower_red2 = np.array([160, 60, 60])
            upper_red2 = np.array([179, 255, 255])

            lower_green = np.array([30, 60, 60])
            upper_green = np.array([94, 255, 255])

            mask_red1 = cv2.inRange(image_hsv, lower_red1, upper_red1)
            mask_red2 = cv2.inRange(image_hsv, lower_red2, upper_red2)
            mask_yellow = cv2.inRange(image_hsv, lower_green, upper_green)


            mask = cv2.bitwise_or(mask_yellow, cv2.bitwise_or(mask_red2,mask_red1))

            hsv_seg = cv2.bitwise_and(image, image, mask=mask)
            hsv_seg = cv2.cvtColor(hsv_seg, cv2.COLOR_BGR2HSV)
            return hsv_seg

    def steer(self, image_hsv):
        if self.stop:    # okasou
            return
        linear_scale = self.get_parameter('linear_scale').value
        angular_scale = self.get_parameter('angular_scale').value

        width = image_hsv.shape[1]
        height = image_hsv.shape[0]
        fifth_screen = width // 5
        self.middle_screen = width // 2

        horizon = min(self.horizon, height - 1)

        lower_roi = image_hsv[horizon:, :, :]
        h_lroi = lower_roi[:, :, 0]
        s_lroi = lower_roi[:, :, 1]
        v_lroi = lower_roi[:, :, 2]

        green_mask = ((h_lroi >= 30) & (h_lroi <= 94) & (s_lroi >= 60) & (v_lroi >= 60))
        red_mask = (((h_lroi <= 10) | (h_lroi >= 160)) & (s_lroi >= 60)& (v_lroi >= 60))

        # 2D binary images for moment calculation
        green_binary = np.zeros(lower_roi.shape[:2], dtype=np.uint8)
        green_binary[green_mask] = 255
        red_binary = np.zeros(lower_roi.shape[:2], dtype=np.uint8)
        red_binary[red_mask] = 255


        green_centroid = self.centroid(green_binary)
        red_centroid = self.centroid(red_binary)

        # Store for display (offset back to full image coordinates)
        self.green_centroid = (green_centroid[0], green_centroid[1] + self.horizon) if green_centroid else None
        self.red_centroid = (red_centroid[0], red_centroid[1] + self.horizon) if red_centroid else None

        self.get_logger().info(f"green_centroid: {green_centroid} | red_centroid: {red_centroid}")
        self.get_logger().info(f"green pixels: {np.sum(green_mask)} | red pixels: {np.sum(red_mask)}")
        self.get_logger().info(f"image shape: {image_hsv.shape} | horizon: {self.horizon}")

        # Upper ROI centroids
        upper_roi = image_hsv[:horizon, :, :]
        h_uroi = upper_roi[:, :, 0]
        s_uroi = upper_roi[:, :, 1]
        v_uroi = upper_roi[:, :, 2]

        upper_green_mask = ((h_uroi >= 30) & (h_uroi <= 94) & (s_uroi >= 60) & (v_uroi >= 60))
        upper_red_mask = (((h_uroi <= 10) | (h_uroi >= 160)) & (s_uroi >= 60)& (v_uroi >= 60))

        upper_green_binary = np.zeros(upper_roi.shape[:2], dtype=np.uint8)
        upper_green_binary[upper_green_mask] = 255
        upper_red_binary = np.zeros(upper_roi.shape[:2], dtype=np.uint8)
        upper_red_binary[upper_red_mask] = 255

        upper_green_centroid = self.centroid(upper_green_binary)
        upper_red_centroid = self.centroid(upper_red_binary)

        # No horizon offset needed since upper ROI starts at row 0
        self.upper_green_centroid = upper_green_centroid if upper_green_centroid else None
        self.upper_red_centroid = upper_red_centroid if upper_red_centroid else None
        
        RAB_cond=(
            self.upper_green_centroid is not None and 
            self.upper_red_centroid is not None and 
            self.upper_green_centroid[0] > self.upper_red_centroid[0] and 
            abs(self.upper_green_centroid[0] - self.upper_red_centroid[0]) < 120 and 
            abs(self.upper_green_centroid[1] - self.upper_red_centroid[1]) < 20
        )

        if RAB_cond:
            self.message.linear.x = 0.0
            self.message.angular.z = 0.0
            self.roundabout_mode=True
            return

        green_valid = (
            green_centroid is not None and
            self.horizon <= green_centroid[1] + horizon < height and 
            green_centroid[0] < 4 * fifth_screen                               
        )
        red_valid = (
            red_centroid is not None and
            self.horizon <= red_centroid[1] + horizon < height and    
            red_centroid[0] > fifth_screen                            
        )

        green_center = (
            green_centroid is not None and
            self.horizon <= green_centroid[1] + horizon < height and 
            fifth_screen < green_centroid[0]                           
        )

        red_center = (
            red_centroid is not None and
            self.horizon <= red_centroid[1] + horizon < height and    
            fifth_screen < red_centroid[0] < 4 * fifth_screen                        
        )


        if green_valid and red_valid:
            # Both lines visible: steer to middle of segment between them
            self.middle_point = (green_centroid[0] + red_centroid[0]) // 2
            drift = self.middle_point - self.middle_screen
            self.get_logger().info(
                f"Both lines | green: {green_centroid[0]} | red: {red_centroid[0]} | "
                f"middle: {self.middle_point} | drift: {drift}"
            )

        elif red_valid and not green_valid:
            # Only red visible: stay at fixed offset to its left
            self.middle_point = red_centroid[0] - self.margin
            drift = self.middle_point - self.middle_screen
            self.get_logger().warn(f"Green lost, steering from red | drift: {drift}")

        elif green_valid and not red_valid:
            # Only green visible: stay at fixed offset to its right
            self.middle_point = green_centroid[0] + self.margin
            drift = self.middle_point - self.middle_screen
            self.get_logger().warn(f"Red lost, steering from green | drift: {drift}")

        else:
            # No centroid found: search
            self.get_logger().warn("No lines detected, searching...")
            self.message.angular.z = 0.3 * angular_scale
            self.message.linear.x = 0.0
            self.publisher.publish(self.message)
            return
        
        
        ang_gain = 1.0
        lin_gain = 0.75
        if green_center or red_center:
            ang_gain = 5.0 # increase this to taste
            lin_gain = 0.85
        else:
            ang_gain = 1.0
            lin_gain = 0.75

        raw_angular = -float(drift) / (self.middle_screen * 2) * angular_scale * ang_gain
        self.message.angular.z = max(-0.3, min(0.3, raw_angular))
        turn_factor = 1.0 - min(abs(raw_angular) / 0.2, 1.0)
        self.message.linear.x = max(0.05, 0.1 * turn_factor) * linear_scale * lin_gain

        self.publisher.publish(self.message)


    def centroid(self, image_bin):
        M = cv2.moments(image_bin)

        if M["m00"] == 0:
            return None

        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        return cX, cY


    def run(self):
        while rclpy.ok():
            # Controle de node
            if not self.active:
                # Si inactive, on ne fait rien (mais on continue de tourner pour écouter les callbacks)
                time.sleep(0.1) ## à augmenter si trop faible 
                continue

            if self.stop:        
                time.sleep(0.05)
                continue

            self.get_logger().info("run() started")

            self.get_logger().info(f"loop tick | image: {self.image is not None} | roundabout: {self.roundabout_mode}")
            image = self.image
            if image is not None:
                self.get_logger().info("image received, calling steer...")
                if self.roundabout_mode and self.roundabout_count<1:
                    self.roundabout_protocol()
                    self.roundabout_count+=1
                    self.roundabout_mode = False
                    self.margin = 300 # #180
                    self.horizon = 200 # #150
                else:
                    image_hsv = self.hsv_segmentation(image.copy())
                    self.steer(image_hsv)
            time.sleep(0.05)

    def roundabout_protocol(self,forward_time=2.75, turn_time=2):
        if self.roundabout_dir=="R":
            linear_scale = self.get_parameter('linear_scale').value
            angular_scale = self.get_parameter('angular_scale').value

            # Go forward
            self.get_logger().info("go_and_turn: moving forward...")
            self.message.linear.x = 0.07 * linear_scale
            self.message.angular.z = 0.0
            self.publisher.publish(self.message)
            time.sleep(forward_time)

            # Turn right
            self.get_logger().info("go_and_turn: turning right...")
            self.message.linear.x = 0.0
            self.message.angular.z = -0.6 * angular_scale
            self.publisher.publish(self.message)
            time.sleep(turn_time)

            # Stop
            self.message.linear.x = 0.0
            self.message.angular.z = 0.0
            self.publisher.publish(self.message)

        elif self.roundabout_dir=="L":
            linear_scale = self.get_parameter('linear_scale').value
            angular_scale = self.get_parameter('angular_scale').value

            # Go forward
            self.get_logger().info("go_and_turn: moving forward...")
            self.message.linear.x = 0.07 * linear_scale
            self.message.angular.z = 0.0
            self.publisher.publish(self.message)
            time.sleep(forward_time)

            # Turn right
            self.get_logger().info("go_and_turn: turning right...")
            self.message.linear.x = 0.0
            self.message.angular.z = 0.6 * angular_scale
            self.publisher.publish(self.message)
            time.sleep(turn_time)

            # Stop
            self.message.linear.x = 0.0
            self.message.angular.z = 0.0
            self.publisher.publish(self.message)
            




def main(args=None):
    rclpy.init(args=args)

    node = LineFollowing()
    thread =  threading.Thread(target=rclpy.spin, args=(node,),daemon=True)
    thread.start()

    try:
        node.run()

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # rclpy.shutdown() # ne pas activer sinon risque de shutdown toutes les nodes
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()