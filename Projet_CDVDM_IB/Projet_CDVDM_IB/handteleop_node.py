import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import numpy as np
import cv2
import cv2
import mediapipe as mp
import time
from std_srvs.srv import SetBool
from sensor_msgs.msg import Image # pour gazebo
from cv_bridge import CvBridge # pour gazebo

mp_hands = mp.solutions.hands
hands = mp_hands.Hands(
    static_image_mode=False,
    max_num_hands=1,          
    model_complexity=0,        
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5
)
mp_draw = mp.solutions.drawing_utils

import threading

from geometry_msgs.msg import Twist

class HandTeleop(Node):
    def __init__(self):
        super().__init__('compressed_image_subscriber')

        self.message = Twist()
        self.publisher = self.create_publisher(Twist,'/cmd_vel',10)
        self.robot_image = None
        
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
        #self.timer = self.create_timer(0.03, self.robot_callback)

        # controle de node
        self.active = False
        self.srv = self.create_service(SetBool, '/activate_handteleop', self.handle_activation)

      # Callback du service
    def handle_activation(self, request, response):
        self.active = request.data
        response.success = True
        response.message = f"HandTeleop node state: {self.active}"
        return response

    def robot_callback(self, msg):
        if not self.active:
            return  # Ignore les images si la node est inactive
        np_arr = np.frombuffer(msg.data, np.uint8)
        
        # Décoder l'image
        #image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) 
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8') ## gazebo

        if image is not None:
            self.robot_image = image.copy()
        

    def run(self):
        cap = cv2.VideoCapture("http://host.docker.internal:8080/video")
        #cap = cv2.VideoCapture(0)

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_FPS, 60)

        while rclpy.ok():

            # Controle de node
            if not self.active:
                # Si inactive, on ne fait rien (mais on continue de tourner pour écouter les callbacks)
                time.sleep(0.1) ## à augmenter si trop faible 
                continue

            ret, frame = cap.read()

            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB) 
        
            result = hands.process(rgb)

            movement_text = "No hand detected"
            self.message.linear.x = 0.0
            self.message.angular.z = 0.0

            if result.multi_hand_landmarks:
                for handLms in result.multi_hand_landmarks:
                    mp_draw.draw_landmarks(frame, handLms, mp_hands.HAND_CONNECTIONS)

                    index_tip=handLms.landmark[8]
                    index_mcp=handLms.landmark[5]

                    THRESHOLD = 0.05  

                    dx = index_tip.x - index_mcp.x
                    dy = index_tip.y - index_mcp.y

                    if  abs(dy) > abs(dx): 
                        self.message.angular.z = 0.0
                        if dy > THRESHOLD:
                            self.message.linear.x = -0.15
                            self.message.linear.z = 0.0
                            movement_text = "BACKWARD"

                        elif dy < -THRESHOLD:
                            self.message.linear.x = 0.15
                            
                            movement_text = "FORWARD"
                            
                        else:
                            self.message.linear.x = 0.0
                            self.message.angular.z = 0.0
                            movement_text = "STOP"

                    elif  abs(dx) > abs(dy):
                        self.message.linear.x = 0.0
                        if dx > THRESHOLD:
                            self.message.angular.z = -0.2
                            movement_text = "TURN RIGHT"
                        elif dx < -THRESHOLD:
                            self.message.angular.z = 0.2
                            movement_text = "TURN LEFT"
                            
                        else:
                            movement_text = "STOP"
                            self.message.angular.z = 0.0
                    else:
                        movement_text = "STOP"
                        self.message.linear.x = 0.0
                        self.message.angular.z = 0.0

                    self.publisher.publish(self.message)
        
            cv2.putText(frame, movement_text, (10, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 2)

            cv2.imshow("Hand Teleop", frame)

            robot_img = self.robot_image
            if robot_img is not None:
                cv2.imshow("Robot Camera", robot_img)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cap.release()

def main():
    rclpy.init()
    node = HandTeleop()

    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
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