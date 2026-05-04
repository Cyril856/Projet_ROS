import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import numpy as np
import cv2
import cv2
import mediapipe as mp

mp_hands = mp.solutions.hands
hands = mp_hands.Hands(max_num_hands=1)
mp_draw = mp.solutions.drawing_utils

import threading

from geometry_msgs.msg import Twist

class HandTeleop(Node):
    def __init__(self):
        super().__init__('compressed_image_subscriber')

        self.message = Twist()
        self.publisher = self.create_publisher(Twist,'/cmd_vel',10)
        self.robot_image = None
        self.subscription = self.create_subscription(
            CompressedImage,
            '/camera/image_raw/compressed',
            self.robot_callback,
            10
        )

    def robot_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if image is not None:
            self.robot_image = image.copy()
        

    def run(self):
        cap = cv2.VideoCapture("http://host.docker.internal:8080/video")

        while rclpy.ok():
            ret, frame = cap.read()

            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB) 
        
            result = hands.process(rgb)

            movement_text = "No hand detected"

            if result.multi_hand_landmarks:
                for handLms in result.multi_hand_landmarks:
                    mp_draw.draw_landmarks(frame, handLms, mp_hands.HAND_CONNECTIONS)

                    index_tip=handLms.landmark[8]
                    index_mcp=handLms.landmark[5]

                    if index_tip.x > index_mcp.x:
                        self.message.linear.x = 0.7
                        movement_text = "FORWARD"
                    elif index_tip.x < index_mcp.x:
                        self.message.linear.x = -0.7
                        movement_text = "BACKWARD"
                    elif index_tip.y > index_mcp.y:
                        self.message.angular.z = 0.7
                        movement_text = "TURN LEFT"
                    elif index_tip.y < index_mcp.y:
                        self.message.angular.z = -0.7
                        movement_text = "TURN RIGHT"
                    else:
                        movement_text = "STOP"

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
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()