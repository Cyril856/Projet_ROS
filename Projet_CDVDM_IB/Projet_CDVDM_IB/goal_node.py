import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import numpy as np
import time
import math
import cv2
import threading

from geometry_msgs.msg import Twist

class Goal(Node):
    def __init__(self):
        super().__init__('compressed_image_subscriber')
        self.subscription = self.create_subscription(
            CompressedImage,
            '/camera/image_raw/compressed',
            self.listener_callback,
            10
        )
        self.subscription  # to prevent unused variable warning

        self.message = Twist()
        self.publisher = self.create_publisher(Twist,'/cmd_vel',10)

    def listener_callback(self, msg):
        # Convertir les données compressées en tableau numpy
        np_arr = np.frombuffer(msg.data, np.uint8)
        # Décoder l'image
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if image is not None:
            self.image = image
            