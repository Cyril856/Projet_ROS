import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import numpy as np
import cv2
import threading

class HSVCalibration(Node):
    def __init__(self):
        super().__init__('hsv_calibration')
        self.subscription = self.create_subscription(
            CompressedImage,
            '/camera/image_raw/compressed',
            self.listener_callback,
            10
        )

        self.image = None
        self.horizon_y = 130

        # Création des fenêtres
        cv2.namedWindow("Original")
        cv2.namedWindow("Mask")
        cv2.namedWindow("HSV Calibration")

        # --- Red trackbars ---
        cv2.createTrackbar("Red H min1", "HSV Calibration", 0,   179, self.nothing)
        cv2.createTrackbar("Red H max1", "HSV Calibration", 10,  179, self.nothing)
        cv2.createTrackbar("Red H min2", "HSV Calibration", 160, 179, self.nothing)
        cv2.createTrackbar("Red H max2", "HSV Calibration", 179, 179, self.nothing)
        cv2.createTrackbar("Red S min",  "HSV Calibration", 20,  255, self.nothing)
        cv2.createTrackbar("Red S max",  "HSV Calibration", 255, 255, self.nothing)
        cv2.createTrackbar("Red V min",  "HSV Calibration", 140, 255, self.nothing)
        cv2.createTrackbar("Red V max",  "HSV Calibration", 255, 255, self.nothing)

        # --- Green trackbars ---
        cv2.createTrackbar("Green H min", "HSV Calibration", 40,  179, self.nothing)
        cv2.createTrackbar("Green H max", "HSV Calibration", 80,  179, self.nothing)
        cv2.createTrackbar("Green S min", "HSV Calibration", 20,  255, self.nothing)
        cv2.createTrackbar("Green S max", "HSV Calibration", 255, 255, self.nothing)
        cv2.createTrackbar("Green V min", "HSV Calibration", 100, 255, self.nothing)
        cv2.createTrackbar("Green V max", "HSV Calibration", 255, 255, self.nothing)

        # --- Yellow trackbars (Nouveau) ---
        cv2.createTrackbar("Yellow H min", "HSV Calibration", 20,  179, self.nothing)
        cv2.createTrackbar("Yellow H max", "HSV Calibration", 35,  179, self.nothing)
        cv2.createTrackbar("Yellow S min", "HSV Calibration", 100, 255, self.nothing)
        cv2.createTrackbar("Yellow S max", "HSV Calibration", 255, 255, self.nothing)
        cv2.createTrackbar("Yellow V min", "HSV Calibration", 100, 255, self.nothing)
        cv2.createTrackbar("Yellow V max", "HSV Calibration", 255, 255, self.nothing)

    def nothing(self, x):
        pass

    def listener_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if image is not None:
            self.image = image.copy()

    def get_trackbar_values(self):
        values = {
            'red': [
                cv2.getTrackbarPos("Red H min1", "HSV Calibration"),
                cv2.getTrackbarPos("Red H max1", "HSV Calibration"),
                cv2.getTrackbarPos("Red H min2", "HSV Calibration"),
                cv2.getTrackbarPos("Red H max2", "HSV Calibration"),
                cv2.getTrackbarPos("Red S min",  "HSV Calibration"),
                cv2.getTrackbarPos("Red S max",  "HSV Calibration"),
                cv2.getTrackbarPos("Red V min",  "HSV Calibration"),
                cv2.getTrackbarPos("Red V max",  "HSV Calibration")
            ],
            'green': [
                cv2.getTrackbarPos("Green H min", "HSV Calibration"),
                cv2.getTrackbarPos("Green H max", "HSV Calibration"),
                cv2.getTrackbarPos("Green S min", "HSV Calibration"),
                cv2.getTrackbarPos("Green S max", "HSV Calibration"),
                cv2.getTrackbarPos("Green V min", "HSV Calibration"),
                cv2.getTrackbarPos("Green V max", "HSV Calibration")
            ],
            'yellow': [
                cv2.getTrackbarPos("Yellow H min", "HSV Calibration"),
                cv2.getTrackbarPos("Yellow H max", "HSV Calibration"),
                cv2.getTrackbarPos("Yellow S min", "HSV Calibration"),
                cv2.getTrackbarPos("Yellow S max", "HSV Calibration"),
                cv2.getTrackbarPos("Yellow V min", "HSV Calibration"),
                cv2.getTrackbarPos("Yellow V max", "HSV Calibration")
            ]
        }
        return values

    def print_values(self, v):
        print("\n" + "="*50)
        print("COPIER-COLLER DANS line_following_node.py :")
        print("="*50)
        print(f"self.horizon = {self.horizon_y}")
        print(f"self.lower_red1 = np.array([{v['red'][0]}, {v['red'][4]}, {v['red'][6]}])")
        print(f"self.upper_red1 = np.array([{v['red'][1]}, {v['red'][5]}, {v['red'][7]}])")
        print(f"self.lower_red2 = np.array([{v['red'][2]}, {v['red'][4]}, {v['red'][6]}])")
        print(f"self.upper_red2 = np.array([{v['red'][3]}, {v['red'][5]}, {v['red'][7]}])")
        print(f"self.lower_green = np.array([{v['green'][0]}, {v['green'][2]}, {v['green'][4]}])")
        print(f"self.upper_green = np.array([{v['green'][1]}, {v['green'][3]}, {v['green'][5]}])")
        print(f"self.lower_yellow = np.array([{v['yellow'][0]}, {v['yellow'][2]}, {v['yellow'][4]}])")
        print(f"self.upper_yellow = np.array([{v['yellow'][1]}, {v['yellow'][3]}, {v['yellow'][5]}])")
        print("="*50 + "\n")

    def run(self):
        rate = self.create_rate(20)
        while rclpy.ok():
            if self.image is not None:
                image = self.image.copy()
                hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
                v = self.get_trackbar_values()

                # Masques
                m_r1 = cv2.inRange(hsv, np.array([v['red'][0], v['red'][4], v['red'][6]]), np.array([v['red'][1], v['red'][5], v['red'][7]]))
                m_r2 = cv2.inRange(hsv, np.array([v['red'][2], v['red'][4], v['red'][6]]), np.array([v['red'][3], v['red'][5], v['red'][7]]))
                mask_red = m_r1 | m_r2
                mask_green = cv2.inRange(hsv, np.array([v['green'][0], v['green'][2], v['green'][4]]), np.array([v['green'][1], v['green'][3], v['green'][5]]))
                mask_yellow = cv2.inRange(hsv, np.array([v['yellow'][0], v['yellow'][2], v['yellow'][4]]), np.array([v['yellow'][1], v['yellow'][3], v['yellow'][5]]))

                # Visualisation
                result = cv2.bitwise_or(cv2.bitwise_or(mask_red, mask_green), mask_yellow)
                
                # Dessin de l'horizon sur l'image originale
                cv2.line(image, (0, self.horizon_y), (image.shape[1], self.horizon_y), (255, 0, 0), 2)
                cv2.putText(image, "HORIZON 130", (10, self.horizon_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

                cv2.imshow("Original", image)
                cv2.imshow("Mask", result)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('p'):
                self.print_values(self.get_trackbar_values())
            elif key == ord('q'):
                break
            rate.sleep()

def main(args=None):
    rclpy.init(args=args)
    node = HSVCalibration()
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()