#!/usr/bin/python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import tf2_ros

from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped, Vector3, Pose
from cv_bridge import CvBridge, CvBridgeError
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

qos_profile = QoSProfile(
          durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
          reliability=QoSReliabilityPolicy.RELIABLE,
          history=QoSHistoryPolicy.KEEP_LAST,
          depth=1)

class RingDetector(Node):
    def __init__(self):
        super().__init__('transform_point')

        # Basic ROS stuff
        timer_frequency = 2
        timer_period = 1/timer_frequency

        # An object we use for converting images between ROS format and OpenCV format
        self.bridge = CvBridge()

        # Marker array object used for visualizations
        self.marker_array = MarkerArray()
        self.marker_num = 1

        # Subscribe to the image and/or depth topic
        self.image_sub = self.create_subscription(Image, "/oakd/rgb/preview/image_raw", self.image_callback, 1)
        self.depth_sub = self.create_subscription(Image, "/oakd/rgb/preview/depth", self.depth_callback, 1)

        self.found_on_rgb = False
        self.found_on_depth = False
        self.is_green_circle = False

        # Publiser for the visualization markers
        # self.marker_pub = self.create_publisher(Marker, "/ring", QoSReliabilityPolicy.BEST_EFFORT)

        # Object we use for transforming between coordinate frames
        # self.tf_buf = tf2_ros.Buffer()
        # self.tf_listener = tf2_ros.TransformListener(self.tf_buf)

        #cv2.namedWindow("Binary Image", cv2.WINDOW_NORMAL)
        #cv2.namedWindow("Detected contours", cv2.WINDOW_NORMAL)
        #cv2.namedWindow("Detected rings", cv2.WINDOW_NORMAL)
        #cv2.namedWindow("Depth window", cv2.WINDOW_NORMAL)  

    def is_circle_present(self, gray):
        # Load image and convert to grayscale

        # Apply GaussianBlur to reduce noise
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # Apply edge detection or thresholding to find contours
        _, thresh = cv2.threshold(blurred, 127, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            # Calculate bounding rectangle for the contour
            x, y, w, h = cv2.boundingRect(cnt)
            aspect_ratio = float(w) / h  # Aspect ratio near 1 indicates a square or circle

            # Calculate area and perimeter
            area = cv2.contourArea(cnt)
            perimeter = cv2.arcLength(cnt, True)
            
            # Circularity: defined as 4*pi*(Area/Perimeter^2)
            # Closer to 1 indicates a perfect circle
            circularity = (4 * np.pi * area) / (perimeter ** 2) if perimeter > 0 else 0

            # Check if the contour is a circle
            if circularity > 0.8 and 0.9 < aspect_ratio < 1.1:
                # This contour is likely a circle
                return True
        
        # If no contour is identified as a circle, return False
        return False

    def detect_circles(self, gray):

        # Apply GaussianBlur to reduce noise and improve circle detection
        gray = cv2.GaussianBlur(gray, (9, 9), 2)

        # Detect circles using HoughCircles
        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, dp=2, minDist=60,
                                param1=90, param2=50, minRadius=0, maxRadius=0)

        # value = self.is_circle_present(gray)

        # Ensure at least some circles were found
        if circles is not None and value:
            # circles = np.uint16(np.around(circles))
            self.found_on_rgb = True
            # for i in circles[0, :]:
            #     # Draw the outer circle
            #     cv2.circle(img, (i[0], i[1]), i[2], (0, 255, 0), 2)
            #     # Draw the center of the circle
            #     cv2.circle(img, (i[0], i[1]), 2, (0, 0, 255), 3)

        else:
            self.found_on_rgb = False

        # Display the result
        # cv2.imshow('Detected Circles', img)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()

    def check_in_line(self, green_image):
        counter = 0
        for line in green_image:
            found_green = False
            found_black = False
            for x in line:
                if x[1] != 0 and not found_green:
                    found_green = True
                elif x[1] == 0 and found_green:
                    found_black = True
                elif x[1] != 0 and found_black:
                    counter += 1
            if counter > 30:
                return True

        return False


    def image_callback(self, data):
            self.get_logger().info(f"I got a new image! Will try to find rings...")
        # cv2.imshow('Detected Circles', img)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()  
            try:
                cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            except CvBridgeError as e:
                print(e)

            blue = cv_image[:,:,0]
            green = cv_image[:,:,1]
            red = cv_image[:,:,2]

            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

            # Define range of green color in HSV
            lower_green = np.array([40, 40, 40])
            upper_green = np.array([70, 255, 255])

            # Threshold the HSV image to get only green colors
            mask = cv2.inRange(hsv, lower_green, upper_green)

            # Bitwise-AND mask and original image
            green_image = cv2.bitwise_and(cv_image, cv_image, mask=mask)
            cv2.imshow('Green Objects', green_image)
            og_image = cv_image
            self.is_green_circle = self.check_in_line(green_image)

            # Tranform image to grayscale
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            gray = gray[:gray.shape[0] // 4]
            # gray = red

            # self.detect_circles(gray)

            value = self.is_circle_present(gray)
            if value:
                self.found_on_rgb = True


            # Apply Gaussian Blur
            # gray = cv2.GaussianBlur(gray,(3,3),0)

            # Do histogram equalization
            # gray = cv2.equalizeHist(gray)

            # Binarize the image, there are different ways to do it
            #ret, thresh = cv2.threshold(img, 50, 255, 0)
            #ret, thresh = cv2.threshold(img, 70, 255, cv2.THRESH_BINARY)
            # thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 15, 30)
            # cv2.imshow("GrayscaleImage", thresh)
            # cv2.waitKey(1)

            # Extract contours
            # contours, hierarchy = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

            # Example of how to draw the contours, only for visualization purposes
            # cv2.drawContours(gray, contours, -1, (255, 0, 0), 3)
            # cv2.imshow("Detected contours", gray)
            # cv2.waitKey(1)
            cv2.imshow("Camera Image", og_image)



    def depth_callback(self, data):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
        except CvBridgeError as e:
            print(e)

        depth_image[depth_image == np.inf] = 0

        # Convert depth image to uint8 for visualization
        depth_image_visualization = (depth_image / np.max(depth_image) * 255).astype(np.uint8)

        # Apply thresholding to detect regions of interest (possible rings)
        #_, binary_depth = cv2.threshold(depth_image_visualization, 80, 255, cv2.THRESH_BINARY)
        #cv2.imshow("Binary Image", binary_depth)


        cols = np.arange(depth_image_visualization.shape[1])

        cut_images = []
        for start in range(0, 320, 20):
            end = start + 20
            # Correct the column exclusion to include all columns between start and end inclusive
            mask = np.ones(cols.shape, dtype=bool)
            mask[start:end+1] = False  # Now correctly excludes all columns between start and end
            cut_image = depth_image_visualization[:, mask].astype(np.uint8)
            cut_images.append(cut_image)


        for image in cut_images:
            image = np.ascontiguousarray(image)
            # Find contours
            contours, _ = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # Example of how to draw the contours, only for visualization purposes
            detected_contours = cv2.drawContours(image, contours, -1, (255, 0, 0), 3)
            #cv2.imshow("Detected contours", detected_contours)
            cv2.waitKey(1)

            # Filter contours based on area or other criteria if needed
            valid_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > 200 and cv2.contourArea(cnt) < 700 ]

            if len(valid_contours)>0:
                self.found_on_depth = True

            else:
                self.found_on_depth = False


            print(self.found_on_depth, self.is_green_circle)
            if self.found_on_depth and self.is_green_circle:
                # Draw contours on the depth image for visualization
                depth_image_detected = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
                cv2.drawContours(depth_image_detected, valid_contours, -1, (0, 255, 0), 2)
                cv2.imshow("Detected ring", depth_image_detected)

    def both_sensors_callback(self, image,source):
        if source=="depth":
            cv2.imshow("Depth Detected rings", image)
        else:
            cv2.imshow("RGB Detected rings", image)

    def ring_detection(self, contours, cv_image):
        # Fit elipses to all extracted contours
        elps = []
        for cnt in contours:
            #     print cnt
            #     print cnt.shape
            if cnt.shape[0] >= 20:
                ellipse = cv2.fitEllipse(cnt)
                elps.append(ellipse)


        # Find two elipses with same centers
        candidates = []
        for n in range(len(elps)):
            for m in range(n + 1, len(elps)):
                # e[0] is the center of the ellipse (x,y), e[1] are the lengths of major and minor axis (major, minor), e[2] is the rotation in degrees
                
                e1 = elps[n]
                e2 = elps[m]
                dist = np.sqrt(((e1[0][0] - e2[0][0]) ** 2 + (e1[0][1] - e2[0][1]) ** 2))
                angle_diff = np.abs(e1[2] - e2[2])

                # The centers of the two elipses should be within 5 pixels of each other (is there a better treshold?)
                if dist >= 5:
                    continue

                # The rotation of the elipses should be whitin 4 degrees of eachother
                if angle_diff>4:
                    continue

                e1_minor_axis = e1[1][0]
                e1_major_axis = e1[1][1]

                e2_minor_axis = e2[1][0]
                e2_major_axis = e2[1][1]

                if e1_major_axis>=e2_major_axis and e1_minor_axis>=e2_minor_axis: # the larger ellipse should have both axis larger
                    le = e1 # e1 is larger ellipse
                    se = e2 # e2 is smaller ellipse
                elif e2_major_axis>=e1_major_axis and e2_minor_axis>=e1_minor_axis:
                    le = e2 # e2 is larger ellipse
                    se = e1 # e1 is smaller ellipse
                else:
                    continue # if one ellipse does not contain the other, it is not a ring
                
                # # The widths of the ring along the major and minor axis should be roughly the same
                # border_major = (le[1][1]-se[1][1])/2
                # border_minor = (le[1][0]-se[1][0])/2
                # border_diff = np.abs(border_major - border_minor)

                # if border_diff>4:
                #     continue
                    
                candidates.append((e1,e2))

        print("Processing is done! found", len(candidates), "candidates for rings")

        # Plot the rings on the image
        for c in candidates:

            # the centers of the ellipses
            e1 = c[0]
            e2 = c[1]

            # drawing the ellipses on the image
            cv2.ellipse(cv_image, e1, (0, 255, 0), 2)
            cv2.ellipse(cv_image, e2, (0, 255, 0), 2)

            # Get a bounding box, around the first ellipse ('average' of both elipsis)
            size = (e1[1][0]+e1[1][1])/2
            center = (e1[0][1], e1[0][0])

            x1 = int(center[0] - size / 2)
            x2 = int(center[0] + size / 2)
            x_min = x1 if x1>0 else 0
            x_max = x2 if x2<cv_image.shape[0] else cv_image.shape[0]

            y1 = int(center[1] - size / 2)
            y2 = int(center[1] + size / 2)
            y_min = y1 if y1 > 0 else 0
            y_max = y2 if y2 < cv_image.shape[1] else cv_image.shape[1]


        return cv_image


def main():

    rclpy.init(args=None)
    rd_node = RingDetector()

    rclpy.spin(rd_node)

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()