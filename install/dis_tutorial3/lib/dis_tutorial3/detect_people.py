#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSReliabilityPolicy

from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2

from visualization_msgs.msg import Marker

from tensorflow.keras.models import load_model

from scipy.spatial.distance import euclidean

from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

from ultralytics import YOLO

import math

from geometry_msgs.msg import Twist


# from rclpy.parameter import Parameter
# from rcl_interfaces.msg import SetParametersResult

class detect_faces(Node):

	def __init__(self):
		super().__init__('detect_faces')

		self.declare_parameters(
			namespace='',
			parameters=[
				('device', ''),
		])

		marker_topic = "/people_marker"

		self.detection_color = (0,0,255)
		self.device = self.get_parameter('device').get_parameter_value().string_value

		self.bridge = CvBridge()
		self.scan = None

		self.rgb_image_sub = self.create_subscription(Image, "/oakd/rgb/preview/image_raw", self.rgb_callback, qos_profile_sensor_data)
		self.pointcloud_sub = self.create_subscription(PointCloud2, "/oakd/rgb/preview/depth/points", self.pointcloud_callback, qos_profile_sensor_data)

		# PARKING		
		self.parking_sub = self.create_subscription(Image, "/top_camera/rgb/preview/image_raw", self.parking_callback, 10)
		self.parking_pub = self.create_publisher(Twist, "/cmd_vel", 10)


		self.marker_pub = self.create_publisher(Marker, marker_topic, QoSReliabilityPolicy.BEST_EFFORT)
		self.new_face_pub = self.create_publisher(Marker, 'new_face', 10)

		self.model = YOLO("yolov8n.pt")

		self.faces = []

		self.edges = []

		self.detected_faces = []
		self.current_num_of_faces = 0
		self.sift = cv2.SIFT_create()

		self.get_logger().info(f"Node has been initialized! Will publish face markers to {marker_topic}.")

	def detect_face(self, image):
		keypoints, descriptors = self.sift.detectAndCompute(image, None)
		return keypoints, descriptors

	def is_face_detected(self, descriptors_to_match):
		for stored_descriptors in self.detected_faces:
			try:
				matcher = cv2.BFMatcher()
				matches = matcher.knnMatch(descriptors_to_match, stored_descriptors, k=2)

				good_matches = 0
				for m, n in matches:
					if m.distance < 0.8 * n.distance:
						good_matches += 1

				if good_matches > 0:
					return True
			except:
				print("yo")
			

		return False

	def rgb_callback(self, data):

		self.faces = []

		self.normal = np.array([])

		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

			# self.get_logger().info(f"Running inference on image...") ### THIS PRINTS IF THE FACE DETECT IS RUNNING

			# run inference
			res = self.model.predict(cv_image, imgsz=(256, 320), show=False, verbose=False, classes=[0], device=self.device)

			# iterate over results
			for x in res:
				bbox = x.boxes.xyxy
				if bbox.nelement() == 0: # skip if empty
					continue

				# self.get_logger().info(f"Person has been detected!")

				bbox = bbox[0] # this is the bounding box of the face that was detected

			########################################
				x1, y1, x2, y2 = bbox.tolist()

				face_roi = cv_image[int(y1):int(y2), int(x1):int(x2)]
				width = len(face_roi[0])
				print(width)

				keypoints, descriptors = self.detect_face(face_roi)

				if width > 28 and not self.is_face_detected(descriptors):
					self.detected_faces.append(descriptors)
					# self.new_face_pub.publish(Marker())
					cv2.imshow("detected face", face_roi)
					print("New face detected!")
							
				else:
					print("Face already detected!")


				# print("ALL FACES DETECTED: ", len(self.detected_faces))



				self.edges.append((int(x1), int(y1), int(x2), int(y2)))

			#########################################

				# draw rectangle
				cv_image = cv2.rectangle(cv_image, (int(bbox[0]), int(bbox[1])), (int(bbox[2]), int(bbox[3])), self.detection_color, 3)

				cx = int((bbox[0]+bbox[2])/2)
				cy = int((bbox[1]+bbox[3])/2) # here the center is calculated. but we can calculate two vectors and get the normal of the face

				# draw the center of bounding box
				cv_image = cv2.circle(cv_image, (cx,cy), 5, self.detection_color, -1)

				self.faces.append((cx,cy))

			cv2.imshow("image", cv_image)
			key = cv2.waitKey(1)
			if key==27:
				print("exiting")
				exit()
			
		except CvBridgeError as e:
			print(e)

	def pointcloud_callback(self, data):

		# get point cloud attributes
		height = data.height
		width = data.width
		point_step = data.point_step
		row_step = data.row_step		

		# iterate over face coordinates
		for x,y in self.faces:

			# get 3-channel representation of the poitn cloud in numpy format
			a = pc2.read_points_numpy(data, field_names= ("x", "y", "z"))
			a = a.reshape((height,width,3))

			# read center coordinates
			d = a[y,x,:]

			# create marker
			marker = Marker()

			# marker.header.parking_image_id = "/base_link"
			marker.header.stamp = data.header.stamp

			marker.type = 2
			marker.id = 0 # We can make it so that the marker ID for the position of the face is 0, and the ID of the marker of the normal is 1

			# Set the scale of the marker
			scale = 0.1
			marker.scale.x = scale
			marker.scale.y = scale
			marker.scale.z = scale

			# Set the color
			marker.color.r = 1.0
			marker.color.g = 1.0
			marker.color.b = 1.0
			marker.color.a = 1.0

			# Set the pose of the marker
			marker.pose.position.x = float(d[0])
			marker.pose.position.y = float(d[1])
			marker.pose.position.z = float(d[2])

			

			x2 = x + 5
			y2 = y + 5

			if x2 > len(a[0]) - 1:
				x2 -= 10

			if y2 > len(a) - 1:
				y2 -= 10

			d = np.array(d)
			# print("DDDDDDDD", d)
			one_point = np.array(a[y2, x,:])		
			another_point = np.array(a[y, x2,:])

			one_vector = one_point - d
			another_vector = another_point - d

			one_vector /= np.linalg.norm(one_vector)
			another_vector /= np.linalg.norm(another_vector)

			normal = np.cross(one_vector, another_vector)
			normal /= np.linalg.norm(normal)
			# normal *= -1

			# self.get_logger().info(f"NORMAL: {normal}")
		
			marker2 = Marker()

			# marker2.header.parking_image_id = "/base_link"
			marker2.header.stamp = data.header.stamp

			marker2.type = 0
			marker2.id = 1

			# Set the scale of the marker
			scale = 0.1
			marker2.scale.x = scale
			marker2.scale.y = scale
			marker2.scale.z = scale

			# Set the color
			marker2.color.r = 1.0
			marker2.color.g = 165 / 255
			marker2.color.b = 0.0
			marker2.color.a = 1.0

			# Set the pose of the marker
			marker2.pose.position.x = float(normal[0])
			marker2.pose.position.y = float(normal[1])
			marker2.pose.position.z = float(normal[2])

			if len(self.detected_faces) > self.current_num_of_faces: 
				self.new_face_pub.publish(marker)
				self.marker_pub.publish(marker2)
				self.current_num_of_faces = len(self.detected_faces)
			self.marker_pub.publish(marker)

	def segment_circle(self, image):
		# Convert image to grayscale
		gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
		gray = gray[:-40]
		
		# Apply Gaussian blur to reduce noise
		blurred = cv2.GaussianBlur(gray, (5, 5), 0)
		
		circles = cv2.HoughCircles(blurred, cv2.HOUGH_GRADIENT, dp=1, minDist=20, param1=50, param2=30, minRadius=0, maxRadius=0)
    
		# Create a mask to store the segmented circle
		mask = np.zeros_like(gray)
		
		# If circles are detected
		if circles is not None:
			# Convert the (x, y, radius) parameters to integers
			circles = np.round(circles[0, :]).astype("int")
			
			# Draw the circles on the mask
			for (x, y, r) in circles:
				cv2.circle(mask, (x, y), r, (255), -1)
		
		# Apply threshold to create binary mask
		_, binary_mask = cv2.threshold(mask, 1, 255, cv2.THRESH_BINARY)
		
		return binary_mask


	def parking_callback(self, data):
		img = self.bridge.imgmsg_to_cv2(data, "bgr8")
		segmented_circle = self.segment_circle(img)



		circle_indices = np.where(segmented_circle != 0)
		x_avg = np.sum(circle_indices[1]) / len(circle_indices[1])
		y_avg = np.sum(circle_indices[0]) / len(circle_indices[0])

		direction = Twist()
		direction.linear.x = 2.0
		direction.linear.y = 0.0
		direction.linear.z = 0.0
		direction.angular.x = 0.0
		direction.angular.y = 0.0
		direction.angular.z = 1.8
		self.parking_pub.publish(direction)

		# print("X: ", x_avg, "Y: ", y_avg)

		cv2.imshow("parking", segmented_circle)
		key = cv2.waitKey(1)
		cv2.imshow("ogImage", img)
		key = cv2.waitKey(1)
		if key==27:
			print("exiting")
			exit()
		

def main():
	print('Face detection node starting.')

	rclpy.init(args=None)
	node = detect_faces()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()