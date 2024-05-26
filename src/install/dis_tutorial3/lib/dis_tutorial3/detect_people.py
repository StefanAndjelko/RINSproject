#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSReliabilityPolicy

from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2

from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from tensorflow.keras.models import load_model

from scipy.spatial.distance import euclidean
from tf2_ros import TransformStamped
import tf2_ros

from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

from ultralytics import YOLO

import math
import time

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
		self.odometry_sub = self.create_subscription(Odometry, '/odom', self.odometry_callback, 10)
		self.parking_pub = self.create_publisher(Twist, "/cmd_vel", 10)
		self.arm_command_pub = self.create_publisher(String, "/arm_command", 10)

		arm_msg = String()
		arm_msg.data = "look_for_parking1"
		self.arm_command_pub.publish(arm_msg)
		
		self.node = rclpy.create_node('robot_controller')
		self.current_yaw = 0.0
		self.tf_buffer = tf2_ros.Buffer()
		self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)

		self.current_pos = 2
		self.searching = True
		# \PARKING

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

			################## DETECT MONA LISA ###################
			
			# Define the target RGB values and tolerance
			colors = [
				(70, 75, 47),
				(51, 37, 31),
				(188, 157, 79),
				(22, 16, 27),
				(61, 34, 18)
			]
			margin = 2

			# Convert BGR image to RGB
			img_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

			# Mask to check presence of each color within the margin
			found_colors = []
			for color in colors:
				lower_bound = np.array([c - margin for c in color])
				upper_bound = np.array([c + margin for c in color])
				
				# Create a mask for the current color within the margin
				mask = cv2.inRange(img_rgb, lower_bound, upper_bound)
				
				# Check if any pixel matches the color
				if np.any(mask):
					found_colors.append(color)
					#print(f"Color {color} is present within margin.")
				#else:
					#print(f"Color {color} is not present.")

			# Check if all specified colors are found
			if len(found_colors) == len(colors):
				is_lisa=True
				print("All specified colors are present within the margin.")
			else:
				is_lisa=False
				print("Not all specified colors are found.")

			##########################################################

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
				keypoints, descriptors = self.detect_face(face_roi)

				if width > 28 and not self.is_face_detected(descriptors):
					self.detected_faces.append(descriptors)
					# self.new_face_pub.publish(Marker())
					cv2.imshow("detected face", face_roi)
					#print("New face detected!")
							
				#else:
					#print("Face already detected!")


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

	def quaternion_to_yaw(self, quaternion):
		# Convert quaternion to Euler angles (yaw)
		q = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
		roll, pitch, yaw = tf2_ros.transformations.euler_from_quaternion(q)
		return yaw

	def odometry_callback(self, msg):
		try:
			transform = self.tf_buffer.lookup_transform('base_link', 'odom', rclpy.time.Time())
			quaternion = transform.transform.rotation
			self.current_yaw = self.quaternion_to_yaw(quaternion)
		except Exception as e:
			pass
			# print("Error:", e)

	def segment_circle(self, image):
		# Convert image to grayscale
		gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
		if self.current_pos != 1:
			gray = gray[20:]
		else:
			gray = gray[:-20]
		
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

	def calculate_velocity(self, direction):
		# Convert target vector to polar coordinates
		# angle = math.atan2(direction[1], direction[0])
		direction[1] *= -1
		angle = np.arccos(np.dot(direction, np.array([1, 0])) / np.linalg.norm(direction))

		if direction[0] < 0:
			angle = -1 * (math.pi - angle)

		# Calculate angular velocity (to align with target direction)
		angular_velocity = angle - self.current_yaw

		# Set linear velocity (move forward)
		linear_velocity = 0.4  # Adjust speed as needed

		print("linear: ", linear_velocity, "angular: ", angular_velocity)

		return linear_velocity, angular_velocity

	def fix_angle(self, current_pos, angle):
		if current_pos == 2:
			return angle + math.pi
		elif current_pos == 3:
			return angle - math.pi / 2
		elif currnet_pos == 4:
			return angle + math.pi / 2

		return angle

	## TO DO TO DO TO DO!!! NAPRAVI TAKO DA U ZAVISNOSTI OD TOGA GDE SE NALAZI KAMERA DA DRUGACIJE ODREZE SLIKU
	def parking_callback(self, data):
		img = self.bridge.imgmsg_to_cv2(data, "bgr8") 
		middle_point_x = np.shape(img)[1] / 2
		middle_point_y = np.shape(img)[0] / 2 + 100

		# print(f"middle point [{middle_point_x}, {middle_point_y}]")

		segmented_circle = self.segment_circle(img)

		circle_indices = np.where(segmented_circle != 0)
		x_avg = np.sum(circle_indices[1]) / len(circle_indices[1])
		y_avg = np.sum(circle_indices[0]) / len(circle_indices[0])

		# print(f"average [{x_avg}, {y_avg}]")

		move_x = float(x_avg - middle_point_x)
		move_y = float(y_avg - middle_point_y)

		# print(f"direction vector [{move_x}, {move_y}]")


		if not math.isnan(move_x) and not math.isnan(move_y):
			self.searching = False
			linear, angular = self.calculate_velocity(np.array([move_x, move_y]))
			direction = Twist()
			# direction.linear.x = linear
			direction.angular.z = -0.75 * angular
			print(f"Rotating for {angular} degrees")
			self.parking_pub.publish(direction)
			time.sleep(2)

			direction.linear.x = linear
			direction.angular.z = 0.0
			self.parking_pub.publish(direction)
			time.sleep(2)
		elif not self.searching:
			goal_arm_pos = String()
			goal_arm_pos.data = "look_for_parking" + f"{self.current_pos}"
			print(goal_arm_pos)
			self.arm_command_pub.publish(goal_arm_pos)
			print("published")
			time.sleep(4)
			self.current_pos += 1
			if self.current_pos > 4:
				self.current_pos = 1
				self.searching = True

		
		# print("X: ", x_avg, "Y: ", y_avg)

		cv2.imshow("parking", segmented_circle)
		key = cv2.waitKey(1)		# Ensure angular velocity is within [-pi, pi] range
		# if angular_velocity > math.pi:
		# 	angular_velocity -= 2 * math.pi
		# elif angular_velocity < -math.pi:
		# 	angular_velocity += 2 * math.pi
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