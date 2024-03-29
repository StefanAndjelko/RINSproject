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

		self.marker_pub = self.create_publisher(Marker, marker_topic, QoSReliabilityPolicy.BEST_EFFORT)

		self.model = YOLO("yolov8n.pt")

		self.faces = []

		self.all_embeddings = []

		self.get_logger().info(f"Node has been initialized! Will publish face markers to {marker_topic}.")

	def preprocess_face(face_image):
		"""Preprocess the cropped face image for the FaceNet model."""
		# Resize face for the model
		face_image = cv2.resize(face_image, (160, 160))
		
		# Preprocess the face in the same way as the training data
		face_image = face_image.astype('float32')
		mean, std = face_image.mean(), face_image.std()
		face_image = (face_image - mean) / std
		
		return face_image

	def get_embedding(model, face_image):
		"""Get the embedding of a face."""

		face_image = np.expand_dims(face_image, axis=0)
		embedding = model.predict(face_image)
		return embedding[0]


	def rgb_callback(self, data):

		self.faces = []

		self.normal = np.array([])
		self.edges = []

		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

			self.get_logger().info(f"Running inference on image...")

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

				# cv2.imshow("yo", face_roi)
				model_path = 'facenet_keras.h5'
				model = load_model(model_path)

				embedding = get_embedding(model, face_roi)

				# good_matches = self.match_faces(descriptor, descriptor)
				# print("NUM OF GOOD MATCHES: ", len(good_matches))

				if len(self.all_embeddings) == 0:
					self.all_embeddings.append(embedding)
					print("ADDED NEW FACE")
				else:
					already_exists = False
					for emb in all_embeddings:
						distance = euclidean(embedding, emb)
						if distance < 10.0:
							already_exists = True

					if not already_exists:
						self.all_embeddings.append(embedding)

					

				print("ALL FACES DETECTED: ", len(self.all_embeddings))



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

			marker.header.frame_id = "/base_link"
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

			self.marker_pub.publish(marker)

			#############################

		for x1, y1, x2, y2 in self.edges:

			self.get_logger().info(f"INDICES: {x1}, {y1}, {x2}, {y2}")

			b = pc2.read_points_numpy(data, field_names= ("x", "y", "z"))
			b = b.reshape((height,width,3))

			x1 = min(x1, len(b[0]) - 1)
			x2 = min(x2, len(b[0]) - 1)
			y1 = min(y1, len(b) - 1)
			y2 = min(y2, len(b) - 1)

			top_left = b[y1, x1, :]
			for ix in range(len(top_left)):
				if math.isinf(top_left[ix]):
					top_left[ix] = 1
			top_right = b[y1, x2, :]
			bottom_left = b[y2, x1, :]
			bottom_right = b[y2, x2, :]

			self.get_logger().info(f"TOP LEFT: {top_left}")
			self.get_logger().info(f"BOTTOM LEFT: {bottom_left}")

			up_vector = top_left - bottom_left
			side_vector = bottom_right - bottom_left

			self.get_logger().info(f"UP: {up_vector}")
			self.get_logger().info(f"SIDE: {side_vector}")


			up_vector /= np.linalg.norm(up_vector)
			side_vector /= np.linalg.norm(side_vector)

			self.normal = np.cross(side_vector, up_vector)
			
			marker2 = Marker()

			marker2.header.frame_id = "/base_link"
			marker2.header.stamp = data.header.stamp

			marker2.type = 0
			marker2.id = 1 # We can make it so that the marker ID for the position of the face is 0, and the ID of the marker of the normal is 1

			# Set the scale of the marker
			scale = 0.1
			marker2.scale.x = scale
			marker2.scale.y = scale
			marker2.scale.z = scale

			# Set the color
			marker2.color.r = 1.0
			marker2.color.g = 0.0
			marker2.color.b = 1.0
			marker2.color.a = 1.0

			# Set the pose of the marker
			marker2.pose.position.x = float(self.normal[0])
			marker2.pose.position.y = float(self.normal[1])
			marker2.pose.position.z = float(self.normal[2])

			# axis = np.cross([1, 0, 0], self.normal)
			# angle = np.arccos(np.dot([1, 0, 0], self.normal))

			# quaternion = tf.transformations.quaternion_about_axis(angle, axis)
			# marker.pose.orientation.x = quaternion[0]
			# marker.pose.orientation.y = quaternion[1]
			# marker.pose.orientation.z = quaternion[2]
			# marker.pose.orientation.w = quaternion[3]

			self.marker_pub.publish(marker2)

def main():
	print('Face detection node starting.')

	rclpy.init(args=None)
	node = detect_faces()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()