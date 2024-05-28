#! /usr/bin/env python3
# Mofidied from Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


from enum import Enum
import time
import numpy as np

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from rclpy.duration import Duration as Dur ###!!!
from geometry_msgs.msg import Quaternion, PoseStamped, PoseWithCovarianceStamped, PointStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import Spin, NavigateToPose
from turtle_tf2_py.turtle_tf2_broadcaster import quaternion_from_euler

from irobot_create_msgs.action import Dock, Undock
from irobot_create_msgs.msg import DockStatus

from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry

import tf2_geometry_msgs as tfg
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf_transformations

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration as rclpyDuration
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data

import math
from pygame import mixer

class TaskResult(Enum):
    UNKNOWN = 0
    SUCCEEDED = 1
    CANCELED = 2
    FAILED = 3

amcl_pose_qos = QoSProfile(
          durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
          reliability=QoSReliabilityPolicy.RELIABLE,
          history=QoSHistoryPolicy.KEEP_LAST,
          depth=1)

class RobotCommander(Node):

    def __init__(self, node_name='robot_commander', namespace=''):
        super().__init__(node_name=node_name, namespace=namespace)
        
        self.pose_frame_id = 'map'
        
        # Flags and helper variables
        self.goal_handle = None
        self.result_future = None
        self.feedback = None
        self.status = None
        self.initial_pose_received = False
        self.is_docked = None
        self.faceDetected = False
        self.should_move = False
        self.currentGoalPos = []
        self.currentNormal = []

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.face_counter = 0
        self.new_face_detected_status = [0, 0, 0]
        self.faces_to_visit = []
        self.current_yaw = 0.0

        # ROS2 subscribers
        self.create_subscription(DockStatus,
                                 'dock_status',
                                 self._dockCallback,
                                 qos_profile_sensor_data)
        
        self.localization_pose_sub = self.create_subscription(PoseWithCovarianceStamped,
                                                              'amcl_pose',
                                                              self._amclPoseCallback,
                                                              amcl_pose_qos)

        self.face_location_sub = self.create_subscription(Marker, 'people_marker', self.faceRecognizedCallback, 10)

        self.is_new_face = self.create_subscription(Marker, 'new_face', self.newFaceCallback, 10)

        self.odometry_sub = self.create_subscription(Odometry, '/odom', self.odometry_callback, 10)

        
        # ROS2 publishers
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped,
                                                      'initialpose',
                                                      10)
        
        # ROS2 Action clients
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.spin_client = ActionClient(self, Spin, 'spin')
        self.undock_action_client = ActionClient(self, Undock, 'undock')
        self.dock_action_client = ActionClient(self, Dock, 'dock')

        self.get_logger().info(f"Robot commander has been initialized!")
        
    def destroyNode(self):
        self.nav_to_pose_client.destroy()
        super().destroy_node()     

    def goToPose(self, pose, behavior_tree=''):
        """Send a `NavToPose` action request."""
        self.debug("Waiting for 'NavigateToPose' action server")
        while not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.info("'NavigateToPose' action server not available, waiting...")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        goal_msg.behavior_tree = behavior_tree

        self.info('Navigating to goal: ' + str(pose.pose.position.x) + ' ' +
                  str(pose.pose.position.y) + '...')
        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg,
                                                                   self._feedbackCallback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error('Goal to ' + str(pose.pose.position.x) + ' ' +
                       str(pose.pose.position.y) + ' was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True

    def spin(self, spin_dist=1.57, time_allowance=10):
        self.debug("Waiting for 'Spin' action server")
        while not self.spin_client.wait_for_server(timeout_sec=1.0):
            self.info("'Spin' action server not available, waiting...")
        goal_msg = Spin.Goal()
        goal_msg.target_yaw = spin_dist
        goal_msg.time_allowance = Duration(sec=time_allowance)

        self.info(f'Spinning to angle {goal_msg.target_yaw}....')
        send_goal_future = self.spin_client.send_goal_async(goal_msg, self._feedbackCallback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error('Spin request was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True
    
    def undock(self):
        """Perform Undock action."""
        self.info('Undocking...')
        self.undock_send_goal()

        while not self.isUndockComplete():
            time.sleep(0.1)

    def undock_send_goal(self):
        goal_msg = Undock.Goal()
        self.undock_action_client.wait_for_server()
        goal_future = self.undock_action_client.send_goal_async(goal_msg)

        rclpy.spin_until_future_complete(self, goal_future)

        self.undock_goal_handle = goal_future.result()

        if not self.undock_goal_handle.accepted:
            self.error('Undock goal rejected')
            return

        self.undock_result_future = self.undock_goal_handle.get_result_async()

    def isUndockComplete(self):
        """
        Get status of Undock action.

        :return: ``True`` if undocked, ``False`` otherwise.
        """
        if self.undock_result_future is None or not self.undock_result_future:
            return True

        rclpy.spin_until_future_complete(self, self.undock_result_future, timeout_sec=0.1)

        if self.undock_result_future.result():
            self.undock_status = self.undock_result_future.result().status
            if self.undock_status != GoalStatus.STATUS_SUCCEEDED:
                self.info(f'Goal with failed with status code: {self.status}')
                return True
        else:
            return False

        self.info('Undock succeeded')
        return True

    def cancelTask(self):
        """Cancel pending task request of any type."""
        self.info('Canceling current task.')
        if self.result_future:
            future = self.goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, future)
        return

    def isTaskComplete(self):
        """Check if the task request of any type is complete yet."""
        if not self.result_future:
            # task was cancelled or completed
            return True
        rclpy.spin_until_future_complete(self, self.result_future, timeout_sec=0.10)
        if self.result_future.result():
            self.status = self.result_future.result().status
            if self.status != GoalStatus.STATUS_SUCCEEDED:
                self.debug(f'Task with failed with status code: {self.status}')
                return True
        else:
            # Timed out, still processing, not complete yet
            return False

        self.debug('Task succeeded!')
        return True

    def getFeedback(self):
        """Get the pending action feedback message."""
        return self.feedback

    def getResult(self):
        """Get the pending action result message."""
        if self.status == GoalStatus.STATUS_SUCCEEDED:
            return TaskResult.SUCCEEDED
        elif self.status == GoalStatus.STATUS_ABORTED:
            return TaskResult.FAILED
        elif self.status == GoalStatus.STATUS_CANCELED:
            return TaskResult.CANCELED
        else:
            return TaskResult.UNKNOWN

    def waitUntilNav2Active(self, navigator='bt_navigator', localizer='amcl'):
        """Block until the full navigation system is up and running."""
        self._waitForNodeToActivate(localizer)
        if not self.initial_pose_received:
            time.sleep(1)
        self._waitForNodeToActivate(navigator)
        self.info('Nav2 is ready for use!')
        return

    def _waitForNodeToActivate(self, node_name):
        # Waits for the node within the tester namespace to become active
        self.debug(f'Waiting for {node_name} to become active..')
        node_service = f'{node_name}/get_state'
        state_client = self.create_client(GetState, node_service)
        while not state_client.wait_for_service(timeout_sec=1.0):
            self.info(f'{node_service} service not available, waiting...')

        req = GetState.Request()
        state = 'unknown'
        while state != 'active':
            self.debug(f'Getting {node_name} state...')
            future = state_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                state = future.result().current_state.label
                self.debug(f'Result of get_state: {state}')
            time.sleep(2)
        return
    
    def YawToQuaternion(self, angle_z = 0.):
        quat_tf = quaternion_from_euler(0, 0, angle_z)

        # Convert a list to geometry_msgs.msg.Quaternion
        quat_msg = Quaternion(x=quat_tf[0], y=quat_tf[1], z=quat_tf[2], w=quat_tf[3])
        return quat_msg

    def _amclPoseCallback(self, msg):
        self.debug('Received amcl pose')
        self.initial_pose_received = True
        self.current_pose = msg.pose
        return

    def _feedbackCallback(self, msg):
        self.debug('Received action feedback message')
        self.feedback = msg.feedback
        return
    
    def _dockCallback(self, msg: DockStatus):
        self.is_docked = msg.is_docked

    def setInitialPose(self, pose):
        msg = PoseWithCovarianceStamped()
        msg.pose.pose = pose
        msg.header.frame_id = self.pose_frame_id
        msg.header.stamp = 0
        self.info('Publishing Initial Pose')
        self.initial_pose_pub.publish(msg)
        return

    def info(self, msg):
        self.get_logger().info(msg)
        return

    def warn(self, msg):
        self.get_logger().warn(msg)
        return

    def error(self, msg):
        self.get_logger().error(msg)
        return

    def debug(self, msg):
        self.get_logger().debug(msg)
        return

    def cancel_current_goal(self):
        if self.goal_handle is not None:
            self.info("Cancelling current goal.")
            cancel_future = self.goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=1.0)
            self.goal_handle = None

    def moveRobotToPosition(self, x, y):
        # Cancel any existing goal first.
        self.cancel_current_goal()

        # Now, proceed to set up the new goal.
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = 2.7
        goal_pose.pose.position.y = -1.3
        goal_pose.pose.orientation = self.YawToQuaternion(0.0)

        # Send the new goal.
        self.goToPose(goal_pose)
        self.info(f"Moving to a new position: x={x}, y={y}")

    def cancel_current_goal(self):
        if self.goal_handle:
            self.info("Cancelling current goal")
            cancel_future = self.goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=1.0)
            self.goal_handle = None
            self.info("Cancellation request sent")


    def faceRecognizedCallback(self, msg):
        if msg.id == 0:
            pass
            # if not self.should_move:
            #     x = msg.pose.position.x
            #     y = msg.pose.position.y
            #     self.should_move = True
            #     # self.currentGoalPos = [x, y]
        elif msg.id == 1:
            print("currentNormal: ", msg)
            self.currentNormal = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]

    def newFaceCallback(self, msg):
        self.faces_to_visit.append([msg.pose.position.x, msg.pose.position.y])
        self.faceDetected = True


        # self.moveRobotToPosition(x, y)

    def quaternion_to_yaw(self, quaternion):
        # Convert quaternion to Euler angles (yaw)
        q = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        roll, pitch, yaw = tf_transformations.euler_from_quaternion(q)
        return yaw

    def odometry_callback(self, msg):
        try:
            transform = self.tf_buffer.lookup_transform('base_link', 'odom', rclpy.time.Time())
            quaternion = transform.transform.rotation
            self.current_yaw = self.quaternion_to_yaw(quaternion)
        except Exception as e:
            self.get_logger().error(f"Error: {e}")

    def transformCoordinates(self, x, y):
        # Create a PointStamped in the /base_link frame of the robot
        # The point is located 0.5m in from of the robot
        # "Stamped" means that the message type contains a Header
        point_in_robot_frame = PointStamped()
        point_in_robot_frame.header.frame_id = "/base_link"
        point_in_robot_frame.header.stamp = self.get_clock().now().to_msg()

        point_in_robot_frame.point.x = x
        point_in_robot_frame.point.y = y
        point_in_robot_frame.point.z = 0.

        # Now we look up the transform between the base_link and the map frames
        # and then we apply it to our PointStamped
        time_now = rclpy.time.Time()
        timeout = Dur(seconds=0.1)
        try:
            # An example of how you can get a transform from /base_link frame to the /map frame
            # as it is at time_now, wait for timeout for it to become available
            trans = self.tf_buffer.lookup_transform("map", "base_link", time_now, timeout)
            self.get_logger().info(f"Looks like the transform is available.")

            # Now we apply the transform to transform the point_in_robot_frame to the map frame
            # The header in the result will be copied from the Header of the transform
            point_in_map_frame = tfg.do_transform_point(point_in_robot_frame, trans)
            self.get_logger().info(f"We transformed a PointStamped!")

            return point_in_map_frame

        except TransformException as te:
            self.get_logger().info(f"Cound not get the transform: {te}")

            return None

def polar_to_cartesian(r, theta):
    theta = math.atan2(math.sin(theta), math.cos(theta))
    x = r * math.cos(theta)
    y = r * math.sin(theta)
    return x, y

def calculate_angle(r, theta, x2, y2):
    x1, y1 = polar_to_cartesian(r, theta)
    dot_product = x1 * x2 + y1 * y2
    magnitude_a = math.sqrt(x1**2 + y1**2)
    magnitude_b = math.sqrt(x2**2 + y2**2)
    if magnitude_a == 0 or magnitude_b == 0:
        return 0
    cos_angle = dot_product / (magnitude_a * magnitude_b)
    angle = math.acos(cos_angle)
    return angle
    
def main(args=None):
    
    rclpy.init(args=args)
    rc = RobotCommander()

    # Wait until Nav2 and Localizer are available
    rc.waitUntilNav2Active()

    # Check if the robot is docked, only continue when a message is recieved
    while rc.is_docked is None:
        rclpy.spin_once(rc, timeout_sec=0.5)

    # If it is docked, undock it first
    if rc.is_docked:
        rc.undock()
    
    # Finally send it a goal to reach
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = rc.get_clock().now().to_msg()

    goal_points = [[2.6, -1.3], [2.0, 1.5], [0.1, -0.1]]
    should_stop = False


    for point in goal_points:
        rc.currentGoalPos = [point[0], point[1]]
        goal_pose.pose.position.x = rc.currentGoalPos[0]
        goal_pose.pose.position.y = rc.currentGoalPos[1]
        # goal_pose.pose.orientation = rc.YawToQuaternion(0.57)

        rc.goToPose(goal_pose)
        while not rc.isTaskComplete():
            rc.info("Waiting for the task to complete...")
            print("NUM OF FACES TO GO TO: ", len(rc.faces_to_visit))
            if len(rc.faces_to_visit) > 0:
                goal_pose.pose.position.x = rc.faces_to_visit[0][0]
                goal_pose.pose.position.x = rc.faces_to_visit[0][1]

                rc.faces_to_visit.pop(0)

                # print("FOUND NEW FACE")
                # rc.info(f"Face recognized at x={rc.currentGoalPos[0]}, y={rc.currentGoalPos[1]}, redirecting robot.")

                cancel = rc.goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(rc, cancel, timeout_sec=1.0)

                time.sleep(2)

                goal_pose = PoseStamped()
                goal_pose.header.frame_id = 'map'
                goal_pose.header.stamp = rc.get_clock().now().to_msg()
                transCoords = rc.transformCoordinates(goal_pose.pose.position.x, goal_pose.pose.position.y) 
                goal_pose.pose.position.x = transCoords.point.x
                goal_pose.pose.position.y = transCoords.point.y
                yaw_degrees = 0.0
                print("CURRENT ROTATION", rc.current_yaw)
                print("NORMAL: ", rc.currentNormal)
                if len(rc.currentNormal) < 3:
                    rc.currentNormal = [0, 0, 0]
                else:
                    yaw_degrees = calculate_angle(1, 0, rc.currentNormal[0], rc.currentNormal[1])
                    yaw_degrees -= math.pi / 2
                    # yaw_degrees += math.degrees(rc.current_yaw)
                    print("YAW", yaw_degrees)


                goal_pose.pose.position.x += rc.currentNormal[1] * 0.01
                goal_pose.pose.position.y += rc.currentNormal[0] * 0.01
                # goal_pose.pose.orientation = rc.YawToQuaternion(0)
                rc.goToPose(goal_pose)
                
                rc.info("GOING TO NEW POSE")

                while not rc.isTaskComplete():
                    rc.info("Going to the face location...")
                    time.sleep(1)
                
                rc.spin(-rc.current_yaw)
                while not rc.isTaskComplete():
                    rc.info("Rotating to zero...")
                    time.sleep(1)

                time.sleep(2)
                print("NOW ROTATING TO FACE")

                
                rc.spin(yaw_degrees)
                while not rc.isTaskComplete():
                    rc.info("Rotating towards face...")
                    time.sleep(1)

                time.sleep(2)

                # mixer.init()
                # sound = mixer.Sound("hello.wav")
                # sound.play()
                # time.sleep(5)

                rc.faceDetected = False
                rc.should_move = False
                rc.face_counter += 1

                rc.info("SAID HELLO")

                # if rc.face_counter >= 3:
                #     print("THREE FACES DETECTED, STOPPING")
                #     stopping = rc.goal_handle.cancel_goal_async()
                #     rclpy.spin_until_future_complete(rc, stopping, timeout_sec=1.0)
                #     should_stop = True
                #     break

                goal_pose.pose.position.x = point[0]
                goal_pose.pose.position.y = point[1]
                # goal_pose.pose.orientation = rc.YawToQuaternion(0.99)

                rc.info(f"GOING BACK TO THE GOAL x={point[0]}, y={point[1]}")
                rc.goToPose(goal_pose)

            if should_stop:
                break

            time.sleep(1)


    rc.destroyNode()

    # And a simple example
if __name__=="__main__":
    main()