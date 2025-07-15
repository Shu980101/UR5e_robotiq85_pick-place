#!/usr/bin/env python3
"""
Simple UR5e Pick and Place - Two targets only
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
import numpy as np
import time
from threading import Thread

# Core messages
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker, MarkerArray
from moveit_msgs.srv import GetPositionIK
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as R
from linkattacher_msgs.srv import AttachLink, DetachLink


class SimpleGripper(Node):
    """Simple gripper control"""
    
    def __init__(self):
        super().__init__('simple_gripper')
        
        # Try joint control for gripper
        self.joint_pubs = {}
        joint_names = ['robotiq_85_left_knuckle_joint', 'robotiq_85_right_knuckle_joint']
        
        for joint_name in joint_names:
            topic = f'/robotiq_gripper/{joint_name}/command'
            self.joint_pubs[joint_name] = self.create_publisher(Float64, topic, 10)
    
    def open(self):
        """Open gripper"""
        for pub in self.joint_pubs.values():
            msg = Float64()
            msg.data = 0.0  # Open position
            pub.publish(msg)
        time.sleep(1.0)
        self.get_logger().info("Gripper opened")
        
    def close(self):
        """Close gripper"""
        for pub in self.joint_pubs.values():
            msg = Float64()
            msg.data = 0.8  # Closed position
            pub.publish(msg)
        time.sleep(1.0)
        self.get_logger().info("Gripper closed")

class SimpleUR5e(Node):
    """Simple UR5e controller"""
    
    def __init__(self):
        super().__init__('simple_ur5e')
        
        # Joint control
        self.joint_client = ActionClient(self, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')
        
        # Joint names
        self.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        self.marker_pub = self.create_publisher(MarkerArray, '/target_poses', 10)
        # Gripper
        self.gripper = SimpleGripper()
        
        # Predefined positions (pose -> joint angles)
        self.positions = {
            'home': [0.0, -1.57, 0.0, -1.57, 0.0, 0.0],
            'ready1': [-1.57, -1.57, -1.57, -1.57, 1.57, 0.0],
            'ready2': [0.0, -1.57, -1.57, -1.57, 1.57, 0.0],
        }

        self.attach_client = self.create_client(AttachLink, '/ATTACHLINK')
        self.detach_client = self.create_client(DetachLink, '/DETACHLINK')

        self.get_logger().info("Simple UR5e initialized")

    def attach_object(self, model1, link1, model2, link2):
        """Simulate grasping by calling /ATTACHLINK"""
        req = AttachLink.Request()
        req.model1_name = model1
        req.link1_name = link1
        req.model2_name = model2
        req.link2_name = link2
        
        if not self.attach_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("AttachLink service not available")
            return False
        
        future = self.attach_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info(f"✅ Attached {model2} to {model1}")
        return True

    def detach_object(self, model1, link1, model2, link2):
        """Simulate releasing by calling /DETACHLINK"""
        req = DetachLink.Request()
        req.model1_name = model1
        req.link1_name = link1
        req.model2_name = model2
        req.link2_name = link2
        
        if not self.detach_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("DetachLink service not available")
            return False
        
        future = self.detach_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info(f"✅ Detached {model2} from {model1}")
        return True

    def open(self):

        for pub in self.joint_pubs.values():
            msg = Float64()
            msg.data = 0.0  # Open position
            pub.publish(msg)
        time.sleep(1.0)
        self.get_logger().info("Gripper opened")

    def close(self):
        """Close gripper"""
        for pub in self.joint_pubs.values():
            msg = Float64()
            msg.data = 0.8  # Closed position
            pub.publish(msg)
        time.sleep(1.0)
        self.get_logger().info("Gripper closed")

    def move_to_position(self, position_name, duration=3.0):
        """Move to a predefined position"""
        if position_name not in self.positions:
            self.get_logger().error(f"Position '{position_name}' not found")
            return False
            
        joint_positions = self.positions[position_name]
        
        # Create trajectory
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        
        trajectory.points = [point]
        
        # Send goal
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        
        self.get_logger().info(f"Moving to {position_name}")
        
        future = self.joint_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() and future.result().accepted:
            result_future = future.result().get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
            
            if result_future.result().result.error_code == 0:
                self.get_logger().info(f"Reached {position_name}")
                return True
                
        self.get_logger().error(f"Failed to reach {position_name}")
        return False
        
    def pick_object(self):
        """Pick up object"""
        self.get_logger().info("=== PICKING OBJECT ===")
        
        # Open gripper
        self.gripper.open()
        
        # Move to approach position
        if not self.move_to_position('pick_approach'):
            return False
            
        # Move down to pick
        if not self.move_to_position('pick'):
            return False
            
        # Close gripper
        self.gripper.close()
        
        # Move back up
        if not self.move_to_position('pick_approach'):
            return False
            
        self.get_logger().info("✅ Object picked successfully")
        return True
        
    def place_object(self):
        """Place object"""
        self.get_logger().info("=== PLACING OBJECT ===")
        
        # Move to approach position
        if not self.move_to_position('place_approach'):
            return False
            
        # Move down to place
        if not self.move_to_position('place'):
            return False
            
        # Open gripper
        self.gripper.open()
        
        # Move back up
        if not self.move_to_position('place_approach'):
            return False
            
        self.get_logger().info("✅ Object placed successfully")
        return True
        
    def pose_to_joints(self, x, y, z, roll=0.0, pitch=0.0, yaw=0.0):
        """
        Use MoveIt IK service to convert pose to joint angles
        """

        # Create service client
        ik_client = self.create_client(GetPositionIK, '/compute_ik')

        if not ik_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error('❌ IK service /compute_ik not available.')
            return None

        # Convert to quaternion
        r = R.from_euler('xyz', [roll, pitch, yaw])
        q = r.as_quat()  # Returns [x, y, z, w]


        # Create PoseStamped
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = 'base_link'
        pose_stamped.pose.position.x = x
        pose_stamped.pose.position.y = y
        pose_stamped.pose.position.z = z
        pose_stamped.pose.orientation.x = q[0]
        pose_stamped.pose.orientation.y = q[1]
        pose_stamped.pose.orientation.z = q[2]
        pose_stamped.pose.orientation.w = q[3]

        # Prepare IK request
        request = GetPositionIK.Request()
        request.ik_request.group_name = 'arm_manipulator'  # Make sure this matches your MoveIt planning group
        request.ik_request.pose_stamped = pose_stamped
        request.ik_request.timeout.sec = 1

        future = ik_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            joint_state = future.result().solution.joint_state
            joint_positions = []
            for name in self.joint_names:
                if name in joint_state.name:
                    idx = joint_state.name.index(name)
                    joint_positions.append(joint_state.position[idx])

            self.get_logger().info(f"✅ MoveIt IK: ({x:.3f}, {y:.3f}, {z:.3f}) → joints: {joint_positions}")
            return joint_positions
        else:
            self.get_logger().error("❌ IK service failed to return result")
            return None

            
    def create_pose_position(self, x, y, z, roll=0.0 , pitch=0.0, yaw=0.0):
        """
        Create a new position from pose and add it to predefined positions
        
        Args:
            x, y, z: Position in meters
            roll, pitch, yaw: Orientation in radians
            
        Returns:
            str: Name of the created position, or None if failed
        """
        joint_angles = self.pose_to_joints(x, y, z, roll, pitch, yaw)
        
        if joint_angles is not None:
            # Create unique name
            position_name = f"pose_{x:.2f}_{y:.2f}_{z:.2f}".replace('.', '_').replace('-', 'n')
            
            # Add to positions dictionary
            self.positions[position_name] = joint_angles
            
            self.get_logger().info(f"Created position '{position_name}' from pose")
            return position_name
        else:
            return None
            
    def move_to_pose(self, x, y, z, roll=0.0 , pitch=0.0, yaw=0.0, duration=3.0):
        """
        Move robot to a pose (x, y, z, roll, pitch, yaw)
        
        Args:
            x, y, z: Position in meters (relative to base_link)
            roll, pitch, yaw: Orientation in radians
            duration: Movement duration in seconds
            
        Returns:
            bool: True if successful, False otherwise
        """
        self.get_logger().info(f"Moving to pose: x={x:.3f}, y={y:.3f}, z={z:.3f}")
        
        # Convert pose to joint angles
        joint_angles = self.pose_to_joints(x, y, z, roll, pitch, yaw)
        
        if joint_angles is None:
            self.get_logger().error("Failed to convert pose to joint angles")
            return False
            
        # Move to calculated joint positions
        return self.move_to_joints(joint_angles, duration)
        
    def move_to_joints(self, joint_angles, duration=3.0):
        """
        Move robot to specific joint angles
        
        Args:
            joint_angles: List of 6 joint angles in radians
            duration: Movement duration in seconds
            
        Returns:
            bool: True if successful, False otherwise
        """
        if len(joint_angles) != 6:
            self.get_logger().error("Need exactly 6 joint angles")
            return False
            
        # Create trajectory
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = joint_angles
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        
        trajectory.points = [point]
        
        # Send goal
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        
        self.get_logger().info(f"Moving to joints: {[f'{j:.3f}' for j in joint_angles]}")
        
        future = self.joint_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() and future.result().accepted:
            result_future = future.result().get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
            
            if result_future.result().result.error_code == 0:
                self.get_logger().info("Movement completed successfully")
                return True
                
        self.get_logger().error("Movement failed")
        return False

    def visualize_target_pose(self, name, x, y, z, roll=0.0 , pitch=0.0, yaw=0.0):
        """Visualize a target pose in RViz"""
        
        # Create pose marker
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = f"target_{name}"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        # Set position
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        
        # Convert orientation
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)
        
        marker.pose.orientation.w = cr * cp * cy + sr * sp * sy
        marker.pose.orientation.x = sr * cp * cy - cr * sp * sy
        marker.pose.orientation.y = cr * sp * cy + sr * cp * sy
        marker.pose.orientation.z = cr * cp * sy - sr * sp * cy
        
        # Set appearance
        marker.scale.x = 0.1
        marker.scale.y = 0.01
        marker.scale.z = 0.01
        
        if name == 'pick':
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
        elif name == 'place':
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
        else:
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            
        marker.color.a = 1.0
        
        # Publish marker
        marker_array = MarkerArray()
        marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)
        
        self.get_logger().info(f"Visualized target pose: {name} at ({x:.3f}, {y:.3f}, {z:.3f})")



def main():
    """Main function"""
    rclpy.init()
    
    try:
        # Create robot
        robot = SimpleUR5e()
        
        # Create executor
        executor = MultiThreadedExecutor()
        executor.add_node(robot)
        executor.add_node(robot.gripper)
        
        # Start executor
        executor_thread = Thread(target=executor.spin, daemon=True)
        executor_thread.start()
        
        # Wait for initialization
        time.sleep(2.0)
        
        # === OPTION 1: Use predefined joint positions ===
        robot.get_logger().info("=== Using predefined joint positions ===")
        robot.move_to_position('home')
        
        # Go to ready position
        robot.move_to_position('ready1')
        

        # === OPTION 2: Use pose-to-joint conversion ===
        robot.get_logger().info("=== Using pose-to-joint conversion ===")
        
        # Define pick and place poses (easier to understand)
        pick_pose = {
            'x': 0.4,     # 40cm forward
            'y': 0.2,     # 20cm to the right
            'z': 0.9,     # 10cm up
            'roll': 0.0,  # No roll
            'pitch': 0.0 , # Point down (90 degrees)
            'yaw': 0.0    # No yaw
        }
        
        place_pose = {
            'x': 0.4,     # 40cm forward  
            'y': -0.2,    # 20cm to the left
            'z': 0.9,     # 10cm up
            'roll': 0.0,  # No roll
            'pitch': 0.0 , # Point down (90 degrees)
            'yaw': 0.0    # No yaw
        }
        
        # Convert poses to joint positions and update robot
        pick_joints = robot.pose_to_joints(**pick_pose)
        place_joints = robot.pose_to_joints(**place_pose)

        robot.gripper.open()
        time.sleep(1.0)
        
        robot.move_to_pose(0.3,0.5,0.318,0.0,3.14,0.0)
        time.sleep(1.0)
        robot.move_to_pose(0.3,0.5,0.118,0.0,3.14,0.0)
        time.sleep(1.0)
        robot.gripper.close()
        robot.attach_object('cobot', 'robotiq_85_left_finger_tip_link', 'golf_ball', 'ball_link')
        time.sleep(1.0)
        robot.move_to_pose(0.3,0.5,0.318,0.0,3.14,0.0)
        time.sleep(1.0)

        robot.move_to_position('ready1')
        robot.move_to_position('ready2')

        robot.move_to_pose(-0.3,-0.5,0.318,0.0,3.14,0.0)
        time.sleep(1.0)
        robot.move_to_pose(-0.3,-0.5,0.118,0.0,3.14,0.0)
        time.sleep(1.0)
        robot.detach_object('cobot', 'robotiq_85_left_finger_tip_link', 'golf_ball', 'ball_link')
        robot.gripper.open()
        time.sleep(1.0)
        robot.move_to_pose(-0.3,-0.5,0.318,0.0,3.14,0.0)
        time.sleep(1.0)
        
        
        # === OPTION 3: Direct pose movement ===
        robot.get_logger().info("=== Testing direct pose movement ===")
        
        
        # Return home
        robot.move_to_position('home')
        
            
    except KeyboardInterrupt:
        print("Interrupted by user")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        try:
            executor.shutdown()
            rclpy.shutdown()
        except:
            pass

if __name__ == "__main__":
    main()
