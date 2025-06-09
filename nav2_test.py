#!/usr/bin/env python3
# Shebang: tells the system to execute this script using the Python 3 interpreter

# Import ROS 2 client library
import rclpy

# Import the BasicNavigator class for simple navigation commands
from nav2_simple_commander.robot_navigator import BasicNavigator

# Import message type for 3D position and orientation (pose)
from geometry_msgs.msg import PoseStamped

# Import transformation utilities to convert Euler angles to quaternions
import tf_transformations

# --- Function to generate a PoseStamped message with given position and yaw (orientation_z)
def create_pose_stamped(navigator, position_x, position_y, orientation_z):
    # Convert Euler angles (roll=0, pitch=0, yaw=orientation_z) to quaternion
    q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, orientation_z)
    
    # Create a PoseStamped message to define position and orientation
    pose = PoseStamped()
    pose.header.frame_id = 'map'  # Reference frame is 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()  # Use current time
    
    # Set the position coordinates
    pose.pose.position.x = position_x
    pose.pose.position.y = position_y
    pose.pose.position.z = 0.0  # Typically 0 for ground robots

    # Set the orientation using the quaternion
    pose.pose.orientation.x = q_x
    pose.pose.orientation.y = q_y
    pose.pose.orientation.z = q_z
    pose.pose.orientation.w = q_w
    
    return pose

def main():
    # Initialize the ROS 2 Python client library
    rclpy.init()

    # Create a BasicNavigator object to control navigation behavior
    nav = BasicNavigator()
    
    # Create and assign the robot's initial pose
    initial_pose = create_pose_stamped(nav, 0.0, 0.0, 0.0)

    # Provide the initial pose to the navigation system
    # nav.setInitialPose(initial_pose)  # Uncomment if needed; should be called only once at startup
    
    # Wait for the navigation stack to become active
    nav.waitUntilNav2Active()
    
    # --- Define multiple goal poses (waypoints) with position and orientation
    goal_pose1 = create_pose_stamped(nav, 3.5, 1.0, 1.57)   # 90 degrees yaw
    goal_pose2 = create_pose_stamped(nav, 2.0, 2.5, 3.14)   # 180 degrees yaw
    goal_pose3 = create_pose_stamped(nav, 0.5, 1.0, -1.57)  # -90 degrees yaw

    # --- Optionally navigate to a single pose (commented out for waypoint mode)
    # nav.goToPose(goal_pose1)
    # while not nav.isTaskComplete():
    #     feedback = nav.getFeedback()
    #     # print(feedback)  # Display feedback if needed

    # --- Navigate through a series of waypoints
    waypoints = [goal_pose1, goal_pose2, goal_pose3]
    nav.followWaypoints(waypoints)

    # Monitor feedback while the robot navigates
    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        # print(feedback)  # Uncomment to see live feedback (e.g., distance to goal)

    # Print the final result after reaching the goal(s)
    print(nav.getResult())

    # Clean shutdown of ROS 2 node
    rclpy.shutdown()

# Execute the script's main function
if __name__ == '__main__':
    main()
