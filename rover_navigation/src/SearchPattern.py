from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
import rclpy

import math
from geometry_msgs.msg import Pose

from visualization_msgs.msg import Marker


from scipy.spatial.transform import Rotation as R

def calculate_standoff_pose(marker_position, standoff_distance=1.0):
    """
    Calculates a Pose 1 meter away from the marker, on the line intersecting 
    the robot and the marker, with the orientation facing the marker.
    
    Args:
        marker_position: geometry_msgs.msg.Point (from the Marker message)
        standoff_distance: float (distance from the object in meters)
        
    Returns:
        geometry_msgs.msg.Pose
    """
    # 1. Get the vector FROM the object TO the robot
    dx = marker_position.x
    dy = marker_position.y
    
    # 2. Calculate the distance between them (2D planar)
    distance = math.hypot(dx, dy)
    
    if distance < 1:
        # Edge case: Robot is perfectly centered on the object. 
        # Cannot compute a line, return safely.
        print("Uh oh")
        return None
        
    # 3. Calculate the unit vector (direction)
    unit_x = dx / distance
    unit_y = dy / distance
    
    # 4. Calculate the target position 
    target_x = marker_position.x - (unit_x * standoff_distance)
    target_y = marker_position.y - (unit_y * standoff_distance)
    target_z = 0  # Assuming we keep the target Z the same
    
    # 5. Calculate the orientation (facing the object)
    # The vector from target TO object is exactly (-unit_x, -unit_y)
    yaw = math.atan2(unit_y, unit_x)
    
    # 6. Convert yaw to quaternion (assuming roll=0, pitch=0)
    qw = math.cos(yaw / 2.0)
    qx = 0.0
    qy = 0.0
    qz = math.sin(yaw / 2.0)
    
    # 7. Construct and return the new Pose
    target_pose = Pose()
    target_pose.position.x = target_x
    target_pose.position.y = target_y
    target_pose.position.z = target_z
    
    target_pose.orientation.x = qx
    target_pose.orientation.y = qy
    target_pose.orientation.z = qz
    target_pose.orientation.w = qw
    
    return target_pose

class SearchPattern(rclpy.node.Node):

    def __init__(self):
        super().__init__("search_pattern_node")
        self.navigator = BasicNavigator()
        self.subscriber = self.create_subscription(Marker, "aruco_detections", self.detection_callback, 5)

    def detection_callback(self, msg):
        self.navigator.cancelTask()
        final_pose = PoseStamped()
        final_pose.header.frame_id = msg.header.frame_id
        final_pose.header.stamp = self.get_clock().now().to_msg()
        final_pose.pose = calculate_standoff_pose(msg.pose.position)

        start_pose = PoseStamped()
        start_pose.header.frame_id = "base_link"
        start_pose.header.stamp = self.get_clock().now().to_msg()
        self.navigator.getPath(start_pose, final_pose)

# first_pose = PoseStamped()
# first_pose.header.frame_id = 'map'
# first_pose.header.stamp = navigator.get_clock().now().to_msg()
# first_pose.pose.position.x = 5
# first_pose.pose.position.y = 0
# first_pose.pose.orientation.z 

if __name__ == "__main__":
    rclpy.init()
    search_node = SearchPattern()
    
    rclpy.spin(search_node)

    search_node.destroy_node()
    rclpy.shutdown()