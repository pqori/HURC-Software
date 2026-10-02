# 3D Object Detection from ZED SDK
# Enhanced detection with native 3D bounding box and tracking

# Standard 2D detection info
vision_msgs/Detection2D detection_2d

# 3D Bounding Box (in camera/world frame)
geometry_msgs/Point position          # Center position in 3D
geometry_msgs/Vector3 dimensions      # Width, height, depth
geometry_msgs/Quaternion orientation  # 3D orientation

# Tracking and Quality
int32 tracking_id                     # Persistent object ID (-1 if not tracked)
string tracking_state                 # "OK", "SEARCHING", "TERMINATE"
float32 confidence_3d                 # 3D detection confidence (0-1)
float32 distance                      # Distance from camera (meters)

# Motion (if tracking enabled)
geometry_msgs/Vector3 velocity        # 3D velocity vector
geometry_msgs/Vector3 bounding_box_2d # 2D bounding box in image coordinates

# Mission metadata
string mission_category               # "science", "equipment", "navigation", "human", "obstacle"
float32 mission_priority             # Mission-specific priority (0-1)
float32 interaction_distance         # Required approach distance
