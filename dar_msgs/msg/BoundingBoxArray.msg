# The BoundingBoxArray message is designed to handle multiple detected objects within a visual scene. 


# The standard ROS header that includes timestamp and frame information.
# The header is crucial for synchronizing and organizing data within a ROS system.
std_msgs/Header header

# An array containing individual BoundingBox messages, each representing a detected object.
# Each BoundingBox message within the array combines class  spatial localization 
BoundingBox[] bounding_boxes