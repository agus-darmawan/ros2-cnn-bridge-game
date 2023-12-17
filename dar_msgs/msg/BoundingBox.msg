# This custom BoundingBox message seamlessly integrates object detection and class information


# This variable describes the class name associated with the detected object.
string label
string class_name
uint8 number

# Specifies a rectangular region enclosing a particular object within an image or frame.
vision_msgs/BoundingBox2D bounding_box