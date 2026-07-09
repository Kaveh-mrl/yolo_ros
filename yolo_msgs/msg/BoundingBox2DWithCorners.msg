# 2D oriented bounding box with explicit corners.

# Same structure as BoundingBox2D.
# center.position is in pixel coordinates.
# size is in pixels.
# theta is the bbox orientation.
yolo_msgs/Pose2D center
yolo_msgs/Vector2 size

# Four bbox corners in normalized image coordinates.
# Convention:
#   (0, 0) is image center
#   x in [-1, 1], +x is right
#   y in [-1, 1], +y is down
geometry_msgs/Polygon normalized_corners