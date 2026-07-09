# defines a YOLO detection result

# class probability
string class_name
float64 score

# ID for tracking (if needed in the future)
# string id

# 2D bounding box surrounding the object in pixels
yolo_msgs/BoundingBox2DWithCorners bbox