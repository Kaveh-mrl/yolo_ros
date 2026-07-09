# represents all YOLO detections in one frame with explicit 2D bbox corners

std_msgs/Header header

# Image size used to compute normalized_corners (evaluate).
uint32 image_width
uint32 image_height

yolo_msgs/DetectionWithCorners[] detections