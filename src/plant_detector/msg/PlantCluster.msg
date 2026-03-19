std_msgs/Header header

# Cluster ID in this frame
int32 id

# Centroid position relative to the LiDAR frame (metres)
geometry_msgs/Point position

# Axis-aligned bounding box dimensions (metres)
float32 width   # x
float32 depth   # y
float32 height  # z

# Number of points in the cluster
int32 point_count

# Confidence score [0-1] that this is a potted plant
float32 confidence