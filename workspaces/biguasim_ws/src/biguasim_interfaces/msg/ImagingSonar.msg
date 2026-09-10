# ImagingSonar message
# Contains timestamp, bins information, and image data
#
# NOTE: recreated field-for-field from the schema embedded in a recorded
# BiguaSim rosbag, for use until the upstream biguasim_interfaces package is
# installed in the container. Field order and types match the schema exactly.

std_msgs/Header header

# Number of azimuth bins
int32 bins_azimuth

# Number of range bins
int32 bins_range

# Sonar geometry (from the sensor configuration) -- needed to interpret
# raw_image without an external YAML config.
float32 range_min                # metros
float32 range_max                # metros
float32 azimuth_aperture_deg     # apertura angular horizontal total
float32 elevation_aperture_deg   # apertura angular vertical total

# Raw sonar image (as received)
sensor_msgs/Image raw_image

# Ground-truth intensity (float, same size as raw)
sensor_msgs/Image intensity

# Ground-truth elevation (per-pixel angle or height)
sensor_msgs/Image elevation

# Ground-truth 3D points
sensor_msgs/PointCloud2 point_cloud
