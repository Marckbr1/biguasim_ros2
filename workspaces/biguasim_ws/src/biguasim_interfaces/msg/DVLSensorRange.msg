# DVLSensor message
# Contains header, velocity, and range measurements
#
# NOTE: recreated from the schema embedded in a recorded BiguaSim rosbag
# (see ImagingSonar.msg).

std_msgs/Header header

# Range measurements in meters from the 4 sonar beams
float32[4] range
