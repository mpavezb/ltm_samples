# static fields
uint32 uid
uint32 log_uid
string name
string city
string country
sensor_msgs/Image image

# dynamic fields
# - assume points in /map frame
geometry_msgs/Point center
geometry_msgs/Polygon shape
string map_name