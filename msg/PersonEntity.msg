# static fields
uint32 uid
uint32 log_uid
string name
string last_name
uint8 age
uint8 genre
string country
string city
ltm/Date birthday
sensor_msgs/Image face
sensor_msgs/Image body

# dynamic fields
string emotion
string stance
time last_seen
time last_interacted



# Available GENRES
uint8 MASCULINE=0
uint8 FEMININE=1