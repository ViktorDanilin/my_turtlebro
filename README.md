# connection 
* hostname ip : 192.168.19.140
* turtlebro ip : 192.169.19.192
* connect to turtlebro : ssh pi@turtlebro73.local
* pwd: brobro
# redaction ~/.bashrc
* sudo nano ~/.bashrc
* laptop: export ROS_MASTER_URI='http://192.168.19.192:11311/'
* laptop: export ROS_HOSTNAME=192.168.19.140
* robot: export ROS_MASTER_URI='http://192.168.19.192:11311/'
* robot: export ROS_HOSTNAME=192.168.19.140

