source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
export ROS_IP=127.0.0.1
unset ROS_HOSTNAME
echo "ROS_MASTER_URI=$ROS_MASTER_URI"
