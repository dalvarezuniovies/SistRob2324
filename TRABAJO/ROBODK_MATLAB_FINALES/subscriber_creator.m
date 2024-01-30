function [pose,scan] = subscriber_creator(nodo)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
pose=ros2subscriber(nodo, "/amcl_pose", "geometry_msgs/PoseWithCovarianceStamped"); 

scan=ros2subscriber(nodo, "/scan", "sensor_msgs/LaserScan");

end