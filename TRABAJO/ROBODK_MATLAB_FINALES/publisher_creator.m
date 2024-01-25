function [publisherGoal, publisherVel] = publisher_creator(nodo)
publisherGoal = ros2publisher(nodo,"/goal_pose","geometry_msgs/PoseStamped");
publisherVel =ros2publisher(nodo,"/cmd_vel", "geometry_msgs/Twist");
end