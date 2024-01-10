%conectar robola2g 74269891
 nodo = ros2node("/nodo_navegacion");
chatterPub = ros2publisher(nodo,"/goal_pose","geometry_msgs/PoseStamped") %crear publisher
% Creamos un mensaje y lo rellenamos 
chatterMsg = ros2message(chatterPub)
chatterMsg.header.frame_id ='map'
chatterMsg.pose.position.x=0.106
chatterMsg.pose.position.y=-2.52
chatterMsg.pose.orientation.w=1
%send(chatterPub,chatterMsg) %envío mensaje 
%ros2 topic list -t tipo mensajes




