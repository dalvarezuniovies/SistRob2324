%%
% Archivo para comunicarse con Turtlebot4 a través de Matlab"
%%
%CREAR NODO
nodo = ros2node("/nodo_inicial")
%CREAR PUBLISHER PARA PUBLICAR EN /GOAL_POSE
publisher=ros2publisher(nodo,"/goal_pose","geometry_msgs/PoseStamped")
%CREAR UNA VARIABLE MENSAJE PARA PUBLICAR EN EL TOPIC
goalMsg=ros2message(publisher)
%EN FRAME_ID ESCRIBIR MAP
goalMsg.header.frame_id='map'
%
%% MANDAR TURTLEBOT4 A RECOGER LA PIEZA
%
%INDICAR LA COORDENADA X
goalMsg.pose.position.x=0.2105
%INDICAR LA COORDENADA Y
goalMsg.pose.position.y=-2.7
%ORIENTACIÓN
goalMsg.pose.orientation.z=-0.8996
goalMsg.pose.orientation.w=0.4367     
%MANDARLE EL MENSAJE
send(publisher, goalMsg)
%
%% MANDAR A TURTLEBOT4 A DEJAR LA PIEZA BUENA EN SU BASE
%
%INDICAR LA COORDENADA X
goalMsg.pose.position.x=-0.4602
%INDICAR LA COORDENADA Y
goalMsg.pose.position.y=-0.0141
%ORIENTACIÓN
goalMsg.pose.orientation.z=0.0726
goalMsg.pose.orientation.w=0.9974
%MANDARLE EL MENSAJE
send(publisher, goalMsg)
%
%% DEJAR LAS PIEZAS ROTAS
%
%INDICAR LA COORDENADA X
goalMsg.pose.position.x=-1.6462
%INDICAR LA COORDENADA Y    
goalMsg.pose.position.y=0.2491
%ORIENTACIÓN
goalMsg.pose.orientation.z=0.8375
goalMsg.pose.orientation.w=0.5465
%MANDARLE EL MENSAJE
send(publisher, goalMsg)