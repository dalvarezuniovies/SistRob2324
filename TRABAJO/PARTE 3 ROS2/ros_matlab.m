%conectar robola2g 74269891
% nodo = ros2node("/nodo_inicial");
% ros2 node list
% ros2 topic list
%chatterSub = ros2subscriber(nodo,"/cmd_vel") %suscribirse al topic chatter
%mensaje_testSub = receive(chatterSub,10) %leer cada 10s
chatterPub = ros2publisher(nodo,"/turtle1/cmd_vel","geometry_msgs/Twist") %crear publisher
% Creamos un mensaje y lo rellenamos 
chatterMsg = ros2message(chatterPub)
chatterMsg.linear.x =1.0
%chatterMsg.angular='[0.0, 0.0, 0.0]'
send(chatterPub,chatterMsg) %envío mensaje 
%ros2 topic list -t tipo mensajes




