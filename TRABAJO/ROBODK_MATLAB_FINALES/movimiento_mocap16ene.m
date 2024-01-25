%--------------Cargar datos----------------------
load('WS_17_enero.mat') %Se cargan los datos de la trayectoria entrenada usando el sistema OptiTrack
load('red.mat') %Se cargan los datos de la red neuronal entrenada

%%
%%--------------Generar programa RoboDK-----------
% Generate a Robolink object RDK. This object interface with RoboDK
RDK = Robolink;

% Get the library path
path=RDK.getParam('PATH_LIBRARY');

% Display a list of all items
RDK.ItemList();

% Get some items in the station by their name. Each item is visible in the
% current project tree
robot = RDK.Item('UR3e');
fprintf('Robot selected:\t%s\n', robot.Name());
%%
% robot frame, hide it
Br=robot.Parent();
Br.setVisible(0);

% define a new rotated frame to be base frame
B0=RDK.AddFrame('B0 frame', Br);
Hr0=transl(70,-90,0)*rotx(pi/2); %*rotx(pi/2);  %transl(0,-50,-10)*rotz(-pi/2)*rotx(pi/2);  %transl(195,-30,-5)*rotz(-pi/2)*rotx(pi/2);  %transl(0,-0,0)*rotz(-pi/2)*rotx(pi/2);  %transl(430,-430,0)*rotz(-pi/2)*rotx(pi/2); 
B0.setPose(Hr0);
% from now on, all motions are taken with respect to B0
robot.setPoseFrame(B0);

% Set the rest position joints
Jrest=[0 -90 0 -90 0 0]';
robot.setJoints(Jrest);

% Create home target and program My Prog

% Select first program
prog=RDK.AddProgram('My Prog');

% Create a joint target home
target = RDK.AddTarget('Home',B0,robot);
target.setAsJointTarget();
target.setJoints(Jrest);

% Add joint movement into the program
prog.MoveJ(target);

% Download data

W=W(1:2500);
X=X(1:2500);
Y=Y(1:2500);
Z=Z(1:2500);
X1=X1(1:2500);
Y1=Y1(1:2500);
Z1=Z1(1:2500);
q=quaternion(W,X,Y,Z);
longq=length(q);

% Create target points as Cartesian Targets

%Primer target (coincide con el 901 de la trayectoria grabada)
targetname = sprintf('Target%i',1);
target = RDK.AddTarget(targetname, B0, robot);
ad=[zeros(4,3), [X1(901);Y1(901);Z1(901);1]];
qh=quat2tform(q(901))+ ad;
qh=qh*rotz(pi*24/18)*roty(pi/18);
disp(qh);
target.setPose(qh);
prog.MoveJ(target);

% Segundo target (coincide con el 1001 de la trayectoria grabada)
targetname = sprintf('Target%i',2);
target = RDK.AddTarget(targetname, B0, robot);
ad=[zeros(4,3), [X1(1001);Y1(1001);Z1(1001);1]];
qh=quat2tform(q(1001))+ ad;
qh=qh*rotz(pi*24/18)*roty(pi/18);
disp(qh);
target.setPose(qh);
prog.MoveJ(target);

%Instrucción para volver a home
target = RDK.AddTarget('Home',B0,robot);
target.setAsJointTarget();
target.setJoints(Jrest);
prog.MoveJ(target);

%%
%-----------Tomar foto a la pieza a examinar y cargarla
% Ruta de la imagen a clasificar

imagen_a_clasificar = imread('imagen.jpg'); 
 
% Mostrar la imagen
imshow(imagen_a_clasificar);

%% ----------Enviar al Turtlebot al lugar de recogida de las piezas
%CREAR NODO
nodo = ros2node("/nodo_inicial")
[publisherGoal, publisherVel] = publisher_creator(nodo);
[pose,scan] = subscriber_creator(nodo);


%CREAR PUBLISHER PARA PUBLICAR EN /GOAL_POSE
%publisher=ros2publisher(nodo,"/goal_pose","geometry_msgs/PoseStamped")
%CREAR UNA VARIABLE MENSAJE PARA PUBLICAR EN EL TOPIC
goalMsg=ros2message(publisherGoal)
%EN FRAME_ID ESCRIBIR MAP
goalMsg.header.frame_id='map'

%MANDAR TURTLEBOT4 A RECOGER LA PIEZA
%INDICAR LA COORDENADA X
goalMsg.pose.position.x=0.2105
%INDICAR LA COORDENADA Y
goalMsg.pose.position.y=-2.7
%ORIENTACIÓN
goalMsg.pose.orientation.z=-0.8996
goalMsg.pose.orientation.w=0.4367
pause(5)
%MANDARLE EL MENSAJE
send(publisherGoal, goalMsg)
pause(5)
%% ACERCAR ROBOT

velMsg=ros2message(publisherVel)
velMsg.linear.x=1.5
pause(2)
send(publisherVel, velMsg)
pause(5)
%%
velMsg=ros2message(publisherVel)
velMsg.angular.z=-1.5;
velMsg.linear.x=0;
pause(2);
send(publisherVel, velMsg)
pause(5)


%%

%------- Realizar la predicción
etiqueta_predicha = classify(net, imagen_a_clasificar);
% Mostrar la etiqueta predicha
disp(['La red predice que la imagen pertenece a la clase: ' char(etiqueta_predicha)]);
% Guardar la prediccion en una variable booleana
es_rectangulo_roto=etiqueta_predicha=='rectangulo_roto';
es_cuadrado_roto=etiqueta_predicha=='cuadrado_roto';
es_rectangulo_valido=etiqueta_predicha=='rectangulo_valido';
es_cuadrado_valido=etiqueta_predicha=='cuadrado_valido';


%%
%--------Establecer el modo de funcionamiento para que se ejecute en el
%robot real el programa de RoboDK

RDK.setRunMode(4);
    
% Conectarse al robot real y ejecutar el programa
robot.Connect();
prog.RunProgram();
% Wait for the movement to finish
while robot.Busy()
    pause(1);
    fprintf('Waiting for the robot to finish...\n');
end

%------En funcion de la pieza mandar a un sitio o a otro
%% ALEJAR ROBOT
velMsg=ros2message(publisherVel)
velMsg.angular.z=+1.5;
velMsg.linear.x=0;
pause(5);
send(publisherVel, velMsg)
pause(5)

for i=1:1:2
velMsg=ros2message(publisherVel)
velMsg.linear.x=-4
velMsg.angular.z=0
send(publisherVel, velMsg)
pause(10)
end

%%
if es_cuadrado_valido || es_rectangulo_valido
%MANDAR A TURTLEBOT4 A DEJAR LA PIEZA BUENA EN SU BASE

%INDICAR LA COORDENADA X
goalMsg.pose.position.x=0
%INDICAR LA COORDENADA Y
goalMsg.pose.position.y=0
%ORIENTACIÓN
goalMsg.pose.orientation.z=0.0726
goalMsg.pose.orientation.w=0.9974
pause(5)
%MANDARLE EL MENSAJE
send(publisherGoal, goalMsg)
pause(5)

else
% DEJAR LAS PIEZAS ROTAS

%INDICAR LA COORDENADA X
goalMsg.pose.position.x=-1.6462
%INDICAR LA COORDENADA Y    
goalMsg.pose.position.y=0.2491
%ORIENTACIÓN
goalMsg.pose.orientation.z=0.8375
goalMsg.pose.orientation.w=0.5465
%MANDARLE EL MENSAJE
send(publisherGoal, goalMsg)  

end
