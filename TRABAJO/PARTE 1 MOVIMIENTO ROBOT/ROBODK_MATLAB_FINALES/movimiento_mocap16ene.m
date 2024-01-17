%%
% Generate a Robolink object RDK. This object interface with RoboDK
RDK = Robolink;

% Get the library path
path=RDK.getParam('PATH_LIBRARY');



% Display a list of all items
RDK.ItemList();

%%
% Get some items in the station by their name. Each item is visible in the
% current project tree
robot = RDK.Item('UR3e');
fprintf('Robot selected:\t%s\n', robot.Name());

%%
% robot frame, hide it
Br=robot.Parent();
Br.setVisible(0);

%%
% define a new rotated frame to be base frame
B0=RDK.AddFrame('B0 frame', Br);
Hr0=transl(70,-90,0)*rotx(pi/2); %*rotx(pi/2);  %transl(0,-50,-10)*rotz(-pi/2)*rotx(pi/2);  %transl(195,-30,-5)*rotz(-pi/2)*rotx(pi/2);  %transl(0,-0,0)*rotz(-pi/2)*rotx(pi/2);  %transl(430,-430,0)*rotz(-pi/2)*rotx(pi/2); 
B0.setPose(Hr0);
% from now on, all motions are taken with respect to B0
robot.setPoseFrame(B0);

%%
% Set the rest position joints
Jrest=[0 -90 0 -90 0 0]';
robot.setJoints(Jrest);

% %% RobolinkItem/setPose()
% % This is the key method for setting the pose of a frame, robot target, ect
% % Requires a 4x4 homogeneos matrix as input
% 
% % Create a target defined by Cartesian pose in B0
% target = RDK.AddTarget('Target', B0);
% target.setAsCartesianTarget();
% 
% % define the pose (H transform)
% H_target=transl(550,0,-100)*rotz(pi/6)*rotx(pi/4);
% 
% % set the pose
% target.setPose(H_target);



%% Create home target and program My Prog

% Select first program
prog=RDK.AddProgram('My Prog');

% Create a joint target home
target = RDK.AddTarget('Home',B0,robot);
target.setAsJointTarget();
target.setJoints(Jrest);

% Add joint movement into the program
prog.MoveJ(target);

% Download data
load('WS_17_enero.mat')
W=W(1:2500);
X=X(1:2500);
Y=Y(1:2500);
Z=Z(1:2500);
X1=X1(1:2500);
Y1=Y1(1:2500);
Z1=Z1(1:2500);
q=quaternion(W,X,Y,Z);
longq=length(q);


%% Create target points as Cartesian Targets

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

%% Establecer el modo de funcionamiento para que se ejecute en el robot real

RDK.setRunMode(4);
    
%% Conectarse al robot real y ejecutar el programa

robot.Connect();
prog.RunProgram();
% Wait for the movement to finish
while robot.Busy()
    pause(1);
    fprintf('Waiting for the robot to finish...\n');
end

% Run the program once again
fprintf('Running the program again...\n');








