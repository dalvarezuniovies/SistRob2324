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
Hr0=transl(0,-50,-10)*rotz(-pi/2)*rotx(pi/2);  %transl(195,-30,-5)*rotz(-pi/2)*rotx(pi/2);  %transl(0,-0,0)*rotz(-pi/2)*rotx(pi/2);  %transl(430,-430,0)*rotz(-pi/2)*rotx(pi/2); 
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



%% SolveIK: for inverse kinematics
%para calcular todas las rotaciones conocido el punto objetivo

prog=RDK.AddProgram('My Prog');

% Create a joint target home
target = RDK.AddTarget('Home',B0,robot);
target.setAsJointTarget();
target.setJoints(Jrest);
%sacar orientacion inicial
m_inicial=target.Pose();
m=m_inicial(1:3,1:3);


% Add joint movement into the program
prog.MoveJ(target);
load('WS_punto_18dic.mat')
% define the pose (H transforms) Cambiar las unidades
% Download date
% load('WS_14dic.mat')
% W=W;
% X=X;
% Y=Y;
% Z=Z;
% X1=X1*1000;
% Y1=Y1*1000;
% Z1=Z1*1000;
% q=quaternion(W,X,Y,Z);
% longq=length(q);
X=round(X);
Y=round(Y);
Z=round(Z);
longq=length(X);

%%

% for i=1:400:longq
%      targetname = sprintf('Target%i',i);
%      target = RDK.AddTarget(targetname, B0, robot);
%      target.setAsJointTarget();
%      %ad=[zeros(4,3), [X1(i);Y1(i);Z1(i);0]];
%      ad=[zeros(4,3), [X1(i);Y1(i);Z1(i);1]]
%      qh=0*ones(4,4)+ad;
%      qh=quat2tform(q(i))+ ad;
%      disp(qh);
%      target.setPose(qh);
%      prog.MoveJ(target);
%     
% end
% for i=401:400:longq
%      targetname = sprintf('Target%i',i);
%      target = RDK.AddTarget(targetname, B0, robot);
%      target.setAsJointTarget();
%      %ad=[zeros(4,3), [X1(i);Y1(i);Z1(i);0]];
%      ad=floor([zeros(4,3), [X(i);Y(i);Z(i);1]]);
%      qh=round([[m zeros(3,1) ] ; zeros(1,4)],3)+round(ad);
%      disp(qh);
%      target.setPose(round(qh));
%      prog.MoveJ(target);
%     
% end
for i=401:400:longq
     targetname = sprintf('Target%i',i);
     target = RDK.AddTarget(targetname, B0, robot);
     target.setAsJointTarget();
     H_target=transl(X(i),Y(i),Z(i))*rotx(pi/2)*rotz(pi/2);
     target.setPose(H_target);
     prog.MoveJ(target);
    
end

% define the pose (H transform)


% set the pose

    
%% 
robot.Connect();
% Run the program we just created
prog.RunProgram();

% Wait for the movement to finish
while robot.Busy()
    pause(1);
    fprintf('Waiting for the robot to finish...\n');
end

% Run the program once again
fprintf('Running the program again...\n');
prog.RunProgram();




