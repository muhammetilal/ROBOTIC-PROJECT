clc;
clear;
clear all;
rosshutdown
ipaddress = "192.168.108.131";
rosinit(ipaddress,11311)
%% joint ayarlama
physicsClient = rossvcclient('gazebo/unpause_physics');
physicsResp = call(physicsClient,'Timeout',3);
CommandActivateGripperROSGazebo('off');
[trajPub,trajCmd] = rospublisher('/husky_gen3/gen3_joint_trajectory_controller/command');

jointWaypoints = [0 0 180 266 0 272 90]*pi/180; % bu jointleri ayarlıyor

jointWaypointTimes = 5;

reachJointConfiguration(trajPub,trajCmd,jointWaypoints,jointWaypointTimes);
%% robot position
odomSub = rossubscriber("/gazebo/model_states","DataFormat","struct");
odomMsg = receive(odomSub,3);
% pose = odomMsg.Pose.Pose;
X = odomMsg.Pose(16).Position.X;
Y = odomMsg.Pose(16).Position.Y;
z = odomMsg.Pose(16).Position.Z;


% X = model_states.Pose(huskyIdx).Position.X;
% Y = model_states.Pose(huskyIdx).Position.Y;
Xo = odomMsg.Pose(16).Orientation.X;
Yo = odomMsg.Pose(16).Orientation.Y;
Zo = odomMsg.Pose(16).Orientation.Z;
Wo = odomMsg.Pose(16).Orientation.W;

eul = quat2eul([Wo,Xo,Yo,Zo]);
yaw = eul(1);

position = [X,Y,yaw]; %robot position % current robot position olarak kullan
%position=[0 0 yaw];

%linkleri gzlinkten yapabilirsin




%% finding object position



%% location object
load('helperKINOVAGen3MobileArmPickPlace.mat'); 

gen3 = robot;

% Retrive Camera properties
jSub = rossubscriber('/camera/color/camera_info');
jMsg = receive(jSub,4);

imageWidth = single(jMsg.Width);
imageHeight = single(jMsg.Height);
focalLength = single(jMsg.K(2));

% End Effector
endEffectorFrame = "gripper";

% Read current Joint Angles
jSub = rossubscriber('/husky_gen3/gen3_joint_trajectory_controller/state');
jMsg = receive(jSub,10);
CurrentRobotJConfig = wrapToPi(jMsg.Actual.Positions(1:7)');

% Get transformation to end effector
cameraTransf = getTransform(gen3, CurrentRobotJConfig, 'EndEffector_Link');
cameraZ = cameraTransf(3,4);
zDistance = cameraZ - 0.05590 + 0.10372; % - Width of the object + Width of the stool;



% Read the object's position and orientation data.
lidarSub = rossubscriber("/husky_gen3/scan","DataFormat","struct");
scanMsg = receive(lidarSub,10);
%rosPlot(scanMsg)

gzinit(ipaddress);
modelList = gzmodel("list")

[r_position,g_selfcollide] = gzmodel("get","Green Can","Position","SelfCollide")
[g_position,r_selfcollide] = gzmodel("get","Red Bottle","Position","SelfCollide")
[deger angle]=min(scanMsg.Ranges);
angle=angle/2;
centerPixel = [imageHeight/2, imageWidth/2];
centerBox = [imageHeight-r_position(1) imageWidth-r_position(2)];
centerBoxwrtCenterPixel = centerBox - centerPixel; % in pixels
worldCenterBoxwrtCenterPixel = (zDistance/focalLength)*centerBoxwrtCenterPixel; % in meters
actualCameraTransf = cameraTransf * trvec2tform([0, 0.041, 0.0]);
actualpartXY = actualCameraTransf(1:2,4)' + worldCenterBoxwrtCenterPixel;
part_centerPoint = [actualpartXY(1),actualpartXY(2),zDistance];





ik = inverseKinematics('RigidBodyTree',gen3);
ik.SolverParameters.AllowRandomRestart = false;
ik.SolverParameters.EnforceJointLimits = false;

% Calculate Final Pose to grasp the object
GraspPose = trvec2tform(part_centerPoint + [-0.008 0 -zDistance-0.055])*eul2tform([deg2rad(angle) pi 0]);

% Calculate first position Grasppose - 0.15 in Z axes
taskFinal = double(GraspPose*trvec2tform([0,0,-0.15]));

jointWaypoints1 = ik(endEffectorFrame,taskFinal,[1 1 1 1 1 1],CurrentRobotJConfig);
% jointWaypoints1=jointWaypoints1*pi/180;
jointWaypointTimes = 5;

% Go to Position 1
reachJointConfiguration(trajPub,trajCmd,jointWaypoints1,jointWaypointTimes);


%%
% Read current Joint Angles
jSub = rossubscriber('/husky_gen3/gen3_joint_trajectory_controller/state');
jMsg = receive(jSub,10);
CurrentRobotJConfig = wrapToPi(jMsg.Actual.Positions(1:7)');

jointWaypoints2 = ik(endEffectorFrame,double(GraspPose),[1 1 1 1 1 1],CurrentRobotJConfig);
% jointWaypoints1=jointWaypoints1*pi/180;
jointWaypointTimes = 5;
reachJointConfiguration(trajPub,trajCmd,jointWaypoints2,jointWaypointTimes);


%%
CommandActivateGripperROSGazebo('on');

%% belt
jointWaypointTimes = 5;
reachJointConfiguration(trajPub,trajCmd,jointWaypoints1,jointWaypointTimes);    

% %beltCmd = rossubscriber("/gazebo/link_states");
% beltCmd = rospublisher("/gazebo/link_states") ;
% %beltMsg = receive(beltCmd,10); % 21 21
% belt_velocity=5;
% beltMsg = rosmessage(beltCmd);
% beltMsg.Twist(22,1).Linear.X = belt_velocity;
% send(beltCmd,beltMsg)
% send(beltCmd,beltMsg)
