clc
clear
clear all
rosshutdown
ipaddress = "192.168.108.131";
rosinit(ipaddress,11311)
physicsClient = rossvcclient('gazebo/unpause_physics');
physicsResp = call(physicsClient,'Timeout',3);
CommandActivateGripperROSGazebo('off');
[trajPub,trajCmd] = rospublisher('/husky_gen3/gen3_joint_trajectory_controller/command');

jointWaypoints = [0 0 180 266 0 272 90]*pi/180; % bu jointleri ayarlıyor

jointWaypointTimes = 5;

reachJointConfiguration(trajPub,trajCmd,jointWaypoints,jointWaypointTimes);



% [gripAct,gripGoal] = rosactionclient('/husky_gen3/custom_gripper_controller/gripper_cmd');
% gripperCommand = rosmessage('control_msgs/GripperCommand');
% gripperCommand.Position = 0.0;  
% gripGoal.Command = gripperCommand;
% sendGoal(gripAct,gripGoal);
% rosaction list

%% Load Rigid Body Tree Model of Kinova Gen3 with Gripper 
% husky = rossubscriber('/gazebo/model_states');
% husky_m = receive(husky,10);
% husky_m = rigidBodyTree("MaxNumBodies",N,"DataFormat",dataFormat)

%% Retrive Camera properties
jSub = rossubscriber('/camera/color/camera_info');
jMsg = receive(jSub,10);

imageWidth = single(jMsg.Width);
imageHeight = single(jMsg.Height);
focalLength = single(jMsg.K(1));

%% End Effector
endEffectorFrame = "gripper";

%% Read current Joint Angles
jSub = rossubscriber('/husky_gen3/joint_states');
jMsg = receive(jSub,10);
CurrentRobotJConfig = wrapToPi(jMsg.Position(2:8)');



%% creating robot
% load exampleRobots.mat
gen3=load('helperKINOVAGen3MobileArmPickPlace.mat');
%  gen3 = rossubscriber('/diagnostics');
%  husky_gen3 = receive(gen3,10);
%  gen3=husky_gen3.Pose(1,1).Position;


%% Get transformation to end effector
cameraTransf = getTransform(gen3, CurrentRobotJConfig, 'EndEffector_Link');
cameraZ = cameraTransf(3,4);
zDistance = cameraZ - 0.05590 + 0.10372; % - Width of the object + Width of the stool;


%% reading scan data
scan = rossubscriber("/husky_gen3/scan","DataFormat","struct");
scanMsg= receive(scan,10);


%% Get center and rotation of the object
tmpsub = rossubscriber('/opencv/centerx');
tmp = receive(tmpsub,10);
result(1) = tmp.Data;

tmpsub = rossubscriber('/opencv/centery');
tmp = receive(tmpsub,10);
result(2) = tmp.Data;

tmpsub = rossubscriber('/opencv/angle');
tmp = receive(tmpsub,10);
result(3) = tmp.Data;


centerPixel = [imageHeight/2, imageWidth/2];
centerBox = [imageHeight-result(1) imageWidth-result(2)];
centerBoxwrtCenterPixel = centerBox - centerPixel; % in pixels
worldCenterBoxwrtCenterPixel = (zDistance/focalLength)*centerBoxwrtCenterPixel; % in meters
actualCameraTransf = cameraTransf * trvec2tform([0, 0.041, 0.0]);
actualpartXY = actualCameraTransf(1:2,4)' + worldCenterBoxwrtCenterPixel;
part_centerPoint = [actualpartXY(1),actualpartXY(2),zDistance];

ik = inverseKinematics('RigidBodyTree',gen3);
ik.SolverParameters.AllowRandomRestart = 0;

% Calculate Final Pose to grasp the object
GraspPose = trvec2tform(part_centerPoint + [-0.008 0 -zDistance-0.055])*eul2tform([deg2rad(result(3)) pi 0]);

% Calculate first position Grasppose - 0.15 in Z axes
taskFinal = double(GraspPose*trvec2tform([0,0,-0.15]));

jointWaypoints1 = ik(endEffectorFrame,taskFinal,[1 1 1 1 1 1],CurrentRobotJConfig);
jointWaypointTimes = 5;

%% Go to Position 1
reachJointConfiguration(trajPub,trajCmd,jointWaypoints1,jointWaypointTimes);

% Read current Joint Angles
jSub = rossubscriber('/husky_gen3/joint_states');
jMsg = receive(jSub,10);
CurrentRobotJConfig = wrapToPi(jMsg.Position(2:8)');

jointWaypoints2 = ik(endEffectorFrame,double(GraspPose),[1 1 1 1 1 1],CurrentRobotJConfig);

jointWaypointTimes = 3;
reachJointConfiguration(trajPub,trajCmd,jointWaypoints2,jointWaypointTimes);
pause(1);
%% Close the gripper
CommandActivateGripperROSGazebo('on');
pause(3);
jointWaypointTimes = 5;
reachJointConfiguration(trajPub,trajCmd,jointWaypoints1,jointWaypointTimes);
rosshutdown;
clear;




