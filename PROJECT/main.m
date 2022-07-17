clc;
clear;
clear all;
%% connect to the address
rosshutdown
ipaddress = "192.168.108.131";
rosinit(ipaddress)
%% sorry for the unclear code comments and explanations. We could not use the time efficently
%% robot initial positions // this was used just for early position finding. Not used later

% inodomPub = rospublisher("/gazebo/set_model_state","DataFormat","struct");
% inodomMsg = rosmessage(inodomPub);
% 
% 
% 
% % pose = odomMsg.Pose.Pose;
% imodomMsg.Pose(16).Position.X=2.2941;
% inodomMsg.Pose(16).Position.Y=10.6;
% % z = odomMsg.Pose(16).Position.Z;
% 
% 
% % X = model_states.Pose(huskyIdx).Position.X;
% % Y = model_states.Pose(huskyIdx).Position.Y;
% % Xo = odomMsg.Pose(16).Orientation.X;
% % Yo = odomMsg.Pose(16).Orientation.Y;
% % Zo = odomMsg.Pose(16).Orientation.Z;
% % Wo = odomMsg.Pose(16).Orientation.W;
% % 
% % velMsg.Linear.X = 5;
% send(inodomPub,inodomMsg)


%% Initial joint calculations
physicsClient = rossvcclient('gazebo/unpause_physics');
physicsResp = call(physicsClient,'Timeout',3);
CommandActivateGripperROSGazebo('off');
[trajPub,trajCmd] = rospublisher('/husky_gen3/gen3_joint_trajectory_controller/command');

%jointWaypoints = [0 0 180 266 0 272 90]*pi/180; % bu jointleri ayarlıyor
jointWaypoints = [0 -10 180 275 0 277 100]*pi/180; % bu jointleri ayarlıyor

jointWaypointTimes = 1;

reachJointConfiguration(trajPub,trajCmd,jointWaypoints,jointWaypointTimes)
pause(1);

%% Load some mat files from MATHWORKS.com
  temp = load('helperYolo2DetectorMobileArm.mat');
robotCmd = rospublisher("/husky_gen3/husky_velocity_controller/cmd_vel","DataFormat","struct") ;
cameraSub = rossubscriber("/camera/color/image_raw", "DataFormat", "struct");
cameraDep = rossubscriber("/camera/depth/points", "DataFormat", "struct");

%% object detection using camera
for i=1:100
    

cameraMsg = receive(cameraSub,1);
    image =rosReadImage(cameraMsg);
%     figure
%     imshow(image);

    
    
  
    cameraDepMsg = receive(cameraDep,1);


    xyz = rosReadXYZ(cameraDepMsg);
points = reshape(xyz,480,270,3);
   
  DetectorModel = temp.detectorYolo2;   
     %  hfov = 1.211269;
     centerPixel = [round(size(image,1)/2), round(size(image,2)/2)];
     imshow(image);
     [bboxes,~,labels] = detect(DetectorModel,image);
     size_det=size(bboxes);
     if size_det(1,1)<1   
velMsg = rosmessage(robotCmd);
velMsg.Linear.X = 5;
send(robotCmd,velMsg)
     else
velMsg = rosmessage(robotCmd);
velMsg.Linear.X = 0
send(robotCmd,velMsg)
    break
     end
          pause(1);
end
     
     
%% Load another helmer.mat for calculations
load('exampleHelperKINOVAGen3GripperGazeboRRTScene.mat'); 
gen3 = robot;
show(robot);

% Retrive Camera properties
jSub = rossubscriber('/camera/color/camera_info');
jMsg = receive(jSub,1);

imageWidth = single(jMsg.Width);
imageHeight = single(jMsg.Height);
focalLength = single(jMsg.K(1));

% End Effector
endEffectorFrame = "gripper";

% Read current Joint Angles
jSub = rossubscriber('/husky_gen3/gen3_joint_trajectory_controller/state');
jMsg = receive(jSub,1);
CurrentRobotJConfig = wrapToPi(jMsg.Actual.Positions(1:7)');

% Get transformation to end effector
cameraTransf = getTransform(gen3, CurrentRobotJConfig, 'EndEffector_Link');
cameraZ = cameraTransf(3,4);
zDistance = cameraZ - 0.05590 + 0.10372; % - Width of the object + Width of the stool;
     
     
     %detecting function
     [objectResults,labeledImg] = detectObjects(bboxes, labels, points, image, cameraTransf);
     imshow(labeledImg)



% Approach detected object 
jSub = rossubscriber('/husky_gen3/gen3_joint_trajectory_controller/state');
jMsg = receive(jSub,1);
currentArmPose = wrapToPi(jMsg.Actual.Positions(1:7)');


taskActive = true;detectedObject=objectResults;
graspPoseLocal = computeGraspPose(detectedObject);
graspPose = graspPoseLocal;  % compute grasp pose based on the location of detected object

graspPose=graspPose(1:4,1:4);




%% mapping trials for bonus
 
 
 load("helperRecyclingWarehouseMap.mat");
% show(logicalMap)
% rosShow(logicalMap)
imshow(logicalMap)
 %%
     load('exampleHelperKINOVAGen3GripperGazeboRRTScene.mat'); 

gen3 = robot;

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

position = [X,Y,yaw]; 
%position=[0 0 yaw];

% X = model_states.Pose(huskyIdx).Position.X;
% Y = model_states.Pose(huskyIdx).Position.Y;
Xr = odomMsg.Pose(12).Orientation.X;
Yr = odomMsg.Pose(12).Orientation.Y;
Zr = odomMsg.Pose(12).Orientation.Z;
Wr = odomMsg.Pose(12).Orientation.W;

eulr = quat2eul([Wr,Xr,Yr,Zr]);
yawr = eulr(1);


Xr = odomMsg.Pose(12).Position.X;
Yr = odomMsg.Pose(12).Position.Y;
Zr = odomMsg.Pose(12).Position.Z;



r_position = [Xr,Yr,Zr]; %robot position % current robot position olarak kullan


jSub = rossubscriber('/camera/color/camera_info');
jMsg = receive(jSub,4);

imageWidth = single(jMsg.Width);
imageHeight = single(jMsg.Height);
focalLength = single(jMsg.K(1));

% End Effector
endEffectorFrame = "gripper";

% Read current Joint Angles
jSub = rossubscriber('/husky_gen3/gen3_joint_trajectory_controller/state');
jMsg = receive(jSub,10);
CurrentRobotJConfig = wrapToPi(jMsg.Actual.Positions(1:7)');

% Get transformation to end effector
cameraTransf = getTransform(gen3, CurrentRobotJConfig, 'EndEffector_Link');
cameraZ = cameraTransf(3,4);
zDistance = cameraZ - 0.05591 + 0.1034+0.26; % - Width of the object + Width of the stool;



% Read the object's position and orientation data.
lidarSub = rossubscriber("/husky_gen3/scan","DataFormat","struct");
scanMsg = receive(lidarSub,10);

[cart,angles] = rosReadCartesian(scanMsg);

[deger angle]=min(scanMsg.Ranges);
angle=angle/2;
centerPixel = [imageHeight/2, imageWidth/2];
centerBox = [imageHeight-r_position(1) imageWidth-r_position(2)];
centerBoxwrtCenterPixel = centerBox - centerPixel; % in pixels
worldCenterBoxwrtCenterPixel = (zDistance/focalLength)*centerBoxwrtCenterPixel; % in meters
actualCameraTransf = cameraTransf * trvec2tform([0, 0.041, 0.0]);
actualpartXY = actualCameraTransf(1:2,4)' + worldCenterBoxwrtCenterPixel;
part_centerPoint = [actualpartXY(1)-0.17,actualpartXY(2)-0.65,zDistance+0.50];
% part_centerPoint = [actualpartXY(1)+0.5,actualpartXY(2)-0.9,zDistance+0.45];

%r_position(3)=zDistance;
%part_centerPoint=r_position;


ik = inverseKinematics('RigidBodyTree',gen3);
ik.SolverParameters.AllowRandomRestart = 0;

% Calculate Final Pose to grasp the object
GraspPose = trvec2tform(part_centerPoint + [-0.008 0 -zDistance-0.055])*eul2tform([deg2rad(0) pi 0]);



% Calculate first position Grasppose - 0.15 in Z axes
taskFinal = double(GraspPose*trvec2tform([0,0,-0.15]));


jointWaypointTimes = 1;
jointWaypoints1 = ik(endEffectorFrame,taskFinal,[1 1 1 1 1 1],CurrentRobotJConfig);

% Go to Position 1
reachJointConfiguration(trajPub,trajCmd,jointWaypoints1,jointWaypointTimes);
%% finding object position
jointWaypointsnew = [0 0 180 266 0 272 90]*pi/180;

jointWaypointTimes = 1;

reachJointConfiguration(trajPub,trajCmd,jointWaypointsnew,jointWaypointTimes);


%% location object
 load('exampleHelperKINOVAGen3GripperGazeboRRTScene.mat'); 
 gen3 = robot;




%taskFinal=r_position;
weights=[1 1 1 1 1 1];


CommandActivateGripperROSGazebo('off');
pause(1);
   


%%
% Read current Joint Angles
jSub = rossubscriber('/husky_gen3/gen3_joint_trajectory_controller/state');
jMsg = receive(jSub,10);
CurrentRobotJConfig = wrapToPi(jMsg.Actual.Positions(1:7)');

jointWaypoints2 = ik(endEffectorFrame,double(GraspPose),[1 1 1 1 1 1],CurrentRobotJConfig);
% jointWaypoints1=jointWaypoints1*pi/180;
jointWaypointTimes = 1;
reachJointConfiguration(trajPub,trajCmd,jointWaypoints2,jointWaypointTimes);
pause(1);
% CommandActivateGripperROSGazebo('off');
%%
CommandActivateGripperROSGazebo('on');
pause(1);
%% belt
jointWaypointTimes = 1;
reachJointConfiguration(trajPub,trajCmd,jointWaypoints,jointWaypointTimes);    


%% moving to conteiner

lidarSub = rossubscriber("/husky_gen3/scan","DataFormat","struct");
robotCmd = rospublisher("/husky_gen3/husky_velocity_controller/cmd_vel","DataFormat","struct") ;
velMsg = rosmessage(robotCmd);
for k=1:100
 
scanMsg = receive(lidarSub,1);
if scanMsg.Ranges(360)>0.60

velMsg.Linear.X = 5
send(robotCmd,velMsg)
else
velMsg.Linear.X = 0
send(robotCmd,velMsg)
break
pause(1)
end
end

jointWaypoints_end = [20 -10 180 275 0 277 100]*pi/180; % bu jointleri ayarlıyor
jointWaypointTimes = 1;
reachJointConfiguration(trajPub,trajCmd,jointWaypoints_end,jointWaypointTimes);    