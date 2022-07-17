clc;
clear;
clear all;
rosshutdown
ipaddress = "192.168.108.131";
rosinit(ipaddress,11311)

%% sending velocity command
rosshutdown
ipaddress = "192.168.108.131";
rosinit(ipaddress,11311)
velocity = 16;
robotCmd = rospublisher("/husky_gen3/husky_velocity_controller/cmd_vel","DataFormat","struct") ;
for i=1:5
velMsg = rosmessage(robotCmd);
velMsg.Linear.X = velocity;
send(robotCmd,velMsg)
pause(1)
% velMsg.Linear.X = 0;
% send(robotCmd,velMsg)
% pause(1)
velMsg.Linear.Y = velocity;
send(robotCmd,velMsg)
velMsg = rosmessage(robotCmd);
velMsg.Angular.Z = 0;
send(robotCmd,velMsg)
% pause(1)
% velMsg = rosmessage(robotCmd);
% velMsg.Angular.Z = 0;
% send(robotCmd,velMsg)


odomSub = rossubscriber("/husky_gen3/odometry/filtered","DataFormat","struct");
odomMsg = receive(odomSub,3);
pose = odomMsg.Pose.Pose;
x = pose.Position.X;
y = pose.Position.Y;
z = pose.Position.Z;



lidarSub = rossubscriber("/husky_gen3/scan","DataFormat","struct");
scanMsg(i) = receive(lidarSub,10);


end
figure
rosPlot(scanMsg(1))
hold on
rosPlot(scanMsg(2))
hold on
rosPlot(scanMsg(3))
hold on
rosPlot(scanMsg(4))
hold on

%% reading position of the robot
rostopic list


%% reading scan datas
% ipaddress = "192.168.108.128";
% rosinit(ipaddress,11311)



%% deneme
gzinit(ipaddress);
modelList = gzmodel("list") % modellerin ne olduğunu gösterir
[status,message] = gzmodel("set","husky_gen3","Position" ...
    ,[2 2 0.5],"SelfCollide","on") % bu istediğimz modelin pozisyonunu değiştiriyor


[position,selfcollide] = gzmodel("get","husky_gen3","Position","SelfCollide") %bu buldunduğu pozisyonu veriyor
linkList = gzlink("list","unit_box_2")
[status,message] = gzlink("set","unit_box_2","link","Mass",2,"Gravity","off")
[mass,gravity] = gzlink("get","unit_box_2","link","Mass","Gravity")
jointList = gzjoint("list","husky_gen3")%tüm linkeleri listeler
[status,message] = gzjoint("set","husky_gen3","joint_1","Axis","0","Damping",0.25)
damping = gzjoint("get","husky_gen3","joint_1","Axis0","Damping")
gzworld("reset")

[status,message] = gzjoint('set','husky_gen3','joint_5','Axis','0','Angle',0); %bu işe yarıyor ama düzeltmek lazım


%% manipulator
rosshutdown
ipaddress = "192.168.108.130";
rosinit(ipaddress,11311)
[gripAct,gripGoal] = rosactionclient('/husky_gen3/custom_gripper_controller/gripper_cmd');
gripperCommand = rosmessage('control_msgs/GripperCommand');
gripperCommand.Position = 0.0;  
gripGoal.Command = gripperCommand;
sendGoal(gripAct,gripGoal);