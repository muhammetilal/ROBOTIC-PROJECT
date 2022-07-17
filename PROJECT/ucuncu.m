clc
clear
clear all

% Load the robot model from a .mat file as a rigidBodyTree object
load('exampleHelperKINOVAGen3GripperCollManip.mat');
gen3 = robot;
gen3.DataFormat = 'column';

% Define initial pose by specifying the joint values (in radians) at ['Shoulder_Link', 'HalfArm1_Link',	'HalfArm2_Link', 'ForeArm_Link', 'Wrist1_Link',	'Wrist2_Link','Bracelet_Link']
q_home = [0 0.35 pi -1.20 0 -pi/2 pi/2]'; 

% Get the end effector transform
endEffectorTransf  = getTransform(gen3,q_home,'EndEffector_Link');
