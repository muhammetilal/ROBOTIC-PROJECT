function CommandActivateGripperROSGazebo(state)% This class is for internal use and may be removed in a future release
       if strcmp(state,'on') == 1
           % Activate gripper
            [gripAct,gripGoal] = rosactionclient('/husky_gen3/custom_gripper_controller/gripper_cmd');
            gripperCommand = rosmessage('control_msgs/GripperCommand');
            gripperCommand.Position = 0.1; % 0.1 fully closed, 0 fully open
            gripperCommand.MaxEffort = 500;
            gripGoal.Command = gripperCommand;            
            pause(1);
            
            % Send command
            sendGoal(gripAct,gripGoal); 
            disp('Gripper closed...');
       else
           % Deactivate gripper
            [gripAct,gripGoal] = rosactionclient('/husky_gen3/custom_gripper_controller/gripper_cmd');
            gripperCommand = rosmessage('control_msgs/GripperCommand');
            gripperCommand.Position = 0.0; % 0.1 fully closed, 0 fully open
            gripperCommand.MaxEffort = 500;
            gripGoal.Command = gripperCommand;            
            pause(1);
            
            % Send command
            sendGoal(gripAct,gripGoal);
            disp('Gripper open...');
       end
       
       pause(2);
end