function reachJointConfiguration(trajPub,trajCmd,jointWaypoints,jointWaypointTimes)
    
    trajCmd.JointNames = {'joint_1','joint_2','joint_3','joint_4', ... 
                      'joint_5','joint_6','joint_7'}; 
    jointWaypoints = wrapToPi(jointWaypoints);          
    trajCmd.Points = rosmessage('trajectory_msgs/JointTrajectoryPoint');
    trajCmd.Points.TimeFromStart = rosduration(jointWaypointTimes);
    trajCmd.Points.Positions = jointWaypoints;
    trajCmd.Points.Velocities = zeros(size(jointWaypoints));
    trajCmd.Points.Accelerations = zeros(size(jointWaypoints));
    trajCmd.Points.Effort = zeros(size(jointWaypoints));

    send(trajPub,trajCmd);
    pause(1);
    disp('Moving to Position');

    jSub = rossubscriber('/husky_gen3/joint_states');
    err = 1;
    while ( err > 1e-2)
        jMsg = receive(jSub);
        err = max(abs(jMsg.Velocity(2:8)));
    end
    disp('Position Reached');
end