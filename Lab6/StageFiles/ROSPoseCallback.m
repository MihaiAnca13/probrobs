function ROSPoseCallback(~, message)
    %exampleHelperROSPoseCallback Subscriber callback function for pose data    
    %   exampleHelperROSPoseCallback(~,MESSAGE) returns no arguments - it instead sets 
    %   global variables to the values of position and orientation that are
    %   received in the ROS message MESSAGE.
    %   
    %   See also ROSPublishAndSubscribeExample.
    
    %   Copyright 2014-2015 The MathWorks, Inc.
    
    % Declare global variables to store position and orientation
    %global pos
    %global orient
    global GL_robot_pose
    % Extract 2D position and theta orientation from the ROS message and assign the
    % data to the global variable.
    % rosmsg show geometry_msgs/Pose
    % Point Position  {X Y Z}
    % Quaternion Orientation {X Y Z W}
    % convert quaternion orientation to Euler angles (alpha, beta, gamma) 
    quatern = [message.Pose.Pose.Orientation.X message.Pose.Pose.Orientation.Y message.Pose.Pose.Orientation.Z message.Pose.Pose.Orientation.W];
    eula = quat2eul(quatern);
    % only pass back rotation around Z axis (gamma euler angle) that we
    % refer to as theta
    GL_robot_pose = [message.Pose.Pose.Position.X message.Pose.Pose.Position.Y eula(3)];
end