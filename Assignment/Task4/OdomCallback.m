function OdomCallback(~, message)
    global robot_pose

    quatern = [message.Pose.Pose.Orientation.X message.Pose.Pose.Orientation.Y message.Pose.Pose.Orientation.Z message.Pose.Pose.Orientation.W];
    eula = quat2eul(quatern);
    
    robot_pose = [message.Pose.Pose.Position.X message.Pose.Pose.Position.Y eula(3)];
end