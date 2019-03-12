%%
% Transform sensor into Global reference and find maximum extent of perceptual
% field in cartesian coordinates
%%
function [index sensor_pose_G] = get_perceptual_field_sq(sensor_pose_R, robot_pose, fov, z_max, scale)

z_max_cells = round(z_max/scale);                           % equivalent z_max in cells

x=[];
y=[];
index=[];
sensor_pose_G = [];

%% transform sensor pose from robot to global reference frame
R=[cos(robot_pose(3)) -sin(robot_pose(3)); 
   sin(robot_pose(3)) cos(robot_pose(3))];                  % form Rotation matrix
sensor_pose_G(1:2) = (R * sensor_pose_R(1:2))';             % rotate sensor position to current robot pose in world frame
sensor_pose_G(1:2) = sensor_pose_G(1:2) + robot_pose(1:2);  % translate to robot location in world
sensor_pose_G(3) = sensor_pose_R(3)+robot_pose(3);          % add robot rotation

%% Find extent of perceptual field of sensor
index(1) = round(sensor_pose_G(1) - z_max_cells);               %  } min..
index(2) = round(sensor_pose_G(1) + z_max_cells);               %  } max..extent of perceptual field in x-axis

index(3) = round(sensor_pose_G(2) - z_max_cells);               %  } min..
index(4) = round(sensor_pose_G(2) + z_max_cells);               %  } max..extent of perceptual field in y-axis
