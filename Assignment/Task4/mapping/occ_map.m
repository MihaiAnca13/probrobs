% load some data from playerstage robot
load('../data.mat');
odometry = data;
load('../ranges.mat');

prior = 0.5;                    % no knowledge of occupancy
free  = 0.45;			% probability of free space
occ   = 0.8;			% probability of occupancy

scale = 0.05;                   % size of each cell in m
Env_size_x = 10+4;                % size of environment (width) in m + sensor range both ways
Env_size_y = 6+4;                 % size of environment (heigth) in m + sensor range both ways
l_free = log(free/(1-free));    % log odds of free cell
l_occ  = log(occ/(1-occ));      % log odds of occupied cell
l_prior =log(prior/(1-prior));  % log-odds of prior

dimx = round(Env_size_x/scale);   % find number of cells in x-dimension
dimy = round(Env_size_y/scale);   % find number of cells in y-dimension

% initialise map to prior occupancy
map = prior*ones(dimx,dimy);

% offset used for odometry
% odom is between -4.8 and 4.8 | -2.9 and 2.9
% this is equivalent to -96 to 96 | -58 to 58
% 0-40 and 240-280 | 0-40 and 160-200 are sensor max ranges
% so the range must be offsetted by 96+40 = 136 | 58+40 = 98
offset = [136 98 0];

numits = length(odometry);

odometry(:,1)=odometry(:,1)/scale; % convert x to cell scale
odometry(:,2)=odometry(:,2)/scale; % convert y to cell scale

figure(3)
clf;

% step through data set, update map and display
for(x=1:numits)
    pose=odometry(x,:)+offset;
    map = add_robot(pose, map, scale, l_free, l_occ, l_prior, ranges(x,:));
    image([1 dimx], [1 dimy], repmat((1-map)', [1 1 3]));
    set(gca, 'ydir', 'normal')
    xlabel('x-axis');
    ylabel('y-axis');
    drawnow
    pause(0.001)
end





