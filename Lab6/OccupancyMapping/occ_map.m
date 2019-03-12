prior = 0.5;                    % no knowledge of occupancy
free  = 0.3;			% probability of free space
occ   = 0.8;			% probability of occupancy

scale = 0.05;                   % size of each cell in m
Env_size = 10;                  % size of environment in m (assume square for now)
l_free = log(free/(1-free));    % log odds of free cell
l_occ  = log(occ/(1-occ));      % log odds of occupied cell
l_prior =log(prior/(1-prior));  % log-odds of prior

dimx = round(Env_size/scale);   % find number of cells in x-dimension
dimy = dimx;                    % same number of cells in y-dimension

% initialise map to prior occupancy
map = prior*ones(dimx,dimy);

%initial pose of robot in the map
start_pose = [100 100 0];

% load some data from playerstage robot
odometry = load('../data/odometry.dat');
ranges = load('../data/ranges.dat');

numits = length(odometry);

odometry(:,1)=odometry(:,1)/scale; % convert x to cell scale
odometry(:,2)=odometry(:,2)/scale; % convert y to cell scale

figure(3)
clf;

%step through data set, update map and display
for(x=1:numits)
    pose=odometry(x,:)+start_pose;
    map = add_robot(pose, map, scale, l_free, l_occ, l_prior, ranges(x,:));
    image([1 dimx], [1 dimy], repmat((1-map)', [1 1 3]));
    set(gca, 'ydir', 'normal')
    xlabel('x-axis');
    ylabel('y-axis');
    drawnow
    pause(0.01)
end





