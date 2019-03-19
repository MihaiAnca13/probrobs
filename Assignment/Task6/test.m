%%
vel.Linear.X = 1;
vel.Angular.Z = 0;
last_pose = [0 0 3.14];
xs = [];
ys = [];
i = 0;
while(i<20)
    last_pose = vel_model(vel, last_pose);
    xs = [xs last_pose(1)];
    ys = [ys last_pose(2)];
    i = i+1;
end
i = 0;
vel.Linear.X = 0;
vel.Angular.Z = 1;
while(i<20)
    last_pose = vel_model(vel, last_pose);
    xs = [xs last_pose(1)];
    ys = [ys last_pose(2)];
    i = i+1;
end
i = 0;
vel.Linear.X = 1;
vel.Angular.Z = 0;
while(i<20)
    last_pose = vel_model(vel, last_pose);
    xs = [xs last_pose(1)];
    ys = [ys last_pose(2)];
    i = i+1;
end

%%
figure(2);
plot(xs, ys, 'b');
axis([-5 5 -2.5 2.5]);