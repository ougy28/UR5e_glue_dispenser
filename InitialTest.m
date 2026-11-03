% Load the original UR5 robot.
robot = loadrobot('universalUR5e', 'DataFormat', 'row');

% Get its default joint configuration.
qHome = homeConfiguration(robot);

% Display the robot.
figure('Name', 'Stage 1 - UR5 model');
show(robot, qHome);
title('UR5 - Initial configuration');
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
axis equal;
grid on;
view(135, 25);

% Display the robot's links and joints.
showdetails(robot);

% Confirm the number of joint coordinates.
disp('Number of joint coordinates:');
disp(numel(qHome));

%% joint test
% Start from the home configuration.
qStart = qHome;

% Set the target configuration.
qTarget = qStart;
qTarget(1) = qStart(1) + deg2rad(30);

% Create a figure.
figure('Name', 'UR5e - First joint movement');
ax = axes;

show(robot, qStart, 'Parent', ax);
view(ax, 135, 25);
axis(ax, 'equal');
grid(ax, 'on');

% Keep the viewing area fixed during movement.
xlim(ax, [-1 1]);
ylim(ax, [-1 1]);
zlim(ax, [-0.5 1.2]);

% Gradually move from the start to the target.
numberOfSteps = 100;

for k = 1:numberOfSteps
    fraction = (k - 1) / (numberOfSteps - 1);

    q = qStart + fraction * (qTarget - qStart);

    show(robot, q, ...
        'Parent', ax, ...
        'PreservePlot', false, ...
        'FastUpdate', true);

    title(ax, 'UR5e - Rotating the base joint by 30 degrees');
    drawnow;
    pause(0.02);
end

disp('Final joint angles in degrees:');
disp(rad2deg(q));
