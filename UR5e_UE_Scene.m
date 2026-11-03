% Start from the colored robot.
robotUEScene = copy(robotUEColored);

% The bench is fixed to the robot's base coordinate system.
bench = rigidBody('workbenchVisual');
bench.Joint = rigidBodyJoint('workbenchFixed','fixed');
setFixedTransform(bench.Joint, eye(4));

topColor = [0.30 0.32 0.35];
legColor = [0.06 0.07 0.08];

% Tabletop: upper surface at Z = 0.
addVisual(bench, 'Box', [1.10 0.80 0.05], ...
    trvec2tform([0.30 0 -0.025]), ...
    'FaceColor', topColor);

% Four legs: extend down to Z = -0.80 m.
for x = [-0.20 0.80]
    for y = [-0.35 0.35]
        addVisual(bench, 'Box', [0.05 0.05 0.75], ...
            trvec2tform([x y -0.425]), ...
            'FaceColor', legColor);
    end
end

addBody(robotUEScene, bench, robotUEScene.BaseName);

% Confirm the fixed bench has not added a movable joint.
assert(numel(homeConfiguration(robotUEScene)) == 6);

% Confirm the tool position and orientation are unchanged.
Tbefore = getTransform(robotUEColored, qUE, 'glueTCP');
Tafter  = getTransform(robotUEScene, qUE, 'glueTCP');

assert(norm(Tbefore - Tafter, 'fro') < 1e-10);
disp('Bench added: six joints and unchanged TCP.');

% Preview.
figure;
show(robotUEScene, qUE, ...
    'Visuals','on','Collisions','off','Frames','off');
axis equal;
view(135,25);
title('UR5e and bench — layout checkpoint');

%% Scence with workpiece 
% Work on a copy of the robot-and-bench scene.
robotUEWorkcell = copy(robotUEScene);

% Create an oval cover mesh. Dimensions are in metres.
N = 96;
theta = (0:N-1)' * (2*pi/N);

% Each row: [X radius, Y radius, height].
% Follow the surface from the outside bottom to the inside floor.
profile = [
    0.128  0.078  0.000
    0.128  0.078  0.035
    0.150  0.100  0.035
    0.150  0.100  0.045
    0.120  0.070  0.045
    0.120  0.070  0.008
];

V = zeros(6*N + 2, 3);

for k = 1:6
    rows = (k-1)*N + (1:N);
    V(rows,:) = [
        profile(k,1)*cos(theta), ...
        profile(k,2)*sin(theta), ...
        profile(k,3)*ones(N,1)
    ];
end

% Join successive rings with triangles.
F = zeros(12*N,3);
row = 0;

for k = 1:5
    for i = 1:N
        j = mod(i,N) + 1;

        a = (k-1)*N + i;
        b = (k-1)*N + j;
        c = k*N + i;
        d = k*N + j;

        F(row+1,:) = [a b d];
        F(row+2,:) = [a d c];
        row = row + 2;
    end
end

% Close the underside and the cavity floor.
bottomCentre = 6*N + 1;
floorCentre  = 6*N + 2;
V(bottomCentre,:) = [0 0 0];
V(floorCentre,:)  = [0 0 0.008];

for i = 1:N
    j = mod(i,N) + 1;

    F(row+1,:) = [bottomCentre j i];
    F(row+2,:) = [floorCentre 5*N+i 5*N+j];
    row = row + 2;
end

coverMeshFile = fullfile(pwd,'gearbox_cover_visual.stl');
stlwrite(triangulation(F,V), coverMeshFile);

% Fix the cover to the scene's base coordinate system.
cover = rigidBody('gearboxCoverVisual');
cover.Joint = rigidBodyJoint('gearboxCoverFixed','fixed');

setFixedTransform(cover.Joint, ...
    trvec2tform([0.45 0 0]));

addVisual(cover,'Mesh',coverMeshFile, ...
    'FaceColor',[0.58 0.60 0.63]);

addBody(robotUEWorkcell,cover,robotUEWorkcell.BaseName);

% Check that the robot configuration is unchanged.
assert(numel(homeConfiguration(robotUEWorkcell)) == 6);

Tbefore = getTransform(robotUEScene,qUE,'glueTCP');
Tafter  = getTransform(robotUEWorkcell,qUE,'glueTCP');
assert(norm(Tbefore-Tafter,'fro') < 1e-10);

fprintf('Cover added. Rim height: 45 mm above tabletop.\n');
fprintf('Current TCP height above rim: %.1f mm\n', ...
    1000*(Tafter(3,4)-0.045));

figure;
show(robotUEWorkcell,qUE, ...
    'Visuals','on','Collisions','off','Frames','off');
axis equal;
view(135,25);
title('UR5e, bench and gearbox cover');
%% motion 
% Disconnect the Constant block and add a From Workspace block
% Regenerate the timed trajectory only if it is missing.
if ~exist('qTimed','var') || ~exist('time','var')
    UR5e_Animate_Glue
end

assert(exist('robotUEWorkcell','var') == 1, ...
    'Create the Unreal workcell with the bench and cover first.');

% Validate the trajectory dimensions and timestamps.
tMotion = double(time(:));
qMotion = double(qTimed);

assert(size(qMotion,2) == 6, ...
    'Expected six joint angles per sample.');
assert(size(qMotion,1) == numel(tMotion), ...
    'Time and joint-data lengths do not match.');
assert(all(isfinite(qMotion(:))) && all(isfinite(tMotion)));
assert(all(diff(tMotion) > 0), ...
    'Timestamps must be strictly increasing.');

tMotion = tMotion - tMotion(1);

% Hold at the starting hover pose for two seconds.
startDelay = 2;
jointMotionUE = [
    0,                         qMotion(1,:);
    tMotion + startDelay,      qMotion
];

% Initial pose and simulation duration.
qUE = qMotion(1,:);
stopTimeUE = ceil((tMotion(end) + startDelay + 2)/0.02)*0.02;

fprintf('Motion duration: %.2f s\n', tMotion(end));
fprintf('Simulation stop time: %.2f s\n', stopTimeUE);