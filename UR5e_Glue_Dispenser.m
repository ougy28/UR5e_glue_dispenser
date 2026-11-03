%% UR5e glue dispenser concept - MATLAB R2025b
% Requires Robotics System Toolbox. Run this entire script.
% All dimensions are provisional, in METRES. No CAD files needed.
% Visual/kinematic concept only: no validated bolt pattern, mass/inertia,
% pressure/flow model, hose dynamics, or collision certification.
% tool0: mounting origin. Gun axis parallel to local +Z, offset in +X.
% Reloading the robot makes this script safe to rerun.
robot = loadrobot('universalUR5e','DataFormat','row');
qHome = homeConfiguration(robot);

%% Editable concept dimensions
offset = 0.070;             % Barrel centreline offset from tool0
rearZ = 0.025;              % Barrel rear relative to tool0
barrelLength = 0.180;
barrelRadius = 0.025;
capLength = 0.020;
nozzleLength = 0.065;
frontZ = rearZ + barrelLength;
nozzleStart = frontZ + capLength;
tipZ = nozzleStart + nozzleLength;
tcpOffset = [offset 0 tipZ]; % [0.070 0 0.290] metres

blue = [0.035 0.30 0.59];
darkBlue = [0.025 0.12 0.28];
metal = [0.70 0.73 0.77];
black = [0.12 0.13 0.15];
ivory = [0.95 0.93 0.84];

%% One fixed body containing the dispenser assembly
gun = rigidBody('glueGun');
mount = rigidBodyJoint('glueGunMount','fixed');
setFixedTransform(mount,eye(4));
gun.Joint = mount;

% Concept flange adapter and offset bracket (no machined bolt pattern).
addVisual(gun,'Cylinder',[0.0315 0.012], ...
    trvec2tform([0 0 0.006]),'FaceColor',metal);
addVisual(gun,'Box',[0.070 0.032 0.014], ...
    trvec2tform([0.035 0 0.019]),'FaceColor',metal);
addVisual(gun,'Box',[0.014 0.032 0.060], ...
    trvec2tform([0.035 0 0.056]),'FaceColor',metal);

% Blue cartridge housing, rear cap and front cap.
addVisual(gun,'Cylinder',[barrelRadius barrelLength], ...
    trvec2tform([offset 0 rearZ+barrelLength/2]),'FaceColor',blue);
addVisual(gun,'Cylinder',[0.028 0.018], ...
    trvec2tform([offset 0 rearZ+0.009]),'FaceColor',darkBlue);
addVisual(gun,'Cylinder',[0.028 capLength], ...
    trvec2tform([offset 0 frontZ+capLength/2]),'FaceColor',darkBlue);

% Generate portable STL meshes beside this script (units: metres).
meshFolder = fullfile(fileparts(mfilename('fullpath')),'dispenser_meshes');
if ~exist(meshFolder,'dir'), mkdir(meshFolder); end
ringFile = fullfile(meshFolder,'clamp.stl');
writeRevolvedMesh([0.025 -0.012; 0.029 -0.012; ...
    0.029 0.012; 0.025 0.012],ringFile);
addVisual(gun,'Mesh',ringFile, ...
    trvec2tform([offset 0 0.075]),'FaceColor',metal);
addVisual(gun,'Box',[0.018 0.032 0.024], ...
    trvec2tform([0.044 0 0.075]),'FaceColor',metal);

% Two visible clamp screw heads, aligned along local X.
for y = [-0.010 0.010]
    screwPose = trvec2tform([0.032 y 0.075]) * ...
        axang2tform([0 1 0 pi/2]);
    addVisual(gun,'Cylinder',[0.004 0.005],screwPose,'FaceColor',black);
end

% Tapered hollow nozzle, with a 1.6 mm outlet bore.
nozzleFile = fullfile(meshFolder,'nozzle.stl');
writeRevolvedMesh([0.0008 0; 0.010 0; ...
    0.002 nozzleLength; 0.0008 nozzleLength],nozzleFile);
addVisual(gun,'Mesh',nozzleFile, ...
    trvec2tform([offset 0 nozzleStart]),'FaceColor',ivory);

% Rear pneumatic fitting: decorative, no hose dynamics.
addVisual(gun,'Cylinder',[0.006 0.014], ...
    trvec2tform([offset 0 rearZ-0.007]),'FaceColor',metal);
addVisual(gun,'Cylinder',[0.007 0.008], ...
    trvec2tform([offset 0 rearZ-0.018]),'FaceColor',black);

addBody(robot,gun,'tool0');

%% TCP at the centre of the nozzle outlet
tcp = rigidBody('glueTCP');
tcpJoint = rigidBodyJoint('glueTCPJoint','fixed');
setFixedTransform(tcpJoint,trvec2tform(tcpOffset));
tcp.Joint = tcpJoint;
addBody(robot,tcp,'glueGun');

%% View 1: dispenser alone, in the mounting coordinate system
toolPreview = rigidBodyTree('DataFormat','row');
addBody(toolPreview,copy(gun),toolPreview.BaseName);
figure('Name','Dispenser close-up','Color','w');
ax1 = axes;
show(toolPreview,'Parent',ax1,'Frames','off');
hold(ax1,'on');
plot3(ax1,tcpOffset(1),tcpOffset(2),tcpOffset(3), ...
    'ro','MarkerFaceColor','r','MarkerSize',7);
text(ax1,offset,0,tipZ+0.012,'TCP / nozzle outlet','Color','r');
plot3(ax1,0,0,0,'ko','MarkerFaceColor','k');
text(ax1,0,0,-0.015,'Wrist mount');
axis(ax1,'equal'); grid(ax1,'on'); view(ax1,135,20);
xlabel(ax1,'X (m)'); ylabel(ax1,'Y (m)'); zlabel(ax1,'Z (m)');
title(ax1,'Dispenser concept - provisional dimensions');

%% View 2: assembly on UR5e
% Display posture only; not a planned or collision-validated motion.
qShow = deg2rad([0 -60 90 -120 -90 0]);
figure('Name','UR5e with dispenser','Color','w');
ax2 = axes;
show(robot,qShow,'Parent',ax2,'Frames','off');
hold(ax2,'on');
Ttip = getTransform(robot,qShow,'glueTCP');
p = Ttip(1:3,4);
plot3(ax2,p(1),p(2),p(3),'ro','MarkerFaceColor','r','MarkerSize',7);
text(ax2,p(1),p(2),p(3),'  TCP','Color','r');
axis(ax2,'equal'); grid(ax2,'on'); view(ax2,135,25);
xlabel(ax2,'X (m)'); ylabel(ax2,'Y (m)'); zlabel(ax2,'Z (m)');
title(ax2,'UR5e with offset dispenser');

%% Check fixed attachment in two robot configurations
for qTest = [qHome; qShow]'
    Trelative = getTransform(robot,qTest','glueTCP','tool0');
    assert(norm(Trelative-trvec2tform(tcpOffset),'fro') < 1e-9, ...
        'TCP attachment check failed.');
end
assert(numel(homeConfiguration(robot)) == 6, ...
    'The fixed tool should not add an actuated joint.');
fprintf('TCP offset relative to tool0: [%.3f %.3f %.3f] m\n',tcpOffset);
disp('Fixed TCP attachment checks passed; robot still has six joints.');
disp('Next checkpoint: inspect the close-up and wrist attachment.');
% robot, qHome, qShow and tcpOffset remain in the workspace for next steps.

function writeRevolvedMesh(profile,filename)
% Revolve a closed, counterclockwise [radius,z] profile about local Z.
% Positive radii avoid degenerate triangles at the axis.
    n = 96;
    m = size(profile,1);
    theta = (0:n-1)' * (2*pi/n);
    vertices = zeros(m*n,3);
    for j = 1:m
        ids = (j-1)*n + (1:n);
        vertices(ids,:) = [profile(j,1)*cos(theta), ...
            profile(j,1)*sin(theta), repmat(profile(j,2),n,1)];
    end
    faces = zeros(2*m*n,3);
    row = 0;
    for j = 1:m
        nextJ = mod(j,m)+1;
        for k = 1:n
            nextK = mod(k,n)+1;
            a = (j-1)*n+k;
            b = (j-1)*n+nextK;
            c = (nextJ-1)*n+nextK;
            d = (nextJ-1)*n+k;
            faces(row+1,:) = [a b c];
            faces(row+2,:) = [a c d];
            row = row+2;
        end
    end
    stlwrite(triangulation(faces,vertices),filename);
end
