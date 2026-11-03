%% Stage: bench, concept gearbox cover, and one hover pose
% MATLAB R2025b + Robotics System Toolbox.
% Put this file beside the corrected UR5e_Glue_Dispenser.m and Run.
% Coordinates are metres relative to the robot base. Bench top is Z=0.
% Static layout only: no robot motion, glue flow, or collision validation.
projectFolder = fileparts(mfilename('fullpath'));
toolScript = fullfile(projectFolder,'UR5e_Glue_Dispenser.m');
assert(isfile(toolScript),'Place the corrected UR5e_Glue_Dispenser.m beside this file.');
run(toolScript);

%% Provisional workcell dimensions
coverCentre = [0.45 0];
rimZ = 0.045;
hoverGap = 0.060; % 60 mm approach clearance; NOT dispensing distance
% An open shallow cover: 300 x 200 mm outer flange, 45 mm high.
% Elliptical concept geometry, not a production gearbox CAD model.
theta = (0:95)'*2*pi/96;
outer = [coverCentre(1)+0.150*cos(theta), coverCentre(2)+0.100*sin(theta)];
inner = [coverCentre(1)+0.120*cos(theta), coverCentre(2)+0.070*sin(theta)];
wallOuter = [coverCentre(1)+0.128*cos(theta), coverCentre(2)+0.078*sin(theta)];

%% Target above the near-side sealing flange
hoverPosition = [coverCentre(1), coverCentre(2)-0.085, rimZ+hoverGap];
% Local tool +Z points down; local +X stays along base +X.
targetPose = trvec2tform(hoverPosition)*axang2tform([1 0 0 pi]);
ik = inverseKinematics('RigidBodyTree',robot);
weights = [1 1 1 1 1 1];
[qHover,ikInfo] = ik('glueTCP',targetPose,weights,qShow);
actualPose = getTransform(robot,qHover,'glueTCP');
positionError = norm(actualPose(1:3,4)-hoverPosition');
rotationError = actualPose(1:3,1:3)'*targetPose(1:3,1:3);
angleError = acos(max(-1,min(1,(trace(rotationError)-1)/2)));
fprintf('\nHover position error: %.4f mm\n',1000*positionError);
fprintf('Hover orientation error: %.4f degrees\n',rad2deg(angleError));
fprintf('IK solver status: %s\n',ikInfo.Status);
assert(positionError < 0.001 && angleError < deg2rad(0.5), ...
    'Hover pose did not meet tolerance. Send the printed errors before continuing.');

%% Display the achieved static pose
figure('Name','Stage - Workcell layout','Color','w');
ax = axes;
show(robot,qHover,'Parent',ax,'Frames','off');
hold(ax,'on');

% Bench: top surface is at Z=0; floor is at Z=-0.80.
drawBox(ax,[0.30 0 -0.025],[1.10 0.80 0.05],[0.55 0.58 0.61]);
for bx = [-0.20 0.80]
    for by = [-0.35 0.35]
        drawBox(ax,[bx by -0.425],[0.05 0.05 0.75],[0.22 0.25 0.28]);
    end
end

% Cover bottom, hollow walls, and wider horizontal sealing flange.
drawSolid(ax,wallOuter,0,0.008,[0.52 0.56 0.61]);
drawRing(ax,wallOuter,inner,0.008,0.035,[0.62 0.67 0.72]);
drawRing(ax,outer,inner,0.035,rimZ,[0.78 0.81 0.85]);

% Actual TCP (red) and the surface point directly below the requested pose.
pActual = actualPose(1:3,4);
plot3(ax,pActual(1),pActual(2),pActual(3),'ro', ...
    'MarkerFaceColor','r','MarkerSize',7);
plot3(ax,hoverPosition(1),hoverPosition(2),rimZ,'go', ...
    'MarkerFaceColor',[0.1 0.7 0.2],'MarkerSize',7);
plot3(ax,[hoverPosition(1) hoverPosition(1)], ...
    [hoverPosition(2) hoverPosition(2)],[rimZ hoverPosition(3)],'k--');
text(ax,hoverPosition(1)+0.02,hoverPosition(2),hoverPosition(3),'60 mm hover');
axis(ax,'equal'); grid(ax,'on'); view(ax,135,25);
xlim(ax,[-0.30 0.90]); ylim(ax,[-0.45 0.45]); zlim(ax,[-0.82 0.95]);
xlabel(ax,'X (m)'); ylabel(ax,'Y (m)'); zlabel(ax,'Z (m)');
title(ax,'Workcell concept - static hover pose, no collision check yet');
disp('Pose accuracy check passed. Inspect arm, tool, bench and cover clearances.');
disp('This proves one pose only, not a collision-free route or the full glue path.');
% Retained for the next stage: robot, qHover, coverCentre, rimZ, targetPose.

function drawBox(ax,centre,dims,colour)
    xy = centre(1:2)+[-1 -1;1 -1;1 1;-1 1].*(dims(1:2)/2);
    drawSolid(ax,xy,centre(3)-dims(3)/2,centre(3)+dims(3)/2,colour);
end

function drawSolid(ax,xy,z0,z1,colour)
    n=size(xy,1);
    patch(ax,'XData',xy(:,1),'YData',xy(:,2),'ZData',z1*ones(n,1), ...
        'FaceColor',colour,'EdgeColor','none');
    patch(ax,'XData',xy(:,1),'YData',xy(:,2),'ZData',z0*ones(n,1), ...
        'FaceColor',colour,'EdgeColor','none');
    for k=1:n
        j=mod(k,n)+1;
        quad(ax,[xy(k,:) z0;xy(j,:) z0;xy(j,:) z1;xy(k,:) z1],colour);
    end
end

function drawRing(ax,outer,inner,z0,z1,colour)
    n=size(outer,1);
    for k=1:n
        j=mod(k,n)+1;
        quad(ax,[outer(k,:) z1;outer(j,:) z1;inner(j,:) z1;inner(k,:) z1],colour);
        quad(ax,[outer(k,:) z0;inner(k,:) z0;inner(j,:) z0;outer(j,:) z0],colour);
        quad(ax,[outer(k,:) z0;outer(j,:) z0;outer(j,:) z1;outer(k,:) z1],colour);
        quad(ax,[inner(k,:) z0;inner(k,:) z1;inner(j,:) z1;inner(j,:) z0],colour*0.85);
    end
end

function quad(ax,v,colour)
    patch(ax,'Vertices',v,'Faces',[1 2 3 4],'FaceColor',colour,'EdgeColor','none');
end
