%% Prepare the UR5e glue demonstration for Simulink R2025b.
% Run this entire script from the Command Window. Read README.md for the
% one-time Simulink model setup. This does not create an SLX model.
% The existing validation scripts and MATLAB animation run first.
ur5eProjectRoot = fileparts(mfilename('fullpath'));
addpath(ur5eProjectRoot);
run(fullfile(ur5eProjectRoot,'UR5e_Animate_Glue.m'));
assert(collisionPassed,'Collision checkpoint must pass before preparation.');

%% Build a visual workcell using the original robot as the color source.
robotUE = copy(robot);
robotUE.DataFormat = 'row';
robotUEColored = UR5e_Apply_Colors(robotUE);
robotUEScene = copy(robotUEColored);
benchBody = rigidBody('workbenchVisual');
benchBody.Joint = rigidBodyJoint('workbenchFixed','fixed');
setFixedTransform(benchBody.Joint,eye(4));
addVisual(benchBody,'Box',[1.10 .80 .05],trvec2tform([.30 0 -.025]), ...
    'FaceColor',[.30 .32 .35]);
for benchX = [-.20 .80]
    for benchY = [-.35 .35]
        addVisual(benchBody,'Box',[.05 .05 .75], ...
            trvec2tform([benchX benchY -.425]),'FaceColor',[.06 .07 .08]);
    end
end
addBody(robotUEScene,benchBody,robotUEScene.BaseName);
robotUEWorkcell = copy(robotUEScene);
coverFile = fullfile(ur5eProjectRoot,'gearbox_cover_visual.stl');
writeCoverMesh(coverFile);
coverBody = rigidBody('gearboxCoverVisual');
coverBody.Joint = rigidBodyJoint('gearboxCoverFixed','fixed');
setFixedTransform(coverBody.Joint,trvec2tform([.45 0 0]));
addVisual(coverBody,'Mesh',coverFile,eye(4),'FaceColor',[.58 .60 .63]);
addBody(robotUEWorkcell,coverBody,robotUEWorkcell.BaseName);
assert(numel(homeConfiguration(robotUEWorkcell))==6);
assert(norm(getTransform(robotUEWorkcell,qHover,'glueTCP')- ...
    getTransform(robot,qHover,'glueTCP'),'fro')<1e-10);

%% From Workspace matrix: time (seconds), followed by six joint radians.
tMotion = double(time(:));
qMotion = double(qTimed);
assert(size(qMotion,2)==6 && size(qMotion,1)==numel(tMotion));
assert(all(isfinite(qMotion),'all') && all(isfinite(tMotion)));
assert(all(diff(tMotion)>0));
tMotion = tMotion-tMotion(1);
startDelay = 2;
jointMotionUE = [0 qMotion(1,:); tMotion+startDelay qMotion];
qUE = qMotion(1,:);
stopTimeUE = ceil((jointMotionUE(end,1)+2)/.02)*.02;

%% Generate separate bead instance input and the actor initialization data.
glueInstancesUE = UR5e_Glue_Bead('prepare', ...
    robotUEWorkcell,jointMotionUE,phaseDurations);
fprintf('\nWorkspace ready. Stop time: %.2f s\n',stopTimeUE);
disp('Robot tree: robotUEWorkcell; robot Config input: jointMotionUE');
disp('Bead Instance input: glueInstancesUE via int32 conversion');
disp('Actor Valid is a status output; connect it to a Terminator.');
disp('Follow README.md to create or open UR5e_Unreal_Checkpoint.slx.');

function writeCoverMesh(filename)
% Closed stepped oval shell. All dimensions are metres.
n=96; theta=(0:n-1)'*2*pi/n;
profile=[.128 .078 0; .128 .078 .035; .150 .100 .035; ...
         .150 .100 .045; .120 .070 .045; .120 .070 .008];
vertices=zeros(6*n+2,3); faces=zeros(12*n,3); row=0;
for ring=1:6
    ids=(ring-1)*n+(1:n);
    vertices(ids,:)=[profile(ring,1)*cos(theta), ...
        profile(ring,2)*sin(theta),profile(ring,3)*ones(n,1)];
end
for ring=1:5
    for i=1:n
        j=mod(i,n)+1;
        a=(ring-1)*n+i; b=(ring-1)*n+j;
        c=ring*n+i; d=ring*n+j;
        faces(row+(1:2),:)=[a b d; a d c]; row=row+2;
    end
end
bottom=6*n+1; floorPoint=6*n+2;
vertices(bottom,:)=[0 0 0]; vertices(floorPoint,:)=[0 0 .008];
for i=1:n
    j=mod(i,n)+1;
    faces(row+(1:2),:)=[bottom j i; floorPoint 5*n+i 5*n+j];
    row=row+2;
end
stlwrite(triangulation(faces,vertices),filename);
end
