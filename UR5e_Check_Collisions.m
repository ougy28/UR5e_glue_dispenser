%% Stage: sampled collision check, MATLAB R2025b
% Place beside all three previous scripts. Run this entire file.
% This is a conservative geometry check, NOT continuous collision proof.
% No motion is sent to hardware. No speed or singularity checks here.
collisionProject = fileparts(mfilename('fullpath'));
run(fullfile(collisionProject,'UR5e_Check_Glue_Path.m'));
robotCollision = copy(robot);
gunCollision = getBody(robotCollision,'glueGun');
clearCollision(gunCollision);
% Explicit specifications shared by the full model and diagnostic.
% Columns: primitive type, dimensions, local transform.
toolSpecs = {
 'cylinder', [0.0315 0.012], trvec2tform([0 0 0.006]);
 'box', [0.070 0.032 0.014], trvec2tform([0.035 0 0.019]);
 'box', [0.014 0.032 0.060], trvec2tform([0.035 0 0.056]);
 'cylinder', [barrelRadius barrelLength], trvec2tform([offset 0 rearZ+barrelLength/2]);
 'cylinder', [0.028 0.018], trvec2tform([offset 0 rearZ+0.009]);
 'cylinder', [0.028 capLength], trvec2tform([offset 0 frontZ+capLength/2]);
 'cylinder', [0.029 0.024], trvec2tform([offset 0 0.075]);
 'box', [0.018 0.032 0.024], trvec2tform([0.044 0 0.075]);
 'cylinder', [0.004 0.005], trvec2tform([0.032 -0.010 0.075])*axang2tform([0 1 0 pi/2]);
 'cylinder', [0.004 0.005], trvec2tform([0.032 0.010 0.075])*axang2tform([0 1 0 pi/2]);
 'cylinder', [0.010 nozzleLength], trvec2tform([offset 0 nozzleStart+nozzleLength/2]);
 'cylinder', [0.006 0.014], trvec2tform([offset 0 rearZ-0.007]);
 'cylinder', [0.007 0.008], trvec2tform([offset 0 rearZ-0.018])
};
for component = 2:size(toolSpecs,1)
    addCollision(gunCollision,toolSpecs{component,1}, ...
        toolSpecs{component,2},toolSpecs{component,3});
end
replaceBody(robotCollision,'glueGun',gunCollision);

% Separate the adapter so only its intended mounting interfaces are exempt.
% Geometry stays in the same place; the TCP and robot motion do not change.
chainBody = getBody(robotCollision,'glueGun');
while ~strcmp(chainBody.Name,'wrist_3_link')
    assert(~strcmp(chainBody.Name,robotCollision.BaseName) && ...
        strcmp(chainBody.Joint.Type,'fixed'), ...
        'Expected a fixed attachment chain from glueGun to wrist_3_link.');
    chainBody = chainBody.Parent;
end
Tadapter = getTransform(robotCollision,qPath(1,:),'glueGun','wrist_3_link');
adapterBody = rigidBody('flangeAdapterCollision');
adapterJoint = rigidBodyJoint('flangeAdapterCollisionJoint','fixed');
setFixedTransform(adapterJoint,Tadapter);
adapterBody.Joint = adapterJoint;
addCollision(adapterBody,toolSpecs{1,1},toolSpecs{1,2},toolSpecs{1,3});
addBody(robotCollision,adapterBody,'wrist_3_link');
% Preserve standard direct parent-child exclusions and add adapter/bracket.
skipPairs = cell(robotCollision.NumBodies+1,2);
for b = 1:robotCollision.NumBodies
    skipPairs(b,:) = {robotCollision.Bodies{b}.Parent.Name,robotCollision.Bodies{b}.Name};
end
skipPairs(end,:) = {'flangeAdapterCollision','glueGun'};
assert(~any((strcmp(skipPairs(:,1),'wrist_3_link') & strcmp(skipPairs(:,2),'glueGun')) | ...
    (strcmp(skipPairs(:,2),'wrist_3_link') & strcmp(skipPairs(:,1),'glueGun'))), ...
    'The dispenser must remain checked against the wrist.');
assert(numel(homeConfiguration(robotCollision))==6,'Adapter must not add a moving joint.');
for k = [1 size(qPath,1)]
    originalTCP = getTransform(robot,qPath(k,:),'glueTCP');
    checkedTCP = getTransform(robotCollision,qPath(k,:),'glueTCP');
    assert(norm(originalTCP-checkedTCP,'fro')<1e-10,'TCP changed unexpectedly.');
end


% Confirm every actuated link has collision geometry from the robot library.
for b = 1:robotCollision.NumBodies
    body = robotCollision.Bodies{b};
    if ~strcmp(body.Joint.Type,'fixed') && isempty(body.Collisions)
        error('Missing collision geometry on %s. Stop and report.',body.Name);
    end
end

% World objects: tabletop, four legs, and a conservative solid cover envelope.
worldObjects = {makeBox([1.10 0.80 0.05],[0.30 0 -0.025])};
worldNames = {'tabletop'};
for bx = [-0.20 0.80]
    for by = [-0.35 0.35]
        worldObjects{end+1} = makeBox([0.05 0.05 0.75],[bx by -0.425]);
        worldNames{end+1} = sprintf('bench leg x=%.2f y=%.2f',bx,by);
    end
end
worldObjects{end+1} = makeBox([0.300 0.200 rimZ], ...
    [coverCentre rimZ/2]);
worldNames{end+1} = 'cover solid bounding box';

% Only fixed-to-base bodies are allowed to contact tabletop at the mounting.
% They remain checked against other objects and against robot links.
bodyNames = [robotCollision.BodyNames {robotCollision.BaseName}];
B = numel(bodyNames);
staticBaseRows = false(B,1);
staticBaseRows(end) = true;
for b = 1:robotCollision.NumBodies
    ancestor = robotCollision.Bodies{b};
    stationary = true;
    while ~strcmp(ancestor.Name,robotCollision.BaseName)
        if ~strcmp(ancestor.Joint.Type,'fixed')
            stationary = false; break;
        end
        ancestor = ancestor.Parent;
    end
    staticBaseRows(b) = stationary;
end
fprintf('\nAllowed tabletop mounting contact only for: %s\n', ...
    strjoin(bodyNames(staticBaseRows),', '));
disp('Allowed mounting interfaces: wrist_3_link / flangeAdapterCollision (parent-child),');
disp('and flangeAdapterCollision / glueGun. This is an assembly-contact assumption.');
disp('The remaining 12 dispenser parts are still checked against wrist_3_link.');
disp('Adapter remains checked against the environment and other non-parent arm links.');

% Four subintervals per waypoint segment; includes original waypoints.
subdivisions = 4;
qDense = zeros((size(qPath,1)-1)*subdivisions+1,6);
row = 1;
for segment = 1:size(qPath,1)-1
    for part = 0:subdivisions-1
        u = part/subdivisions;
        qDense(row,:) = (1-u)*qPath(segment,:)+u*qPath(segment+1,:);
        row = row+1;
    end
end
qDense(end,:) = qPath(end,:);
minimumWorldClearance = inf(size(qDense,1),1);
collisionPassed = false;
failedIndex = 0;
collisionPairs = cell(0,2);
fprintf('Checking %d poses, including joint-interpolated samples...\n',size(qDense,1));
for sample = 1:size(qDense,1)
    [~,D] = checkCollision(robotCollision,qDense(sample,:),worldObjects, ...
        'IgnoreSelfCollision','off','SkippedSelfCollisions',skipPairs,'Exhaustive','on');
    assert(isequal(size(D),[B B+numel(worldObjects)]),'Unexpected collision matrix dimensions.');
    % Remove ONLY intentional fixed-base/tabletop contact from results.
    D(staticBaseRows,B+1) = Inf;
    selfD = D(:,1:B);
    worldD = D(:,B+1:end);
    [si,sj] = find(triu(isnan(selfD),1));
    [wi,wj] = find(isnan(worldD));
    if ~isempty(si) || ~isempty(wi)
        failedIndex = sample;
        for k=1:numel(si)
            collisionPairs(end+1,:) = {bodyNames{si(k)},bodyNames{sj(k)}};
        end
        for k=1:numel(wi)
            collisionPairs(end+1,:) = {bodyNames{wi(k)},worldNames{wj(k)}};
        end
        break;
    end
    minimumWorldClearance(sample) = min(worldD,[],'all');
    if mod(sample,100)==0
        fprintf('Checked %d / %d poses\n',sample,size(qDense,1)); drawnow;
    end
end

% Display collision geometry at first detected overlap, or at closest pose.
if failedIndex>0
    fprintf('\nCOLLISION REVIEW REQUIRED at dense sample %d\n',failedIndex);
    disp(cell2table(collisionPairs,'VariableNames',{'RobotBody','OtherBodyOrObject'}));
    qInspect = qDense(failedIndex,:);
    if any(strcmp(collisionPairs(:,1),'wrist_3_link') & ...
           strcmp(collisionPairs(:,2),'glueGun')) || ...
       any(strcmp(collisionPairs(:,2),'wrist_3_link') & ...
           strcmp(collisionPairs(:,1),'glueGun'))
        diagnoseToolContact(robotCollision,qInspect,toolSpecs,skipPairs);
    end
    plotTitle = sprintf('Review overlap at sample %d - conservative geometry',failedIndex);
else
    collisionPassed = true;
    [closestDistance,closestIndex] = min(minimumWorldClearance);
    qInspect = qDense(closestIndex,:);
    fprintf('\nSAMPLED COLLISION CHECK: PASS (%d poses)\n',size(qDense,1));
    fprintf('Minimum world clearance, excluding fixed base/tabletop: %.4f mm\n',1000*closestDistance);
    plotTitle = 'Collision envelopes at smallest sampled world clearance';
end
figure('Name','Collision inspection','Color','w');
collisionAxes = axes;
show(robotCollision,qInspect,'Parent',collisionAxes,'Visuals','off','Collisions','on','Frames','off');
hold(collisionAxes,'on');
for k=1:numel(worldObjects)
    show(worldObjects{k},'Parent',collisionAxes);
end
axis(collisionAxes,'equal'); grid(collisionAxes,'on'); view(collisionAxes,135,25);
xlabel(collisionAxes,'X (m)'); ylabel(collisionAxes,'Y (m)'); zlabel(collisionAxes,'Z (m)');
title(collisionAxes,plotTitle);
disp('Limits: finite samples; conservative tool/cover envelopes; no hoses, fixtures or dynamics.');
disp('Only the hover-to-loop-to-hover route was checked, not travel from home.');
% collisionPassed is true ONLY after all sampled configurations pass.

function object = makeBox(dimensions,position)
    object = collisionBox(dimensions(1),dimensions(2),dimensions(3));
    object.Pose = trvec2tform(position);
end

function diagnoseToolContact(fullRobot,q,toolSpecs,skipPairs)
% Isolate each tool envelope; do not suppress the reported collision.
    labels = {'Flange adapter'; 'Horizontal bracket'; 'Vertical bracket'; ...
        'Blue barrel'; 'Rear cap'; 'Front cap'; 'Clamp band'; ...
        'Clamp block'; 'Screw 1'; 'Screw 2'; 'Nozzle envelope'; ...
        'Air fitting'; 'Rear connector'};
    labels = labels(2:end);
    toolSpecs = toolSpecs(2:end,:);
    partCount = size(toolSpecs,1);
    assert(partCount==numel(labels),'Tool component label count mismatch.');
    names = [fullRobot.BodyNames {fullRobot.BaseName}];
    wristIndex = find(strcmp(names,'wrist_3_link'));
    gunIndex = find(strcmp(names,'glueGun'));
    contact = false(partCount,1);
    clearanceMM = nan(partCount,1);
    for partIndex = 1:partCount
        isolatedRobot = copy(fullRobot);
        isolatedGun = getBody(isolatedRobot,'glueGun');
        clearCollision(isolatedGun);
        % Recreate from the same explicit primitive specification.
        addCollision(isolatedGun,toolSpecs{partIndex,1}, ...
            toolSpecs{partIndex,2},toolSpecs{partIndex,3});
        replaceBody(isolatedRobot,'glueGun',isolatedGun);
        [~,distances] = checkCollision(isolatedRobot,q, ...
            'SkippedSelfCollisions',skipPairs,'Exhaustive','on');
        distance = distances(wristIndex,gunIndex);
        contact(partIndex) = isnan(distance);
        clearanceMM(partIndex) = distance*1000;
    end
    fprintf('\nWRIST-TOOL COMPONENT DIAGNOSTIC\n');
    disp(table(labels,contact,clearanceMM, ...
        'VariableNames',{'ToolPart','TouchesOrOverlapsWrist','Clearance_mm'}));
    disp('NaN clearance means touching/overlap, not a measured penetration depth.');
    disp('Only the declared mounting interfaces are excluded. This additional contact remains unresolved.');
    relative = getTransform(fullRobot,q,'glueGun','wrist_3_link');
    disp('Tool attachment transform relative to wrist_3_link:');
    disp(relative);
    figure('Name','Wrist attachment close-up','Color','w');
    detailAxes = axes;
    show(fullRobot,q,'Parent',detailAxes,'Visuals','on','Collisions','off','Frames','off');
    Tmount = getTransform(fullRobot,q,'tool0');
    centre = Tmount(1:3,4);
    xlim(detailAxes,centre(1)+[-0.13 0.13]);
    ylim(detailAxes,centre(2)+[-0.13 0.13]);
    zlim(detailAxes,centre(3)+[-0.13 0.13]);
    daspect(detailAxes,[1 1 1]); grid(detailAxes,'on'); view(detailAxes,135,25);
    title(detailAxes,'Mounting detail - inspect component diagnostic table');
end
