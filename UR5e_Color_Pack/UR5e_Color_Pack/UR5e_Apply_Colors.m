function coloredRobot = UR5e_Apply_Colors(sourceRobot)
% UR5e visual-only color approximation for MATLAB R2025b.
% Usage: robotUEColored = UR5e_Apply_Colors(robotUE);
% Set the Simulation 3D Robot block's rigid body tree to robotUEColored.
% Original model, joints, collision objects, and tool visuals are untouched.
root = fileparts(mfilename('fullpath'));
data = load(fullfile(root,'UR5e_Color_Data.mat'),'parts','palette');
coloredRobot = copy(sourceRobot);
meshDir = fullfile(root,'colored_meshes_v4');
if ~isfolder(meshDir), mkdir(meshDir); end
for k = 1:numel(data.parts)
    if iscell(data.parts), part = data.parts{k};
    else, part = data.parts(k); end
    name = char(part.BodyName);
    assert(any(strcmp(sourceRobot.BodyNames,name)), ...
        'Missing expected UR5e body: %s',name);
    body = getBody(coloredRobot,name);
    current = getVisual(body);
    assert(numel(current)==1, ...
        'Expected original single visual on %s. Pass the original robotUE.',name);
    assert(norm(double(current.Tform)-double(part.Tform),'fro')<1e-9, ...
        'Visual transform does not match the exported model: %s',name);
    assert(all(abs(double(current.Scale(:))-1)<1e-12) && ...
           all(abs(double(part.Scale(:))-1)<1e-12), ...
        'This pack expects the unit mesh scales in your export.');
    originalPoints = current.Triangulation.Points;
    assert(max(abs(min(originalPoints,[],1)-min(part.Vertices,[],1)))<1e-8 && ...
           max(abs(max(originalPoints,[],1)-max(part.Vertices,[],1)))<1e-8, ...
        'Visual bounds do not match the exported model: %s',name);
    paths = cell(size(data.palette,1),1);
    for group = 1:size(data.palette,1)
        faceRows = part.Faces(part.Labels(:)==group,:);
        if isempty(faceRows), continue; end
        [used,~,mapped] = unique(faceRows(:));
        mesh = triangulation(reshape(mapped,size(faceRows)),part.Vertices(used,:));
        paths{group} = fullfile(meshDir,sprintf('%s_color%d.stl',name,group));
        stlwrite(mesh,paths{group});
    end
    clearVisual(body);
    for group = 1:size(data.palette,1)
        if isempty(paths{group}), continue; end
        addVisual(body,'Mesh',paths{group},double(part.Tform), ...
            'FaceColor',data.palette(group,:),'FaceAlpha',1);
    end
end
% Verify every joint's fixed transforms and all collision-object counts.
assert(isequal(sourceRobot.BodyNames,coloredRobot.BodyNames));
for k = 1:numel(sourceRobot.Bodies)
    before = sourceRobot.Bodies{k}; after = coloredRobot.Bodies{k};
    assert(strcmp(before.Joint.Type,after.Joint.Type));
    assert(isequal(before.Joint.JointToParentTransform,after.Joint.JointToParentTransform));
    assert(isequal(before.Joint.ChildToJointTransform,after.Joint.ChildToJointTransform));
    assert(numel(before.Collisions)==numel(after.Collisions));
end
fprintf('Color update complete: joints and collision-object counts unchanged.\n');
fprintf('Set the Simulation 3D Robot rigid body tree to robotUEColored, then restart simulation.\n');
end
