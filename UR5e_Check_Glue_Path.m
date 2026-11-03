%% Stage: sampled kinematic feasibility of approach, glue loop, and retract
% MATLAB R2025b. Put beside the two previous scripts and Run entire file.
% No animation or hardware commands. No collision or dynamic checks yet.
% The path has no timestamps: sampling spacing is NOT a speed command.
pathFolder = fileparts(mfilename('fullpath'));
run(fullfile(pathFolder,'UR5e_Workcell_Layout.m'));

%% Proposed nozzle path: 3 mm above the flange, pointing downward
% 3 mm is a demonstration assumption, not a validated dispensing setting.
dispensingGap = 0.003;
pathZ = rimZ + dispensingGap;
loopCount = 241; % Includes both start and end at the same Cartesian point
pathAngles = linspace(-pi/2,3*pi/2,loopCount)';
glueLoop = [coverCentre(1)+0.135*cos(pathAngles), ...
           coverCentre(2)+0.085*sin(pathAngles), ...
           pathZ*ones(loopCount,1)];
glueLoop(end,:) = glueLoop(1,:);
approachCount = 31;
descent = [repmat(glueLoop(1,1:2),approachCount,1), ...
           linspace(rimZ+hoverGap,pathZ,approachCount)'];
retract = flipud(descent);
tcpWaypoints = [descent; glueLoop(2:end,:); retract(2:end,:)];
N = size(tcpWaypoints,1);
loopIndices = approachCount:(approachCount+loopCount-1);

% Check the proposed deposition centreline lies between both flange edges.
xy = glueLoop(:,1:2)-coverCentre;
outerMetric = (xy(:,1)/0.150).^2+(xy(:,2)/0.100).^2;
innerMetric = (xy(:,1)/0.120).^2+(xy(:,2)/0.070).^2;
assert(all(outerMetric < 1) && all(innerMetric > 1), ...
    'The proposed glue centreline is outside the concept flange.');

%% Solve consecutively, using the previous result as the next initial guess
pathIK = inverseKinematics('RigidBodyTree',robot);
pathIK.SolverParameters.AllowRandomRestart = false;
pathIK.SolverParameters.SolutionTolerance = 1e-7;
qPath = zeros(N,6);
positionErrors = zeros(N,1);
orientationErrors = zeros(N,1);
actualTCP = zeros(N,3);
qSeed = qHover;
fprintf('\nChecking %d sampled poses...\n',N);
for idx = 1:N
    desired = trvec2tform(tcpWaypoints(idx,:))*axang2tform([1 0 0 pi]);
    [qCandidate,~] = pathIK('glueTCP',desired,[1 1 1 1 1 1],qSeed);
    reached = getTransform(robot,qCandidate,'glueTCP');
    positionErrors(idx) = norm(reached(1:3,4)-desired(1:3,4));
    Rerror = reached(1:3,1:3)'*desired(1:3,1:3);
    orientationErrors(idx) = acos(max(-1,min(1,(trace(Rerror)-1)/2)));
    if positionErrors(idx)>0.0001 || orientationErrors(idx)>deg2rad(0.1)
        error('Pose %d/%d failed: %.4f mm, %.4f deg. Stop and report this.', ...
            idx,N,1000*positionErrors(idx),rad2deg(orientationErrors(idx)));
    end
    % Use raw joint differences: do not hide a full-turn jump by wrapping.
    if max(abs(qCandidate-qSeed)) > deg2rad(5)
        error('Joint change exceeds 5 degrees at sample %d. Report before continuing.',idx);
    end
    qPath(idx,:) = qCandidate;
    actualTCP(idx,:) = reached(1:3,4)';
    qSeed = qCandidate;
end
maxJointStep = max(abs(diff([qHover;qPath])),[],'all');
jointLoopClosure = max(abs(qPath(loopIndices(end),:)-qPath(loopIndices(1),:)));

%% Show desired path on the existing workcell (robot stays at hover)
plot3(ax,glueLoop(:,1),glueLoop(:,2),glueLoop(:,3), ...
    'Color',[0.0 0.65 0.25],'LineWidth',2.5);
title(ax,'Proposed glue path - robot shown at hover, not moving');

%% Close-up plan view and diagnostics
figure('Name','Glue path checks','Color','w');
tiledlayout(2,2);
nexttile;
plot(outer([1:end 1],1),outer([1:end 1],2),'k-', ...
     inner([1:end 1],1),inner([1:end 1],2),'k-');
hold on;
plot(glueLoop(:,1),glueLoop(:,2),'g-','LineWidth',2);
plot(glueLoop(1,1),glueLoop(1,2),'ro','MarkerFaceColor','r');
axis equal; grid on; xlabel('X (m)'); ylabel('Y (m)');
title('Top view: path between flange edges');
legend('Outer edge','Inner edge','Proposed glue path','Start / end','Location','best');
nexttile;
plot(1:N,rad2deg(qPath),'LineWidth',1); grid on;
xlabel('Waypoint index (not time)'); ylabel('Joint angle (deg)');
title('Joint continuity'); legend('J1','J2','J3','J4','J5','J6','Location','best');
nexttile;
plot(1:N,1000*positionErrors); grid on;
xlabel('Waypoint index'); ylabel('Position error (mm)'); title('TCP position error');
nexttile;
plot(1:N,rad2deg(orientationErrors)); grid on;
xlabel('Waypoint index'); ylabel('Orientation error (deg)'); title('TCP orientation error');

fprintf('Sampled path kinematics: PASS (%d poses)\n',N);
fprintf('Maximum position error: %.6f mm\n',1000*max(positionErrors));
fprintf('Maximum orientation error: %.6f deg\n',rad2deg(max(orientationErrors)));
fprintf('Largest joint step: %.4f deg\n',rad2deg(maxJointStep));
fprintf('Joint difference at loop closure: %.4f deg\n',rad2deg(jointLoopClosure));
fprintf('Demonstration nozzle gap: %.1f mm\n',1000*dispensingGap);
disp('Not yet checked: collisions, singularities, between-sample motion, speeds or accelerations.');
% Retain robot, qPath, tcpWaypoints, loopIndices for the next validation stage.
