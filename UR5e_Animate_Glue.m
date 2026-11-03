%% Timed glue demonstration - MATLAB R2025b
% Keep beside all four earlier scripts. Run entire file.
% Visual bead only, not fluid physics or a hardware-ready controller.
% Piecewise-linear joint interpolation is retained so the geometric route
% matches the collision-check interpolation. Velocity may change at knots;
% acceleration/jerk and dynamics remain unvalidated.
animFolder = fileparts(mfilename('fullpath'));
run(fullfile(animFolder,'UR5e_Check_Collisions.m'));
assert(collisionPassed,'Resolve the collision report before this demonstration.');

%% Demonstration settings (NOT manufacturer limits)
maxGlueSpeed = 0.040;       % 40 mm/s nominal path-speed maximum
maxApproachSpeed = 0.020;   % 20 mm/s nominal path-speed maximum
jointSpeedScreen = deg2rad(60); % chosen demonstration screening limit
samplePeriod = 0.04;        % playback sample interval, seconds
playbackSpeed = 1;          % 1 = nominal real-time; 2 = twice as fast visually
characteristicLength = 0.5; % metres, for dimensionally scaled Jacobian
ratioThreshold = 1e-3;      % heuristic singularity screen, not UR specification

%% Screen dense collision-checked poses for Jacobian conditioning
sigmaRatio = zeros(size(qDense,1),1);
for k=1:size(qDense,1)
    J = geometricJacobian(robot,qDense(k,:),'glueTCP');
    % MATLAB ordering: angular rows first, linear rows last.
    Jscaled = [J(1:3,:); J(4:6,:)/characteristicLength];
    singularValues = svd(Jscaled);
    sigmaRatio(k) = singularValues(end)/singularValues(1);
end
fprintf('\nMinimum scaled Jacobian singular-value ratio: %.6f\n',min(sigmaRatio));
assert(min(sigmaRatio)>ratioThreshold, ...
    'Singularity screen flagged a pose. Send the ratio before continuing.');

%% Three timed phases, with quintic progress from rest to rest
% Each phase follows the existing joint-space polyline. Arc length is based
% on desired TCP waypoints, so actual TCP speed can differ slightly.
phaseRows = {1:loopIndices(1), loopIndices, loopIndices(end):size(qPath,1)};
phaseSpeeds = [maxApproachSpeed maxGlueSpeed maxApproachSpeed];
phaseNames = {'DESCEND - glue OFF','DISPENSE - glue ON','RETRACT - glue OFF'};
time = []; qTimed = []; phaseID = []; nominalSpeed = [];
elapsed = 0;
phaseDurations = zeros(3,1);
exactJointSpeedBound = 0;
for phase=1:3
    ids=phaseRows{phase};
    p=tcpWaypoints(ids,:); qPhase=qPath(ids,:);
    arc=[0;cumsum(vecnorm(diff(p),2,2))];
    L=arc(end);
    assert(all(diff(arc)>0),'Repeated waypoints in a timing phase.');
    dqds=diff(qPhase)./diff(arc);
    % Quintic smoothstep has maximum derivative 1.875.
    duration=max(1.875*L/phaseSpeeds(phase), ...
        1.875*L*max(abs(dqds),[],'all')/jointSpeedScreen);
    duration=1.05*duration; % small timing margin
    localTime=linspace(0,duration,ceil(duration/samplePeriod)+1)';
    u=localTime/duration;
    progress=10*u.^3-15*u.^4+6*u.^5;
    progress=min(1,max(0,progress));
    speed=L/duration*(30*u.^2-60*u.^3+30*u.^4);
    qt=interp1(arc,qPhase,L*progress,'linear');
    bound=1.875*L/duration*max(abs(dqds),[],'all');
    exactJointSpeedBound=max(exactJointSpeedBound,bound);
    if phase>1
        localTime=localTime(2:end); qt=qt(2:end,:); speed=speed(2:end);
    end
    time=[time;elapsed+localTime]; %#ok<AGROW>
    qTimed=[qTimed;qt]; %#ok<AGROW>
    phaseID=[phaseID;phase*ones(size(localTime))]; %#ok<AGROW>
    nominalSpeed=[nominalSpeed;speed]; %#ok<AGROW>
    elapsed=elapsed+duration;
    phaseDurations(phase)=duration;
end
assert(all(isfinite(qTimed),'all'),'Nonfinite timed configuration.');
assert(exactJointSpeedBound<=jointSpeedScreen,'Joint speed screen failed.');

% Compute actual nozzle locations for rendering and speed diagnostics.
tcpActual=zeros(numel(time),3);
for k=1:numel(time)
    T=getTransform(robot,qTimed(k,:),'glueTCP');
    tcpActual(k,:)=T(1:3,4)';
end
segmentSpeed=vecnorm(diff(tcpActual),2,2)./diff(time);
fprintf('Nominal cycle duration: %.2f s\n',time(end));
fprintf('Joint-speed upper bound on interpolated route: %.2f deg/s\n',rad2deg(exactJointSpeedBound));
fprintf('Maximum sampled actual TCP speed: %.2f mm/s\n',1000*max(segmentSpeed));
disp('Timing and sampled singularity screens passed. Acceleration/jerk NOT validated.');

%% New scene, including the same bench and cover
animationFigure=figure('Name','Timed glue application','Color','w');
animationAxes=axes('Parent',animationFigure);
show(robot,qTimed(1,:),'Parent',animationAxes,'Frames','off','PreservePlot',false);
hold(animationAxes,'on');
drawBox(animationAxes,[0.30 0 -0.025],[1.10 0.80 0.05],[0.55 0.58 0.61]);
for bx=[-0.20 0.80]
    for by=[-0.35 0.35]
        drawBox(animationAxes,[bx by -0.425],[0.05 0.05 0.75],[0.22 0.25 0.28]);
    end
end
drawSolid(animationAxes,wallOuter,0,0.008,[0.52 0.56 0.61]);
drawRing(animationAxes,wallOuter,inner,0.008,0.035,[0.62 0.67 0.72]);
drawRing(animationAxes,outer,inner,0.035,rimZ,[0.78 0.81 0.85]);
bead=animatedline(animationAxes,'Color',[0.1 0.8 0.2],'LineWidth',4);
% Bead is projected onto the surface; thickness is in screen points, not mm.
axis(animationAxes,'equal'); grid(animationAxes,'on'); view(animationAxes,135,25);
xlim(animationAxes,[-0.25 0.85]); ylim(animationAxes,[-0.42 0.42]);
zlim(animationAxes,[-0.15 0.85]);
xlabel(animationAxes,'X (m)'); ylabel(animationAxes,'Y (m)'); zlabel(animationAxes,'Z (m)');
wallClock=tic;
beadStarted=false;
for k=1:numel(time)
    if ~isgraphics(animationFigure), break; end
    show(robot,qTimed(k,:),'Parent',animationAxes,'Frames','off', ...
        'PreservePlot',false,'FastUpdate',true);
    if phaseID(k)==2
        if ~beadStarted
            startK=find(phaseID==1,1,'last');
            addpoints(bead,tcpActual(startK,1),tcpActual(startK,2),rimZ+0.0005);
            beadStarted=true;
        end
        addpoints(bead,tcpActual(k,1),tcpActual(k,2),rimZ+0.0005);
    end
    title(animationAxes,sprintf('%s | t = %.1f / %.1f s', ...
        phaseNames{phaseID(k)},time(k),time(end)));
    drawnow;
    if k<numel(time)
        pause(max(0,time(k+1)/playbackSpeed-toc(wallClock)));
    end
end
% Time is simulated time. Slow rendering may make playback take longer.
% Flow should eventually be linked to speed for a physical constant bead.
% Current trace does not predict bead width, flow, curing, or start/stop blobs.
figure('Name','Timing diagnostics','Color','w');
tiledlayout(2,1);
nexttile;
plot(time,nominalSpeed*1000,'LineWidth',1.2); grid on;
xlabel('Simulation time (s)'); ylabel('Nominal path speed (mm/s)');
title('Progress ramps within each phase; glue phase is not constant speed');
nexttile;
plot(1:numel(sigmaRatio),sigmaRatio,'LineWidth',1.2); hold on;
yline(ratioThreshold,'r--'); grid on;
xlabel('Dense path sample'); ylabel('Scaled singular-value ratio');
title('Heuristic singularity screen (characteristic length 0.5 m)');

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
