function output = UR5e_Glue_Bead(action,varargin)
% Visual deposition for existing timed UR5e motion, MATLAB R2025b.
% Prepare: signal=UR5e_Glue_Bead('prepare',robotUEWorkcell,jointMotionUE,phaseDurations);
% Simulation 3D Actor initialization: UR5e_Glue_Bead('create',Actor);
root=fileparts(mfilename('fullpath'));
dataFile=fullfile(root,'UR5e_Bead_Data.mat');
output=[];
switch lower(char(action))
 case 'prepare'
  robot=varargin{1}; motion=double(varargin{2}); durations=double(varargin{3}(:));
  assert(size(motion,2)==7 && size(motion,1)>2,'Expected [time, six joint angles].');
  assert(all(isfinite(motion),'all') && all(diff(motion(:,1))>0));
  assert(numel(durations)==3 && all(durations>0));
  delay=motion(2,1); % first timed pose follows initial hold
  assert(max(abs(motion(1,2:end)-motion(2,2:end)))<1e-10);
  assert(abs(motion(end,1)-delay-sum(durations))<1e-6, ...
      'Motion and phase durations do not match. Recreate both from the same run.');
  dt=.02; first=delay+durations(1); last=first+durations(2);
  t=(0:dt:ceil((motion(end,1)+2)/dt)*dt)';
  active=find(t>=first & t<=last);
  assert(~isempty(active));
  q=interp1(motion(:,1),motion(:,2:end),t(active),'linear');
  xyz=zeros(numel(active),3);
  for k=1:numel(active)
   T=getTransform(robot,q(k,:),'glueTCP');xyz(k,:)=T(1:3,4)';
  end
  assert(max(abs(xyz(:,3)-.048))<.0002, ...
      'This bead setup expects the existing 45 mm rim and 3 mm nozzle gap.');
  % Verify the path is on the current oval flange before projecting onto it.
  xy=xyz(:,1:2)-[.45 0];
  assert(all(sum((xy./[.15 .10]).^2,2)<1) && ...
         all(sum((xy./[.12 .07]).^2,2)>1),'TCP path is outside the expected rim.');
  keep=1;
  for k=2:size(xyz,1)
   if norm(xyz(k,1:2)-xyz(keep(end),1:2))>=.001
    keep(end+1)=k; %#ok<AGROW>
   end
  end
  if keep(end)~=size(xyz,1),keep(end+1)=size(xyz,1);end
  positions=xyz(keep,:);positions(:,3)=.0455;
  diameter=.003; color=[.95 .75 .18]; % amber for visibility
  gaps=vecnorm(diff(positions(:,1:2)),2,2);
  assert(all(gaps<.0025),'Deposits would leave gaps. Reduce sample time or increase bead diameter.');
  assert(norm(positions(end,1:2)-positions(1,1:2))<.0025,'Bead loop does not close.');
  instance=zeros(size(t));
  instance(active(keep))=1:numel(keep);
  for k=2:numel(t)
   if instance(k)==0,instance(k)=instance(k-1);end
  end
  output=[t instance];
  save(dataFile,'positions','diameter','color');
  fprintf('Bead prepared: %d deposits, %.1f mm nominal width.\n',numel(keep),1000*diameter);
  fprintf('Deposition window: %.2f to %.2f s.\n',first,last);
 case 'create'
  actor=varargin{1};
  token=regexp(char(actor.Name),'^GlueBead(\d+)$','tokens','once');
  if isempty(token),return;end % block preview may use the unsuffixed name
  index=str2double(token{1});
  data=load(dataFile,'positions','diameter','color');
  assert(index>=1 && index<=size(data.positions,1),'Invalid glue actor instance.');
  actor.CoordinateSystem='MATLAB';
  createShape(actor,'sphere',data.diameter*[1 1 1],12);
  actor.Translation=data.positions(index,:);
  actor.Color=data.color;
  actor.Physics=false;
  actor.Gravity=false;
  actor.Collisions=false;
  actor.Shininess=.3;
  actor.Metallic=0;
 otherwise
  error('Use prepare or create.');
end
end
