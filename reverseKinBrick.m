function xx = reverseKinBrick(p,brickpos,forces)
% objective is to match the forces at each state

% TODO: take some constraints on the states of the actuators

N = size(brickpos,2);
xx = zeros(getNumStates(p),N);
display('Running reverse kin opt...');
for j=1:N
  numvars = getNumStates(p)-12;
  w = warning('off','optim:fminunc:SwitchingMethod');
  extra_states = fminunc(@(x)forcedif(x,p,brickpos(:,j),forces(:,j)),zeros(numvars,1),struct('Display','on'));
  warning(w);
  xx(:,j) = [brickpos(1:6,j);extra_states(1:numel(extra_states)/2);brickpos(7:12,j);extra_states(numel(extra_states)/2+1:end)];
end

end

function df = forcedif(x,p,brick,force)
  q = [brick(1:6);x(1:numel(x)/2)];
  qd = [brick(7:12);x(numel(x)/2+1:end)];
  kinsol = doKinematics(p,q);
  force_ext = zeros(size(force));
  for i=1:numel(p.force)
    force_element = p.force{i};
    if isprop(force_element,'child_body')
      body_ind = force_element.child_body;
    else
      body_frame = getFrame(p,force_element.kinframe);
      body_ind = body_frame.body_ind;
    end
    f_ext = computeSpatialForce(force_element,p,q,qd);
    joint_wrench = f_ext(:,body_ind);
    body_wrench = inv(p.body(body_ind).X_joint_to_body)'*joint_wrench;
    pos = forwardKin(p,kinsol,body_ind,[zeros(3,1),body_wrench(1:3),body_wrench(4:6)]);
    point = pos(:,1);
    % torque_ext = pos(:,2)-point;
    force_ext(3*(i-1)+1:3*(i-1)+3) = pos(:,3)-point;
  end
  df = (force_ext-force)'*(force_ext-force);
end