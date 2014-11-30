function xtraj = reverseKinBrick(p,bricktraj,forcetraj)

t = bricktraj.getBreaks();
N = numel(t);
bx = bricktraj.eval(t);
fx = forcetraj.eval(t);

prog = NonlinearProgram(getNumStates(p)*N);

% objective is to match the forces at each state
prog = prog.addCost(FunctionHandleConstraint(-Inf,Inf,prog.num_vars,@(x)cost(x,p,fx),0));

% use the brick traj for a partial initial guess
x0 = zeros(getNumStates(p),N);
x0(1:6,:) = bx(1:6,:);
x0(getNumStates(p)/2+[1:6],:) = bx(7:12,:);
x0 = reshape(x0,[],1);

[x,~,exitflag] = prog.solve(x0);

x = reshape(x,getNumStates(p),[]);
xtraj = PPTrajectory(spline(t,x));

end

function g = cost(xvar,p,fx)
  w = warning('off','Drake:TaylorVar:DoubleConversion');
  x = double(reshape(xvar,getNumStates(p),[]));
  warning(w);
  q = x(1:getNumStates(p)/2,:);
  qd = x(getNumStates(p)/2+1:end,:);
  N = size(x,2);
  
  df = zeros(3*numel(p.force),N);
  for i=1:N
    for j=1:numel(p.force)
       rbf = p.force{j};
       frame = getFrame(p,rbf.kinframe);
       wrench = rbf.computeSpatialForce(p,q(:,i),qd(:,i));
       df(3*(j-1)+1:3*j,i) = wrench(1:3,frame.body_ind); % should be handled more carefully
    end
  end
  df = reshape(df,[],1);
  
  g = (df-reshape(fx,[],1))'*(df-reshape(fx,[],1));
end