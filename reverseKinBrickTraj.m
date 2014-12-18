function [xtraj,utraj] = reverseKinBrickTraj(p,bricktraj,forcetraj,options)
% objective is to match the forces at each state
% doing the optimization as a full trajectory planning
% maximizing the forces match

N = options.N;
minimum_duration = options.minimum_duration;
maximum_duration = options.maximum_duration;
prog = DirtranTrajectoryOptimization(p,N,[minimum_duration maximum_duration],struct('integration_method',2));

% prog = prog.addStateConstraint(BoundingBoxConstraint(options.xlb,options.xub),1:N);

% parametrizing the forces with the x position (monotonically increasing)
breaks = bricktraj.getBreaks();
brickpos = bricktraj.eval(breaks);
xforcetraj = PPTrajectory(spline(brickpos(1,:),forcetraj.eval(breaks)));
prog = prog.addStateConstraint(BoundingBoxConstraint([brickpos(1,1)',-Inf*ones(1,getNumStates(p)-1)]',[brickpos(1,1)',Inf*ones(1,getNumStates(p)-1)]'),1);
prog = prog.addStateConstraint(BoundingBoxConstraint([brickpos(1,end)',-Inf*ones(1,getNumStates(p)-1)]',[brickpos(1,end)',Inf*ones(1,getNumStates(p)-1)]'),N);

Q = zeros(prog.num_vars);
Q(prog.h_inds,prog.h_inds) = 2;
prog = prog.addConstraint(QuadraticConstraint(0.0001,.6,Q,zeros(prog.num_vars,1)));

prog = prog.addRunningCost(@(dt,x,u)cost(dt,x,u,p,xforcetraj));
prog = prog.addFinalCost(@finalCost);
% prog = prog.addTrajectoryDisplayFunction(@(t,x,u)plotDircolTraj(t,x,u,options.xlb(1),options.xub(1),options.xlb(3),options.xub(3)));

tf0 = .5*(minimum_duration+maximum_duration);
extra_states = getNumStates(p)-12;
xtraj0 = PPTrajectory(spline(breaks,[brickpos(1:6,:);zeros(extra_states/2,N);brickpos(7:12,:);zeros(extra_states/2,N)]));
traj_init.x = xtraj0;
traj_init.u = ConstantTrajectory(zeros(getNumInputs(p),1));

prog = prog.setSolverOptions('snopt','superbasicslimit',prog.num_vars+1);
prog = prog.setSolverOptions('snopt','majoriterationslimit',2000);
% prog = prog.setSolverOptions('snopt','iterationslimit',50000);
% prog = prog.setSolverOptions('snopt','print','snopt.out');

% prog = prog.setSolver('fmincon');
prog = prog.setSolver('snopt');

display('Running reverse kinematics (traj version)...');
tic
[xtraj,utraj,~,~,info] = prog.solveTraj(tf0,traj_init);
toc

info

end

function [g,dg] = cost(dt,x,u,p,xforcetraj)
  fdes = xforcetraj.eval(x(1));
  fdes = reshape(fdes,3,[]);
  forces = cell(1,numel(p.force));
  for i=1:numel(p.force)
    force_element = p.force{i};
    if isprop(force_element,'child_body')
      body_ind = force_element.child_body;
    else
      body_frame = getFrame(p,force_element.kinframe);
      body_ind = body_frame.body_ind;
    end
    f_ext = zeros(6,p.featherstone.NB);
    f_ext(1:3,body_ind) = fdes(:,i);
    forces{i} = f_ext;
  end
  q = x(1:numel(x)/2);
  qd = x(numel(x)/2+1:end);
  [~,Cdes,~,~,dCdes] = p.manipulatorDynamicsGivenForces(q,qd,true,forces);
  [~,C,~,~,dC] = p.manipulatorDynamicsNoTorque(q,qd,true);
  %Q = 0*diag([0 0 0 1 1 1 1 1 1 1 1 1 1 1]);
  %R = 0;
  %g = (C-Cdes)'*(C-Cdes) + x'*Q*x + u'*R*u;
  %dg = [0,2*(C-Cdes)'*(dC-dCdes)+2*x'*Q,2*u'*R];
  g = (C-Cdes)'*(C-Cdes);
  dg = [0,2*(C-Cdes)'*(dC-dCdes),zeros(1,numel(u))];
end

function [h,dh] = finalCost(t,x)
  h = t;
  if (nargout>1)
    dh = [1,zeros(1,numel(x))];
  end
end

function plotDircolTraj(t,x,u,xmin,xmax,zmin,zmax)
  figure(1)
  clf;
  hold on
  plot(x(1,:),x(3,:),'.-');
  hold off
  axis([xmin xmax zmin zmax]);
  drawnow;
end
