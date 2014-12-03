function [xtraj,utraj] = runTrajOpt(p,options)

N = options.N;
minimum_duration = options.minimum_duration;
maximum_duration = options.maximum_duration;
prog = DircolTrajectoryOptimization(p,N,[minimum_duration maximum_duration]);
prog = prog.addStateConstraint(BoundingBoxConstraint(options.x0lb,options.x0ub),1);
prog = prog.addStateConstraint(BoundingBoxConstraint(options.xlb,options.xub),1:N);
prog = prog.addStateConstraint(BoundingBoxConstraint(options.xflb,options.xfub),N);

%prog = prog.addRunningCost(@cost);
%prog = prog.addFinalCost(@finalCost);
prog = prog.addTrajectoryDisplayFunction(@plotDircolTraj);

if (isfield(options,'xtraj0')&&isfield(options,'utraj0'))
  tf0 = options.xtraj0.tspan(2);
  traj_init0.x = options.xtraj0;
  traj_init0.u = options.utraj0;
else
  tf0 = .5*(options.minimum_duration+options.maximum_duration);
  traj_init0.x = PPTrajectory(foh([0,tf0],[options.x0lb,options.xflb]));
  traj_init0.u = ConstantTrajectory(zeros(getNumInputs(p),1));
  % traj_init0 = struct();
end

display('Running full body traj opt...');
for i=1:1
  tic
  [xtraj,utraj,z,F,info,infeasible_constraint_name] = prog.solveTraj(tf0,traj_init0);
  toc
  if info==1, break; end
end
if info~=1, error('Failed to find a trajectory'); end

end

function [g,dg] = cost(dt,x,u)
  R = eye(numel(u));
  g = u'*R*u;
  if (nargout>1)
    dg = [0,zeros(1,length(x)),2*u'*R];
  end
end

function [h,dh] = finalCost(t,x)
  h = t;
  if (nargout>1)
    dh = [1,zeros(1,numel(x))];
  end
end

function plotDircolTraj(t,x,u)
  figure(1)
  clf;
  hold on
  plot(x(1,:),x(3,:),'.-');
  hold off
  axis([-15 15 -15 15]);
  drawnow;
  %delete(h);
end