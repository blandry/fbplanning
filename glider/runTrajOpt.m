function [xtraj,utraj] = runTrajOpt(p,options)

N = options.N;
minimum_duration = options.minimum_duration;
maximum_duration = options.maximum_duration;

prog = DircolTrajectoryOptimization(p,N,[minimum_duration maximum_duration]);

x0 = options.x0; 
xf = options.xf;
prog = prog.addStateConstraint(ConstantConstraint(double(x0)),1);
prog = prog.addStateConstraint(ConstantConstraint(double(xf)),N);

prog = prog.addRunningCost(@cost);
prog = prog.addFinalCost(@finalCost);

tf0 = options.xtraj0.tspan(2);
traj_init.x = options.xtraj0;
traj_init.u = ConstantTrajectory(zeros(getNumInputs(p),1));

display('Running full body traj opt...');
info=0;
while (info~=1)
  tic
  [xtraj,utraj,~,~,info] = prog.solveTraj(tf0,traj_init);
  toc
end

end

function [g,dg] = cost(dt,x,u)
  R = eye(numel(u));
  g = u'*R*u;
  dg = [zeros(1,1+numel(x)),2*u'*R];
end

function [h,dh] = finalCost(t,x)
  h = 100*t;
  dh = [100,zeros(1,size(x,1))];
end