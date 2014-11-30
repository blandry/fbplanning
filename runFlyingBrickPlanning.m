function [bricktraj, forcetraj] = runFlyingBrickPlanning(p,options)

r = FlyingBrickPlant(p);

N = options.N;
minimum_duration = options.minimum_duration;
maximum_duration = options.maximum_duration;

prog = DircolTrajectoryOptimization(r,N,[minimum_duration maximum_duration]);

num_states = numel(options.x0);
x0 = options.x0; x0 = [x0(1:6);x0(num_states/2+[1:6])];
xf = options.xf; xf = [xf(1:6);xf(num_states/2+[1:6])];
prog = prog.addStateConstraint(ConstantConstraint(double(x0)),1);
prog = prog.addStateConstraint(ConstantConstraint(double(xf)),N);

% CONSTRAINTS ON THE FORCES
% TODO!!!
% 1 - passive elements = wings can only remove energy
% 2 - forces must be smooth = |derivative| of u must be < epsilon
% 3 - magnitudes must in a certain range

prog = prog.addRunningCost(@cost);
prog = prog.addFinalCost(@finalCost);

tf0 = .5*(minimum_duration+maximum_duration);
traj_init.x = PPTrajectory(foh([0,tf0],[double(x0),double(xf)]));
traj_init.u = ConstantTrajectory(zeros(getNumInputs(r),1));

display('Running flying brick traj opt...');
info=0;
while (info~=1)
  tic
  [bricktraj,forcetraj,~,~,info] = prog.solveTraj(tf0,traj_init);
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
