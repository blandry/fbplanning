function [bricktraj, forcetraj] = runFlyingBrickPlanning(p,options)

r = FlyingBrickPlant(p);

N = options.N;
minimum_duration = options.minimum_duration;
maximum_duration = options.maximum_duration;
prog = DircolTrajectoryOptimization(r,N,[minimum_duration maximum_duration]);

num_states = numel(options.x0lb);
x0lb = options.x0lb; x0lb = [x0lb(1:6);x0lb(num_states/2+[1:6])];
x0ub = options.x0ub; x0ub = [x0ub(1:6);x0ub(num_states/2+[1:6])];
xlb = options.xlb; xlb = [xlb(1:6);xlb(num_states/2+[1:6])];
xub = options.xub; xub = [xub(1:6);xub(num_states/2+[1:6])];
xflb = options.xflb; xflb = [xflb(1:6);xflb(num_states/2+[1:6])];
xfub = options.xfub; xfub = [xfub(1:6);xfub(num_states/2+[1:6])];
prog = prog.addStateConstraint(BoundingBoxConstraint(options.x0lb,options.x0ub),1);
prog = prog.addStateConstraint(BoundingBoxConstraint(options.xlb,options.xub),1:N);
prog = prog.addStateConstraint(BoundingBoxConstraint(options.xflb,options.xfub),N);

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
