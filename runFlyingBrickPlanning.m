function [bricktraj, forcetraj] = runFlyingBrickPlanning(p,options)

r = FlyingBrickPlant(p);
% Note that the states of the brick
% are not rolldot, yawdot and pitchdot
% but angular velocities!!!

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

% convert the rdot,pdot,ydot to angular vel constraints
x0lb(10:12) = rpydot2angularvel(x0lb(4:6),x0lb(10:12));
x0ub(10:12) = rpydot2angularvel(x0ub(4:6),x0ub(10:12));
xlb(10:12) = rpydot2angularvel(xlb(4:6),xlb(10:12));
xub(10:12) = rpydot2angularvel(xub(4:6),xub(10:12));
xflb(10:12) = rpydot2angularvel(xflb(4:6),xflb(10:12));
xfub(10:12) = rpydot2angularvel(xfub(4:6),xfub(10:12));

prog = prog.addStateConstraint(BoundingBoxConstraint(x0lb,x0ub),1);
prog = prog.addStateConstraint(BoundingBoxConstraint(xlb,xub),1:N);
prog = prog.addStateConstraint(BoundingBoxConstraint(xflb,xfub),N);

% CONSTRAINTS ON THE FORCES
% 1 - passive elements = wings can only remove energy (unless you are
% flapping them...)
P = zeros(6,3*size(r.Fpos,2));
for i=1:size(r.Fpos,2)
  Pi = [0 -r.Fpos(3,i) r.Fpos(2,i); r.Fpos(3,i) 0 -r.Fpos(1,i); -r.Fpos(2,i) r.Fpos(1,i) 0];
  P(:,3*(i-1)+1:3*(i-1)+3) = [eye(3);Pi];
end
for j=1:N
  Q = sparse(prog.num_vars,prog.num_vars);
  qd_inds = prog.x_inds(7:12,j);
  fext_inds = prog.u_inds(:,j);
  Q(qd_inds,fext_inds) = P;
  Q = Q'+Q;
  b = sparse(prog.num_vars,1);
  Q(prog.x_inds(7:9,j),prog.x_inds(7:9,j))=.1*eye(3);
  prog = prog.addConstraint(QuadraticConstraint(-Inf,0,Q,b));
end

% TODO - FIND MORE!!!
% 2 - forces must be smooth = |derivative| of u must be < epsilon
% 3 - magnitudes must in a certain range

prog = prog.addRunningCost(@cost);
prog = prog.addFinalCost(@finalCost);

tf0 = .5*(minimum_duration+maximum_duration);
traj_init.x = ConstantTrajectory(zeros(getNumStates(r),1));
traj_init.u = ConstantTrajectory(zeros(getNumInputs(r),1));

display('Running flying brick traj opt...');
info=0;
while (info~=1)
  tic
  [bricktrajangular,forcetraj,~,~,info] = prog.solveTraj(tf0,traj_init);
  toc
end

% converts the angular velocities back to rpydot
bb = bricktrajangular.eval(bricktrajangular.getBreaks());
for i=1:size(bb,2)
  bb(10:12,i) = angularvel2rpydot(bb(4:6,i),bb(10:12,i)); 
end
bricktraj = PPTrajectory(spline(bricktrajangular.getBreaks(),bb));

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
