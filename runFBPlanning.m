function runFBPlanning()

options.floating = true;
p = RigidBodyManipulator('GliderBalanced.urdf',options);
r = FlyingBrickPlant(p);

N = 21;
minimum_duration = .1;
maximum_duration = 4;
prog = DircolTrajectoryOptimization(r,N,[minimum_duration maximum_duration]);

x0 = [-3.3 0 .4 0 0 0 7 0 0 0 0 0]';
xf = [0 0 0 0 -pi/4 0 0 0 -1 0 0 0]';
prog = prog.addStateConstraint(ConstantConstraint(double(x0)),1);
prog = prog.addStateConstraint(ConstantConstraint(double(xf)),N);

% CONSTRAINTS ON THE FORCES
% 1 - passive elements = wings can only remove energy
% Energy need to include the rotational velocity...
% for i=1:N
%   xdot_ind = prog.x_inds(7,i);
%   ydot_ind = prog.y_inds(8,i);
%   zdot_ind = prog.x_inds(9,i);
%   F_inds = prog.u_inds(:,i);
%   Q = sparse(prog.num_vars,prog.num_vars);
%   Q(repmat(xdot_ind,1,numel(F_inds)),F_inds) = 1;
%   Q(repmat(ydot_ind,1,numel(F_inds)),F_inds) = 1;
%   Q(repmat(zdot_ind,1,numel(F_inds)),F_inds) = 1;
%   Q = Q+Q';
%   b = sparse(prog.num_vars,1);
%   b(xdot_ind) = -r.m*r.g;
%   b(ydot_ind) = -r.m*r.g;
%   prog = prog.addConstraint(QuadraticConstraint(-Inf,0,Q,b));
% end

% 2 - forces must be smooth = |derivative| of u must be < epsilon
% epsilon = 1;
% lb = -epsilon*ones(4*(N-1),1);
% ub = epsilon*ones(4*(N-1),1);
% prog = prog.addConstraint(FunctionHandleConstraint(lb,ub,1:prog.num_vars,@(x)fodConstraint(x,h_inds,u_inds)));

% 3 - magnitudes must in a certain range
% TODO

prog = prog.addRunningCost(@cost);
prog = prog.addFinalCost(@finalCost);

tf0 = .5*(minimum_duration+maximum_duration);
traj_init.x = PPTrajectory(foh([0,tf0],[double(x0),double(xf)]));
traj_init.u = ConstantTrajectory(zeros(getNumInputs(r),1));

info=0;
while (info~=1)
  tic
  [btraj,ftraj,~,~,info] = prog.solveTraj(tf0,traj_init);
  toc
end

% should use the RBM visualizer
tsamples = linspace(0,btraj.tspan(2),100);
bb = btraj.eval(tsamples);
ff = ftraj.eval(tsamples);

figure(5);
plot(bb(1,:),bb(2:end,:));

figure(6);
plot(tsamples,ff);

% Energy need to include the rotational velocity...
% E = [];
% for i=1:numel(tsamples)
%     E = [E,0.5*r.m*(bb(7,i)+xx(8,i)+xx(9,i))^2+r.glider.m*r.glider.g*xx(2,i)];
% end
% figure(7)
% plot(tsamples,E);

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
