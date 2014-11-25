function runFpmPlanning()

r = GliderForcePlant();

N = 21;
minimum_duration = .1;
maximum_duration = 4;
prog = DircolTrajectoryOptimization(r,N,[minimum_duration maximum_duration]);  

x0 = [-3.5 0.1 7 0]';
xf = [0 0 0 0]';
u0 = [0 0 0 0]';

prog = prog.addStateConstraint(ConstantConstraint(double(x0)),1);
prog = prog.addStateConstraint(ConstantConstraint(double(xf)),N);
prog = prog.addInputConstraint(ConstantConstraint(double(u0)),1);

% CONSTRAINTS ON THE FORCES
% 1 - passive elements = wings can only remove energy
for i=1:N
  xdot_ind = prog.x_inds(3,i);
  zdot_ind = prog.x_inds(4,i);
  F_inds = prog.u_inds(:,i);
  Q = sparse(prog.num_vars,prog.num_vars);
  Q(repmat(xdot_ind,1,numel(F_inds)),F_inds) = 1;
  Q(repmat(zdot_ind,1,numel(F_inds)),F_inds) = 1;
  Q = Q+Q';
  b = sparse(prog.num_vars,1);
  b(xdot_ind) = -r.glider.m*r.glider.g;
  prog = prog.addConstraint(QuadraticConstraint(-Inf,0,Q,b));
end

% 2 - forces must be smooth = derivative of u must be < epsilon

% 3 - magnitudes must in a certain range

prog = prog.addRunningCost(@cost);
prog = prog.addFinalCost(@finalCost);

tf0 = 2;
traj_init.x = PPTrajectory(foh([0,tf0],[double(x0),double(xf)]));
traj_init.u = ConstantTrajectory(u0);

info=0;
while (info~=1)
  tic
  [xtraj,utraj,~,~,info] = prog.solveTraj(tf0,traj_init);
  toc
end

if (info==1)    
  tsamples = linspace(0,xtraj.tspan(2),100);
  xx = xtraj.eval(tsamples);
  uu = utraj.eval(tsamples);
  figure(5);
  plot(xx(1,:),xx(2:3,:));
  figure(6);
  plot(tsamples,uu);
  E = [];
  for i=1:numel(tsamples)
      E = [E,0.5*r.glider.m*(xx(3,i)+xx(4,i))^2+r.glider.m*r.glider.g*xx(2,i)];
  end
  figure(7)
  plot(tsamples,E);
end

end

function [g,dg] = cost(dt,x,u)
  R = eye(4);
  g = u'*R*u;
  dg = [zeros(1,1+numel(x)),2*u'*R];
end

function [h,dh] = finalCost(t,x)
  h = 100*t;
  dh = [100,zeros(1,size(x,1))];
end