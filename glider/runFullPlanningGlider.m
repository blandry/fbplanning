megaclear();

options.floating = true;
p = RigidBodyManipulator('GliderBalanced.urdf',options);

options.N = 21;
options.minimum_duration = .1;
options.maximum_duration = 4;

options.x0lb = [-10 0 .1 0 0 0 0 0 0 0 0 0 0 0]';
options.x0ub = [-5 0 3 0 0 0 0 7 0 0 0 0 0 0]';

options.xlb = [-15 0 -10 0 -pi/2 0 -pi/2 -10 0 -10 0 -100 0 -100]';
options.xub = [15 0 10 0 pi/2 0 pi/2 10 0 100 0 10 0 100]';

options.xflb = [0 0 0 0 -pi/2 0 -pi/2 -10 0 -10 0 -100 0 -100]';
options.xfub = [0 0 0 0 -pi/3 0 pi/2 10 0 10 0 100 0 100]';

% 1- SOLVE FOR A TRAJECTORY USING A BRICK
[bricktraj, forcetraj] = runFlyingBrickPlanning(p,options);

% sanity check of the brick plan
breaks = bricktraj.getBreaks();
xx = bricktraj.eval(breaks);
extra_states = getNumStates(p)-12;
xtraj = PPTrajectory(spline(breaks,[xx(1:6,:);zeros(extra_states/2,numel(breaks));xx(7:12,:);zeros(extra_states/2,numel(breaks))]));
xtraj = setOutputFrame(xtraj,getStateFrame(p));
v = p.constructVisualizer();
drawForceTraj(p,xtraj,forcetraj);
v.playback(xtraj,struct('slider',true));

% verify that the energy decreases
E = zeros(1,options.N);
R = [getMass(p)*eye(3) zeros(3); zeros(3) p.body(2).inertia];
for i=1:options.N
  qd = [xx(7:9,i);rpydot2angularvel(xx(4:6,i),xx(10:12,i))];
  E(i) = 0.5*qd'*R*qd - [p.gravity;zeros(3,1)]'*R*qd;
end
figure(25);
plot(1:options.N,E);
title('Energy of the glider over time');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% % 2- SOLVE THE REVERSE KINEMATICS PROBLEM
t = bricktraj.getBreaks();
brickpos = bricktraj.eval(t);
forces = forcetraj.eval(t);
w = warning('off','Drake:TaylorVar:DoubleConversion');
xx = reverseKinBrick(p,brickpos,forces);
warning(w);
xtraj0 = PPTrajectory(spline(t,xx));

% sanity check of the reverse kin for the forces
xtraj0 = setOutputFrame(xtraj0,getStateFrame(p));
% v = p.constructVisualizer();
v.playback(xtraj0,struct('slider',true,'visualized_system',p));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% 3- SOLVE FULL TRAJECTORY USING THE PREVIOUS RESULTS AS SEED
% options.xtraj0 = xtraj0;
% [xtraj, utraj] = runTrajOpt(p,options);
% 
% % display the final trajectory
% v = p.constructVisualizer();
% v.playback(xtraj,struct('slider',true));

% simulates the final trajectory
% TODO
