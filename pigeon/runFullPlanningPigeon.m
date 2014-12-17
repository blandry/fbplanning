
options.floating = true;
p = RigidBodyManipulator('pigeon.URDF', options);

options.N = 21;
options.minimum_duration = .1;
options.maximum_duration = 4;

n = getNumStates(p);

options.x0lb = [-15 0 .5 0 0 0 zeros(1,29) .001 0 0 0 0 0 zeros(1,29)]';
options.x0ub = [-5 0 5 0 0 0 zeros(1,29) 7 0 0 0 0 0 zeros(1,29)]';

options.xlb = [-15 0 0 0 -pi/3 0 zeros(1,29) .001 0 -10 0 -100 0 zeros(1,29)]';
options.xub = [0 0 5 0 pi/3 0 zeros(1,29) 7 0 10 0 100 0 zeros(1,29)]';

options.xflb = [0 0 .1 0 -pi/2 0 zeros(1,29) .001 0 0 0 0 0 zeros(1,29)]';
options.xfub = [0 0 .1 0 -pi/3 0 zeros(1,29) .001 0 0 0 0 0 zeros(1,29)]';

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

% % % 2- SOLVE THE REVERSE KINEMATICS PROBLEM
% w = warning('off','Drake:TaylorVar:DoubleConversion');
% [xtraj0,utraj0] = reverseKinBrickTraj(p,bricktraj,forcetraj,options);
% warning(w);
% t = bricktraj.getBreaks();
% brickpos = bricktraj.eval(t);
% forces = forcetraj.eval(t);
% w = warning('off','Drake:TaylorVar:DoubleConversion');
% xx = reverseKinBrick(p,brickpos,forces);
% warning(w);
% xtraj0 = PPTrajectory(spline(t,xx));
% 
% % sanity check of the reverse kin for the forces
% xtraj0 = setOutputFrame(xtraj0,getStateFrame(p));
% % v = p.constructVisualizer();
% v.playback(xtraj0,struct('slider',true,'visualized_system',p));