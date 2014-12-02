
megaclear();

options.floating = true;
p = RigidBodyManipulator('GliderBalanced.urdf',options);

options.N = 21;
options.minimum_duration = .1;
options.maximum_duration = 4;
options.x0 = [-3.3 0 .4 0 0 0 0  7 0 0 0 0 0 0]';
options.xf = [0 0 0 0 -pi/4 0 0  0 0 -1 0 0 0 0]';

% 1- SOLVE FOR A TRAJECTORY USING A BRICK
[bricktraj, forcetraj] = runFlyingBrickPlanning(p,options);
% data = load('brick.mat');
% bricktraj = data.bricktraj;
% forcetraj = data.forcetraj;

% sanity check of the brick plan
% breaks = bricktraj.getBreaks();
% xx = bricktraj.eval(breaks);
% xtraj = PPTrajectory(spline(breaks,[xx(1:6,:);zeros(1,numel(breaks));xx(7:12,:);zeros(1,numel(breaks))]));
% xtraj = setOutputFrame(xtraj,getStateFrame(p));
% v = p.constructVisualizer();
% drawForceTraj(p,xtraj,forcetraj);
% v.playback(xtraj,struct('slider',true));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% 2- SOLVE THE REVERSE KINEMATICS PROBLEM
t = bricktraj.getBreaks();
brickpos = bricktraj.eval(t);
forces = forcetraj.eval(t);
w = warning('off','Drake:TaylorVar:DoubleConversion');
xx = reverseKinBrick(p,brickpos,forces);
warning(w);
xtraj0 = PPTrajectory(spline(t,xx));

% sanity check of the reverse kin for the forces
% xtraj0 = setOutputFrame(xtraj0,getStateFrame(p));
% v = p.constructVisualizer();
% v.playback(xtraj0,struct('slider',true,'visualized_system',p));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% 3- SOLVE FULL TRAJECTORY USING THE PREVIOUS RESULTS AS SEED
options.xtraj0 = xtraj0;
[xtraj, utraj] = runTrajOpt(p,options);

% display the final trajectory
v = p.constructVisualizer();
v.playback(xtraj,struct('slider',true));

% simulates the final trajectory
