
% megaclear();

options.floating = true;
p = RigidBodyManipulator('GliderBalanced.urdf',options);

options.N = 21;
options.minimum_duration = .1;
options.maximum_duration = 4;
options.x0 = [-3.3 0 .4 0 0 0 0  7 0 0 0 0 0 0]';
options.xf = [0 0 0 0 -pi/4 0 0  0 0 -1 0 0 0 0]';

% 1- Solve for a trajectory using the brick
% [bricktraj, forcetraj] = runFlyingBrickPlanning(p,options);
data = load('brick.mat');
bricktraj = data.bricktraj;
forcetraj = data.forcetraj;

% 2- Solve reverse kinematics problem to get full state
xtraj0 = reverseKinBrick(p,bricktraj,forcetraj);

% 3- Solve full trajectory using previous result as seed
options.xtraj0 = xtraj0;
[xtraj, utraj] = runTrajOpt(p,options);

v = p.constructVisualizer();
v.playback(xtraj,struct('slider',true));