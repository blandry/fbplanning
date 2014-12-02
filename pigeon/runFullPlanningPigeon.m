
megaclear();

options.floating = true;
p = RigidBodyManipulator('pigeon.URDF',options);
p = p.weldJoint('tail_roll');
p = p.weldJoint('tail_yaw');
p = p.weldJoint('left_hip_roll');
p = p.weldJoint('left_hip_pitch');
p = p.weldJoint('left_knee_pitch');
p = p.weldJoint('left_ankle_pitch');
p = p.weldJoint('left_thumb_pitch');
p = p.weldJoint('left_fingers_pitch');
p = p.weldJoint('right_hip_roll');
p = p.weldJoint('right_hip_pitch');
p = p.weldJoint('right_knee_pitch');
p = p.weldJoint('right_ankle_pitch');
p = p.weldJoint('right_thumb_pitch');
p = p.weldJoint('right_fingers_pitch');
p = p.compile();

options.N = 21;
options.minimum_duration = .1;
options.maximum_duration = 4;

frame = getStateFrame(p);
x0 = Point(frame);
xf = Point(frame);
x0.base_x = -3.3;
x0.base_z = .4;
x0.base_xdot = 7;
xf.base_pitch = -pi/4;
xf.base_zdot = -1;
options.x0 = double(x0);
options.xf = double(xf);

% 1- SOLVE FOR A TRAJECTORY USING A BRICK
[bricktraj, forcetraj] = runFlyingBrickPlanning(p,options);

% sanity check of the brick plan
breaks = bricktraj.getBreaks();
xx = bricktraj.eval(breaks);
xtraj = PPTrajectory(spline(breaks,[xx(1:6,:);zeros(1,numel(breaks));xx(7:12,:);zeros(1,numel(breaks))]));
xtraj = setOutputFrame(xtraj,getStateFrame(p));
v = p.constructVisualizer();
drawForceTraj(p,xtraj,forcetraj);
v.playback(xtraj,struct('slider',true));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% 2- SOLVE THE REVERSE KINEMATICS PROBLEM
% t = bricktraj.getBreaks();
% brickpos = bricktraj.eval(t);
% forces = forcetraj.eval(t);
% w = warning('off','Drake:TaylorVar:DoubleConversion');
% xx = reverseKinBrick(p,brickpos,forces);
% warning(w);
% xtraj0 = PPTrajectory(spline(t,xx));

% sanity check of the reverse kin for the forces
% xtraj0 = setOutputFrame(xtraj0,getStateFrame(p));
% v = p.constructVisualizer();
% v.playback(xtraj0,struct('slider',true,'visualized_system',p));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% 3- SOLVE FULL TRAJECTORY USING THE PREVIOUS RESULTS AS SEED
% options.xtraj0 = xtraj0;
% [xtraj, utraj] = runTrajOpt(p,options);
% 
% % display the final trajectory
% v = p.constructVisualizer();
% v.playback(xtraj,struct('slider',true));

% simulates the final trajectory
