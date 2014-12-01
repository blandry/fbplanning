function drawForceTraj(p,xtraj,forcetraj)

lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton,'force-traj');

breaks = forcetraj.getBreaks();
xx = xtraj.eval(breaks);
ff = forcetraj.eval(breaks);
N = size(ff,2);
numf = numel(p.force);

for j=1:N
  kinsol = doKinematics(p,xx(1:getNumPositions(p),j));
  for i=1:numf
    force = ff(3*(i-1)+1:3*(i-1)+3,j);
    force_element = p.force{i};
    if isprop(force_element,'child_body')
      body_ind = force_element.child_body;
    else
      body_frame = getFrame(p,force_element.kinframe);
      body_ind = body_frame.body_ind;
    end
    pos = forwardKin(p,kinsol,body_ind,zeros(3,1));
    lcmgl.glColor3f(0,0,i/numel(p.force));
    lcmgl.drawVector3d(pos,force);
  end
end
lcmgl.switchBuffers();

end

