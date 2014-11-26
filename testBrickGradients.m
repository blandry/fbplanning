function testBrickGradients()

options.floating = true;
p = RigidBodyManipulator('GliderBalanced.urdf',options);
r = FlyingBrickPlant(p);

% some random states to test
x = 100*rand(12,10);
u = 100*rand(getNumInputs(r),10);

for i=1:size(x,2)
  f1=cell(1,2);
  [f1{:}]=geval(1,@r.dynamics,0,x(:,i),u(:,i),struct('grad_method','user'));

  f2=cell(1,2);
  [f2{:}]=geval(1,@r.dynamics,0,x(:,i),u(:,i),struct('grad_method','numerical'));
  
  if (any(any(abs(f1{1}-f2{1}))>1e-5))
    error('xdot when computing gradients don''t match!');
  end
  if (any(any(abs(f1{2}-f2{2})>1e-5)))
    error('gradients don''t match!');
  end
  
end


end

