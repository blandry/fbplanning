classdef GliderForcePlant < DrakeSystem

  % x(1) = center of mass x position
  % x(2) = center of mass z position
  % x(3) = center of mass x velocity
  % x(4) = center of mass z velocity
  % u(1) = x force produced by the wing
  % u(2) = z force produced by the wing
  % u(3) = x force produced by the elevator
  % u(4) = z force produced by the elevator
     
  properties
    glider = GliderPlant();
  end
  
  methods
    function obj = GliderForcePlant()
      obj = obj@DrakeSystem(4,0,4,4);
      obj = setDirectFeedthrough(obj,0);
      obj = setOutputFrame(obj,getStateFrame(obj));
      obj = obj.setTIFlag(true);
    end
    
    function [xdot,df] = dynamics(obj,t,x,u)
      m = obj.glider.m;
      g = obj.glider.g;
      
      F_w = u(1:2);
      F_e = u(3:4);
      
      xdot = [x(3:4);(1/m)*(F_w+F_e-[0;m*g])];
      if (nargout>1)
         df = [zeros(4,1),zeros(4,2),[1 0 0 0]',[0 1 0 0]',[0 0 1/m 0]',[0 0 0 1/m]',[0 0 1/m 0]',[0 0 0 1/m]'];  
      end
      
    end
    
    function [y,dy] = output(obj,t,x,u)
      y = x;
      if (nargout>1)
        dy=[zeros(obj.num_y,1),eye(obj.num_y),zeros(obj.num_y,obj.num_u)];
      end
    end
    
    function x = getInitialState(obj)
      x = [0 0 0 0]';
    end
    
  end  
  
end