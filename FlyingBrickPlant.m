classdef FlyingBrickPlant < DrakeSystem

    properties
      manip;
      g;
      m;
      I;
      Fpos;
    end

    methods
      function obj = FlyingBrickPlant(manip)
        obj = obj@DrakeSystem(12,0,3*numel(manip.force),12);
        obj = setDirectFeedthrough(obj,0);
        obj = setOutputFrame(obj,getStateFrame(obj));
        obj = obj.setTIFlag(true);

        obj.manip = manip;
        obj.g = manip.gravity;
        obj.m = getMass(manip);
        obj.I = manip.body(2).inertia;
        obj.Fpos = zeros(3,numel(manip.force));
        q0 = zeros(getNumPositions(manip),1);
        kinsol = doKinematics(manip,q0);
        for i=1:numel(manip.force)
          force_element = manip.force{i};
          if isprop(force_element,'child_body')
            body_ind = force_element.child_body;
          else
            body_frame = getFrame(manip,force_element.kinframe);
            body_ind = body_frame.body_ind;
          end
          obj.Fpos(:,i) =  forwardKin(manip,kinsol,body_ind,zeros(3,1));
        end
      end

      function [xdot,dxdot] = dynamics(obj,t,x,u)
        forces = reshape(u,3,[]);
        [Phi, dPhi] = angularvel2rpydotMatrix(x(4:6));
        xdot = [x(7:9);
                Phi*x(10:12);
               (1/obj.m)*sum(forces,2)+obj.g;
               obj.I\sum(cross(obj.Fpos,forces,1),2)];
        if (nargout>1)
          dxdot = [zeros(3,7),eye(3),zeros(3,3),zeros(3,numel(u));
                   zeros(3,4),reshape(dPhi(:,1),3,3)*x(10:12),reshape(dPhi(:,2),3,3)*x(10:12),reshape(dPhi(:,3),3,3)*x(10:12),zeros(3,3),Phi*eye(3),zeros(3,numel(u));
                   zeros(3,13),repmat((1/obj.m)*eye(3),1,size(forces,2));
                   zeros(3,13),obj.I\cross(reshape(repmat(obj.Fpos,3,1),3,[]),repmat(eye(3),1,size(forces,2)),1)];
        end
      end

      function [y,dy] = output(obj,t,x,u)
        y = x;
        if (nargout>1)
          dy=[zeros(obj.num_y,1),eye(obj.num_y),zeros(obj.num_y,obj.num_u)];
        end
      end

      function x = getInitialState(obj)
        x = zeros(12,1);
      end
    end

end
