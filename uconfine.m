 function [cu] = uconfine(u)
        %disp('ENTER IN UCONFINE')
        cu=[0;0;0];
        %约束u(1)
        if u(1)>= 0.5
            cu(1) = 0.5;
        elseif u(1)<= -0.1
            cu(1) = -0.1;
         else
             cu(1) = u(1);
         end
         
         
         %约束u(2)
          if u(2)>= 0.5
            cu(2) = 0.5;
          elseif u(2)<= -0.1
            cu(2) = -0.1;
         else
             cu(2)= u(2);
         end
         
         %约束u(3)
          if u(3)>= 1
            cu(3) = 1;
          elseif u(3)<= 0
            cu(3) = 0;
         else
             cu(3)= u(3);
         end
         
         
 end