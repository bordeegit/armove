function [coords] = EFEquation(z,parameters)
% EFEquation Outputs the x and y coordinates of the End Effector, given the
% current configuration

a1 = parameters.a1;
a2 = parameters.a2;
a3 = parameters.a3;

th1 = z(1,1);
th2 = z(3,1);
th3 = z(5,1);

coords = [a1*cos(th1)+a2*cos(th1+th2)+a3*cos(th1+th2+th3);
          a1*sin(th1)+a2*sin(th1+th2)+a3*sin(th1+th2+th3)];
      