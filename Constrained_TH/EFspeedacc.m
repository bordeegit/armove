function [EFspeed] = EFspeedacc(z,u,parameters)
% This function computes the speed of the end effector by computing X and Y
% The computation is based on the derivative of the kinematic relationship
a1 = parameters.a1;
a2 = parameters.a2;
a3 = parameters.a3;

th1 = z(1:6:end,1);
th1d = z(2:6:end,1);
th2 = z(3:6:end,1);
th2d = z(4:6:end,1);
th3 = z(5:6:end,1);
th3d = z(6:6:end,1);

c1=cos(th1);
s1=sin(th1);
c12=cos(th1+th2);
s12=sin(th1+th2);
c123=cos(th1+th2+th3);
s123=sin(th1+th2+th3);

th1dd = [0; u(1:3:end,1)];
th2dd = [0; u(2:3:end,1)];
th3dd = [0; u(3:3:end,1)];

EFspeedxy = [-(a3*s123+a2*s12+a1*s1).*th1d-(a3*s123+a2*s12).*th2d-a3*s123.*th3d;
           (a3*c123+a2*c12+a1*c1).*th1d+(a3*c123+a2*c12).*th2d+a3*c123.*th3d];
EFspeed = sqrt((EFspeedxy(1:end/2,1)).^2+(EFspeedxy(end/2+1:end,1)).^2);


