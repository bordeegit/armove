function [J]=jacob(q,parameters)
%JACOB Computes the jacobian of the End Effector position

a2=parameters.a1;
a3=parameters.a2;
a4=parameters.a3;
theta2=q(1,1);
theta3=q(2,1);
theta4=q(3,1);

c2=cos(theta2);
s2=sin(theta2);
c23=cos(theta2+theta3);
s23=sin(theta2+theta3);
c234=cos(theta2+theta3+theta4);
s234=sin(theta2+theta3+theta4);

J= [-(a4*s234+a3*s23+a2*s2),    -(a4*s234+a3*s23),  -a4*s234;...
    a4*c234+a3*c23+a2*c2,       a4*c234+a3*c23,     a4*c234];


