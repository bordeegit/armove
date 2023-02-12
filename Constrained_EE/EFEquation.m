function [coords] = EFEquation(z,parameters)
%EFEQUATION Computes the vector of x-y coordinate of the end effector,
%given the vector of states z and the physical parameter of the Arm

a1 = parameters.a1;
a2 = parameters.a2;
a3 = parameters.a3;
N = parameters.N;

th1 = z(1:6:end,1);
th2 = z(3:6:end,1);
th3 = z(5:6:end,1);
coords = zeros(2*N,1);

coords(1:2:end) = a1*cos(th1(1:end,1))+a2*cos(th1(1:end,1)+th2(1:end,1))+a3*cos(th1(1:end,1)+th2(1:end,1)+th3(1:end,1));
coords(2:2:end) = a1*sin(th1(1:end,1))+a2*sin(th1(1:end,1)+th2(1:end,1))+a3*sin(th1(1:end,1)+th2(1:end,1)+th3(1:end,1));

end 