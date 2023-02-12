function [zdot] = ArmEquation(z,u)
%ARMEQUATION Computes the state equation of the system

% th1 = z(1,1);
th1d = z(2,1);
% th2 = z(3,1);
th2d = z(4,1);
% th3 = z(5,1);
th3d = z(6,1);

th1dd = u(1,1);
th2dd = u(2,1);
th3dd = u(3,1);

zdot = [th1d;
        th1dd;
        th2d;
        th2dd;
        th3d;
        th3dd];

end
