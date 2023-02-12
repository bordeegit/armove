function [f,zsim] = Arm_cost(U, z0, parameters)
%ARM_COST Cost function of the optimization problem

N = length(U)/3;

zsim = zeros(6*(N+1),1);
zsim(1:6,1) = z0;
ztemp = z0;

Q = parameters.Q;
Qf = parameters.Qf;
R = parameters.R;
Ts = parameters.Ts;


F  = [zeros(2*N,1);
      zeros(2,1);
      zeros(3*N,1)]; % for U  
  
F_z = [zeros(3*N,1);
       zeros(3,1)];

for ind = 2:N+1
    [zdot] = ArmEquation(ztemp, U((ind-2)*3+1:(ind-1)*3,1));
    
    ztemp = ztemp + Ts*zdot;
    zsim((ind-1)*6+1:ind*6,1) = ztemp;
    
    F((ind-2)*2+1:(ind-1)*2,1) = sqrt(Q)*(parameters.obj - EFEquation(ztemp,parameters));
    %Limiting Joints Accelerations
    F(2*N+2+(ind-2)*3+1:2*N+2+(ind-1)*3) = sqrt(R)*U((ind-2)*3+1:(ind-1)*3,1);
    %Limiting Joints Velocities
    F_z((ind-2)*3+1:(ind-1)*3,1) = sqrt(100*R)*([ztemp(2);ztemp(4);ztemp(6)]);
    
end

F(2*N+1:2*N+2) = sqrt(Qf)*(parameters.obj - EFEquation(ztemp,parameters));
F_z(3*N+1:3*N+3) = sqrt(100*R)*[ztemp(2);ztemp(4);ztemp(6)]; %final speed to 0

 
[~,B]=barrier_cost_GN(zsim, parameters);


Ftot = [F;F_z;B];
f = Ftot'*Ftot;
