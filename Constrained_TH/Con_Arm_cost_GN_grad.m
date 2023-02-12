function [Ftot,gradFtot,zsim] = Con_Arm_cost_GN_grad(U,z0,parameters)

N = parameters.N;
theta = parameters.theta;

%It was decided to use a block diagonal matrix V composed by the diagonal
%matrices diagQ, Qf and diagR, equivalent to the Con_Arm_cost_GN case
V = parameters.V;

circle.x=parameters.circle.x;
circle.y=parameters.circle.y;
circle.r=parameters.circle.r;

a1 = parameters.a1;
a2 = parameters.a2;
a3 = parameters.a3;


zsim = zeros(6*(N+1),1);
zsim(1:6,1) = z0;

zsim(7:end,1) = parameters.M*U + parameters.L*z0;
coords = EFEquation(zsim(7:end,1),parameters);

F  = [zeros(3*N-3,1);  %3N-3
      zeros(3,1);      %3 
      zeros(3*N,1)];       %3N    = 6N %NEW

jacobF = zeros(6*N,3*N);   %jacobian of F         
jacobG = zeros(3,3*N);     %jacobian of g
jacobH = zeros(N,3*N);    %jacobian of h (only circle)
  
g = zeros(3,1); 
h = zeros(N,1); %N from obstacole avoidance (just from EE), 
                    
% Cost function generation f = z'Qz + zf'Qfzf + U'RU    
F(1:end,1)= sqrt(V)*[(repmat(theta,N-1,1) - zsim(7:2:end-6,1)); ...
                     (theta - zsim(end-5:2:end,1)); ...
                      U];

% Jacobian of F generation
jacobF(1:3*N-3,:) = -sqrt(V(1:3*N-3,1:3*N-3))*parameters.M(1:2:end-6,:);
jacobF(3*N-2:3*N,:) = -sqrt(V(3*N-2:3*N,3*N-2:3*N))*parameters.M(end-5:2:end,:);
jacobF(3*N+1:6*N,:) = sqrt(V(3*N+1:end,3*N+1:end));


g(1:3,1) = [zsim(end-4);
            zsim(end-2);
            zsim(end)];

jacobG(:,:) = parameters.M(end-4:2:end,:);

% Circle Constrain
h(1:N,1)  = (coords(1:2:end) - circle.x*ones(N,1)).^2 + (coords(2:2:end) - circle.y*ones(N,1)).^2 - (circle.r*ones(N,1)).^2;

%jacobian of circle constrain
for i = 1:N
    z = zsim(i*6+1:(i+1)*6);
    deltax = 2*(coords((i-1)*2+1) - circle.x);
    deltay = 2*(coords((i-1)*2+2) - circle.y);
    vectemp = [deltax*(-a1*sin(z(1))-a2*sin(z(1)+z(3))-a3*sin(z(1)+z(3)+z(5)))+deltay*(a1*cos(z(1))+a2*cos(z(1)+z(3))+a3*cos(z(1)+z(3)+z(5)));
               0;
               deltax*(-a2*sin(z(1)+z(3))-a3*sin(z(1)+z(3)+z(5)))+deltay*(a2*cos(z(1)+z(3))+a3*cos(z(1)+z(3)+z(5)));
               0;
               deltax*(-a3*sin(z(1)+z(3)+z(5)))+deltay*(a3*cos(z(1)+z(3)+z(5)));
               0];
               
    jacobH(i,:) = vectemp'*parameters.M((i-1)*6+1:i*6,:);
end 


gradFtot = [jacobF;jacobG;jacobH]';
Ftot = [F;g;h];
end

