function [Ftot,zsim] = Con_Arm_cost_GN(U,z0,parameters)

N = parameters.N;
Qf = parameters.Qf;
theta = parameters.theta;

diagQ = kron(eye(N-1),Q); %same but faster
diagR = kron(eye(N), R);

circle.x=parameters.circle.x;
circle.y=parameters.circle.y;
circle.r=parameters.circle.r;

zsim = zeros(6*(N+1),1);
zsim(1:6,1) = z0;

zsim(7:end,1) = parameters.M*U + parameters.L*z0;
coords = EFEquation(zsim(7:end,1),parameters);

F  = [zeros(3*N-3,1);
      zeros(3,1);
      diagR*U];  
  
g = zeros(3,1); 
h = zeros(N,1); %N from obstacole avoidance (just from EE), 
             

% Cost function generation f = Q'zQ + Qf'zQf + R'*u*R    
F(1:3*N-3,1) = diagQ*(repmat(theta,N-1,1) - zsim(7:2:end-6,1));   %NEW  
F(3*N-2:3*N) = Qf*(theta - zsim(end-5:2:end,1));

% Final velocities to 0
g(1:3,1) = [zsim(end-4);
          zsim(end-2);
          zsim(end)];

% Circle Constrain
h(1:N,1)  = (coords(1:2:end) - circle.x*ones(N,1)).^2 + (coords(2:2:end) - circle.y*ones(N,1)).^2 -(circle.r*ones(N,1)).^2;

Ftot = [F;g;h];
end
