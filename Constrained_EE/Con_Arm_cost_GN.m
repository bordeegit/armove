function [Ftot,zsim] = Con_Arm_cost_GN(U,z0,parameters)

N = parameters.N;
Q = parameters.Q;
Qf = parameters.Qf;

diagQ = parameters.diagQ;
diagR = parameters.diagR;

circle.x=parameters.circle.x;
circle.y=parameters.circle.y;
circle.r=parameters.circle.r;

zsim = zeros(6*(N+1),1);
zsim(1:6,1) = z0;

zsim(7:end,1) = parameters.M*U + parameters.L*z0;
coords = EFEquation(zsim(7:end,1),parameters);

F  = [zeros(2*N-2,1); 
      zeros(2,1);     
      diagR*U];       
  
g = zeros(3,1); %end speed equal to 0
h = zeros(N,1); %N from obstacole avoidance (just from EE),                    
                    
% Cost function generation f = Q' e_EndEff Q + Qf' e_EndEff Qf + U' R U
F(1:2*N-2)= diagQ*(repmat(parameters.obj,N-1,1) - coords(1:end-2));
F(2*N-1:2*N) = Qf*(parameters.obj - coords(end-1:end));

% Constrain on final speed
g(1:3,1) = [zsim(end-4);
            zsim(end-2);
            zsim(end)];

% Circle Constrain
h(1:N,1)  = (coords(1:2:end) - circle.x*ones(N,1)).^2 + (coords(2:2:end) - circle.y*ones(N,1)).^2 - (circle.r*ones(N,1)).^2;


Ftot = [F;g;h];
end

