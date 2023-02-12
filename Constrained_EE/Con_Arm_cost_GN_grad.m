function [Ftot,gradFtot,zsim] = Con_Arm_cost_GN_grad(U,z0,parameters)

N = parameters.N;
Q = parameters.Q;
Qf = parameters.Qf;

diagQ = parameters.diagQ;
diagR = parameters.diagR;

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

F  = [zeros(2*N-2,1); 
      zeros(2,1);     
      diagR*U];       
  
g = zeros(3,1); %end speed equal to 0
h = zeros(N,1); %N from obstacole avoidance (just from EE), 

jacobF = zeros(5*N,3*N);   %jacobian of F
jacobG = zeros(3,3*N);     %jacobian of g
jacobH = zeros(N,3*N);     %jacobian of h (only circle)
                   
                    
% Cost function generation f = Q' e_EndEff Q + Qf' e_EndEff Qf + U' R U
F(1:2*N-2)= diagQ*(repmat(parameters.obj,N-1,1) - coords(1:end-2));
F(2*N-1:2*N) = Qf*(parameters.obj - coords(end-1:end));

% Jacobian of F  and Jacobian of H (circle constraint) in order not to 
%  recompute dx and dy
for i = 1:N
    z = zsim(i*6+1:(i+1)*6);
    dx = [-a1*sin(z(1))-a2*sin(z(1)+z(3))-a3*sin(z(1)+z(3)+z(5));
          0;
          -a2*sin(z(1)+z(3))-a3*sin(z(1)+z(3)+z(5));
          0;
          -a3*sin(z(1)+z(3)+z(5));
          0];

    dy = [a1*cos(z(1))+a2*cos(z(1)+z(3))+a3*cos(z(1)+z(3)+z(5));
          0;
          a2*cos(z(1)+z(3))+a3*cos(z(1)+z(3)+z(5));
          0;
          a3*cos(z(1)+z(3)+z(5));
          0];

    if (i == N)
        jacobF(2*(i-1)+1,:) = -Qf(1,1)*dx'*parameters.M((i-1)*6+1:i*6,:);
        jacobF(2*i,:) = -Qf(2,2)*dy'*parameters.M((i-1)*6+1:i*6,:);
    else
        jacobF(2*(i-1)+1,:) = -Q(1,1)*dx'*parameters.M((i-1)*6+1:i*6,:);
        jacobF(2*i,:) = -Q(2,2)*dy'*parameters.M((i-1)*6+1:i*6,:);
    end

    % circle constrain 
    deltax = 2*(coords((i-1)*2+1) - circle.x);
    deltay = 2*(coords((i-1)*2+2) - circle.y);
    vectemp = deltax*dx + deltay*dy;
    jacobH(i,:) = vectemp'*parameters.M((i-1)*6+1:i*6,:);
end 

jacobF(2*N+1:5*N,:) = diagR;

% Constrain on final speed
g(1:3,1) = [zsim(end-4);
            zsim(end-2);
            zsim(end)];

jacobG = full(parameters.M(end-4:2:end,:));

% Circle Constrain -- OK
h(1:N,1)  = (coords(1:2:end) - circle.x*ones(N,1)).^2 + (coords(2:2:end) - circle.y*ones(N,1)).^2 - (circle.r*ones(N,1)).^2;


gradFtot = [jacobF;jacobG;jacobH]';
Ftot = [F;g;h];
end

