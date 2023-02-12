function [b_cost,B_cost]= barrier_cost_GN(z, parameters)
%% creates an exponential function to deal with no feasibility of non constrained cost function  
        %INPUT
                %theta=vector of angles position (rad) 
                %alpha, beta= weights;
                %bound=vector of bounded angles

N = parameters.N;

th_max=parameters.theta_max;
thd_max=parameters.thetad_max;
z_max=[ th_max(1,1); thd_max(1,1);...
        th_max(2,1); thd_max(2,1);...
        th_max(3,1); thd_max(3,1)];
th_min=parameters.theta_min;
thd_min=parameters.thetad_min;
z_min=[ th_min(1,1); thd_min(1,1);...
        th_min(2,1); thd_min(2,1);...
        th_min(3,1); thd_min(3,1)];
alpha=diag(repmat(diag(parameters.alpha),N,1));
delta=diag(repmat(diag(parameters.delta),N,1));

% B_max=zeros(6*N,1);
% B_min=zeros(6*N,1);

%(theta-boundaries)
B_max = alpha*exp(-delta*(-z(7:end,1) + repmat(z_max,N,1)));
B_min = alpha*exp(-delta*( z(7:end,1) - repmat(z_min,N,1)));

B_cost= [B_max; B_min];
b_cost= B_cost'*B_cost;

end
