%% Unconstrained Optimization project, Main
clear
close all
clc

%% Model/Control Parameters

parameters.a1           = 0.125;                                            % Link1 length
parameters.a2           = 0.125;                                            % Link2 length
parameters.a3           = 0.200;                                            % Link3 length
z0_init=90*pi/180;                                                          % Simmetry point for first Joint
parameters.theta_max    = [75*pi/180+z0_init;90*pi/180;90*pi/180];          % Constraints on theta
parameters.theta_min    = [-75*pi/180+z0_init;-90*pi/180;-90*pi/180];                  
parameters.thetad_max   = 5.2*[1;1;1];                                      % Constraints on theta dot
parameters.thetad_min   = -5.2*[1;1;1];
parameters.origin       = [0; 0];                                           % Origin of the manipolator
parameters.obj          = [-0.3; -0.1];                                     % Desired final position

%% Check feasibility of final target point
if ~isvalidTargetPoint(parameters.obj,parameters.origin,parameters)
    return;
end

%% Optimization problem weights and initialization parameters

parameters.Ts           = 0.05;                                             % Sampling time
parameters.Tend         = 2;                                                % Total time
parameters.N            = parameters.Tend/parameters.Ts;                    % Prediction steps
U0                      = +0.45*ones(3*parameters.N,1);                     % Input initialization            
z0                      = [pi/2;0;0;0;0;0];                                 % State initialization
parameters.Q            = 1e2*eye(2);                                       % Tracking error weight 
parameters.R            = 1e-2*diag([3;2;1]);                               % Input acceleration weight
parameters.Qf           = 1e7*eye(2);                                       % Terminal weight
parameters.alpha        = 1e3*diag([1; 1; 1; 1; 1; 1]);                     % Bound on barrier moltiplies exponential
parameters.delta        = 1.5e1*diag([1; 1; 1; 1; 1; 1]);                   % Bound on barrier inside exponential
 
%% Optimization Parameters

myoptions               = myoptimset;
myoptions.Hessmethod    = 'BFGS';  % Select BFGS or GN
myoptions.gradmethod    = 'CD';
myoptions.graddx        = 2^-17;
myoptions.ls_beta       = 0.2;    
myoptions.ls_c          = 0.1;
myoptions.nitermax      = 200;
myoptions.xsequence 	= 'on';
myoptions.BFGS_gamma 	= 0.1; 
myoptions.GN_sigma      = 0;
myoptions.GN_funF       = @(U)Arm_cost_GN_mex(U,z0,parameters);

%% Generating code (mex files) for faster computation (to be ran only once whenever Tend changes)

codegen Arm_cost -args {U0,z0,parameters} -lang:c++
codegen Arm_cost_GN -args {U0,z0,parameters} -lang:c++

%% Running the optimization routine

tic
[Ustar,fxstar,k,exitflag,xsequence] = myfminunc(@(U)Arm_cost_mex(U,z0,parameters),U0,myoptions);
toc

%% Post-processing the results

[fstar,zstar] = Arm_cost_mex(Ustar,z0,parameters);

print_Sim(zstar,parameters)

print_Traj(zstar,parameters)

print_Plot(zstar,Ustar,parameters)

