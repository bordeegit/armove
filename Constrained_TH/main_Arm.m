%% Constrained Optimization with Theta Cost Function, Main
clear
close all
clc

%% Model/Control Parameters

% Parameters
parameters.a1 = 0.125;
parameters.a2 = 0.125;
parameters.a3 = 0.200;
z0_init=15*pi/180;    %baricenter of the middle bounds with respect to bounds
parameters.theta_max=[75*pi/180+z0_init;90*pi/180;90*pi/180];
parameters.theta_min=[-75*pi/180+z0_init;-90*pi/180;-90*pi/180];
   %  Less restrictive Constraints
% parameters.theta_max=[360*pi/180;179*pi/180;179*pi/180];
% parameters.theta_min=[-360*pi/180;-179*pi/180;-179*pi/180];
parameters.thetad_max=5.2*[1;1;1];
parameters.thetad_min=-parameters.thetad_max;
parameters.thetadd_max=10*[1; 1; 1];
parameters.thetadd_min=-parameters.thetadd_max;

parameters.origin  =     [0;0];                                 % Origin of the manipolator
z0                  =     [pi/180*85;0;pi/180*0;0;pi/180*0;0];  % Starting Configuration
parameters.obj     =    [0.3; -0.3];                            % Objective Point
% Circle parameters
parameters.circle.x=0.4;
parameters.circle.y=0.4;
parameters.circle.r=0.25;

parameters.FinalPoseMode = 1;                                   %0 to show final configuration during simulation

% Parameters for Kinematic Inversion Algorithm
parameters.NMAX=1000;
parameters.TOL=0.001;

if ~isvalidTargetPoint(parameters.obj,parameters.origin,parameters)
    return;
end

parameters.Ts      = 0.05;                                  % Sampling time
parameters.Tend    = 2;                                     % Total time
parameters.N       = parameters.Tend/parameters.Ts;         % Prediction steps
U0                 = 0*ones(3*parameters.N,1);              % Input initialization                   
parameters.Q       = 0e0*diag([3;2;1]);                     % Tracking error weight                       
parameters.Qf      = 1e3*diag([3;2;1]) ;                    % Terminal weight
parameters.R       = 1e-2*diag([3;2;1]);                    % Input effort weight  
                 

%% Creation of states Matrix, Final Pose and Matrices for fmincon

q = 1*parameters.N;   
[parameters.L,parameters.M,C,d] = matConstruction(parameters.Ts,parameters.N,parameters,z0);
[parameters.theta,~]=inv_kin([z0(1); z0(3); z0(5)],parameters);
diagQ = kron(eye(parameters.N-1),parameters.Q); 
diagR = kron(eye(parameters.N), parameters.R);
parameters.V = blkdiag(diagQ,parameters.Qf,diagR);

%% Optimization Parameters

myoptions               =   myoptimset;
myoptions.Hessmethod  	=	'GN';
myoptions.gradmethod  	=	'UP';       %Select CD or UP
myoptions.graddx     	=	2^-17;      %Optimal for CD
myoptions.tolgrad    	=	1e-6;       %default : 1e-6
myoptions.tolfun        =	1e-12;      %default : 1e-12
myoptions.ls_beta       =	0.8;        %default : 0.8
myoptions.ls_c          =	0.1;        %default : 0.1
myoptions.ls_nitermax   =	20;         %default : 20
myoptions.nitermax      =   200;
myoptions.xsequence     =	'off';
myoptions.GN_sigma      =	0; 
myoptions.QPoptions     =   optimset('Display','none','Algorithm','interior-point-convex'); 
myoptions.GN_funF       =   @(U)Con_Arm_cost_GN(U,z0,parameters); 
if(myoptions.gradmethod == "UP")
    myoptions.GN_funF       =   @(U)Con_Arm_cost_GN_grad(U,z0,parameters);
end
myoptions.outputfcn     =   @(U)EF_traj(U,z0,parameters);

%% Running the optimization routine
tic
[Ustar,fxstar,k,exitflag,~] = myfmincon(@(U)Con_Arm_cost(U,z0,parameters),U0,[],[],C,d,3,q,myoptions);
toc

%% Post-processing the results
[fstar,zstar] = Con_Arm_cost(Ustar,z0,parameters);

print_Sim(zstar,parameters)
% print_Traj(zstar,parameters)
print_Plot(zstar,Ustar,parameters)
