%% Constrained Optimization with End Effector Cost Function, Main
clear
close all
clc

%% Model/Control Parameters

% Manipulator Parameters and Limits
parameters.a1 = 0.125;
parameters.a2 = 0.125;
parameters.a3 = 0.200;
z0_init=15*pi/180;
parameters.theta_max=[75*pi/180+z0_init;90*pi/180;90*pi/180];
parameters.theta_min=[-75*pi/180+z0_init;-90*pi/180;-90*pi/180];
   %  Less restrictive Constraints
% parameters.theta_max=[179*pi/180;179*pi/180;179*pi/180];
% parameters.theta_min=[-179*pi/180;-179*pi/180;-179*pi/180];
parameters.thetad_max=5.2*[1;1;1];
parameters.thetad_min=-parameters.thetad_max;
parameters.thetadd_max=10*[1; 1; 1];
parameters.thetadd_min=-parameters.thetadd_max; 

parameters.origin  = [0;0];                                  % Origin of the manipolator
z0                 = [pi/180*90;0;pi/180*0;0;pi/180*10;0];   % Starting Configuration
parameters.obj     = [0.3; -0.3];                            % Objective Point
% Circle parameters
parameters.circle.x= 0.4;                                    
parameters.circle.y= 0.4;
parameters.circle.r= 0.30;

if ~isvalidTargetPoint(parameters.obj,parameters.origin,parameters)
    return;
end

parameters.Ts      = 0.05;                                  % Sampling time
parameters.Tend    = 2;                                     % Total time
parameters.N       = parameters.Tend/parameters.Ts;         % Prediction steps  
U0                 = 0*ones(3*parameters.N,1);              % Input initialization                   
parameters.Q       = 1e0*diag([1;1]);                       % Tracking error weight                       
parameters.Qf      = 1e2*diag([1;1]) ;                      % Terminal weight
parameters.R       = 1e-2*diag([3;3;3]);                    % Input effort weight                  
                

%% Creation of states Matrix, Final Pose and Matrices for fmincon

q = 1*parameters.N;   
[parameters.L,parameters.M,C,d] = matConstruction(parameters.Ts,parameters.N,parameters,z0);
parameters.diagQ =  kron(eye(parameters.N-1),parameters.Q);
parameters.diagR = kron(eye(parameters.N),parameters.R);


%% Generating code (mex files) for faster computation

% codegen Con_Arm_cost_GN_grad -args {U0,z0,parameters} -lang:c


%% Optimization Parameters

myoptions               =   myoptimset;
myoptions.Hessmethod  	=	'GN';               %GN
myoptions.gradmethod  	=	'UP';               %CD or UP
myoptions.graddx        =	2^-17;
myoptions.tolgrad    	=	1e-6;               %default : 1e-6
myoptions.tolfun        =	1e-12;              %default : 1e-12
myoptions.ls_beta       =	0.8;                %default : 0.8
myoptions.ls_c          =	0.1;                %default : 0.1
myoptions.ls_nitermax   =	20;                 %default : 20
myoptions.nitermax      =	200;
myoptions.xsequence     =	'off';
myoptions.GN_sigma      =	0;
myoptions.QPoptions     =   optimset('Display','none','Algorithm','interior-point-convex'); 
myoptions.GN_funF       =   @(U)Con_Arm_cost_GN(U,z0,parameters); 
if(myoptions.gradmethod == "UP")
    % Modify to Con_Arm_cost_GN_grad_mex to use the compiled version
    myoptions.GN_funF       =   @(U)Con_Arm_cost_GN_grad(U,z0,parameters);
end
myoptions.outputfcn     =   @(U)EF_traj(U,z0,parameters);


%% Running the optimization routine
tic
[Ustar,fxstar,k,exitflag,xsequence] = myfmincon(@(U)Con_Arm_cost(U,z0,parameters),U0,[],[],C,d,3,q,myoptions);
toc


%% Post-processing the results

[fstar,zstar] = Con_Arm_cost(Ustar,z0,parameters);

print_Sim(zstar,parameters)
% print_Traj(zstar,parameters)
print_Plot(zstar,Ustar,parameters)
