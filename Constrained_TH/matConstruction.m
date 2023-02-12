function [l, M, C, d] = matConstruction(Ts,N,parameters,z0)
%MATCONSTRUCTION Creates matrices useful for optimization
% M and l are used to simulate the system
% C and d are used in the definition of the linear inequality constraints

theta_max=parameters.theta_max;
theta_min=parameters.theta_min;
thetad_max=parameters.thetad_max;
thetad_min=parameters.thetad_min;
thetadd_max=parameters.thetadd_max;
thetadd_min=parameters.thetadd_min;

ps=[theta_min(1,1);thetad_min(1,1);theta_min(2,1);thetad_min(2,1);theta_min(3,1);thetad_min(3,1);
    -theta_max(1,1);-thetad_max(1,1);-theta_max(2,1);-thetad_max(2,1);-theta_max(3,1);-thetad_max(3,1)];
p=repmat(ps,N,1);


A = zeros(6,6);
A(1,2) = 1;
A(3,4) = 1;
A(5,6) = 1;
B = zeros(6,3);
B(2,1) = 1;
B(4,2) = 1;
B(6,3) = 1;
I = eye(6);
A_dis = I + Ts*A;
B_dis = Ts*B;

Ns = size(A_dis,2);
Nu = size(B_dis,2);
l = sparse(zeros(Ns*N,Ns));
M = sparse(zeros(Ns*N,Ns*N));
Atmp = eye(Ns);
for k=1:N
	M = M + kron(sparse(diag(ones(N-k+1,1),-k+1)),Atmp);
	Atmp = Atmp*A_dis;
	l((1:Ns)+(k-1)*Ns,:) = Atmp;
end

M = M * kron(sparse(eye(N)),B_dis);

%constraction of U part in C
Mat=[-eye(3); eye(3)];                                                  
Matr = repmat(Mat, 1, parameters.N);                                
Matc = mat2cell(Matr, size(Mat,1), repmat(size(Mat,2),1,parameters.N));    
Cu = blkdiag(Matc{:});  
du=repmat([-thetadd_max; thetadd_min],(parameters.N),1);

%constaction of z part in C
Es=zeros(12,6);
Es(1,1)=1; Es(2,2)=1; Es(3,3)=1; Es(4,4)=1; Es(5,5)=1; Es(6,6)=1; 
Es(7:end,:)=-Es(1:6,:);
Ec = repmat({Es}, 1, N);
E = blkdiag(Ec{:});
Cz=E*M;
dz=p-E*l*z0;

%final C and d
C=[Cz;Cu];
d=[dz;du];
end