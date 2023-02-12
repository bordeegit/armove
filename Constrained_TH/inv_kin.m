function[q,error,n]=inv_kin(q0,parameters)
%INV_KIN Computes a feasible final configuration of the manipulator q given
%the initial configuration q0 with an Inverse Kinematics Algorithm

xref= parameters.obj;
TOL=parameters.TOL;
N_MAX=parameters.NMAX;
q=zeros(3,1);
k=1*eye(2);   % vector of P action of the loop t.f.
J=jacob(q0,parameters);

[~,x]=manip3dof(q0,parameters);
error=norm(xref(1:2,1)-x(1:2,1),2);
n=0;

while norm(xref(1:2,1)-x(1:2,1),2)>TOL && n<N_MAX
    
    q=q+J'*k*(xref(1:2,1)-x(1:2,1));
    J=jacob(q,parameters);
    [~,x]=manip3dof(q,parameters);
    n=n+1;
    error=[error,norm(xref(1:2,1)-x(1:2,1),2)];
    
end
    

