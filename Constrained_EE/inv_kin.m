function[q,error,n]=inv_kin(q0,parameters)
    xref= parameters.obj1;
    TOL=parameters.TOL;
    N_MAX=parameters.NMAX;
    q=zeros(3,1);
    x=zeros(2,1);
    k=1*eye(2);   % vector of P action of the loop t.f.
    %N_MAXMAX=N_MAX*100;
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
    

