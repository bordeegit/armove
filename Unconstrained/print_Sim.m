function print_Sim(zstar,parameters)
%% Function that prints the simulation of the robot Arm
Ts=parameters.Ts;
N = parameters.N;
coords_obj=parameters.obj;
origin=parameters.origin;

% Unpacking of thetas
theta1   = zstar(1:6:end);
theta2   = zstar(3:6:end);
theta3   = zstar(5:6:end);

% Population of Joint positions, useful for plotting
pos1 = zeros(2,3,N+1); 
for ind = 1:N+1
    [pos1(:,:,ind),~] = manip3dof([theta1(ind,1) theta2(ind,1) theta3(ind,1)]',...
                    [parameters.a1 parameters.a2 parameters.a3]'); 
end   

% Computation of speed of end effector and Plotting
for ind = 2:N+1
    
    figure(1), grid on, hold on
    print3robot(pos1(:,:,ind-1),origin); hold on  % Print of manipulator
    
    for j = 1:ind-1       % Print of end effector trajectory
        plot(pos1(1,3,j)+origin(1,1),pos1(2,3,j)+origin(2,1),'+r'); 
    end
    
    %Print of Ellipsoides
    ell_gen([theta1(ind-1,1);theta2(ind-1,1);theta3(ind-1,1)],...
             [parameters.a1;parameters.a2;parameters.a3],...
             origin);
    %Print of Circles
    c_gen([theta1(ind-1,1);theta2(ind-1,1);theta3(ind-1,1)],...
          [parameters.a1;parameters.a2;parameters.a3],...
           origin);

    plot(coords_obj(1,1),coords_obj(2,1),'d'); %Print of target point

    title(['Time: ' num2str((ind-1)*Ts) ' s'])
    axis([-0.6 0.6  -0.6 0.6 ]), axis equal;
    pause(1e-6);
    hold off
end

end