function print_Traj(zstar,parameters)
%% Plot ONLY the EF trajectory and circle, without full simulation
N = parameters.N;
coords_obj=parameters.obj;
origin=parameters.origin;

% Unpacking of thetas
theta1   = zstar(1:6:end);
theta2   = zstar(3:6:end);
theta3   = zstar(5:6:end);

EF = zeros(2,(N+1)); 
for ind = 1:N+1
    [~,EF(:,ind)] = manip3dof([theta1(ind,1) theta2(ind,1) theta3(ind,1)]',...
                    [parameters.a1 parameters.a2 parameters.a3]'); 
end   

figure(2) 

plot(EF(1,:)+origin(1,1),EF(2,:)+origin(2,1),'+r'); hold on 
plot(coords_obj(1,1),coords_obj(2,1),'d'); hold on
title('Trajectory of End Effector')
xlim([-0.7,0.7]), ylim([-0.7,0.7]), xlabel('X(m)'), ylabel('Y(m)'), axis equal , grid on
hold off

