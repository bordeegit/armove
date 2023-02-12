function print_Traj(zstar,parameters)
% Plot ONLY the EF trajectory and circle, without full simulation

N = parameters.N;
coords_obj=parameters.obj;

% Unpacking of thetas
theta1   = zstar(1:6:end);
theta2   = zstar(3:6:end);
theta3   = zstar(5:6:end);

EF1 = zeros(2,(N+1)); 
 
for ind = 1:N+1
    [~,EF1(:,ind)] = manip3dof([theta1(ind,1) theta2(ind,1) theta3(ind,1)]',parameters); 
    
end   

figure(2) 

plot(EF1(1,:)+parameters.origin(1,1),EF1(2,:)+parameters.origin(2,1),'+r'); hold on 

plot(coords_obj(1,1),coords_obj(2,1),'d'); hold on
    
th = 0:pi/50:2*pi; %useful for the print of the circle obstacle
xunit = parameters.circle.r * cos(th) + parameters.circle.x;
yunit = parameters.circle.r * sin(th) + parameters.circle.y;
plot(xunit, yunit);
title('Trajectory of End Effector'),
xlim([-0.7,0.7]), ylim([-0.7,0.7]), xlabel('X(m)'), ylabel('Y(m)'), axis equal , grid on
hold off