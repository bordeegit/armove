function print_Plot(zstar,Ustar,parameters)
% Print the plots for position, velocity and acceleration of angles
% and speed of end effector 
Ts=parameters.Ts;
Tend=parameters.Tend;
N=parameters.N;

% Unpacking of thetas
theta1   = zstar(1:6:end);
theta1d  = zstar(2:6:end);
theta1dd = Ustar(1:3:end);
theta2   = zstar(3:6:end);
theta2d  = zstar(4:6:end);
theta2dd = Ustar(2:3:end);
theta3   = zstar(5:6:end);
theta3d  = zstar(6:6:end);
theta3dd = Ustar(3:3:end);

EFspeed = abs(EFspeedacc(zstar,parameters));

T = 1000*(0:Ts:Tend);
subplotnum = 4;
figure(3), grid on
subplot(subplotnum,1,1),plot(T,theta1),title('Angles'),xlabel('Time [ms]'),hold on
               plot(T,theta2),ylabel('[rad]')
               plot(T,theta3),legend('Theta1','Theta2','Theta3'), hold off
subplot(subplotnum,1,2),plot(T,theta1d),title('Speed of Angles'),xlabel('Time [ms]'),hold on
               plot(T,theta2d),ylabel('[rad/s]')
               plot(T,theta3d),legend('Theta1d','Theta2d','Theta3d'), hold off
subplot(subplotnum,1,3),plot(T,[0;theta1dd]),title('Acceleration of Angles'),xlabel('Time [ms]'),hold on
               plot(T,[0;theta2dd]),ylabel('[rad/s^2]')
               plot(T,[0;theta3dd]),legend('Theta1dd','Theta2dd','Theta3dd'), hold off               
subplot(subplotnum,1,4),plot(T,EFspeed,'r'),title('Speed of end effector'),
               xlabel('Time [ms]'),ylabel('[rad/s]')
end

