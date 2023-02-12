function [F,X,Y] = ellisse(theta,p,a)
%% ellisse, will plot a single elipse starting from joint positions p, and lenght a -length of the joints-, oriented as theta.

a = a/2;
b = 0.05/2;
c = (a^2 - b^2)^(1/2); %% fuochi
global Xplot Yplot;

F1(1,1) = p(1,1) + (a - c) * cos(theta); 
F1(2,1) = p(2,1) + (a - c) * sin(theta);

F2(1,1) = p(1,1) + (a + c) * cos(theta); 
F2(2,1) = p(2,1) + (a + c) * sin(theta);

Xplot = a*cos(0:0.2:2*pi)*cos(theta) - b*sin(0:0.2:2*pi)*sin(theta);
Yplot = a*cos(0:0.2:2*pi)*sin(theta) + b*sin(0:0.2:2*pi)*cos(theta);
Xplot = Xplot + p(1,1) + a*cos(theta);
Yplot = Yplot + p(2,1) + a*sin(theta);

plot(Xplot,Yplot),hold on;
plot(F1(1,1),F1(2,1),'*'),hold on
plot(F2(1,1),F2(2,1),'*'),hold on

X = a*cos(0:0.7:2*pi)*cos(theta) - b*sin(0:0.7:2*pi)*sin(theta);
Y = a*cos(0:0.7:2*pi)*sin(theta) + b*sin(0:0.7:2*pi)*cos(theta);
X = X + p(1,1) + a*cos(theta);
Y = Y + p(2,1) + a*sin(theta);

plot(X,Y, '--r'),hold on;


F = [F1; F2];

end

