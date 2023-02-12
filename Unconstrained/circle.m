function [X,Y] = circle(p, r)
%CIRCLE plot a circunference centered in the joint position with radius r


Xplot = r*cos(0:0.1:2*pi) - r*sin(0:0.1:2*pi);
Yplot = r*cos(0:0.1:2*pi) + r*sin(0:0.1:2*pi);
Xplot = Xplot + p(1,1) ;
Yplot = Yplot + p(2,1) ;

plot(Xplot,Yplot),hold on;

X = r*cos(0:0.7:2*pi) - r*sin(0:0.7:2*pi);
Y = r*cos(0:0.7:2*pi) + r*sin(0:0.7:2*pi);
X = X + p(1,1) ;
Y = Y + p(2,1) ;

plot(X,Y, '--g'),hold on;

end

