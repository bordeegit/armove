function [F, X, Y] = ell_gen(theta,a,origin)

a1=a(1);
a2=a(2);
a3=a(3);

p(1,1)=origin(1)+a1*cos(theta(1));
p(2,1)=origin(2)+a1*sin(theta(1));
p(1,2)=p(1,1)+a2*cos(theta(1)+theta(2));
p(2,2)=p(2,1)+a2*sin(theta(1)+theta(2));
p(1,3)=p(1,2)+a3*cos(theta(1)+theta(2)+theta(3));
p(2,3)=p(2,2)+a3*sin(theta(1)+theta(2)+theta(3));

p_ext(1,:)=[origin(1,1) p(1,1) p(1,2) p(1,3)];

p_ext(2,:)=[origin(2,1) p(2,1) p(2,2) p(2,3)];


[F1, X1, Y1] = ellisse(theta(1,1),p_ext(:,1),a(1,1));
[F2, X2, Y2] = ellisse(theta(2,1)+theta(1,1),p_ext(:,2),a(2,1));
[F3, X3, Y3] = ellisse(theta(3,1)+theta(2,1)+theta(1,1),p_ext(:,3),a(3,1));

F = [F1 F2 F3];
X= [X1 X2 X3];
Y= [Y1 Y2 Y3];


end

