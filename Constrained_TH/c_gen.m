function  [X,Y]=c_gen(theta,a, origin, r)
%C_GEN create the circle representing the joint encumbrance
r=[0.035; 0.02; 0.02; 0.02]; 

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

[X1, Y1]=circle(p_ext(:,1), r(1,1));
[X2, Y2]=circle(p_ext(:,2), r(2,1));
[X3, Y3]=circle(p_ext(:,3), r(3,1));
[X4, Y4]=circle(p_ext(:,4), r(4,1));

X= [X1 X2 X3 X4];
Y= [Y1 Y2 Y3 Y4];

end

