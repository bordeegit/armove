function [p,x] = manip3dof(th,a)
% input:  a  DH parameter
%         th joint angles
% output: p  matrix [2 X 3] x,y coordinate of joints coordinates
%         x  end effector [x,y,phi]' 

th1     =      th(1,1);     
th2     =      th(2,1);     
th3     =      th(3,1);

a1      =       a(1,1);     
a2      =       a(2,1);     
a3      =       a(3,1);

c1=cos(th1);
s1=sin(th1);
c2=cos(th2);
s2=sin(th2);
c3=cos(th3);
s3=sin(th3);

%rotation matrix of frame 1 w.r.t. frame 0, first superscript,second subscript 
A_0_1 = [c1     -s1     0   a1*c1 ;...
         s1     c1      0   a1*s1 ;...
         0      0       1     0   ;...
         0      0       0     1   ];
A_1_2 = [c2     -s2     0   a2*c2 ;...
         s2     c2      0   a2*s2 ;...
         0      0       1     0   ;...
         0      0       0     1   ];
A_2_3 = [c3     -s3     0   a3*c3 ;...
         s3     c3      0   a3*s3 ;...
         0      0       1     0   ;...
         0      0       0     1   ];
     
A_0_2 = A_0_1* A_1_2;
A_0_3 = A_0_1* A_1_2 * A_2_3;

p = [A_0_1(1:2,4)  A_0_2(1:2,4)   A_0_3(1:2,4)];
x = A_0_3(1:2,4);
     
end
