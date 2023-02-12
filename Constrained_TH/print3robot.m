function print3robot(p,origin)
% make a 2D print of a 3 d.o.f. planar robot taking as input a matrix containing
% all the joints coordinates [p1 p2 p3] and the origin  of the robot.
% Origin and each point are a colum vector containing [x,y]' coordinate
% of the point

p1 = p(:,1)+origin;
p2 = p(:,2)+origin;
p3 = p(:,3)+origin;

hold off
plot(p1(1,1),p1(2,1),'*'),grid on, axis equal, hold on,xlabel('X (m)'),ylabel('Y (m)')
plot(p2(1,1),p2(2,1),'*'), hold on
plot(p3(1,1),p3(2,1),'*'), hold on
plot([origin(1,1) p1(1,1)],[origin(2,1) p1(2,1)],'-k*','LineWidth',1.5),hold on
plot([p1(1,1) p2(1,1)],[p1(2,1) p2(2,1)],'-k*','LineWidth',1.5),hold on
plot([p2(1,1) p3(1,1)],[p2(2,1) p3(2,1)],'-k*','LineWidth',1.5),hold on

end

