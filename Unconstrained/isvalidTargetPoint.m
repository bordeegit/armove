function output = isvalidTargetPoint(TP, origin, parameters)
% Boolean function to stop exectution in case of invalid taget point,
% returns true if the point is valid, false if it's invalid
 
if ((TP(1)-origin(1))^2+(TP(2)-origin(2))^2) <= (parameters.a1+parameters.a2+parameters.a3)^2
    fprintf('Valid Target Point!\n')
    output = true;
else
    fprintf('Invalid Target Point for the current manipulator configuration\nInsert a Target Point in the operating condition and try again.\n');
    output = false;
end

end