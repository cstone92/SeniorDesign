function jointAngles = robotInvKin(x,y,angle,ind)
% This functio computes inverse kinematics for the 2020 breast cancer
% detection device for Dr. Olson's senior design project
% Created by Peter Garnache (2020)

% Constants
indenterLength = 245 + ind;
YOffset = 177.5;

jointAngles(1) = x + indenterLength*sind(angle);
jointAngles(2) = y + indenterLength*cosd(angle) - YOffset;
jointAngles(3) = -angle;
jointAngles(4) = ind;