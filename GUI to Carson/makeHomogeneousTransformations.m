function [A, A1, A2, A3, A4] = makeHomogeneousTransformations(d1, d2, degTheta3, d4)
%MAKEHOMOGENEOUSTRANSFORMATIONS Create the DH matrices for the arm.
A = homogeneousTransformation(0, 0, 90, 90);    % Constant offset for MATLAB's coordinate system (with y as vertical instead of z)
A1 = homogeneousTransformation(0, d1, -90, 0);
A2 = homogeneousTransformation(0, 177.5+d2 , -90, 90);
A3 = homogeneousTransformation(0, 0, -90, degTheta3);
A4 = homogeneousTransformation(0, 245+d4 , 0, 0);
end
