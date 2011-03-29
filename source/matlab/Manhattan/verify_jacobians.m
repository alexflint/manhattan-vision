function [ err ] = verify_jacobians( f, x0, h )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

default h = 1e-3;

[ y Jy ] = f(x0);
numeric_Jy = numeric_derivative(f, x0, h);

report y;
report Jy;
report numeric_Jy;

if (norm(Jy) > 1e-6)
    err = norm(Jy - numeric_Jy) / norm(Jy);
    disp(['Relative error is: ' num2str(err)]);
else
    err = norm(Jy - numeric_Jy);
    disp(['Absolute error is: ' num2str(err)]);
end

