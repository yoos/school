% MTH 351, Spring 2015
% cheb_points.m
% A function that returns the roots of the Chebyshev polynomial of degree N
% Parameters:
%   N -- the index of the Chebyshev polynomial
%
% Returns:
%   x -- an array containing the roots of T_N(x) on the interval [-1 1]

function x = cheb_points(N)
    theta = (2*(1:N)-1)*pi/2/N;    %This is just applying the formula from class
    x = cos(theta-pi); 		   %We subtract pi so that the points come out in the right order on [-1,1]
end

