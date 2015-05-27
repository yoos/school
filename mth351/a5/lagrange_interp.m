% MTH 351, Spring 2015
% lagrange_interp.m
% A method for computing the Lagrange interpolant at a provided set of points, using divided differences.
% Parameters:
%   x -- the interpolation node x values
%   y -- the corresponding y values
%   x_out -- the set of x values at which we want to compute the value of the interpolant
%
% Returns:
%   y_out -- a column vector containing the interpolated values corresponding to x_out

function [y_out] = lagrange_interp(x,y,x_out)
    n = numel(x);
    x = x(:); y = y(:); x_out = x_out(:); %this just ensures we have column vectors
    
    
    %divided difference calculation
    DDmatrix = zeros(n);
    DDmatrix(:,1) = y; %first column is just the first order divided differences
    for j = 2:n
        %compute the (j-1)th order divided difference based on the previous column
        DDmatrix(j:end,j) = diff(DDmatrix(j-1:end,j-1)) ./ ( x(j:end) - x(1:end-j+1) ); 
    end  
    
    y_out = zeros(size(x_out));
    %compute value of the Lagrange polynomial at all of the interpolation points x_out
    for j = 1:n
        tmp = DDmatrix(j,j);   %the divided difference coefficient
        for k = 1:j-1
            % we take the jth coefficient and successively multiply by (x-x_k) for k from 1 to j-1
            tmp = tmp .* (x_out - x(k));
        end
        y_out = y_out + tmp;
    end
end