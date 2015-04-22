% MTH 351, Spring 2015
% newtons.m
% A simple implementation of Newton's method for finding a root.
% Parameters:
%   f -- handle to the function for which we want to find a root
%   fp -- handle for computing the derivative
%   x0 -- the starting point
%   tol -- the desired tolerance. The default is 1e-6
%   N   -- the desired number of iterations to run. The default is 50.
%
% Returns:
%   x -- the root found by Newton's method
%   xarray -- the sequence of all x values that were found
% Prints the value of x, f(x), the quadratic ratio (x_{n+1}-x_n)/(x_n - x_{n-1})^2 
% and the linear ratio (x_{n+1}-x_n)/(x_n - x_{n-1}) 

function [x, xarray] = newtons(f,fp,x0, tol, N)

    switch(nargin)
        case 1
            N = 20; tol = 1e-6;     %use default values if the user doesn't specify any
        case 2
            N = inf;                %user wants to guarantee a specified tolerance
        otherwise
            %do nothing
    end
    
    its = 0;                %number of iterations run so far
    
    fx = f(x0); fpx = fp(x0); %compute the first values of f and fp
    x = x0; x_old = inf;    %we set x_old to inf initially so that the while loop will run.
    xarray = [x];             %store the iteration history so that we can examine convergence
    qratio = 0; lratio = 0;
    
    fprintf('   n\t      x\t\t\t      f(x) \t\t     quad. ratio \t\t lin. ratio\n');
    fprintf('-------\t-------------\t-------------\t\t-------------\t    -------------\n')
    while ( abs(x-x_old) > tol && its < N)
        if (its >= 2)
            fprintf('   %d\t%1.8e\t%1.8e\t\t%1.8e\t\t%1.8e\n',its,x,fx,qratio,lratio);	%we can start looking at ratios after 2 iterations
        else
            fprintf('   %d\t%1.8e\t%1.8e\n',its,x,fx);
        end
        x_old = x;
        x = x - fx/fpx;             %update x
        fx = f(x); fpx = fp(x);     %update f and fp
        xarray = [xarray x];
         
        % f(x) is pretty close to zero, so return the root.
        if ( abs(fx) < 1e-20)
            its = its+1;
            return
        end
        its = its+1;
        if its >= 2
            qratio = (xarray(its+1)-xarray(its))/(xarray(its)-xarray(its-1))^2;   %compute the quadratic error ratio
            lratio = (xarray(its+1)-xarray(its))/(xarray(its)-xarray(its-1));	  %compute the linear error ratio
        end
    end
    qratio = (xarray(its+1)-xarray(its))/(xarray(its)-xarray(its-1))^2;   %compute the quadratic error ratio
    lratio = (xarray(its+1)-xarray(its))/(xarray(its)-xarray(its-1));	  %compute the linear error ratio
    fprintf('   %d\t%1.8e\t%1.8e\t\t%1.8e\t\t%1.8e\n',its,x,fx,qratio,lratio);   %print the last line after the loop exits
end