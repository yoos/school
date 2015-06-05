% Simple function implementing the composite trapezoid rule with node points
% given by x and the corresponding function values given by y.
% NOTE: we assume that the points x are equally spaced!! 

function I = comp_trap(x,y)
    h = x(2)-x(1);   % assume equal spacing
    I = y(x(1)) + 4*sum(y(x(2:2:end-1))) + 2*sum(y(x(3:2:end-2))) + y(x(end));
    I = h*I/3;
end
