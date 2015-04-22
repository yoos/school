function r = annuity(A, P, n)
    f  = @(r) P/r*(1- (1+r)^-n) - A;
    fp = @(r) -P/r^2*(1- (1+r)^-n) + P/r*n*(1+r)^(-n-1);
    x0 = .04;
    tol = 1e-16;
    r = newtons(f,fp,x0,tol,50)
end
