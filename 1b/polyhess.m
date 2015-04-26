function h = polyhess(n, r, tau)

% c.f. equation 3.30, section 3.3.1, quadratic optimization on the derivatives of polynomials, 
% adam parker bry, control, estimation, and planning algorithms for aggressive flight using onboard sensing
qril = @(i, l) prod(prod(bsxfun(@minus, [i, l], [0: r - 1]'))) / (i + l - 2 * r + 1) * tau ^ (i + l - 2 * r + 1);
[ig, lg] = meshgrid(r: n); h = 2 * blkdiag (zeros(r), arrayfun(qril, ig, lg));

