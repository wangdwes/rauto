function coeff = rescoeff(n, r0, rtau, tau)

coeff = zeros(numel(r0) + numel(rtau), n + 1);
% c.f. equation 3.33 - 3.37, section 3.3.1, quadratic optimization on the derivatives of polynomials, 
% adam parker bry, control, estimation, and planning algorithms for aggressive flight using onboard sensing
if ~isempty(r0), coeff(sub2ind(size(coeff), 1: numel(r0), r0 + 1)) = arrayfun(@(r) prod(1: r), r0); end
if ~isempty(rtau), coeff(numel(r0) + 1: end, :) = arrayfun(@(n, r) prod(n - r + 1: n) * tau ^ (n - r), ...
  repmat(0: n, numel(rtau), 1), repmat(rtau', 1, n + 1)); end 

