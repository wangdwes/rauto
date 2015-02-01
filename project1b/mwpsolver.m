function coeff = mwpsolver(n, r, wp)

ofs = (size(wp, 1) - 2) * (r + 1); 
% generate the coefficients for the initial, final and continuity conditions...
usc = @(tau, idx) padarray(rescoeff(n, 0: r, 0: r, tau) * (-1) ^ idx, ofs, 'post');
amat = cell2mat(arrayfun(@(tau, idx) {circshift(usc(tau, idx), (r + 1) * idx)}, wp(2: end, end)', 0: size(wp, 1) - 2)); 

% generate the waypoint restriction coefficients...
wpresc = arrayfun(@(tau) {rescoeff(n, 0: r, [], tau)}, wp(2: end - 1, 2));
amat = vertcat(circshift(amat, -[r + 1, 0]), padarray(blkdiag(wpresc{:}), [0, n + 1], 'pre'));

% construct a diagonal block matrix that serves as the coefficients, or the q in quadratic programming, 
% then, construct the boundary conditions, or the b in quadratic programming...
hesscell = arrayfun(@(tau) {polyhess(n, r, tau)}, wp(2: end, end)); hess = blkdiag(hesscell{:});
bdcond = vertcat(zeros(ofs, 1), wp(end, 1: end - 1)' * (-1) ^ (size(wp, 1) - 2), ...
  reshape(wp(1: end - 1, 1: end - 1)', [], 1));   

% finally, check mask and remove the restrictions that we don't really want.
% and invoke the quadratic programming solver on all the matrices we've got so far.  
lm = reshape(circshift(isnan(wp(:, 1: end - 1)), [1, 0])', [], 1);
amat(find(lm) + ofs, :) = []; bdcond(find(lm) + ofs, :) = [];
coeff = flipud(reshape(quadprog(hess, [], [], [], amat, bdcond), n + 1, [])); 
