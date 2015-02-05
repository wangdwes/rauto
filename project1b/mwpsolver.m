function coeff = mwpsolver(n, r, wp, tau)

ofs = (size(wp, 1) - 2) * (r + 1); 
% generate the coefficients for the initial, final and continuity conditions...
sco = @(t, idx) circshift(padarray(rescoeff(n, 0: r, 0: r, t) * (-1) ^ idx, ofs, 'post'), (r + 1) * idx);
amat = circshift(cell2mat(arrayfun(@(t, idx) {sco(t, idx)}, tau(2: end)', 0: size(wp, 1) - 2)), - (r + 1)); 

% generate the waypoint restriction coefficients...
if size(wp, 1) > 2, wpresc = arrayfun(@(t) {rescoeff(n, 0: r, [], t)}, tau(3: end));
  amat = vertcat(amat, padarray(blkdiag(wpresc{:}), [0, n + 1], 'pre')); end

% construct a diagonal block matrix that serves as the coefficients, or the q in quadratic programming, 
% then, construct the boundary conditions, or the b in quadratic programming...
hesscell = arrayfun(@(t) {polyhess(n, r, t)}, tau(2: end)); hess = blkdiag(hesscell{:});
bdcond = vertcat(zeros(ofs, 1), wp(end, :)' * (-1) ^ (size(wp, 1) - 2), reshape(wp(1: end - 1, :)', [], 1));   

% finally, check mask and remove the restrictions that we don't really want.
% and invoke the quadratic programming solver on all the matrices we've got so far.  
lm = reshape(circshift(isnan(wp), 1)', [], 1);
amat(find(lm) + ofs, :) = []; bdcond(find(lm) + ofs, :) = [];
coeff = flipud(reshape(quadprog(hess, [], [], [], amat, bdcond), n + 1, []))';

