function [trajs, t] = mwptraj(coeff, tau),

t = []; trajs = cell(size(coeff)); 
% yeah so i put everything on a single line - bite me! despite the fact that two loops are used, 
% the actual computational burden is not expected to be huge - only ten or twenty iterations needed. 
for idx = 1: numel(tau) - 1, tl = linspace(0, tau(idx + 1)); t = horzcat(t, tl + tau(idx));
  for vi = 1: length(coeff), trajs{vi} = horzcat(trajs{vi}, polyval(coeff{vi}(:, idx), tl)); end, end

