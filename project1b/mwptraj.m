function [trajs, t] = mwptraj(coeff, tau)

t = []; trajs = cell(size(coeff)); cumtau = 0; 
% yeah so i put everything on a single line - bite me! despite the fact that two loops are used, 
% the actual computational burden is not expected to be huge - only ten or twenty iterations needed. 
for idx = 1: numel(tau) - 1, 
  tl = linspace(0, tau(idx + 1)); cumtau = cumtau + tau(idx); t = horzcat(t, tl + cumtau);  
  for vi = 1: length(coeff), trajs{vi} = horzcat(trajs{vi}, polyval(coeff{vi}(:, idx), tl)); end, end

