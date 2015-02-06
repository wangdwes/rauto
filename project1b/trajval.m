function [t, varargout] = trajval(tau, varargin)

% first let me say that replacing these craps with some loops might actually make the entire thing faster.
% anyway, the first line generates a bunch of sample time points relative to the beginning of each segment; 
% and then, the second defines a function that evaluates the piecewise polynomials at these time points, 
% while the third runs all these stuff on each individual input.  

t = cell2mat(arrayfun(@(t) {linspace(0, t)'}, tau(2: end)));
val = @(cfm) cell2mat(arrayfun (@(idx) {polyval(cfm(:, idx), t(:, idx))}, [1: numel(tau) - 1]));
varargout = cellfun(val, varargin, 'UniformOutput', false); t = bsxfun(@plus, cumsum(tau(1: end - 1)), t);  

