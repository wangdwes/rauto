function dercf = dercoeff(coeffmat) 

ncf = size(coeffmat, 1); 
% this method creates a three-dimensional numerical array, with its third dimension as the
% waypoint indices, the second as the orders of the derivatives, and the first as the 
% orders of the polynomial terms. note that the resulting coefficients are in descending order
% with leading zeros and they can be fed to polyval directly as individual polynomials.  

cftri = @(coeff) arrayfun(@(idx) coeff(idx), flipud(hankel(ncf: -1: 1, ones(1, ncf))));
ders = @(coeff) cumprod(tril([ones(ncf, 1), flipud(hankel(1: ncf, ones(ncf - 1, 1) * ncf))]), 2) .* cftri(coeff);
dercfcl = cellfun(@(coeff) {ders(coeff)}, num2cell(coeffmat, 1)); dercf = cat(3, dercfcl{:});

