% Computes the RQ decomposition from the built-in QR decomposition
% Source: http://ksimek.github.io/2012/08/14/decompose/
%   M - Input square matrix
%   R - Output orthogonal matrix
%   Q - Output upper-triangular matrix
function [R, Q] = rq(M)
    [Q, R] = qr(flipud(M)');
%     R = flipud(R');
%     R = fliplr(R)
    R = rot90(R', 2);

    Q = Q';   
    Q = flipud(Q);