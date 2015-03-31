% Gets rotation and translation from the essential matrix
%	E - Essential matrix
%	K1 - Intrinsic matrix of camera 1
%	K2 - Intrinsic matrix of camera 2
%	p1 - A point correspondence in camera 1 image
%	p2 - A point correspondence in camera 2 image
%	R - Estimated rotation matrix from camera 1 to camera 2
%   t - Estimated translation vectro from camera 1 to camera 2

function [R, t] = getRt(E, K1, K2, p1, p2)
    [U, S, V] = svd(E);
    W = [0 -1 0; 1 0 0; 0 0 1];

    % If det(R)==-1, we inverse sign of E to make det(R)==1
    if (det(U*W*V') < 0 && det(U*W'*V') < 0)
        [U, S, V] = svd(-E);
    end

    Rs = {U*W*V', U*W'*V'};
    tx = V*W*S*V';
    t = [tx(3,2) tx(1,3) tx(2,1)]'; % Vee operation to recover t from skew-symmetry
    ts = {t, -t};

    % Find the correct R and t (only one pair will have Z>0)
    for iR=1:length(Rs)
        R = Rs{iR};
        for it=1:length(ts)
            t = ts{it};
            M1 = K1 * [eye(3,3) [0 0 1]'];
            M2 = K2 * [R t];
            P = triangulate(M1, p1, M2, p2);
            if (det(R) > 0 && t(1) < 0 && P(3) > 0)
                t = t / t(3);
                return;
            end
        end
    end
    fprintf('Error: no R and t results...\n');
end