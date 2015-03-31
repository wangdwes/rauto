function F = sevenpoint_norm(pts1, pts2, normalization_constant)
    % Size of points
    N = 7;
    % Least square matrix
    A = zeros(N, 9); % N-by-9
    % Container
    F = cell(3, 1);
    
    % Check if seven point pairs exist
    if (size(pts1, 2) ~= N || size(pts2, 2) ~= N)
        fprintf('ERROR: Need seven points!\n');
        F = 0;
        return
    end
    
    % Normalize the input coordinates
    pts1n = pts1 / normalization_constant;
    pts2n = pts2 / normalization_constant;

    % Construct the least square matrix
    for i=1:N
        x  = pts1n(1, i); y  = pts1n(2, i);
        xp = pts2n(1, i); yp = pts2n(2, i);
        A(i, :) = [xp*x, xp*y, xp, yp*x, yp*y, yp, x, y, 1];
    end
    
    % Null basis for A (two vectors since A is 7x9)
    Z = null(A);
    F1 = reshape(Z(:, 1), 3, 3)';
    F2 = reshape(Z(:, 2), 3, 3)';
    
    % Compute det(alpha*F1+(1-alpha)*F2)=0
    syms a
    eq = det(a*F1 + (1-a)*F2);
    out = roots(sym2poly(eq));

    % Save only the real roots and denormalize
    T = [1/normalization_constant, 0, 0; ...
         0, 1/normalization_constant, 0; ...
         0, 0, 1];
    
    count = 0;
    for ir=1:size(out, 1)
        alpha = out(ir);
        if (isreal(alpha))
            Fhat = alpha * F1 + (1-alpha) * F2;
            F{count+1} = T' * Fhat * T;
            count = count + 1;
        end
    end
    
    F = F(1:count);
end