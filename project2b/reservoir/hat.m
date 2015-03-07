
function mat = hat(vec)

% convert a vector to its corresponding skew-symmetric matrix.
mat = [0, -vec(3), vec(2); vec(3), 0, -vec(1); -vec(2), vec(1), 0];
