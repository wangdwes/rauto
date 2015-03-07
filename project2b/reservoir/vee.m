
function vec = vee(mat)

% transform skew-symmetric matrix to the corresponding vector.  
vec = [mat(3, 2); mat(1, 3); mat(2, 1)];
