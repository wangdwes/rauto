
function [rot, tr] = camat2rt(camat)

  % extract translation and rotation from a camera matrix, using qr decomposition.
  % code adopted from william's code. ultimate source: http://ksimek.github.io/2012/08/14/decompose/ 
  [rot, intriparams] = qr(flipud(camat(:, 1: 3))'); 
  [rot, intriparams] = deal(flipud(rot'), rot90(intriparams', 2)); 
  % force diagonals of intriparams to be positive. 
  [rot] = bsxfun(@times, rot, sign(diag(intriparams)));
  [intriparams] = bsxfun(@times, intriparams, sign(diag(intriparams))'); 

  % recover the translation
  [~, ~, eigenvecs] = svd(camat); 
  [tr] = -rot * eigenvecs(1: end - 1, end) / eigenvecs(end); 

end
