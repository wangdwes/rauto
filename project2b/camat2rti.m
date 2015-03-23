
function [rot, tr, ip] = camat2rti(camat)

  % extract translation and rotation from a camera matrix, using qr decomposition.
  % code adopted from william's code. ultimate source: http://ksimek.github.io/2012/08/14/decompose/ 
  [rot, ip] = qr(flipud(camat(:, 1: 3))'); 
  [rot, ip] = deal(flipud(rot'), rot90(ip', 2)); 
  % force diagonals of ip to be positive.
  rot = bsxfun(@times, rot, sign(diag(ip)));
  ip = bsxfun(@times, ip, sign(diag(ip))'); 

  % recover the translation
  [~, ~, eigenvecs] = svd(camat);
  [tr] = -rot * eigenvecs(1: 3, end) / eigenvecs(end); 

end
