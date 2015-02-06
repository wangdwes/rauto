px = plantraj(9, 4, wpx, tau);
py = plantraj(9, 4, wpy, tau);
[tj, x, y] = trajval(tau, px, py);
plot(x, y);
