px = mwpsolver(9, 4, wpx, tau);
py = mwpsolver(9, 4, wpy, tau);
pz = mwpsolver(5, 2, wpz, tau);
[traj, tj] = mwptraj({px, py, pz}, tau);
plot3(traj{1}, traj{2}, traj{3});
