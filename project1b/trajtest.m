px = mwpsolver(9, 4, [wpx, tau]);
py = mwpsolver(9, 4, [wpy, tau]);
traj = mwptraj({px, py}, tau);
plot(traj{1}, traj{2});
