
addpath('..'); 

% instantiate a navigator. the arguments are the lower bound for x, upper bound for x, 
% lower bound for y, upper bound for y, and a granularity for discretization. all in meters.  
nav = Navigator(-0.4, 0.4, -0.4, 0.4, 0.005); 

% create a circular obstacle at (0, 0) with a radius of 0.2 meter.  
a = nav.createObstacle(0, 0, 0.2); 
imagesc(nav.getArenaImage); % visualize the arena. 
waitforbuttonpress; 

nav.setStart(-0.3, 0); % set the starting position.
nav.setGoal(0.3, 0);   % set the goal position.
b = nav.plan(0.1);     % please get ths done in 0.1 seconds! b is the waypoints.

imagesc(nav.getArenaImage);  
waitforbuttonpress;

c = nav.createObstacle(0.3, 0, 0.1); % create a new obstacle. 
nav.updateObstacle(a, 0, -0.1); % move the obstacle 'a' to (0, 0.1) with radius unchanged.

nav.setGoal(0.3, -0.2); 
d = nav.plan(0.1);
 
imagesc(nav.getArenaImage);  
waitforbuttonpress;
