s=mfilename('fullpath');
pth=s(1:findstr(s,'cmu_quad_matlab')-1);
addpath([pth,'cmu_quad_matlab/dry/src/geometry_utils'])
addpath([pth,'cmu_quad_matlab/dry/src/quadrotor_model'])
%% Add path to custom messages
addpath([pth,'cmu_quad_matlab/dry/install_isolated/share/ipc_bridge/matlab'])
% If custom messages reside in a wet workspace:
% addpath('~/path/to/wet/build/lib');
addpath([pth,'cmu_quad_matlab/wet/build/lib']);

%% Example fixed gains
Kp = diag([0.8687; 0.8687; 2.5896; 0.2370; 0.2370; 0.0217]);
Kd = diag([1.0869; 1.0869; 1.0869; 0.0272; 0.0272; 0.0099]);

Kp_pos = Kp(1:3,1:3);
Kd_pos = Kd(1:3,1:3);

Kp_att = Kp(4:6,4:6);
Kd_att = Kd(4:6,4:6);

%% Create the input-output connections to ROS
% Subscribers receive data from ROS via IPC
% Publishers publish data to ROS via IPC

% Call clear each time to make sure that you clean up old pub/sub objects
clear odom_sub imu_sub pd_pub;

% Create a subscriber for a nav_msgs/Odometry message called 'odom'
% The matching node instantiation in the launch file is:
% <node pkg="ipc_bridge"
%       name="matlab_odom"
%       type="nav_msgs_Odometry_publisher"
%       output="screen">
%   <remap from="~topic" to="/alpha/odom"/>
%   <param name="message" value="odom"/>
% </node>
% Note in the above, the parameter odom corresponds to the second argument
odom_sub = ipc_bridge.createSubscriber('nav_msgs', 'Odometry', 'odom');
imu_sub = ipc_bridge.createSubscriber('sensor_msgs', 'Imu', 'imu');
pd_pub = ipc_bridge.createPublisher('quadrotor_msgs', 'PDCommand', 'pd_cmd');

%% Create the structure of the outgoing message (to populate below)
pd_msg = pd_pub.empty();

seq = randi(1000);

%% Send out zero command
pd_msg.header.stamp = 0;
pd_msg.roll = 0;
pd_msg.pitch = 0;
pd_msg.yaw = 0;
pd_msg.thrust = 0.0;
pd_msg.roll_speed = 0;
pd_msg.pitch_speed = 0;
pd_msg.yaw_speed = 0;
pd_msg.kp_roll = Kp_att(1,1);
pd_msg.kp_pitch = Kp_att(2,2);
pd_msg.kp_yaw = Kp_att(3,3);
pd_msg.kd_roll = Kd_att(1,1);
pd_msg.kd_pitch = Kd_att(2,2);
pd_msg.kd_yaw = Kd_att(3,3);
pd_msg.gains_seq = seq;
pd_msg.speeds_seq = seq;
pd_pub.publish(pd_msg);

%% Disconnect the bridge objects (good practice)
clear odom_sub imu_sub pwm_pub pd_pub;
