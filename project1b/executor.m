s=mfilename('fullpath');
pth=s(1:findstr(s,'cmu_quad_matlab')-1);
addpath([pth,'cmu_quad_matlab/dry/src/geometry_utils'])
addpath([pth,'cmu_quad_matlab/dry/src/quadrotor_model'])
%% Add path to custom messages
addpath([pth,'cmu_quad_matlab/dry/install_isolated/share/ipc_bridge/matlab'])
% If custom messages reside in a wet workspace:
% addpath('~/path/to/wet/build/lib');
addpath([pth,'cmu_quad_matlab/wet/build/lib']);
% If custom messages reside in a dry workspace:
%addpath('~/path/to/dry/install_isolated/share/ipc_bridge/matlab');

%% Instantiate the model
params = quadrotor_model.quadrotor_parameters();
params.gravity = 9.81;
params.mass = 0.340;
params.inertia = diag([0.00077773; 0.00079183; 0.00112944]);
params.length = 0.1043;

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
%       name="matlabodom"
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

%% Clear pending messages
for i = 1:10
    odom_sub.read(10, false);
    imu_sub.read(10, false);
end

samples = 1500;

err_pos = zeros(3, samples);
err_vel = zeros(3, samples);
err_att = zeros(3, samples);
err_ang = zeros(3, samples);
err_time = zeros(1, samples);

pos_odom = zeros(3,samples);
att_odom = zeros(3,samples);
vel_odom = zeros(3,samples);
agv_odom = zeros(3,samples);

att_imu = zeros(3,samples);
agv_imu = zeros(3,samples);

odom_time = zeros(1,samples);
imu_time = zeros(1,samples);
proc_time = zeros(1, samples);

odom_idx = 1;
imu_idx = 1;
err_idx = 1;

%% Reference

pos_des = [0; 0; 0.2];
vel_des = [0; 0; 0];
att_des = [0; 0; 0];
ang_des = [0; 0; 0];

seq = randi(1000);
sim_mode = true;
tstart = tic;
msgwaitt = 3;

for idx = 1: samples

  % here we're entering the data collection cycle. first we attempt to get a message from the odometry
  % subscriber (persumed equivalent to the waitForMessage method), with a maximum waiting time of
  % three seconds. should such a message be received, convert it to a native data structure. the same 
  % also to the readings from the inertial measurement units. after this snippet of code, we have the
  % measured attitude, position, angular velocity and velocity vectors, from odometry; as well as
  % the attitude, angular velocity readings from the inertial measurement units. then, proceed to 
  % data processing if we have both odometry and imu messages incoming.   

  flatten = @(struct) [struct.x; struct.y; struct.z];
  
  odom_msg = odom_sub.read(msgwaitt, false);
  if ~isempty(odom_msg)
    att_odom(:, odom_idx) = geometry_utils.RToZYX(geometry_utils.QuatToR(odom_msg.pose.pose.orientation));
    pos_odom(:, odom_idx) = flatten(odom_msg.pose.pose.position);
    agv_odom(:, odom_idx) = flatten(odom_msg.twist.twist.angular); 
    vel_odom(:, odom_idx) = flatten(odom_msg.twist.twist.linear);
    odom_time(odom_idx) = toc(tstart); odom_idx = odom_idx + 1; 
  end

  imu_msg = imu_sub.read(msgwaitt, false);
  if ~isempty(imu_msg)
    att_imu(:, imu_idx) = geometry_utils.RToZYX(geometry_utils.QuatToR(imu_msg.orientation)); 
    agv_imu(:, imu_idx) = flatten(imu_msg.angular_velocity);
    imu_time(imu_idx) = toc(tstart); imu_idx = imu_idx + 1;
  end

  if odom_idx > 1 && ~isempty(odom_msg) && imu_idx > 1 && ~isempty(imu_msg) 

    % here we're implementing a non-linear controller as described in: v. position controlled flight mode, 
    % control of complex maneuvers for a quadrotor uav using geometric methods on se(3). to get started,  
    % calculate the position and velocity errors, in equation 17 and 18.  

    err_pos(:, err_idx) = pos_odom(:, odom_idx - 1) - pos_des; 
    err_vel(:, err_idx) = vel_odom(:, odom_idx - 1) - vel_des; 

    % there are four variables to be controlled: x, y, z, yaw. in position controlled flight mode,
    % x, y, z are controlled by design; to properly control yaw, we need to find an appropriate b1c,
    % in equation 22, that is orthogonal to b3c and pointing to the desired heading; then, to calculate
    % hat computed omega, we need to take the differentiation of the computed rotation matrix. 

    b3c = - kpos * epos - kvel * evel - params.mass * ([0; 0; params.gravity] - accd); b3c = b3c / norm(b3c);
    b2c = cross(b3c, [sin(hdgd); cos(hdgd); 0]); b2c = b2c / norm(b2c);
    b1c = cross(b2c, b3c); rotc = [b1c, b2c, b3c]; 
    
    drotc = (rotc - rotchx(err_idx - 1)) / (time - timehx(err_idx - 1));
    agvc = hat(rotc' * drotc); dagvc = (dagv - dagvhx(err_idx - 1)) / (time - timehx(err_idx - 1)); 

    % the following are implementations for equation 19 and 20. the errors for rotational matrix (erot)
    % and angular velocity (eagv) are computed, and the control expressions for force and moments are
    % evaluated. actually i believe that everybody can figure this out, but since they look totally messy,
    % i'm just writing some comments to make the code structure look slightly better. 

    erot = vee((rotc' * rot, rot' * rotc) / 2.0); eagv = agv - rot' * rotc * agvc;  
    force = (kpos * epos + kvel * evel + params.mass * ([0; 0; params.gravity] - accd) * rot * [0; 0; 1]; 
    moments = - krot * erot - kagv * eagv + cross(agv, params.inertia * agv) - ... 
      params.inertia * (hat(agv) * rot' * rotc * agvc - rot' * rotc * dagvc);  

    % now we have the needed force and moments. the next step is to compute the thrusts for each individual motor,
    % compute the corresponding pulse-width modulation (pwm) parameters, and publish some command.  



        e_pos(:,err_idx) = pos_odom(:,odom_idx-1) - pos_des;
        e_vel(:,err_idx) = vel_odom(:,odom_idx-1) - vel_des;

        u_pos(:,err_idx) = -Kp_pos*e_pos(:,err_idx) - Kd_pos*e_vel(:,err_idx);

        %% Ideal attitude inputs
        u_r = (u_pos(1,err_idx)*sin(psi) - u_pos(2,err_idx)*cos(psi))/params.gravity;
        u_p = (u_pos(1,err_idx)*cos(psi) + u_pos(2,err_idx)*sin(psi))/params.gravity;
        u_y = att_des(3);

        %% Bias compensated attitude
        att = [att_imu(1:2,imu_idx-1); psi];

        %% Attiude error
        e_att(:, err_idx) = ...
            geometry_utils.shortest_angular_distance([u_r;u_p;u_y], att);
        e_ang(:, err_idx) = agv_imu(:,imu_idx-1) - ang_des;
        e_time(err_idx) = toc(tstart);

        %% Biased attitude inputs
        phi_cmd = u_r;
        theta_cmd = u_p;
        yaw_delta = e_att(3,err_idx);

        %% Onboard attitude control (for plotting)
        u_att(:,err_idx) = -Kp_att*e_att(:,err_idx) - Kd_att*e_ang(:,err_idx);

        %% Thrust input
        uT(err_idx) = params.mass*(u_pos(3,err_idx) + params.gravity);
        % Simulation thrust input presently differs from real system
        if sim_mode
            th_cmd = uT(err_idx);
        else
            th_cmd = uT(err_idx)/(2*params.mass*params.gravity);
        end

        %% Bias compensated inputs
        pd_msg.header.stamp = e_time(err_idx);
        pd_msg.roll = phi_cmd;
        pd_msg.pitch = theta_cmd;
        pd_msg.yaw = yaw_delta;
        pd_msg.thrust = th_cmd;
        pd_msg.roll_speed = ang_des(1);
        pd_msg.pitch_speed = ang_des(2);
        pd_msg.yaw_speed = ang_des(3);
        pd_msg.kp_roll = Kp_att(1,1);
        pd_msg.kp_pitch = Kp_att(2,2);
        pd_msg.kp_yaw = Kp_att(3,3);
        pd_msg.kd_roll = Kd_att(1,1);
        pd_msg.kd_pitch = Kd_att(2,2);
        pd_msg.kd_yaw = Kd_att(3,3);
        pd_msg.gains_seq = seq;
        pd_msg.speeds_seq = seq;
        pd_pub.publish(pd_msg);

        err_idx = err_idx + 1;
    end
end

%% Send out zero command
pd_msg.header.stamp = e_time(err_idx);
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
% The above is the same as calling the following
% odom_sub.disconnect();
% imu_sub.disconnect();
% pwm_pub.disconnect();
