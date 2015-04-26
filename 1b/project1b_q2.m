clear all;
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
%Kp = diag([0.7; 0.7; 3; 10; 10; 6]); % Good for vel
%Kd = diag([1.3; 1.3; 10; 4; 4; 1.6]);
Kp = diag([0.7; 0.7; 2; 10; 10; 6]); % Good for overdamped pos
Kd = diag([1.3; 1.3; 4; 4; 4; 1.6]);
%Kp = diag([9.7; 9.7; 6.7; 10; 10; 6]); % Good for pos (slight overshoot)
%Kd = diag([1.3; 1.3; 4; 4; 4; 1.6]);

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

%% Clear pending messages
for i = 1:10
    odom_sub.read(10, false);
    imu_sub.read(10, false);
end

%% Variables
samples = 50000;

e_pos = zeros(3, samples);
e_vel = zeros(3, samples);
e_att = zeros(3, samples);
e_ang = zeros(3, samples);
e_time = zeros(1, samples);

pos_odom = zeros(3,samples);
att_odom = zeros(3,samples);
vel_odom = zeros(3,samples);
ang_odom = zeros(3,samples);

att_imu = zeros(3,samples);
ang_imu = zeros(3,samples);

odom_time = zeros(1,samples);
imu_time = zeros(1,samples);

thrusts = zeros(1,samples);

odom_idx = 1;
imu_idx = 1;
err_idx = 1;

%% Reference points
pos_des = [0; 0; 0.5];
vel_des = [0; 0; 0];
att_des = [0; 0; 0];
ang_des = [0; 0; 0];

% ##########
% Project 1a: Creating position and velocity profiles
% ##########
% For analysis purpose
pos_desire = zeros(3, samples);
vel_desire = zeros(3, samples);
att_desire = zeros(3, samples);
ang_desire = zeros(3, samples);
% ##########

% ##########
% Project 1b: FSM state definitions
% ##########
sIDLE = 1; sTAKEOFF = 2; sHOVER1 = 3; sTRACKING = 4; sHOVER2 = 5; sLAND = 6; sDONE = 7;
states = {'IDLE', 'TAKEOFF', 'HOVER1', 'TRACKING', 'HOVER2', 'LAND', 'DONE'};
t_dur = 2; % Duration of IDLE, HOVER1/2 states in second(s)
% ##########

% ##########
% Project 1b: Trajectory generation
% ##########
% Fill in track information here...
t_traj = linspace(0, 10, 1000);
traj = [zeros(2, length(t_traj)); 0.1*ones(1, length(t_traj))];
t_trajstart = 0;
t_trajend = 0;
ei_trajstart = 0;
ei_trajend = 0;
oi_trajstart = 0;
oi_trajend = 0;
% ##########

%% Simulation begins

seq = randi(1000);

sim_mode = true;

tstart = tic;

% ##########
% Project 1b: FSM state initialization
% ##########
c_state = sIDLE;
n_state = sIDLE;
transitioned = 0;
t_state = toc(tstart) + t_dur; % End time to leave the current state

fprintf('i: %d   time: %f   state: %s\n', i, toc(tstart), states{c_state});
% ##########

for i = 1:samples

    %% Read the odom message, return empty if no message after 3 ms
    odom_msg = odom_sub.read(3, false);
    odom_updated = false;
    if ~isempty(odom_msg)
        pos_odom(:, odom_idx) = ...
            [odom_msg.pose.pose.position.x;
             odom_msg.pose.pose.position.y;
             odom_msg.pose.pose.position.z];
        vel_odom(:, odom_idx) = ...
            [odom_msg.twist.twist.linear.x;
             odom_msg.twist.twist.linear.y;
             odom_msg.twist.twist.linear.z];
        att_odom(:, odom_idx) = ...
            geometry_utils.RToZYX(geometry_utils.QuatToR(odom_msg.pose.pose.orientation));
        ang_odom(:, odom_idx) = ...
            [odom_msg.twist.twist.angular.x; ...
             odom_msg.twist.twist.angular.y; ...
             odom_msg.twist.twist.angular.z];
        odom_time(odom_idx) = toc(tstart);
        odom_idx = odom_idx + 1;
        odom_updated = true;
    end

    %% Read the imu message, return empty if no message after 3 ms
    imu_msg = imu_sub.read(3, false);
    imu_updated = false;
    if ~isempty(imu_msg)
        att_imu(:,imu_idx) = ...
            geometry_utils.RToZYX(geometry_utils.QuatToR(imu_msg.orientation));
        ang_imu(:,imu_idx) = ...
            [imu_msg.angular_velocity.x; ...
             imu_msg.angular_velocity.y; ...
             imu_msg.angular_velocity.z];
        imu_time(imu_idx) = toc(tstart);
        imu_idx = imu_idx + 1;
        imu_updated = true;
    end
        
    %% Update controller
    if odom_idx > 1 && imu_idx > 1 && odom_updated && imu_updated
        psi = att_odom(3,odom_idx-1);

        e_pos(:,err_idx) = pos_odom(:,odom_idx-1) - pos_des;
        
        pos_desire(:,err_idx)= pos_des;
        vel_desire(:,err_idx)= vel_des;
        att_desire(:,err_idx)= att_des;
        ang_desire(:,err_idx)= ang_des;
        
        e_vel(:,err_idx) = vel_odom(:,odom_idx-1) - vel_des;

        u_pos(:,err_idx) = -Kp_pos*e_pos(:,err_idx) - Kd_pos*e_vel(:,err_idx);

        % ##########
        % Project 1b: FSM logic
        % ##########
        % Next state logic
        switch c_state
            case sIDLE
                if toc(tstart) > t_state
                    n_state = sTAKEOFF;
                    
                    transitioned = 1;
                end
                
            case sTAKEOFF
                if sqrt(e_pos(:,err_idx)'*e_pos(:,err_idx)) < 0.05
                    n_state = sHOVER1;
                    
                    transitioned = 1;
                    t_state = toc(tstart) + t_dur;
                end
                
            case sHOVER1
                if toc(tstart) > t_state
                    n_state = sTRACKING;
                    
                    Kp = diag([1.5; 1.5; 6.7; 10; 10; 10]); % Slight overshoot
                    Kd = diag([1.7; 1.7; 4; 4; 4; 1.5]);
%                     Kp = diag([1.5; 1.5; 6.7; 10; 10; 10]); % Slight overshoot
%                     Kd = diag([1.7; 1.7; 4; 4; 4; 1.5]);
                    Kp_pos = Kp(1:3,1:3);
                    Kd_pos = Kd(1:3,1:3);
                    Kp_att = Kp(4:6,4:6);
                    Kd_att = Kd(4:6,4:6);

                    transitioned = 1;
                    t_trajstart = toc(tstart);
                    t_trajend = t_trajstart + t_traj(end);
                    ei_trajstart = err_idx;
                    oi_trajstart = odom_idx;
                end
                
            case sTRACKING
                if sqrt((pos_odom(:,odom_idx-1)-traj(:,end))'*(pos_odom(:,odom_idx-1)-traj(:,end))) < 0.1 && ...
                    toc(tstart) > t_trajend
                    n_state = sHOVER2;

                    ttt = toc(tstart);
                    fprintf('Tracking Info... start: %f   end: %f   now: %f   dur: %f   late: %f\n' ...
                        , t_trajstart, t_trajend, ttt, ttt-t_trajstart, ttt-t_trajend);

                    transitioned = 1;
                    t_state = toc(tstart) + t_dur;
                    ei_trajend = err_idx;
                    oi_trajend = odom_idx;
                end
                
            case sHOVER2
                if toc(tstart) > t_state
                    n_state = sLAND;
                    
%                     Kp = diag([0.7; 0.7; 2; 10; 10; 6]); % Overdamped
%                     Kd = diag([1.3; 1.3; 4; 4; 4; 1.6]);
%                     Kp_pos = Kp(1:3,1:3);
%                     Kd_pos = Kd(1:3,1:3);
%                     Kp_att = Kp(4:6,4:6);
%                     Kd_att = Kd(4:6,4:6);
                    
                    transitioned = 1;
                end
                
            case sLAND
                if sqrt(e_pos(:,err_idx)'*e_pos(:,err_idx)) < 0.05
                    n_state = sDONE;
                    
                    transitioned = 1;
                end
                
            otherwise % Warn for an unrecognized state
                fprintf('ERROR: Unrecognized state: %d\n', c_state);
                break;
        end
        c_state = n_state;

        % Status Update
        if transitioned == 1
            fprintf('i: %d   time: %f   state: %s\n', i, toc(tstart), states{c_state});
            transitioned = 0;
        end

        % Output logic
        switch c_state
            case sIDLE % Don't do anything...
                continue;
                
            case sTAKEOFF % Set desired position at 1m height
                pos_des = [0; 0; 1];
                vel_des = [0; 0; 0];
                
            case sHOVER1 % Set desired position at 1m height
                pos_des = [0; 0; 1];
                vel_des = [0; 0; 0];
                
            case sTRACKING % Track the expected position at this time
                pos_des = traj(:, sum(t_traj < toc(tstart)-t_trajstart));
                att_des(3) = 15/180*pi; % Change heading angle here
                
            case sHOVER2 % Set desired position at end of track
                pos_des = traj(:, end);
                att_des(3) = 0;
                
            case sLAND % Set desired position on ground
                pos_des = [0; 0; 0.05];
                vel_des = [0; 0; 0];
                
            case sDONE % Done! Leave the loop
                break;
                
            otherwise % Warn for an unrecognized state
                fprintf('ERROR: Unrecognized state: %d\n', c_state);
                break;
        end
        % ##########
    
        %% Ideal attitude inputs
        u_r = (u_pos(1,err_idx)*sin(psi) - u_pos(2,err_idx)*cos(psi))/params.gravity;
        u_p = (u_pos(1,err_idx)*cos(psi) + u_pos(2,err_idx)*sin(psi))/params.gravity;
        u_y = att_des(3);

        %% Bias compensated attitude
        att = [att_imu(1:2,imu_idx-1); psi];

        %% Attiude error
        e_att(:, err_idx) = ...
            geometry_utils.shortest_angular_distance([u_r;u_p;u_y], att);
        e_ang(:, err_idx) = ang_imu(:,imu_idx-1) - ang_des;
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
        th_cmd = min(2.6*params.gravity*params.mass, th_cmd);
        thrusts(err_idx) = th_cmd;

        %% Bias compensated inputs
        pd_msg.header.stamp = e_time(err_idx);
        pd_msg.roll = phi_cmd;
        pd_msg.pitch = theta_cmd;
        pd_msg.yaw_delta = -yaw_delta;
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
fprintf('Sending zero command...\n');
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

%% Debug plots
% figure(1); % Error plots for TRACKING
% colors = ['r', 'g', 'b'];
% for ip=1:3
%     subplot(4,3,ip)
%     plot(e_time(ei_trajstart:ei_trajend), e_pos(ip,ei_trajstart:ei_trajend), colors(ip));
%     xlabel('t');
%     ylabel(sprintf('e_pos(%d)', ip), 'Interpreter', 'none');
%     if ip == 2; title('e_pos', 'Interpreter', 'none'); end
% end
% for ip=1:3
%     subplot(4,3,3+ip)
%     plot(e_time(ei_trajstart:ei_trajend), e_vel(ip,ei_trajstart:ei_trajend), colors(ip));
%     xlabel('t');
%     ylabel(sprintf('e_vel(%d)', ip), 'Interpreter', 'none');
%     if ip == 2; title('e_vel', 'Interpreter', 'none'); end
% end
% for ip=1:3
%     subplot(4,3,6+ip)
%     plot(e_time(ei_trajstart:ei_trajend), e_att(ip,ei_trajstart:ei_trajend), colors(ip));
%     xlabel('t');
%     ylabel(sprintf('e_att(%d)', ip), 'Interpreter', 'none');
%     if ip == 2; title('e_att', 'Interpreter', 'none'); end
% end
% for ip=1:3
%     subplot(4,3,9+ip)
%     plot(e_time(ei_trajstart:ei_trajend), e_ang(ip,ei_trajstart:ei_trajend), colors(ip));
%     xlabel('t');
%     ylabel(sprintf('e_ang(%d)', ip), 'Interpreter', 'none');
%     if ip == 2; title('e_ang', 'Interpreter', 'none'); end
% end
% 
% figure(2); % Odom vs. desired for TRACKING
% colors = ['r', 'g', 'b'];
% deslabel = '--';
% for ip=1:3
%     subplot(4,3,ip);
%     hold on;
%     plot(odom_time(oi_trajstart:oi_trajend), pos_odom(ip,oi_trajstart:oi_trajend), colors(ip));
%     plot(e_time(ei_trajstart:ei_trajend), pos_desire(ip,ei_trajstart:ei_trajend), [colors(ip), deslabel]);
%     xlabel('t');
%     ylabel(sprintf('pos_odom(%d)', ip), 'Interpreter', 'none');
%     if ip == 2; title('pos_odom', 'Interpreter', 'none'); end
% end
% for ip=1:3
%     subplot(4,3,3+ip);
%     hold on;
%     plot(odom_time(oi_trajstart:oi_trajend), vel_odom(ip,oi_trajstart:oi_trajend), colors(ip));
%     plot(e_time(ei_trajstart:ei_trajend), vel_desire(ip,ei_trajstart:ei_trajend), [colors(ip), deslabel]);
%     xlabel('t');
%     ylabel(sprintf('vel_odom(%d)', ip), 'Interpreter', 'none');
%     if ip == 2; title('vel_odom', 'Interpreter', 'none'); end
% end
% for ip=1:3
%     subplot(4,3,6+ip);
%     hold on;
%     plot(odom_time(oi_trajstart:oi_trajend), att_odom(ip,oi_trajstart:oi_trajend), colors(ip));
%     plot(e_time(ei_trajstart:ei_trajend), att_desire(ip,ei_trajstart:ei_trajend), [colors(ip), deslabel]);
%     xlabel('t');
%     ylabel(sprintf('att_odom(%d)', ip), 'Interpreter', 'none');
%     if ip == 2; title('att_odom', 'Interpreter', 'none'); end
% end
% for ip=1:3
%     subplot(4,3,9+ip);
%     hold on;
%     plot(odom_time(oi_trajstart:oi_trajend), ang_odom(ip,oi_trajstart:oi_trajend), colors(ip));
%     plot(e_time(ei_trajstart:ei_trajend), ang_desire(ip,ei_trajstart:ei_trajend), [colors(ip), deslabel]);
%     xlabel('t');
%     ylabel(sprintf('ang_odom(%d)', ip), 'Interpreter', 'none');
%     if ip == 2; title('ang_odom', 'Interpreter', 'none'); end
% end

colors = ['r', 'g', 'b'];
deslabel = '--';

plotdir = 'project1bplots/q2';
qnum = '2a';
saveplot = 0;

handle = figure(1); % e_pos plots for TRACKING
for ip=1:3
    subplot(1,3,ip);
    plot(e_time(ei_trajstart:ei_trajend), e_pos(ip,ei_trajstart:ei_trajend), colors(ip));
    xlabel('t');
    ylabel(sprintf('e_pos(%d)', ip), 'Interpreter', 'none');
    if ip == 2; title('e_pos', 'Interpreter', 'none'); end
end
if saveplot == 1
    saveas(handle, fullfile(plotdir, sprintf('q%s_e_pos.fig', qnum)));
    print(handle, '-depsc', fullfile(plotdir, sprintf('q%s_e_pos.epsc', qnum)));
end

handle = figure(2); % e_vel plots for TRACKING
for ip=1:3
    subplot(1,3,ip);
    plot(e_time(ei_trajstart:ei_trajend), e_vel(ip,ei_trajstart:ei_trajend), colors(ip));
    xlabel('t');
    ylabel(sprintf('e_vel(%d)', ip), 'Interpreter', 'none');
    if ip == 2; title('e_vel', 'Interpreter', 'none'); end
end
if saveplot == 1
    saveas(handle, fullfile(plotdir, sprintf('q%s_e_vel.fig', qnum)));
    print(handle, '-depsc', fullfile(plotdir, sprintf('q%s_e_vel.epsc', qnum)));
end

handle = figure(3); % e_att plots for TRACKING
for ip=1:3
    subplot(1,3,ip);
    plot(e_time(ei_trajstart:ei_trajend), e_att(ip,ei_trajstart:ei_trajend), colors(ip));
    xlabel('t');
    ylabel(sprintf('e_att(%d)', ip), 'Interpreter', 'none');
    if ip == 2; title('e_att', 'Interpreter', 'none'); end
end
if saveplot == 1
    saveas(handle, fullfile(plotdir, sprintf('q%s_e_att.fig', qnum)));
    print(handle, '-depsc', fullfile(plotdir, sprintf('q%s_e_att.epsc', qnum)));
end

handle = figure(4); % e_ang plots for TRACKING
for ip=1:3
    subplot(1,3,ip);
    plot(e_time(ei_trajstart:ei_trajend), e_ang(ip,ei_trajstart:ei_trajend), colors(ip));
    xlabel('t');
    ylabel(sprintf('e_ang(%d)', ip), 'Interpreter', 'none');
    if ip == 2; title('e_ang', 'Interpreter', 'none'); end
end
if saveplot == 1
    saveas(handle, fullfile(plotdir, sprintf('q%s_e_ang.fig', qnum)));
    print(handle, '-depsc', fullfile(plotdir, sprintf('q%s_e_ang.epsc', qnum)));
end

handle = figure(5); % pos_odom vs. pos_desired for TRACKING
for ip=1:3
    subplot(1,3,ip);
    hold on;
    plot(odom_time(oi_trajstart:oi_trajend), pos_odom(ip,oi_trajstart:oi_trajend), colors(ip));
    plot(e_time(ei_trajstart:ei_trajend), pos_desire(ip,ei_trajstart:ei_trajend), [colors(ip), deslabel]);
    xlabel('t');
    ylabel(sprintf('pos_odom(%d)', ip), 'Interpreter', 'none');
    if ip == 2; title('pos_odom', 'Interpreter', 'none'); end
end
if saveplot == 1
    saveas(handle, fullfile(plotdir, sprintf('q%s_pos_odom.fig', qnum)));
    print(handle, '-depsc', fullfile(plotdir, sprintf('q%s_pos_odom.epsc', qnum)));
end

handle = figure(6); % vel_odom vs. vel_desired for TRACKING
for ip=1:3
    subplot(1,3,ip);
    hold on;
    plot(odom_time(oi_trajstart:oi_trajend), vel_odom(ip,oi_trajstart:oi_trajend), colors(ip));
    plot(e_time(ei_trajstart:ei_trajend), vel_desire(ip,ei_trajstart:ei_trajend), [colors(ip), deslabel]);
    xlabel('t');
    ylabel(sprintf('vel_odom(%d)', ip), 'Interpreter', 'none');
    if ip == 2; title('vel_odom', 'Interpreter', 'none'); end
end
if saveplot == 1
    saveas(handle, fullfile(plotdir, sprintf('q%s_vel_odom.fig', qnum)));
    print(handle, '-depsc', fullfile(plotdir, sprintf('q%s_vel_odom.epsc', qnum)));
end

handle = figure(7); % att_odom vs. att_desired for TRACKING
for ip=1:3
    subplot(1,3,ip);
    hold on;
    plot(odom_time(oi_trajstart:oi_trajend), att_odom(ip,oi_trajstart:oi_trajend), colors(ip));
    plot(e_time(ei_trajstart:ei_trajend), att_desire(ip,ei_trajstart:ei_trajend), [colors(ip), deslabel]);
    xlabel('t');
    ylabel(sprintf('att_odom(%d)', ip), 'Interpreter', 'none');
    if ip == 2; title('att_odom', 'Interpreter', 'none'); end
end
if saveplot == 1
    saveas(handle, fullfile(plotdir, sprintf('q%s_att_odom.fig', qnum)));
    print(handle, '-depsc', fullfile(plotdir, sprintf('q%s_att_odom.epsc', qnum)));
end

handle = figure(8); % ang_odom vs. ang_desired for TRACKING
for ip=1:3
    subplot(1,3,ip);
    hold on;
    plot(odom_time(oi_trajstart:oi_trajend), ang_odom(ip,oi_trajstart:oi_trajend), colors(ip));
    plot(e_time(ei_trajstart:ei_trajend), ang_desire(ip,ei_trajstart:ei_trajend), [colors(ip), deslabel]);
    xlabel('t');
    ylabel(sprintf('ang_odom(%d)', ip), 'Interpreter', 'none');
    if ip == 2; title('ang_odom', 'Interpreter', 'none'); end
end
if saveplot == 1
    saveas(handle, fullfile(plotdir, sprintf('q%s_ang_odom.fig', qnum)));
    print(handle, '-depsc', fullfile(plotdir, sprintf('q%s_ang_odom.epsc', qnum)));
end
