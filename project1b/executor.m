
addpath('~/sandbox/cmu_quad_matlab/dry/src/geometry_utils');
addpath('~/sandbox/cmu_quad_matlab/dry/src/quadrotor_model');
addpath('~/sandbox/cmu_quad_matlab/dry/install_isolated/share/ipc_bridge/matlab');
addpath('~/sandbox/cmu_quad_matlab/wet/build/lib');

% data extracted from cmu_model_configs/config/CMUQuad5.yaml. at some point in the future 
% we might have to parse the yaml file directly, and move these assignments to a dedicated function.
% note that the inertia is not exactly the same as in that file - all non-diagonal elements are assigned zero. 

par = struct; 
par.gravity = 9.81;
par.mass = 0.340;
par.inertia = diag([0.00077773; 0.00079183; 0.00112944]);
par.length = 0.1043;
par.pwm.low = 1000;
par.pwm.high = 1950;
par.pwm.rpm_scale = 16500;
par.gains.motor = 20;
par.gains.thrust = 9.0260e-09;
par.gains.moment_scale = 0.006;

% this is the matrix that maps the thrusts on each individual rotors to the total thrust and moments.
% in the data processing cycle below, we derive the desired thrust and moments (control inputs) and
% left-multiply the inverse of this matrix. The resulting vector is then converted to pwm and published. 
% important: different configurations have different mixer matrices. 

mixer = zeros(4); 
mixer(1, :) = par.gains.thrust * [1, 1, 1, 1]; 
mixer(2, :) = par.length * par.gains.thrust * [0, 1, 0, -1]; 
mixer(3, :) = par.length * par.gains.thrust * [-1, 0, 1, 0];
mixer(4, :) = par.gains.moment_scale * par.gains.thrust * [-1, 1, -1, 1]; 

[kpos, kvel, krot, kagv] = deal(3.82, 3.51, 4.82, 4.06);

% Call clear each time to make sure that you clean up old pub/sub objects
clear odomsub imusub pwmpub;

odomsub = ipc_bridge.createSubscriber('nav_msgs', 'Odometry', 'odom');
imusub = ipc_bridge.createSubscriber('sensor_msgs', 'Imu', 'imu');
pwmpub = ipc_bridge.createPublisher('quadrotor_msgs', 'PWMCommand', 'pwm_cmd');

%% Create the structure of the outgoing message (to populate below)
pwmmsg = pwmpub.empty();

%% Clear pending messages
for i = 1:10
    odomsub.read(10, false);
    imusub.read(10, false);
end

samples = 2400; tstart = tic;
[odomidx, imuidx, erridx] = deal(1);
[agvchx, mmthx, erothx, pos_odom, agv_odom, vel_odom] = deal(zeros(3, samples));
[timehx, forcehx, odom_time, imu_time] = deal(zeros(1, samples));
[rotchx, rothx] = deal(zeros(3, 3, samples));
[pwmhx] = deal(zeros(4, samples)); 

posd = @(time) [0; 0; 1.0];
veld = @(time) [0; 0; 0];
accd = @(time) [0; 0; 0];
hdgd = 0; 

for idx = 1: samples

%  if idx == 700, hdgd = pi-0.01; end
  if idx == 700, posd = @(time) [2.0; 2.0; 3.0]; end 

  % here we're entering the data collection cycle. first we attempt to get a message from the odometry
  % subscriber (persumed equivalent to the waitForMessage method), with a maximum waiting time of
  % three seconds. should such a message be received, convert it to a native data structure. the same 
  % also to the readings from the inertial measurement units. after this snippet of code, we have the
  % measured attitude, position, angular velocity and velocity vectors, from odometry; as well as
  % the attitude, angular velocity readings from the inertial measurement units. then, proceed to 
  % data processing if we have both odometry and imu messages incoming.   

  flatten = @(struct) [struct.x; struct.y; struct.z];
  odom_msg = odomsub.read(3, false);
  if ~isempty(odom_msg)
    rothx(:, :, odomidx) = geometry_utils.QuatToR(odom_msg.pose.pose.orientation); 
    pos_odom(:, odomidx) = flatten(odom_msg.pose.pose.position);
    agv_odom(:, odomidx) = flatten(odom_msg.twist.twist.angular);
    vel_odom(:, odomidx) = flatten(odom_msg.twist.twist.linear);
    odom_time(odomidx) = toc(tstart); odomidx = odomidx + 1; 
  end

  % imu_msg = imusub.read(msgwaitt, false);
  % if ~isempty(imu_msg)
  %  att_imu(:, imuidx) = geometry_utils.RToZYX(geometry_utils.QuatToR(imu_msg.orientation)); 
  %  agv_imu(:, imuidx) = flatten(imu_msg.angular_velocity);
  %  imu_time(imuidx) = toc(tstart); imuidx = imuidx + 1;
  % end

  if ~isempty(odom_msg) % MODIFIED && ~isempty(imu_msg) 

    time = toc(tstart); erridx = erridx + 1; agv = agv_odom(:, odomidx - 1); rot = rothx(:, :, odomidx - 1);
    % here we're implementing a non-linear controller as described in: v. position controlled flight mode, 
    % control of complex maneuvers for a quadrotor uav using geometric methods on se(3). to get started,  
    % calculate the position and velocity errors, in equation 17 and 18.  

    epos = pos_odom(:, odomidx - 1) - posd(time); 
    evel = vel_odom(:, odomidx - 1) - veld(time);

    % there are four variables to be controlled: x, y, z, yaw. in position controlled flight mode,
    % x, y, z are controlled by design; to properly control yaw, we need to find an appropriate b1c,
    % in equation 22, that is orthogonal to b3c and pointing to the desired heading; then, to calculate
    % hat computed omega, we need to take the differentiation of the computed rotation matrix. 

    b3c = - kpos * epos - kvel * evel + [0; 0; par.gravity] + accd(time); b3c = b3c / norm(b3c);
    b1c = - cross(b3c, cross(b3c, [cos(hdgd); sin(hdgd); 0])); b1c = b1c / norm(b1c); 
    b2c = cross(b3c, b1c); rotc = [b1c, b2c, b3c];

    drotc = (rotc - rotchx(:, :, erridx - 1)) / (time - timehx(erridx - 1)); 
    agvc = vee(rotc' * drotc); dagvc = (agvc - agvchx(:, erridx - 1)) / (time - timehx(erridx - 1)); 

    % the following are implementations for equation 19 and 20. the errors for rotational matrix (erot)
    % and angular velocity (eagv) are computed, and the control expressions for force and moments are
    % evaluated. actually i believe that everybody can figure this out, but since they look totally messy,
    % i'm just writing some comments to make the code structure look slightly better. 

    erot = vee(rotc' * rot - rot' * rotc) / 2.0; eagv = agv - rot' * rotc * agvc;
    force = - par.mass * (kpos * epos + kvel * evel - [0; 0; par.gravity] - accd(time))' * rot(:, 3);
    moments = par.inertia * (- krot * erot - kagv * eagv) + cross(agv, par.inertia * agv) - ... 
      par.inertia * (hat(agv) * rot' * rotc * agvc - rot' * rotc * dagvc); 

    % now we have the needed force and moments. the next step is to compute the thrusts for each individual motor,
    % compute the corresponding pulse-width modulation (pwm) parameters, and publish some command. 

    rotorspds = sqrt(max(inv(mixer) * [force; moments], 0));
    pulsewds = rotorspds * (par.pwm.high - par.pwm.low) / par.pwm.rpm_scale + par.pwm.low; 
    pulsewds = max(min(pulsewds, par.pwm.high), par.pwm.low);  

    pwmmsg.header.stamp = time;
    pwmmsg.motor_pwm = pulsewds;
    pwmpub.publish(pwmmsg);

    % after dust's settled let's preserve a part of the history. states that must be saved include agvchx
    % or the computed angular velocity, rotchx, or the computed rotational matrix, and time. 

    agvchx(:, erridx) = agvc; timehx(erridx) = time; rotchx(:, :, erridx) = rotc; pwmhx(:, erridx) = pulsewds;
    mmthx(:, idx) = moments; erothx(:, idx) = erot; rothx(:, :, erridx) = rot; forcehx(erridx) = force; 

  end
end

pwmmsg.header.stamp = timehx(erridx);
pwmmsg.motor_pwm = deal(0); 
pwmpub.publish(pwmmsg);

odomsub.disconnect(); imusub.disconnect(); pwmpub.disconnect(); 
