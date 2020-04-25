%%
addpath('~/Documents/MATLAB/Quaternions')
%%
clear all;
close all;
% T = readtable('~/Desktop/sensor_basic_test.csv');
% T = readtable('~/Desktop/moveXYZ.csv');
% T = readtable('~/Desktop/armwave.csv');
% T = readtable('~/Desktop/lrfbud.csv');
% T = readtable('~/Desktop/lrfbud2.csv');
T = readtable('~/Desktop/rccwTiltRrccwFwUp.csv');
% T = readtable('~/Desktop/slowRotate.csv');

time   = (T{:,1}-T{1,1})/1000; % zero origin, ms -> s
accxyz = T{:,2:4};             % acceleration of x,y,z axes of sensor
quat   = T{:,5:end};           % quaternion orientation sensor

samplePeriod = mean(diff(time)); % average time between measurements

clear T

%% Inspect time between samples
plot(diff(time))

%% Plot acceleration
plot(time, accxyz)
legend(["X", "Y", "Z"])
ylabel("Accel. Magnitude")
xlabel("Time [s]")

%% Plot quaternions
plot(time, quat);
legend(["q1", "q2", "q3", "q4"]);
ylabel("Quat. Values");
xlabel("Time [s]");
%%
% axisAngle2quatern([0 0 1], -pi/2)
size(quatern2euler(quat_corr))

%% Compute translational accelerations
% using axis rotation of orientation quat
close all; 

% 1.0 *** Read in acceleration ***  
acc_orig = accxyz; % default

% 1.1 *** Convert sensor accel NED -> ENU
% acc_corr = acc_orig;
acc_corr  = [acc_orig(:,2) acc_orig(:,1) -acc_orig(:,3)]; % NED (Razor) -> ENU (x-IMU)
% acc_corr  = [acc_orig(:,1) acc_orig(:,2) acc_orig(:,3)];

% 2.0 *** Read in quaternion ***
quat_orig = quat;  % default

% 2.1 *** Convert sensor orientation quat NED -> ENU via rotation 
% quat_corr = quat_local;
quat_corr = quaternProd(            ... % quaternProd performs the rotation of the orientation quaternion
    axisAngle2quatern([sqrt(2)/2 sqrt(2)/2 0], pi), ... % rotation vector at 45 degrees
    quat_orig                       ... % input quaternion (sensor orientation)
);

% according to paper: phi=roll, theta=pitch, psi=yaw
% See Eq.1 in "Adaptive Filter for a Miniature MEMS Based Attitude 
% and Heading Reference System"
orientation = quatern2euler(quat_corr);

% 3.1 *** Rotate body accelerations to Earth frame ***
% 3.0 *** Take conjugate of corrected orientation to use as rotation transform
acc = quaternRotate(acc_orig, quat_corr); % Janis's genius move
acc = quaternRotate(acc, axisAngle2quatern([0 0 1], -pi/4)); % Janis's second genius move

acc(:,3) = acc(:,3)+1; % add (?) gravity back to remove it

figure(5)
plotaccel(time, accxyz, acc)
ylim([-0.5 1])

figure(666)
set(gcf, 'Position', [641   389   640   316], 'NumberTitle', 'off', 'Name', 'Orientation');
hold on;
plot(orientation)
colororder([1 0 0; 0 1 0; 0 0 1]); % r,g,b
% according to paper: phi=roll, theta=pitch, psi=yaw
legend('phi-roll-y', 'pitch-theta-x', 'psi-yaw-z');


xlabel('Time (s)'); 
ylabel('rad');

% Find stationary moments
moving = vecnorm(acc, 2, 2) > 0.15; % 0.2

figure(4)
plot(moving, 'o'); hold on;
plot(vecnorm(acc, 2,2));
hold off;
ylim([-0.05 1.05])

% Compute translational velocities

% acc(:,3) = acc(:,3) - 9.81;

% Integrate acceleration to yield velocity
vel = zeros(size(acc));
for t = 2:length(vel)
    if (moving(t))
        vel(t,:) = vel(t-1,:) + acc(t,:) * samplePeriod;
    else
        vel(t,:) = [0 0 0]; % not moving
    end
    
    % ~~~ Mike removed ~~~
%     if(stationary(t) == 1)
%         vel(t,:) = [0 0 0];     % force zero velocity when foot stationary
%     end
end
clear v;

% ~~~ Mike removed ~~~
% % Compute integral drift during non-stationary periods
% velDrift = zeros(size(vel));
% stationaryStart = find([0; diff(stationary)] == -1);
% stationaryEnd = find([0; diff(stationary)] == 1);
% for i = 1:numel(stationaryEnd)
%     driftRate = vel(stationaryEnd(i)-1, :) / (stationaryEnd(i) - stationaryStart(i));
%     enum = 1:(stationaryEnd(i) - stationaryStart(i));
%     drift = [enum'*driftRate(1) enum'*driftRate(2) enum'*driftRate(3)];
%     velDrift(stationaryStart(i):stationaryEnd(i)-1, :) = drift;
% end
% 
% % Remove integral drift
% vel = vel - velDrift;

% Plot translational velocity
figure(1);
set(gcf, 'Position', [641   389   640   316], 'NumberTitle', 'off', 'Name', 'Velocity');
hold on;
plot(time, vel);
plot(time, vecnorm(vel, 2, 2), '--', 'LineWidth', 1.5);
colororder([1 0 0; 0 1 0; 0 0 1; 0 0 0;]); % r,g,b,k
legend('X', 'Y', 'Z', 'mag');
xlabel('Time (s)'); 
ylabel('Velocity (m/s)');

hold off;

% ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
% Compute translational position

% Integrate velocity to yield position
pos = zeros(size(vel));
for t = 2:length(pos)
    pos(t,:) = pos(t-1,:) + vel(t,:) * samplePeriod;    % integrate velocity to yield position
end

pos = pos .* [1 1 -1]; % to try simple axis inversions, etc.
pos = pos * 981; % mike added: convert to cm

% Plot translational position

figure(2)
set(gcf, 'Position', [642     1   639   314], 'NumberTitle', 'off', 'Name', 'Position');
hold on;
plot(time, pos);
colororder([1 0 0; 0 1 0; 0 0 1;]); % r,g,b
legend('X', 'Y', 'Z');

title('Position');
xlabel('Time (s)');
ylabel('Position (cm)');

% xlim([1 6])
hold off;

% 3D plot

figure(3)
set(gcf, 'Position', [10    85   631   522], 'NumberTitle', 'off', 'Name', 'Position 3D');
cols = genColors(0.0, 0.7, length(pos));
scatter3(pos(:,1),pos(:,2),pos(:,3),[],cols, 'filled')
xlabel('X')
ylabel('Y')
zlabel('Z')
colororder(cols)
clear cols

placeFigures()
%% Functions

function qConj = quaternConj(q)
    qConj = [q(:,1) -q(:,2) -q(:,3) -q(:,4)];
end

function v = quaternRotate(v, q)
    [row col] = size(v);
    v0XYZ = quaternProd(quaternProd(q, [zeros(row, 1) v]), quaternConj(q));
    v = v0XYZ(:, 2:4);
end

function ab = quaternProd(a, b)
    ab(:,1) = a(:,1).*b(:,1)-a(:,2).*b(:,2)-a(:,3).*b(:,3)-a(:,4).*b(:,4);
    ab(:,2) = a(:,1).*b(:,2)+a(:,2).*b(:,1)+a(:,3).*b(:,4)-a(:,4).*b(:,3);
    ab(:,3) = a(:,1).*b(:,3)-a(:,2).*b(:,4)+a(:,3).*b(:,1)+a(:,4).*b(:,2);
    ab(:,4) = a(:,1).*b(:,4)+a(:,2).*b(:,3)-a(:,3).*b(:,2)+a(:,4).*b(:,1);
end

function q = axisAngle2quatern(axis, angle)
    q0 = cos(angle./2);
    q1 = -axis(:,1)*sin(angle./2);
    q2 = -axis(:,2)*sin(angle./2);
    q3 = -axis(:,3)*sin(angle./2); 
    q = [q0 q1 q2 q3];
end

function euler = quatern2euler(q)
    % from paper: "Adaptive Filter for a Miniature MEMS Based Attitude and
    % Heading Reference System" by Wang et al, IEEE.
    
    R(1,1,:) = 2.*q(:,1).^2-1+2.*q(:,2).^2;
    R(2,1,:) = 2.*(q(:,2).*q(:,3)-q(:,1).*q(:,4));
    R(3,1,:) = 2.*(q(:,2).*q(:,4)+q(:,1).*q(:,3));
    R(3,2,:) = 2.*(q(:,3).*q(:,4)-q(:,1).*q(:,2));
    R(3,3,:) = 2.*q(:,1).^2-1+2.*q(:,4).^2;    
    
    phi = atan2(R(3,2,:), R(3,3,:) );
    theta = -atan(R(3,1,:) ./ sqrt(1-R(3,1,:).^2) );    
    psi = atan2(R(2,1,:), R(1,1,:) );

    euler = [phi(1,:)' theta(1,:)' psi(1,:)']; 
end

function Q_ENU = ned2enuQuat(Q_NED)

    Qw = Q_NED(1);
    Qx = Q_NED(2);
    Qy = Q_NED(3);
    Qz = Q_NED(4);

    a = [sqrt(2)/2 sqrt(2)/2 0 0];

    Q_orig = [   Qw -Qz  Qy -Qx;
                 Qz  Qw -Qx -Qy;
                -Qy  Qx  Qw -Qz;
                 Qx  Qy  Qz  Qw  ];

    A = [    0      0   -0.707 0.707;
             0      0    0.707 0.707;
           0.707 -0.707    0     0  ;
          -0.707 -0.707    0     0     ];

    Q_ENU = (a*Q_orig)*A; % orignal
end

function plotaccel(time, accxyz, acc)
    % Plot translational accelerations
    ylims = [-2 2.3];
    
    subplot(2,1,1)
    hold on;
    plot(time, accxyz);
    colororder([1 0 0; 0 1 0; 0 0 1])
    title('Acceleration'); ylabel('Acceleration (m/s/s)');
    legend('X', 'Y', 'Z');
    ylim(ylims)
    hold off;

    subplot(2,1,2)
    hold on;
    plot(time, acc);
    colororder([1 0 0; 0 1 0; 0 0 1])
    xlabel('Time (s)'); ylabel('Acceleration (m/s/s)');
    legend('X', 'Y', 'Z');
    ylim(ylims)
    hold off;
end

% %% Compute translational accelerations
% % using matrix from tech note (no conj)
% 
% % 1.0 *** Read in acceleration ***  
% acc_orig = accxyz; % default
% 
% % 1.1 *** Convert sensor accel NED -> ENU
% % acc_corr = acc_orig;
% acc_corr  = [acc_orig(:,2) acc_orig(:,1) -acc_orig(:,3)]; % NED (Razor) -> ENU (x-IMU)
% % acc_corr  = [acc_orig(:,1) acc_orig(:,2) acc_orig(:,3)];
% 
% % 2.0 *** Read in quaternion ***
% quat_orig = quat;  % default
% 
% % 2.1 *** Convert sensor orientation quat NED -> ENU via rotation 
% quat_corr = zeros(size(quat_orig));
% for i = 1:length(quat_orig)
%     quat_corr(i,:) = ned2enuQuat(quat_orig(i,:));
% end
% 
% % 3.1 *** Rotate body accelerations to Earth frame ***
% % acc = quaternRotate(acc_local, quaternConj(quat_local)); % default
% % acc = quaternRotate(acc_corr, quaternConj(quat_corr));
% acc = quaternRotate(acc_corr, quat_corr);
% 
% plotaccel(time, accxyz, acc)

% %% Compute translational accelerations
% 
% % 1.0 *** Read in acceleration ***  
% acc_orig = accxyz; % default
% 
% % 1.1 *** Convert sensor accel NED -> ENU
% % acc_corr = acc_orig;
% acc_corr  = [acc_orig(:,2) acc_orig(:,1) -acc_orig(:,3)]; % NED (Razor) -> ENU (x-IMU)
% % acc_corr  = [acc_orig(:,1) acc_orig(:,2) acc_orig(:,3)];
% 
% % 2.0 *** Read in quaternion ***
% quat_orig = quat;  % default
% 
% % 2.1 *** Convert sensor orientation quat NED -> ENU via rotation 
% % quat_corr = quat_local;
% % quat_corr = quaternProd(            ... % quaternProd performs the rotation of the orientation quaternion
% %     axisAngle2quatern([sqrt(2)/2 sqrt(2)/2 0], pi), ... % rotation vector at 45 degrees
% %     quat_orig                       ... % input quaternion (sensor orientation)
% % );
% 
% quat_corr = zeros(size(quat_orig));
% for i = 1:length(quat_orig)
%     quat_corr(i,:) = ned2enuQuat(quat_orig(i,:));
% end
% 
% % 3.0 *** Take conjugate of corrected orientation to use as rotation transform
% % 3.1 *** Rotate body accelerations to Earth frame ***
% % acc = quaternRotate(acc_local, quaternConj(quat_local)); % default
% % acc = quaternRotate(acc_corr, quaternConj(quat_corr));
% acc = quaternRotate(acc_corr, quat_corr);
% 
% % % Remove gravity from measurements
% % acc = acc - [zeros(length(time), 2) ones(length(time), 1)];     % unnecessary due to velocity integral drift compensation
% 
% % Convert acceleration measurements to m/s/s
% % acc = acc * 9.81;
% 
% plotaccel(time, accxyz, acc)

