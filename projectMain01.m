%% Introduction to Robotics: Final Project

% Task-A: The control objective is the position-only without exploiting the redundancy. 
% The required path is a vertical line along the axis z of appropriate length (without reaching boundary of the workspace or a kinematic singularity).
% The trajectory duration is 2 s with a trapezoidal velocity profile characterized by a proper cruise velocity.

clc;clear;close;

% Define parameters for Denavit-Hartenberg (DH) table 
a     = [0 0 0 0 0 0 0]';                               % link length in [meter]
alpha = [pi/2 pi/2 pi/2 pi/2 pi/2 pi/2 0]';             % link twist in  [radian]
d     = [0.2755 0 -0.410 -0.0098 -0.3111 0 0.2638]';    % link offset in [meter]
theta = [pi/2 0 0 -pi/2 0 0 0]';                        % joint angle in [radian]


% Define parameters for Tajectory
time_init  = 0;                       % initial time [sec]
time_final = 2;                       % final time [sec]
T          = 0.001;                   % sample time [sec]
t          = time_init:T:time_final;  % total time duration for tajectory [sec]
N          = length(t);               % total number of samples
s0         = 0;                       % initial value
s1         = 2;                       % final value
dsc        = 1.60;                    % cruise velocity [m/sec]


% Define some null matrices for the Tajectory 
q              = zeros(7,N);                     % joints' value 
dq             = zeros(7,N);                     % joints' velocity 

pos            = zeros(3,N);                     % current end-effector position 
pos_des        = zeros(3,N);                     % desired position tajectory 

lin_velo_des   = zeros(3,N);                     % desired linear velocity

error_position = zeros(3,N);                     % position error 

% Assign the values of joints 
q(:,1) = theta;                                  % joints' value [q = theta (angles of joints)]

% Construct the Denavit-Hartenberg (DH) table 
DH = [a alpha d theta];
% Compute the Direct Kinematics 
T0 = DirectKinematics(DH);    

% Insert the initial position of the end-effector from the Direct Kinematics
pos(:,1)  = T0(1:3,4,end);                 
pos_init  = pos(:,1);


% Define the final position of the end effector 
pos_final = pos_init - [0;0;0.2];                 % final position of the end-effector [initial position - desired position]
                                                  % the posistion of end-effector changes along z axis by 0.2 meter whereas x,y axis remain constant

% Set gains to amplify errors 
kpos = diag([10 10 10]);                   

% Loop for tajectory of the robot
for i=2:N
    % Compute the Tajector at time t
    [si, dsi, ddsi] = trapezoidal(s0, s1, dsc, time_final, t(i-1));
    
    pos_des(:,i-1)         = pos_init + (pos_final - pos_init) * si;    % desired position of the end-effector
    lin_velo_des(:,i-1)    = (pos_final-pos_init) * dsi;                % desired linear velocity
  
    % Compute the Direct Kinematics
    DH = [a alpha d q(:, i-1)];
    Tim1 = DirectKinematics(DH);

    % Find current position of the end-effector
    pos(:,i-1)      = Tim1(1:3,4,end);                           % current position of the end-effector
    
    % Compute the position error
    error_position(:, i-1) = pos_des(:, i-1) - pos(:, i-1);      % position error (desired postion - current position)
   
    % Calculate the Jacobian 
    J = Jacobian(DH);                       % current Jacobian 
    dq(:, i-1) = pinv(J(1:3,:)) * (lin_velo_des(:, i-1) + kpos * error_position(:,i-1)); % inserting the joints' velocity [considering linear velocity and position error only]
    q(:, i) = q(:, i-1) + dq(:, i-1)*T;     % inserting the joints' value [Numerical integration]
    
    % Plot the initial and final position with some intermadiate posiitons of the robot
    if (mod(i,420)==0)
        DrawRobot(DH);
        drawnow
    end
end
                           
% Compute the tajectory for the last step N
% Calculate the Direct Kinematics
DH = [a alpha d q(:, N)];
TN = DirectKinematics(DH);

% Find the current position of the end-effector from the Direct Kinematics
pos(:, N) = TN(1:3, 4, end);             

% Find the desired position of the end-effector
pos_des(:, N) = pos_init + (pos_final-pos_init) * 1;       


% Calculate the postion error 
error_ps(:, N) = pos_des(:, N) - pos(:, N);   % position error (desired postion - current position)


% Visualization of results

% Plot of joints' Positions
figure()
subplot(4, 2, 1)
plot(t, q(1, :))              % first joint configurations
xlabel('time [s]')
ylabel('q_1 [rad]')

subplot(4, 2, 2)
plot(t, q(2, :))              % second joint configurations
xlabel('time [s]')
ylabel('q_2 [rad]')

subplot(4, 2, 3)
plot(t, q(3, :))              % third joint configurations
xlabel('time [s]')
ylabel('q_3 [rad]')

subplot(4, 2, 4)
plot(t, q(4, :))              % four joint configurations
xlabel('time [s]')
ylabel('q_4 [rad]')

subplot(4, 2, 5)
plot(t, q(5, :))              % five joint configurations
xlabel('time [s]')
ylabel('q_5 [rad]')

subplot(4, 2, 6)
plot(t, q(6, :))              % six joint configurations
xlabel('time [s]')
ylabel('q_6 [rad]')

subplot(4, 2, 7)
plot(t, q(7, :))              % seven joint configurations
xlabel('time [s]')
ylabel('q_7 [rad]')
sgtitle('Plot for Joints Positions') 

% Plot of joints' velocity
figure();
subplot(4,2,1)
plot(t, dq(1, :))             % first joint velocities
xlabel('time [s]')
ylabel('dq_1 [rad/s]')

subplot(4,2,2)
plot(t, dq(2, :))             % second joint velocities
xlabel('time [s]')
ylabel('dq_2 [rad/s]')

subplot(4,2,3)
plot(t, dq(3, :))             % third joint velocities
xlabel('time [s]')
ylabel('dq_3 [rad/s]')

subplot(4,2,4)
plot(t, dq(4, :))             % four joint velocities
xlabel('time [s]')
ylabel('dq_4 [rad/s]')

subplot(4,2,5)
plot(t, dq(5, :))             % five joint velocities
xlabel('time [s]')
ylabel('dq_5 [rad/s]')

subplot(4,2,6)
plot(t, dq(6, :))             % six joint velocities
xlabel('time [s]')
ylabel('dq_6 [rad/s]')

subplot(4,2,7)
plot(t, dq(7, :))             % seven joint velocities
xlabel('time [s]')
ylabel('dq_7 [rad/s]')
sgtitle('Plot for Joints Velocities') 

% Plot of the position errors along x, y, and z axis
figure() 
subplot(3, 1, 1)
plot(t, error_position(1, :)) % postion error along x axis
xlabel('time [s]');
ylabel('epos_x [m]');

subplot(3, 1, 2)
plot(t, error_position(2, :)) % postion error along y axis
xlabel('time [s]');
ylabel('epos_y [m]');

subplot(3, 1, 3)
plot(t, error_position(3, :)) % postion error along z axis
xlabel('time [s]');
ylabel('epos_z [m]');
sgtitle('Plot for Position Errors') 

% End of the Project_______________________________________________________
% Md Imran Hossain, Erasmus Mundus Joint Master Degree in Meidcal Imaging and Applications (MAIA) 
% University of Cassino and Southern Lazio, Italy.
