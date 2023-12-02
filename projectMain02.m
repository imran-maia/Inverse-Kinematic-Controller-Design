%% Introduction to Robotics: Final Project

% Task-B: The control objective is the position-only without exploiting the redundancy. 
% The required path is a vertical square lying on the yz axis with side of appropriate length (without reaching boundary of the workspace or a kinematic singularity). 
% The end-effector occupies the lower-left corner of the square looking from the x axis. 
% Each side needs to be executed with a trajectory of duration 2 s and a trapezoidal velocity profile characterized by a proper cruise velocity. 
% The robots stops for 500 ms at each corner and 2 s once back in the initial end effector position.

clear variables; clc; clear; close all 


% Define parameters for Denavit-Hartenberg (DH) table 
a     = [0 0 0 0 0 0 0]';                               % link length in [meter]
alpha = [pi/2 pi/2 pi/2 pi/2 pi/2 pi/2 0]';             % link twist in  [radian]
d     = [0.2755 0 -0.410 -0.0098 -0.3111 0 0.2638]';    % link offset in [meter]
theta = [pi/2 -pi/16 0 pi/4 -pi/2 pi/3 pi/3]';          % joint angle in [radian]

% Define parameters for Tajectory
time_init  = 0;                       % initial time [sec]
time_final = 2.5;                     % final time [sec]
T          = 0.001;                   % sample time [sec]
t          = time_init:T:time_final;  % total time duration for tajectory [sec]
N          = length(t);               % total number of samples
s0         = 0;                       % initial value
s1         = 1;                       % final value
dsc        = 1.40;                    % cruise velocity [m/s]

% Define some null matrices for the Tajectory 
q              = zeros(7, N);         % joints' value 
dq             = zeros(7, N);         % joints' velocity 

pos            = zeros(3, N);         % current end-effector position
pos_des        = zeros(3, N);         % desired position tajectory 

lin_velo_des   = zeros(3, N);         % desired linear velocity

error_position = zeros(3,N);          % position error 

% Define the number of tajectory
num_taj = 4;                          % we have total five tajector; the first one for bringing the end-effector
                                      % from the initial position to left bottom corner and other four tajectory for drwaing the square.

% Define some null matrices for polting joints' value, joints' velocity, and position error for each tajectory
total_position_error = zeros(3,N,num_taj+1); 
total_q              = zeros(7,N,num_taj+1);
total_dq             = zeros(7,N,num_taj+1);

% Assign the values of joints
q(:,1) = theta;                      % joints' value [q = theta (angles of joints)]

% Construct the Denavit-Hartenberg (DH) table 
DH = [a alpha d theta];
% Compute the Direct Kinematics 
T0 = DirectKinematics(DH); 

% Insert the initial position of the end-effector from the Direct Kinematics
pos(:,1)  = T0(1:3,4,end);                 
pos_init  = pos(:,1);

% Set gains to amplify errors 
kpos = diag([10 10 10]);      

% Define the final positon of each tajectory [last 4 column for the square and first column for initial adjustment]
pos_final = [0.15 0.15 0.15 0.15 0.15;
             0.15 0.35 0.35 0.15 0.15;
             0.15 0.15 0.35 0.35 0.15];

% Loop for tajectory of the robot
for m = 1:num_taj+1

    for i = 2:N
        % Compute the Tajector at time t
        [si, dsi, ddsi] = Trapezoid(s0, s1, dsc, time_init, time_final-0.5, t(i-1));

        pos_des(:, i-1) = pos_init + (pos_final(:,m) - pos_init) * si;   % desired position of the end-effector
        lin_velo_des(:, i-1) = ( pos_final(:,m) - pos_init) * dsi;       % desired linear velocity
        
        % Compute the Direct Kinematics
        DH = [a alpha d q(:, i-1)];
        Tim1 = DirectKinematics(DH);

        % Find current position of the end-effector
        pos(:, i-1) = Tim1(1:3, 4, end);                                  % current position of the end-effector
        
        % Compute the position and orientation error
        error_position(:, i-1) = pos_des(:, i-1)-pos(:, i-1);             % position error (desired postion - current position)


        % Calculate the Jacobian
        J = Jacobian(DH);                    % current Jacobian 
        dq(:, i-1) = pinv(J(1:3,:)) * (lin_velo_des(:, i-1) + kpos * error_position(:, i-1));  % inserting the joints' velocity [considering linear velocity and position error only] 
        q(:, i) = q(:, i-1)+dq(:, i-1)*T;    % inserting the joints' value [Numerical integration]                            
        
        % Plot the initial and final position with some intermadiate posiitons of each tajectory
        if (mod(i,500)==0)
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

    % Calculate the postion error 
    error_position(:, i-1) = pos_final(:,m)-pos(:, N);  
    
    q(:,1) = q(:, N);
    pos_init = pos(:, N);
    
    % Assign joints' value, velocity, and posstion error for each tajectory for plotting 
    total_q(:,:,m)              = q;                         
    total_dq(:,:,m)             = dq;
    total_position_error(:,:,m) = error_position;
end
  
 
  % Total time duration for intial adjustment and square tajectory
  total_time = 0:T:12.504;
  
  % Visualization of robot joints' angle, joints' velocity and position error
  % In the case of plotting, we will ommit the adjustment tajectory (from 0 sec to 2.5 sec) to bring the end-effector near to left botom corner.
  % Only the plot will be shown from 2.5 sec to 12.5 sec (tajectory for square)
  

  % Plot of joints' Positions
  figure()
  subplot(421)                   % first joint configurations
  plot(total_time(2502:end), [total_q(1,:,2) total_q(1,:,3) total_q(1,:,4) total_q(1,:,5)])
  xlabel('time [s]')
  ylabel('q_1 [rad]')
  axis tight
  
  subplot(422)                   % second joint configurations
  plot(total_time(2502:end), [total_q(2,:,2) total_q(2,:,3) total_q(2,:,4) total_q(2,:,5)])
  xlabel('time [s]')
  ylabel('q_2 [rad]')
  axis tight

  subplot(423)                   % third joint configurations
  plot(total_time(2502:end), [total_q(3,:,2) total_q(3,:,3) total_q(3,:,4) total_q(3,:,5)])
  xlabel('time [s]')
  ylabel('q_3 [rad]')
  axis tight

  subplot(424)                   % four joint configurations
  plot(total_time(2502:end), [total_q(4,:,2) total_q(4,:,3) total_q(4,:,4) total_q(4,:,5)])
  xlabel('time [s]')
  ylabel('q_4 [rad]')
  axis tight

  subplot(425)                   % fifth joint configurations
  plot(total_time(2502:end), [total_q(5,:,2) total_q(5,:,3) total_q(5,:,4) total_q(5,:,5)])
  xlabel('time [s]')
  ylabel('q_5 [rad]')
  axis tight

  subplot(426)                   % sixth joint configurations
  plot(total_time(2502:end), [total_q(6,:,2) total_q(6,:,3) total_q(6,:,4) total_q(6,:,5)])
  xlabel('time [s]')
  ylabel('q_6 [rad]')
  axis tight

  subplot(427)                   % seventh joint configurations
  plot(total_time(2502:end), [total_dq(7,:,2) total_dq(7,:,3) total_dq(7,:,4) total_dq(7,:,5)])
  xlabel('time [s]')
  ylabel('q_7 [rad]')
  axis tight
  sgtitle('Plot for Joints Positions') 
  
  % Plot of joints' velocity
  figure()
  subplot(421)                   % first joint velocities
  plot(total_time(2502:end), [total_dq(1,:,2) total_dq(1,:,3) total_dq(1,:,4) total_dq(1,:,5)])
  xlabel('time [s]')
  ylabel('dq_1 [rad/s]')
  axis tight

  subplot(422)                   % second joint velocities
  plot(total_time(2502:end), [total_dq(2,:,2) total_dq(2,:,3) total_dq(2,:,4) total_dq(2,:,5)])
  xlabel('time [s]')
  ylabel('dq_2 [rad/s]')
  axis tight

  subplot(423)                   % third joint velocities
  plot(total_time(2502:end), [total_dq(3,:,2) total_dq(3,:,3) total_dq(3,:,4) total_dq(3,:,5)])
  xlabel('time [s]')
  ylabel('dq_3 [rad/s]')
  axis tight

  subplot(424)                   % four joint velocities
  plot(total_time(2502:end), [total_dq(4,:,2) total_dq(4,:,3) total_dq(4,:,4) total_dq(4,:,5)])
  xlabel('time [s]')
  ylabel('dq_4 [rad/s]')
  axis tight

  subplot(425)                   % five joint velocities     
  plot(total_time(2502:end), [total_dq(5,:,2) total_dq(5,:,3) total_dq(5,:,4) total_dq(5,:,5)])
  xlabel('time [s]')
  ylabel('dq_5 [rad/s]')
  axis tight

  subplot(426)                   % six joint velocities
  plot(total_time(2502:end), [total_dq(6,:,2) total_dq(6,:,3) total_dq(6,:,4) total_dq(6,:,5)])
  xlabel('time [s]')
  ylabel('dq_6 [rad/s]')
  axis tight

  subplot(427)                   % seven joint velocities
  plot(total_time(2502:end), [total_dq(7,:,2) total_dq(7,:,3) total_dq(7,:,4) total_dq(7,:,5)])
  xlabel('time [s]')
  ylabel('dq_7 [rad/s]')
  axis tight
  sgtitle('Plot for Joints Velocities') 

  % Plot of position error
  figure()
  subplot(311)              % position error along x axis
  plot(total_time(2502:end), [total_position_error(1,:,2) total_position_error(1,:,3) total_position_error(1,:,4) total_position_error(1,:,5)])   
  xlabel('time [s]');
  ylabel('epos_x [m]');
  axis tight
 
  subplot(312)              % position error along y axis
  plot(total_time(2502:end), [total_position_error(2,:,2) total_position_error(2,:,3) total_position_error(2,:,4) total_position_error(2,:,5)])
  xlabel('time [s]');
  ylabel('epos_y [m]');
  axis tight

  subplot(313)              % position error along z axis
  plot(total_time(2502:end), [total_position_error(3,:,2) total_position_error(3,:,3) total_position_error(3,:,4) total_position_error(3,:,5)])
  xlabel('time [s]');
  ylabel('epos_z [m]');
  axis tight
  sgtitle('Plot for Position Errors') 

% End of the Project_______________________________________________________
% Md Imran Hossain, Erasmus Mundus Joint Master Degree in Meidcal Imaging and Applications (MAIA) 
% University of Cassino and Southern Lazio, Italy.
 