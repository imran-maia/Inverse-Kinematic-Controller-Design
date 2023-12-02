% Function for constructing the Jacobian Matrix from DH table 
% [Note: this function is able to compute the Jacobian Matrix of n link robot]
% The dimension of the Jacobian Matrix always should be 6XN ; where, N = number of links of the robot

function [J] = Jacobian(DH)
% Input  : DH Table
% Output : Jacobian Matrix [6XN, where N = number of links of the robot/number of the rows in DH table]

n = size(DH,1);      % find the number of links of the robot (number of rows of the DH table)
p0 = [0 0 0]';       % define the position vector of the link 0 (at the origin) of the robot [all coordinates are zero x,y,z = 0,0,0]
z0 = [0 0 1]';       % define the position of z0 at link 0 (z0 = coordinates x,y = 0,0 & z=1 because unit vector in the direction of z)
p = zeros(3,n);      % construct matrices for storing the position of the n links (all links) of the robot [p1, p2 ...., pn; pn=pe, where pe indicates position of end-efector]

% loop for calculating homogeneous matrix,  for n number of link
for i=1:n
    T_i = Homogeneous(DH(i,:));  % Calculating Homogeneous Matrix of n-th link
                                % Input: n-th row of the DH table 
                                % Output: Homogeneous Matrix of the n-th row;
                                % where n-th row indicates each link of the robot.
    
    % If the number of the link 1 of the robot
    if(i==1)
        T(:,:,i) = T_i;          % inserting the Homogeneous matrix of the link 1 into the n number Homogeneous matrix
                                % where nth number Homogeneous matrix = matrices for nth link
    end

    % If the number of the link 2 of the robot
    if(i>=2)
        T(:,:,i) = T(:, :, i-1)*T_i;  % inserting the Homogeneous matrix of the links 2 and onwardsinto the n number Homogeneous matrix
                                      % where nth number Homogeneous matrix = matrices for nth link
    end
    
    % The final Direct Kinematics (Homogeneous Matrix) of all the links of the robot
    T0i = T(:, :, i);                 % inserting all (n number) Homogeneous matrix of all links (n number) into a variable

    % Extract position matrix of each link of the robot
    p(:, i) = T0i(1:3, 4);            % creating position vector matrix by extracting position vectors of the n-th link from the n-t Homogeneous Matrix
end

pe = p(:, n);                         % extracting the position vector of the end efector from position vector matrix [last column of the position matrix]

% Construct Jacobian Matrix [6xN]
J = zeros(6,n);                       % create an empty matrix to store elements for the Jacobian Matrix
J(:,1) = [cross(z0, pe-p0); z0];      % inserting the elements of the Jacobian matrix for link 1 

for i=2:n
    T0im1 = T(:,:,i-1);               % Homogeneous matrix of link 2 to onwards
    R0im1 = T0im1(1:3, 1:3);          % rotation matrix of link 2 to onwards
    z0im1 = R0im1(:,3);               % vector along z axis of link to onwards
    p0im1 = p(:,i-1);                 % position vectors of link 2 to onwards

    J(:,i) = [cross(z0im1, pe-p0im1); z0im1]; % inserting the vaulue of the Jacobian matrix for link 2 to onwards
end


end

% End of the function for computing the Jacobian Matrix for any dimnesion DH Table.
% Intoduction to Robotics Course
