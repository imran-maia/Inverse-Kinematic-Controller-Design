% Function for computing the Direct Kinematics from the DH table 
% [Note: this function is able to compute the Direct Kinematics Matrix of n link robot using Homogeneous Matrix]

function  T0 = DirectKinematics(DH)
% Input  : DH Table
% Output : DirectKinematics [Multiplication of Homogeneous Matrices of n link robots: A01*A12*A23-----A(N-1,N)]

    % Homogeneous Transformation matrices with respect to the arm base frame 
    n = size(DH,1);               % extract the number of link of the robot [number of the rows in the DH table]
    T0 = zeros(4,4,n);            % create empty matrices with the dimension of [rows = 4, columns = 4, and number of matrix = n (total number of link)]
    T  = zeros(4,4,n);

    % Homogeneous Transformation matrix between consecutive frames according to DH convention
    for i=1:n
        T(:,:,i) = Homogeneous(DH(i,:));
    end

    % Homogeneous Transformation matrices with respect to the arm base frame. T0(:,:,n) contains the Homogeneous 
    % Transformation matrix between the end effector and the arm base frame
    T0(:,:,1) = T(:,:,1);

    for i=2:n
        T0(:,:,i) = T0(:,:,i-1) * T(:,:,i);
    end

end

% End of the function for calculating Direct Kinematics for any dimnesion DH Table or n link robot.
% Intoduction to Robotics Course
