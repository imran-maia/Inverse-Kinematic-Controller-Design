% Function for constructing the Homogeneous Transformation Matrix from DH table 
% [Note: this function is able to compute the Homogeneous Matrix of n link robot]

function T = Homogeneous(DH)
% Input: DH Table
% Output: Homogeneous Matrix of the (n) links

% Extract parameters from the DH table of n link 
a     = DH(1);
alpha = DH(2);
d     = DH(3);
theta = DH(4);

ct = cos(theta);
st = sin(theta);
ca = cos(alpha);
sa = sin(alpha);

% Calculating homogeneous matrix for nth link
T = [ct -st*ca st*sa  a*ct
     st ct*ca  -ct*sa a*st
     0  sa     ca     d
     0  0      0      1]; 

end

% End of the function for computing the Homogeneous Matrix for any dimnesion DH Table.
% Intoduction to Robotics Course
