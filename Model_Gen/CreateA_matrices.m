% ***************************************************************************
% Overview: creates the transformation matrices. 
% See equation 3.10
% A(:,:,i) : homogeneous transform matrix from one link to the next link
% AC(:,:,i) : homogeneous transform matrix from one link to the next
%centre of mass
% ***************************************************************************
function [A,AC,theta] = CreateA_matrices(DH_matrix,NLinks)
% The homogeneous transformation matrices Ai 

syms theta1 theta2 theta3 theta4 theta5 theta6 theta7 positive

theta = [theta1; theta2; theta3; theta4; theta5; theta6; theta7];

for i = 1:NLinks
    %A is the homogeneous transform matrix from one link to the next link
    A(:,:,i) = [
        cos(theta(i))   -sin(theta(i))*cos(DH_matrix(i,2))  sin(theta(i))*sin(DH_matrix(i,2))   DH_matrix(i,1)*cos(theta(i));
        sin(theta(i))   cos(theta(i))*cos(DH_matrix(i,2))   -cos(theta(i))*sin(DH_matrix(i,2))  DH_matrix(i,1)*sin(theta(i));
        0               sin(DH_matrix(i,2))                 cos(DH_matrix(i,2))                 DH_matrix(i,3);
        0               0                                   0                                   1;
        ];
end

for i = 1:NLinks
    %AC is the homogeneous transform matrix from one link to the next
    %centre of mass
    AC(:,:,i) = [
        cos(theta(i))   -sin(theta(i))*cos(DH_matrix(i,2))  sin(theta(i))*sin(DH_matrix(i,2))   DH_matrix(i,4)*cos(theta(i));
        sin(theta(i))   cos(theta(i))*cos(DH_matrix(i,2))   -cos(theta(i))*sin(DH_matrix(i,2))  DH_matrix(i,4)*sin(theta(i));
        0               sin(DH_matrix(i,2))                 cos(DH_matrix(i,2))                 DH_matrix(i,5);
        0               0                                   0                                   1;
        ];
end