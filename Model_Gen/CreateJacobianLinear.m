% ------------------------------------------------------------------------
% Overview: Calculate linear part of Jacobian. See section 4.6.3
% JCL: Jacobians to derive the velocity center of
% mass of each link in base frame.
% JL: for deriving velocity on end effector
% ------------------------------------------------------------------------
function [JL,JCL] = CreateJacobianLinear(NLinks,TC,T)
%JCL is the linear part of the momentum

JCL = sym(zeros([3 NLinks NLinks]));

for i = 1:NLinks
    for j = 1:i
        JCL(:,j,i) = cross(T(1:3,3,j),(TC(1:3,4,i+1)-T(1:3,4,j)));
    end
end
    
JL = sym(zeros([3 NLinks]));
for i = 1:NLinks
    JL(:,i) = cross(T(1:3,3,i),(T(1:3,4,NLinks+1) - T(1:3,4,i)));
end