% ***************************************************************************
% Overview: Obtains angular jacobian. See Section 4.6.3 
% JCA: Angular jacobian with center of mass (if uses TC, we obtain same
% results).
% JA: Angular jacobian end effector. 
% ***************************************************************************
function [JA,JCA] = CreateJacobianAngular(NLinks,T)

JCA = sym(zeros(3,NLinks,NLinks));
for i = 1:NLinks
    for j = i:NLinks
        JCA(:,i,j) = T(1:3,3,i);
    end
end

JA = sym(zeros(3,NLinks));
for i = 1:NLinks
        JA(:,i) = T(1:3,3,i);
end