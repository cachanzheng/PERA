% ------------------------------------------------------------------------
% Overview: Creates inertia matrix. See equation 7.50.
% Caveat: This script assumes that the inertia (I_i) is scalar
% for simplicity.
% ------------------------------------------------------------------------
function [D,MM] = CreateInertiaMatrix(NLinks,T,JCA,JCL)

%define masses M and inertia I:
syms m1 m2 m3 m4 m5 m6 m7 I1 I2 I3 I4 I5 I6 I7 positive

MM = [m1 m2 m3 m4 m5 m6 m7];
I = [I1 I2 I3 I4 I5 I6 I7];
M = MM(1:NLinks);
I = I(1:NLinks);

D = 0;
for i = 1:NLinks
    D = D + M(i)*transpose(JCL(:,:,i))*JCL(:,:,i) + transpose(JCA(:,:,i))*T(1:3,1:3,i)*I(i)*transpose(T(1:3,1:3,i))*JCA(:,:,i);
end

