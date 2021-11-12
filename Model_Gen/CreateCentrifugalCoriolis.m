% ------------------------------------------------------------------------
% Overview: Returns centrifugal and coriolis terms. See Section 7.3
% ------------------------------------------------------------------------
function [C, CC] = CreateCentrifugalCoriolis(NLinks,Christoffel)

syms dtheta1 dtheta2 dtheta3 dtheta4 dtheta5 dtheta6 dtheta7
dtheta = [dtheta1; dtheta2; dtheta3; dtheta4; dtheta5; dtheta6; dtheta7];
dtheta = dtheta(1:NLinks);

C = sym(zeros(NLinks, NLinks));
for k = 1:NLinks
    for j = 1:NLinks
        for i = 1:NLinks
            C(k,j) = C(k,j) + Christoffel(i,j,k)*dtheta(i);
        end
    end
end

CC = sym(zeros([NLinks,1]));
for k = 1:NLinks
    for j = 1:NLinks
        for i = 1:NLinks
            CC(k) = CC(k) + Christoffel(i,j,k)*dtheta(i)*dtheta(j);
        end
    end
end






