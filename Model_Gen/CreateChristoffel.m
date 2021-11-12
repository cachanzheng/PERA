% ------------------------------------------------------------------------
% Overview: Creates Christoffel symbols. See equation 7.60
% ------------------------------------------------------------------------
function [Christoffel] = CreateChristoffel(NLinks,D,theta)

Christoffel = sym(zeros([NLinks,NLinks,NLinks]));

for k = 1:NLinks
    for j = 1:NLinks
        for i = 1:NLinks
            Christoffel(i,j,k) = 0.5*(diff(D(k,j),theta(i))+diff(D(k,i),theta(j)) - diff(D(i,j),theta(k)));
        end
    end
end
