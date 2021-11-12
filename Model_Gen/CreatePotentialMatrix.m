% ------------------------------------------------------------------------
% Overview: PERA manipulator points along the negative direction of z axis of
% base frame->[0 0 -1]. See Section 7.2.3.
% G: total potential energy
% dG: potential enrgy differentiated with respect to theta
% ------------------------------------------------------------------------
function [G,dG] = CreatePotentialMatrix(NLinks,MM,TC,theta)
syms g positive


G = sym(0);
for i = 1:NLinks
    G =  G + MM(i)*[0 0 1]*g*TC(1:3,4,i+1);
    %G =  G + MM(i)*[0 1 0]*g*TC(1:3,4,i+1); %For planar manipulator
    %example from Spong
end

dG = sym(zeros([NLinks 1]));
for j = 1:NLinks
    dG(j) = diff(G,theta(j));
end


  