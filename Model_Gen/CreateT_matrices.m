% ***************************************************************************
% Overview: creates the transformation matrices. 
% See equations 3.3-3.6 
% T(:,:,i) : transformation matrices for full links distance.
% From frame i to frame 0
% TC(:,:,i) : transformation matrices with an arbitrary point (e.g. center
% of mass). From CoM of Links i to frame 0. See page 135, example 4.6. 
% ***************************************************************************
function [T,TC] = CreateT_matrices(NLinks,A,AC)

%Initializing 
T(:,:,1) = sym(eye(4));
TC(:,:,1) = sym(eye(4));

if NLinks > 1
    for i=2:NLinks+1
        T(:,:,i)=T(:,:,i-1)*A(:,:,i-1);
    end

    for i=2:NLinks+1
    TC(:,:,i)=T(:,:,i-1)*AC(:,:,i-1);
    end
end