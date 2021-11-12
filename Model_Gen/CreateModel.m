% ------------------------------------------------------------------------
%
% University of Groningen. The Netherlands
% Script developed by: Martine Bol and Dr. Mauricio Muñoz (2012)
% Revised by: Haojun Ma and Carmen Chan-Zheng. (2021)
% Overview: This function returns the model of the Philips Experimental
% Robotic Arm (PERA). Model is based on Spong et. al. "Robot Modelling and
% Control". First edition.(2006)
%
% ------------------------------------------------------------------------
% Symbol        | Size            | Interpretation   
% ------------------------------------------------------------------------
% NLinks          1 x 1            Joint numbers of the target manipulator.
% DH_matrix       NLinks x 5        Standard D-H parameters of the target
%                                   manipulator. [a_i alpha_i d_i ac_i dc_i]
% A               4 x 4 x NLinks   A(:,:,i) denotes the homogeneous
%                                   transformation matrix from
%                                   link i -> link i-1.
% Ac              4 x 4 x NLinks   Ac(:,:,i) denotes the homogeneous
%                                   transformation matrix from
%                                   link i -> center of link i-1.
% T               4 x 4 x NLinks+1 T(:,:,i) denotes the homogeneous 
%                                   transformation matrix from
%                                   link i-1 -> link 0.
% Tc              4 x 4 x NLinks+1 Tc(:,:,i) denotes the homogeneous
%                                   transformation matrices from
%                                   link i-1 -> center of link 0.
% JC              3 x NLinks       Angular Jacobian for calculating the
%                                   body angular velocities of the
%                                   end-effector in base frame.
% JCA             3 x NLinks       Angular Jacobians for calculating the
%                    x NLinks       body angular velocities of each link in
%                                   the base frame.
% JL              3 x NLinks       Linear Jacobian for calculating the
%                                   body linear velocities of the
%                                   end-effector in base frame.
% JCL             3 x NLinks       Linear Jacobians for calculating the
%                    x NLinks       body linear velocities of each link in 
%                                   base frame.
% D               NLinks x NLinks  Mass-Inertial mmatrix of the target
%                                   manipulator.
% MM              1 x Nlinks       MM(i) denotes the mass of link i
% Chritoffel      NLinks x NLinks  Christoffel Symbols of the First Kind,
%                    x NLinks       used to compute the centrifugal and
%                                   Coriolis terms.
% C               NLinks x NLinks  C matrix
% CC              NLinks x 1       All centrifugal and Coriolis terms
% GG              1 x 1            Total potential energy
% G               NLinks x 1       Gravity terms  
% ------------------------------------------------------------------------
function [D,C,CC,G,dG,JA,JCA,JL,JCL] = CreateModel(NLinks)

[DH_matrix] = CreateDH_matrix(NLinks);                     % D-H parameters --standard DH table used here-- **
[A,AC,theta] = CreateA_matrices(DH_matrix,NLinks);          % Homogeneous Transformation matrix from link i -> i-1 p.81 **
[T,TC] = CreateT_matrices(NLinks,A,AC);                     % Transformation matrix from frame i -> base frame 0
[JA,JCA] = CreateJacobianAngular(NLinks,T);                 % Angular Jacobians p.135 **
[JL,JCL] = CreateJacobianLinear(NLinks,TC,T);               % Linear Jacobians p.137 **
[D,MM] = CreateInertiaMatrix(NLinks,TC,JCA,JCL);            % Inertial Matrix p.211 £¨T or TC both are right£©**
[Christoffel] = CreateChristoffel(NLinks,D,theta);          % Christoffel Symbols of the First Kind p.213
[C,CC] = CreateCentrifugalCoriolis(NLinks,Christoffel);     % All centrifugal and Coriolis terms p.214 **
[G,dG] = CreatePotentialMatrix(NLinks,MM,TC,theta);         % Potential energy and gravity terms **

end



