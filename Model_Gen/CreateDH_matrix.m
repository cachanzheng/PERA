% ***************************************************************************
% Overview: defines the Denavit-Hartenberg matrix for the PERA. 
% Uses standard DH convention.
% NLinks: number of joints wanted. 
% Output: Denavit-Hartenberg matrix. 
% ***************************************************************************
function [DH_matrix] = CreateDH_matrix2(NLinks)
    syms al1 al2 al3 al4 al5 al6 al7 positive
    syms alpha1 alpha2 alpha3 alpha4 alpha5 alpha6 alpha7 
    syms dl1 dl2 dl3 dl4 dl5 dl6 dl7 positive
    syms acl1 acl2 acl3 acl4 acl5 acl6 acl7 positive
    syms dcl1 dcl2 dcl3 dcl4 dcl5 dcl6 dcl7 positive

    if NLinks > 7 
        disp('Too many links. Maximum number of links is 7.')
        return
    end

% ***************************************************************************
% Example Planar manipulator from Chap.7 of Robot modeling and Control 
% Second Edition by Mark Spong et al.
% Caveat vector of potential energy in CreatePotentialMatrix.m must be 
% [0 1 0] since gravity is in Y-direction
% DH_matrix = [   al1   0         0        acl1      0         ;...
%                 al2   0         0        acl2      0         ;];
% ***************************************************************************

% ***************************************************************************
% PERA configuration: Roll Shoulder - Pitch Elbow - Roll Elbow
% DH_matrix=[0      pi/2     -dl1     0     -dcl1   ;
%            0      -pi/2       0     0         0   ;
%            0          0    -dl3     0   -dcl3     ;]
% ***************************************************************************

% ***************************************************************************
%PERA Configuration Shoulder: Yaw-Pitch-Roll. Elbow: Pitch-Roll
DH_matrix=[0    pi/2    0   0   0;
           0    -pi/2   0   0   0;
           0    pi/2    -dl3    0 -dcl3;
           0    -pi/2   0       0   0  ;
           0    -pi/2   -dl5    0 -dcl5];
% ***************************************************************************



DH_matrix=DH_matrix(1:NLinks,:);
