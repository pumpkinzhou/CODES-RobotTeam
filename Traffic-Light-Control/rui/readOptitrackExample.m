%--------------------------------------------------------------------------
%
% File Name:      readOptitrackExample.m
% Date Created:   2014/07/09
% Date Modified:  2014/07/10
%
% Author:         Eric Cristofalo
% Contact:        eric.cristofalo@gmail.com
%
% Description:    Example script for obtaining Optitrack data in MATLAB.
%
% Instructions:   Script needs the following functions:
%                    optiTrackSetup.m, parseNatNet.m,
%                    readOptitrack.m, quatrn2rot.m, rot2ZYXeuler.m
%
% Inputs:         N/A
%
% Outputs:        N/A
%
% Example:        N/A
%
%--------------------------------------------------------------------------

clear all; close all; clc;
addpath ('Eric');
% Select Inputs
% frame = 'Optitrack';
frame = 'XY Plane';

% Optitrack Initialization
opti = optitrackSetup(3000);

% Obtain Optitrack Data
while(1)
    opti = readOptitrack(opti,frame);
    % Euler angles in degrees
    opti.pose(4:6,:) = rad2deg(opti.pose(4:6,:));
    opti.pose
    Y = -opti.pose(2,:)
    X = -opti.pose(1,:)
    Theta = opti.pose(6,:);
    pause(0.2)
end












