function [X,Y,Theta]=read_pos(opti,frame)
    % Obtain Optitrack Data
    opti = readOptitrack(opti,frame);
    % Euler angles in degrees
    opti.pose(4:6,:) = rad2deg(opti.pose(4:6,:));
    opti.pose;
    %% optitrack Read
    X= -opti.pose(2,:);
    Y = -opti.pose(1,:);
    Theta = -opti.pose(6,:);
end