 function one_robot_p2p_test()
 % one robot, input leaderPos, point to point navigation
clear all; close all; clc; 
%% camera set optitrack setup
addpath ('Eric');
period_camera_update = 0.1;
% Select Inputs frame = 'Optitrack';
frame = 'XY Plane';
% Optitrack Initialization
opti = optitrackSetup(3000);

%% formation param set
num_pololu =1; 
% timer setting
for i = 1: num_pololu
    t(i) = timer;
    t(i).BusyMode = 'drop';
    t(i).TimerFcn ={@moveforward,num2str(i+2)};
end
timerBusy = zeros(num_pololu,1);

reachTarget = zeros(num_pololu,1);
leaderPos = [1,1];  % initial leader pos

%% Connect serial PC terminal
comNum = 'COM9'; BaudRate = 115200;
serialPC = connect_to_serialPC(comNum,BaudRate);

while(1)
    pause(period_camera_update);

   [x,y,theta]=read_pos(opti,frame);
    
    if reachTarget
        reachTarget = 0;
        leaderPos = input('input LeaderPos([x,y]): ');       %specify leader's next position
    end
    
    % command calculation()
    current_pos = [x, y];
    current_dir = theta;
    [heading_angle,dist] = cal_command(current_pos,current_dir,leaderPos);
    
    
    % execute command()
    %check position, unit: m , degree
    if (dist>0.05 && reachTarget ==0)
        if abs(heading_angle) > 5  % degree
            if  timerBusy == 0         % check timer
                turnPololu('3',serialPC,heading_angle);
            end
        end
    
    else
        fprintf(serialPC, '%c%c\n',strcat('3','s'),'sync');
        fprintf(serialPC, '%c%c\n',strcat('3','s'),'sync');
        fprintf(serialPC, '%c%c\n',strcat('3','s'),'sync');
        reachTarget = 1;
    end
    
end

function turnPololu(carID,serialPC,angle)
    turn_time = 2.1/360*abs(angle);
    id = str2double(carID);
    t(id).StartDelay= turn_time;
    timerBusy(id) = 1;
    start(t(id));
      
    if(angle >0) % turn left
       fprintf(serialPC, '%c%c\n',strcat(carID,'l'),'sync');
       fprintf(serialPC, '%c%c\n',strcat(carID,'l'),'sync');
    else
       fprintf(serialPC, '%c%c\n',strcat(carID,'r'),'sync');
       fprintf(serialPC, '%c%c\n',strcat(carID,'r'),'sync');
    end
        
end

function moveforward(obj,event,handles)
        
    fprintf(serialPC, '%c%c\n',strcat(handles,'f'),'sync');
    fprintf(serialPC, '%c%c\n',strcat(handles,'f'),'sync');
        
    id = str2double(handles);
    timerBusy(id) = 0;  
end

end






