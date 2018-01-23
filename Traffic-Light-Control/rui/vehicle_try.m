function vehicle_try()
 % one robot, input leaderPos, point to point navigation
clear all; close all; clc; 
%define the path of each vehicle

path(1,1,:)=[1 1];path(1,2,:)=[1.7 1];path(1,3,:)=[0 0];%define path1. [0 0] is stopping mark 
path(2,1,:)=[1 1.2];path(2,2,:)=[1 0.3];path(2,3,:)=[0 0];%define path2. [0 0] is stopping mark
%serial number of traffic light in each intersection
light(1,1)=1;light(1,2)=2;light(2,1)=3;light(2,2)=4;
%initial traffic light color
%light_color=[0,1,1,1];
num_pololu =3;%number of cars in system
N=zeros(num_pololu,1);%mark the serial number of next traffic light in path for every car.
current=zeros(num_pololu,1);% record the serial number of the current traffic light
TT=zeros(num_pololu,1);
START=zeros(num_pololu,1);
pathnum=zeros(num_pololu,1);
state=zeros(num_pololu,1);
% % camera set optitrack setup
addpath ('Eric');
period_camera_update = 0.1;
% Select Inputs frame = 'Optitrack';
frame = 'XY Plane';
% Optitrack Initialization
opti = optitrackSetup(3000);
%% formation param set

% timer setting
 for i = 1 : num_pololu
%      t(i) = timer;
%      t(i).BusyMode = 'drop';
%   %   t(i).TimerFcn ={@moveforward,num2str(i)};
     N(i)=1;%initial N and current
     current(i)=1;
 end
pathnum(1)=1;
%pathnum(2)=1;
pathnum(3)=1;
timerBusy = zeros(num_pololu,1);

reachTarget = zeros(num_pololu,1);
%leaderPos = [1,1];  % initial leader pos

%% Connect serial PC terminal
comNum = 'COM3'; BaudRate = 115200;
serialPC = connect_to_serialPC(comNum,BaudRate);

while(1)
   pause(0.05);
   mydata=load('light_color');
   light_color=mydata.light_color;
   [xc,yc,thetac]=read_pos(opti,frame);
   
   
   %************************************************************************************
   %carID=1; pathnum is path number
   CARID='1';
     current_pos = [xc(str2double(CARID)), yc(str2double(CARID))];
    current_dir = thetac(str2double(CARID));
   %leaderPos=path(pathnum,N(str2double(CARID)));  
  if(path(pathnum(str2double(CARID)),current(str2double(CARID)),1)~=0&&path(pathnum(str2double(CARID)),current(str2double(CARID)),2)~=0)
      if START(str2double(CARID))==0
           [heading_angle,dist] = cal_command(current_pos,current_dir,[path(pathnum(str2double(CARID)),N(str2double(CARID)),1),path(pathnum(str2double(CARID)),N(str2double(CARID)),2)]);  
      elseif START(str2double(CARID))==1
        
      if pathnum(str2double(CARID))==2
           [heading_angle,dist] = cal_command(current_pos,current_dir,[1 2]);
      elseif pathnum(str2double(CARID))==1
           [heading_angle,dist] = cal_command(current_pos,current_dir,[0.3 1]);
      end
      
      end
    % command calculation()
  
    % execute command()
    %check position, unit: m , degree
    %light_color(light(num,N))
    if(detect_collision()==1)
     if state(str2double(CARID))==1 && abs(heading_angle)<2
         state(str2double(CARID))=0;
     end
     if state(str2double(CARID))==0
    if dist > 0.05 && reachTarget(str2double(CARID)) == 0
       % current(str2double(CARID))=N(str2double(CARID));
       if abs(heading_angle) <= 8  % degree
        fprintf(serialPC, '%c%c\n',strcat(CARID,'f'),'sync');
        fprintf(serialPC, '%c%c\n',strcat(CARID,'f'),'sync');
        fprintf(serialPC, '%c%c\n',strcat(CARID,'f'),'sync');
       elseif abs(heading_angle) > 8  % degree
           
                turnPololu(CARID,serialPC,heading_angle);
                state(str2double(CARID))=1;
        end
        
    else
        if START(str2double(CARID))==0
        if(light_color(light(pathnum(str2double(CARID)),current((str2double(CARID)))))==1)   %red light
        fprintf(serialPC, '%c%c\n',strcat(CARID,'s'),'sync');
        fprintf(serialPC, '%c%c\n',strcat(CARID,'s'),'sync');
        fprintf(serialPC, '%c%c\n',strcat(CARID,'s'),'sync');
        reachTarget(str2double(CARID)) = 1;
        
        N(str2double(CARID))=current(str2double(CARID))+1;
        elseif light_color(light(pathnum(str2double(CARID)),current(str2double(CARID))))==0   %green light
            reachTarget(str2double(CARID))=0;
            N(str2double(CARID))=current(str2double(CARID))+1;
            current(str2double(CARID))=N(str2double(CARID));
            fprintf(serialPC, '%c%c\n',strcat(CARID,'f'),'sync');
    fprintf(serialPC, '%c%c\n',strcat(CARID,'f'),'sync');
    fprintf(serialPC, '%c%c\n',strcat(CARID,'f'),'sync');
    fprintf(serialPC, '%c%c\n',strcat(CARID,'f'),'sync');
    fprintf(serialPC, '%c%c\n',strcat(CARID,'f'),'sync');
        end 
        elseif START(str2double(CARID))==1
           START(str2double(CARID))=0;
        end
    end
     end
    elseif detect_collision()==0
        TT(str2double(CARID))=1;
     %   timerBusy(str2double(CARID)) = 1;
        fprintf(serialPC, '%c%c\n',strcat(CARID,'s'),'sync');
        fprintf(serialPC, '%c%c\n',strcat(CARID,'s'),'sync');
        fprintf(serialPC, '%c%c\n',strcat(CARID,'s'),'sync');
        fprintf(serialPC, '%c%c\n',strcat(CARID,'s'),'sync');
        
    end
  elseif (path(pathnum(str2double(CARID)),current(str2double(CARID)),1)==0&&path(pathnum(str2double(CARID)),current(str2double(CARID)),2)==0)
      if pathnum(str2double(CARID))==1
          pathnum(str2double(CARID))=2;
          N(str2double(CARID))=1;%renew N and current
          current(str2double(CARID))=1;
      elseif pathnum(str2double(CARID))==2
          pathnum(str2double(CARID))=1;
          N(str2double(CARID))=1;%renew N and current
          current(str2double(CARID))=1;
      end
      START(str2double(CARID))=1;

  end

 
  
    %************************************************************************************
   %carID=3; pathnum is path number
%    CARID='3';
%      current_pos = [xc(str2double(CARID)), yc(str2double(CARID))];
%     current_dir = thetac(str2double(CARID));
%    %leaderPos=path(pathnum,N(str2double(CARID)));  
%   
%   if(path(pathnum(str2double(CARID)),current(str2double(CARID)),1)~=0&&path(pathnum(str2double(CARID)),current(str2double(CARID)),2)~=0)
%       if START(str2double(CARID))==0
%            [heading_angle,dist] = cal_command(current_pos,current_dir,[path(pathnum(str2double(CARID)),N(str2double(CARID)),1),path(pathnum(str2double(CARID)),N(str2double(CARID)),2)]);  
%       elseif START(str2double(CARID))==1
%         
%       if pathnum(str2double(CARID))==2
%            [heading_angle,dist] = cal_command(current_pos,current_dir,[1 2]);
%       elseif pathnum(str2double(CARID))==1
%            [heading_angle,dist] = cal_command(current_pos,current_dir,[0.3 1]);
%       end
%       
%       end
%     % command calculation()
%   
%     % execute command()
%     %check position, unit: m , degree
%     %light_color(light(num,N))
%     if(detect_collision()==1)
%         if TT(str2double(CARID))==1
%             %timerBusy(str2double(CARID))=0;
%             TT(str2double(CARID))=0;
%             fprintf(serialPC, '%c%c\n',strcat(CARID,'f'),'sync');
%             fprintf(serialPC, '%c%c\n',strcat(CARID,'f'),'sync');
%             fprintf(serialPC, '%c%c\n',strcat(CARID,'f'),'sync');
%             fprintf(serialPC, '%c%c\n',strcat(CARID,'f'),'sync');
%             fprintf(serialPC, '%c%c\n',strcat(CARID,'f'),'sync');
%             fprintf(serialPC, '%c%c\n',strcat(CARID,'f'),'sync');
%             fprintf(serialPC, '%c%c\n',strcat(CARID,'f'),'sync');
%         end
%     if dist > 0.05 && reachTarget(str2double(CARID)) == 0
%        % current(str2double(CARID))=N(str2double(CARID));
%         if abs(heading_angle) > 5  % degree
%             if  timerBusy(str2double(CARID)) == 0         % check timer
%                 turnPololu(CARID,serialPC,heading_angle);
%             end
%         end
%         
%     else
%         if START(str2double(CARID))==0
%         if(light_color(light(pathnum(str2double(CARID)),current((str2double(CARID)))))==1)   %red light
%         fprintf(serialPC, '%c%c\n',strcat(CARID,'s'),'sync');
%         fprintf(serialPC, '%c%c\n',strcat(CARID,'s'),'sync');
%         fprintf(serialPC, '%c%c\n',strcat(CARID,'s'),'sync');
%         reachTarget(str2double(CARID)) = 1;
%         N(str2double(CARID))=current(str2double(CARID))+1;
%         elseif light_color(light(pathnum(str2double(CARID)),current(str2double(CARID))))==0   %green light
%             reachTarget(str2double(CARID))=0;
%             N(str2double(CARID))=current(str2double(CARID))+1;
%             current(str2double(CARID))=N(str2double(CARID));
%             fprintf(serialPC, '%c%c\n',strcat(CARID,'f'),'sync');
%     fprintf(serialPC, '%c%c\n',strcat(CARID,'f'),'sync');
%     fprintf(serialPC, '%c%c\n',strcat(CARID,'f'),'sync');
%     fprintf(serialPC, '%c%c\n',strcat(CARID,'f'),'sync');
%     fprintf(serialPC, '%c%c\n',strcat(CARID,'f'),'sync');
%         end 
%         elseif START(str2double(CARID))==1
%            START(str2double(CARID))=0;
%         end
%     end
%     elseif detect_collision()==0
%         TT(str2double(CARID))=1;
%        
%         fprintf(serialPC, '%c%c\n',strcat(CARID,'s'),'sync');
%         fprintf(serialPC, '%c%c\n',strcat(CARID,'s'),'sync');
%         fprintf(serialPC, '%c%c\n',strcat(CARID,'s'),'sync');
%         fprintf(serialPC, '%c%c\n',strcat(CARID,'s'),'sync');
%        
%         
%         
%     end
%   elseif (path(pathnum(str2double(CARID)),current(str2double(CARID)),1)==0&&path(pathnum(str2double(CARID)),current(str2double(CARID)),2)==0)
%       if pathnum(str2double(CARID))==1
%           pathnum(str2double(CARID))=2;
%           N(str2double(CARID))=1;%renew N and current
%           current(str2double(CARID))=1;
%       elseif pathnum(str2double(CARID))==2
%           pathnum(str2double(CARID))=1;
%           N(str2double(CARID))=1;%renew N and current
%           current(str2double(CARID))=1;
%       end
%       START(str2double(CARID))=1;
% 
%   end
  
  %*********************************************************************************************

end



function turnPololu(carID,serialPC,angle)
  
   
    if(angle >0) % turn left
       fprintf(serialPC, '%c%c\n',strcat(carID,'l'),'sync');
       fprintf(serialPC, '%c%c\n',strcat(carID,'l'),'sync');
    else
       fprintf(serialPC, '%c%c\n',strcat(carID,'r'),'sync');
       fprintf(serialPC, '%c%c\n',strcat(carID,'r'),'sync');
    end
        
end


%**********************************************************************
%if result=1, there is no car before;else if result=0, car should stop to avoid collision.
%the serial number of current traffic light in path for every car.
function result=detect_collision()
result=1; 
 for j=1:1:num_pololu
    if(str2double(CARID)~=j)
        if(norm([xc(str2double(CARID)), yc(str2double(CARID))]-[xc(j), yc(j)])<0.2)
            if START(str2double(CARID))==0
            y2=path(pathnum(str2double(CARID)),N(str2double(CARID)),2)-yc(str2double(CARID));
            y1=path(pathnum(str2double(CARID)),N(str2double(CARID)),2)-yc(j);
            x2=path(pathnum(str2double(CARID)),N(str2double(CARID)),1)-xc(str2double(CARID));
            x1=path(pathnum(str2double(CARID)),N(str2double(CARID)),1)-xc(j);
            else
                if pathnum(str2double(CARID))==1
                    y2=1-yc(str2double(CARID));
                    y1=1-yc(j);
                    x2=0.3-xc(str2double(CARID));
                    x1=0.3-xc(j);
                elseif pathnum(str2double(CARID))==2
                    y2=2-yc(str2double(CARID));
                    y1=2-yc(j);
                    x2=1-xc(str2double(CARID));
                    x1=1-xc(j);
                end
            end
            
            if(norm([0,0]-[x2,y2])>norm([0,0]-[x1,y1]))
             error=atan2d(y2,x2) - atan2d(y1,x1);
             if abs(error) > 180
                if error > 0
                   error = error -360;
                else error = error +360;
                end
             end
             if(abs(error)<15)
                result=0;
                break;
             end
           
           end
        end
    end
 end
end
end