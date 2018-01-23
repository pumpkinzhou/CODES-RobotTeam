function vehicle_control_two()
 % one robot, input leaderPos, point to point navigation
clear all; close all; clc; 
%define the path of each vehicle

path(1,1,:)=[1 1];path(1,2,:)=[2.454 -1.8];path(1,3,:)=[0 0];%define path1. [0 0] is stopping mark 
path(2,1,:)=[1.41 2.0];path(2,2,:)=[1.41 0.97 ];path(2,3,:)=[0 0];%define path2. [0 0] is stopping mark
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
    t(i) = timer;
    t(i).BusyMode = 'drop';
    t(i).TimerFcn ={@moveforward,num2str(i)};
    N(i)=1;%initial N and current
    current(i)=1;
end
pathnum(1)=1;
pathnum(2)=1;
pathnum(3)=1;
timerBusy = zeros(num_pololu,1);

reachTarget = zeros(num_pololu,1);
%leaderPos = [1,1];  % initial leader pos

%% Connect serial PC terminal
comNum = 'COM9'; BaudRate = 115200;
serialPC = connect_to_serialPC(comNum,BaudRate);
for i=1:num_pololu
    ii=num2str(i);
    fprintf(serialPC, '%c%c\n',strcat(ii,'l'),'sync');
 fprintf(serialPC, '%c%c\n',strcat(ii,'l'),'sync');
 fprintf(serialPC, '%c%c\n',strcat(ii,'l'),'sync');
 fprintf(serialPC, '%c%c\n',strcat(ii,'l'),'sync');
 fprintf(serialPC, '%c%c\n',strcat(ii,'l'),'sync');
end
while(1)
   pause(0.06);
   mydata=load('light_color');
   light_color=mydata.light_color;
   [xc,yc,thetac]=read_pos(opti,frame);
   
   
   %************************************************************************************
   %carID=1; pathnum is path number
   CARID='3';
     current_pos = [xc(str2double(CARID)), yc(str2double(CARID))];
    current_dir = thetac(str2double(CARID));
   %leaderPos=path(pathnum,N(str2double(CARID)));  
  if(path(pathnum(str2double(CARID)),current(str2double(CARID)),1)~=0&&path(pathnum(str2double(CARID)),current(str2double(CARID)),2)~=0)
      if START(str2double(CARID))==0
           [heading_angle,dist] = cal_command(current_pos,current_dir,[path(pathnum(str2double(CARID)),N(str2double(CARID)),1),path(pathnum(str2double(CARID)),N(str2double(CARID)),2)]);  
      elseif START(str2double(CARID))~=0
        
      if pathnum(str2double(CARID))==2
           [heading_angle,dist] = cal_command(current_pos,current_dir,[1.41 2.652]);
      elseif pathnum(str2double(CARID))==1
           [heading_angle,dist] = cal_command(current_pos,current_dir,[0.3 1.8]);
      end
      
      end
    % command calculation()
  
    % execute command()
    %check position, unit: m , degree
    %light_color(light(num,N))
    if(detect_collision()==1)
        if TT(str2double(CARID))==1
          %  timerBusy(str2double(CARID))=0;
            TT(str2double(CARID))=0;
            fprintf(serialPC, '%c%c\n',strcat(CARID,'f'),'sync');
            fprintf(serialPC, '%c%c\n',strcat(CARID,'f'),'sync');
            fprintf(serialPC, '%c%c\n',strcat(CARID,'f'),'sync');
            fprintf(serialPC, '%c%c\n',strcat(CARID,'f'),'sync');
            fprintf(serialPC, '%c%c\n',strcat(CARID,'f'),'sync');
            fprintf(serialPC, '%c%c\n',strcat(CARID,'f'),'sync');
            fprintf(serialPC, '%c%c\n',strcat(CARID,'f'),'sync');
        end
    if dist > 0.08 && reachTarget(str2double(CARID)) == 0
       % current(str2double(CARID))=N(str2double(CARID));
        if abs(heading_angle) > 5  % degree
            if  timerBusy(str2double(CARID)) == 0         % check timer
                turnPololu(CARID,serialPC,heading_angle);
            end
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
        elseif START(str2double(CARID))~=0
           START(str2double(CARID))=0;
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
           START(str2double(CARID))=1;
      elseif pathnum(str2double(CARID))==2
          pathnum(str2double(CARID))=1;
          N(str2double(CARID))=1;%renew N and current
          current(str2double(CARID))=1;
           START(str2double(CARID))=2;
      end
  

  end

 
  
  %**************************************************************
   %************************************************************************************
   %carID=3; pathnum is path number
%    CARID='3';
%      current_pos = [xc(str2double(CARID)), yc(str2double(CARID))];
%     current_dir = thetac(str2double(CARID));
%    %leaderPos=path(pathnum,N(str2double(CARID)));  
%   if(path(pathnum(str2double(CARID)),current(str2double(CARID)),1)~=0&&path(pathnum(str2double(CARID)),current(str2double(CARID)),2)~=0)
%       if START(str2double(CARID))==0
%            [heading_angle,dist] = cal_command(current_pos,current_dir,[path(pathnum(str2double(CARID)),N(str2double(CARID)),1),path(pathnum(str2double(CARID)),N(str2double(CARID)),2)]);  
%       elseif START(str2double(CARID))~=0
%         
%       if pathnum(str2double(CARID))==2
%            [heading_angle,dist] = cal_command(current_pos,current_dir,[1.41 2.652]);
%       elseif pathnum(str2double(CARID))==1
%            [heading_angle,dist] = cal_command(current_pos,current_dir,[0.3 1.8]);
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
%           %  timerBusy(str2double(CARID))=0;
%             TT(str2double(CARID))=0;
%             fprintf(serialPC, '%c%c\n',strcat(CARID,'f'),'sync');
%             fprintf(serialPC, '%c%c\n',strcat(CARID,'f'),'sync');
%             fprintf(serialPC, '%c%c\n',strcat(CARID,'f'),'sync');
%             fprintf(serialPC, '%c%c\n',strcat(CARID,'f'),'sync');
%             fprintf(serialPC, '%c%c\n',strcat(CARID,'f'),'sync');
%             fprintf(serialPC, '%c%c\n',strcat(CARID,'f'),'sync');
%             fprintf(serialPC, '%c%c\n',strcat(CARID,'f'),'sync');
%         end
%     if dist > 0.08 && reachTarget(str2double(CARID)) == 0
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
%         
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
%         elseif START(str2double(CARID))~=0
%            START(str2double(CARID))=0;
%         end
%     end
%     elseif detect_collision()==0
%         TT(str2double(CARID))=1;
%      %   timerBusy(str2double(CARID)) = 1;
%         fprintf(serialPC, '%c%c\n',strcat(CARID,'s'),'sync');
%         fprintf(serialPC, '%c%c\n',strcat(CARID,'s'),'sync');
%         fprintf(serialPC, '%c%c\n',strcat(CARID,'s'),'sync');
%         fprintf(serialPC, '%c%c\n',strcat(CARID,'s'),'sync');
%         
%     end
%   elseif (path(pathnum(str2double(CARID)),current(str2double(CARID)),1)==0&&path(pathnum(str2double(CARID)),current(str2double(CARID)),2)==0)
%       if pathnum(str2double(CARID))==1
%           pathnum(str2double(CARID))=2;
%           N(str2double(CARID))=1;%renew N and current
%           current(str2double(CARID))=1;
%            START(str2double(CARID))=1;
%       elseif pathnum(str2double(CARID))==2
%           pathnum(str2double(CARID))=1;
%           N(str2double(CARID))=1;%renew N and current
%           current(str2double(CARID))=1;
%            START(str2double(CARID))=2;
%       end
%     
% 
%   end
  %****************************************************

end



function turnPololu(carID,serialPC,angle)
    turn_time = 2.1/360*abs(angle);
    id = str2double(carID);
    t(id).StartDelay= turn_time;
    start(t(id));
      
    if(angle >0) % turn left
       fprintf(serialPC, '%c%c\n',strcat(carID,'l'),'sync');
       fprintf(serialPC, '%c%c\n',strcat(carID,'l'),'sync');
    else
       fprintf(serialPC, '%c%c\n',strcat(carID,'r'),'sync');
       fprintf(serialPC, '%c%c\n',strcat(carID,'r'),'sync');
    end
    timerBusy(id) = 1;
        
end

function moveforward(obj,event,handles)
        
    fprintf(serialPC, '%c%c\n',strcat(handles,'f'),'sync');
    fprintf(serialPC, '%c%c\n',strcat(handles,'f'),'sync');
        
    id = str2double(handles);
    timerBusy(id) = 0;  
end
%**********************************************************************
%if result=1, there is no car before;else if result=0, car should stop to avoid collision.
%the serial number of current traffic light in path for every car.
function result=detect_collision()
result=1; 
 for j=1:1:num_pololu
    
    if(str2double(CARID)~=j)
        if(norm([xc(str2double(CARID)), yc(str2double(CARID))]-[xc(j), yc(j)])<0.29)
             
            if START(str2double(CARID))==0
                if START(j)==0
                    if(pathnum(str2double(CARID))==pathnum(j))
                        if(current(str2double(CARID))==current(j))
                            y2=path(pathnum(str2double(CARID)),current(str2double(CARID)),2)-yc(str2double(CARID));
                            y1=path(pathnum(str2double(CARID)),current(str2double(CARID)),2)-yc(j);
                            x2=path(pathnum(str2double(CARID)),current(str2double(CARID)),1)-xc(str2double(CARID));
                            x1=path(pathnum(str2double(CARID)),current(str2double(CARID)),1)-xc(j);
                            if(norm([0,0]-[x2,y2])>norm([0,0]-[x1,y1]))
                            result=0;
                            break;
                            end
                        elseif (current(str2double(CARID))<current(j))
                            result=0;
                            break;
                        end
                    end
                
                elseif START(j)== pathnum(str2double(CARID))
                    if(path(pathnum(str2double(CARID)),current(str2double(CARID))+1,2)==0&&path(pathnum(str2double(CARID)),current(str2double(CARID))+1,1)==0)
                      result=0;
                      break;
                    end
                    
                end
            else
               if START(j)==START(str2double(CARID))
                if pathnum(str2double(CARID))==1
                    y2=1.8-yc(str2double(CARID));
                    y1=1.8-yc(j);
                    x2=0.3-xc(str2double(CARID));
                    x1=0.3-xc(j);
                elseif pathnum(str2double(CARID))==2
                    y2=2.652-yc(str2double(CARID));
                    y1=2.652-yc(j);
                    x2=1.41-xc(str2double(CARID));
                    x1=1.41-xc(j);
                end
                if(norm([0,0]-[x2,y2])>norm([0,0]-[x1,y1]))
                       result=0;
                       break;
                end
                elseif START(j)==0
                    if pathnum(j)~=START(str2double(CARID))
                        if(current(str2double(CARID))==1)
                        result=0;
                        break;
                        end
                    end
                    
                end
            end
           
        end
    end
 end
end
end