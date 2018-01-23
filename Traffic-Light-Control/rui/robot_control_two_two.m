function robot_control_two_two()
 % one robot, input leaderPos, point to point navigation
clear all; close all; clc; obj=instrfind;delete(obj);
%define the path of each vehicle

%path(1,1,:)=[0.6434 -2.8369];path(1,2,:)=[0.2612 -2.8369];path(1,3,:)=[-1.5522 -2.8369];path(1,4,:)=[0 0];%define path1. [0 0] is stopping mark 
%path(2,1,:)=[0.2612 -3.1738];path(2,2,:)=[0.2612 -2.8369];path(2,3,:)=[0.2846 -1.6583];path(2,4,:)=[0 0];%define path2. [0 0] is stopping mark
%serial number of traffic light in each intersection
path(1,1,:)=[2.2743 0.9790];path(1,2,:)=[ 2.6720 0.98];path(1,3,:)=[ 4.7065 0.9906 ];path(1,4,:)=[0 0];%define path1. [0 0] is stopping mark 
path(2,1,:)=[2.6622 1.3568];path(2,2,:)=[ 2.6720  0.98];path(2,3,:)=[  2.6446 -0.2439 ];path(2,4,:)=[0 0];%define path2. [0 0] is stopping mark
%path(3,1,:)=[2.2743 0.9790];path(3,2,:)=[ 2.6720 0.98];path(3,3,:)=[  2.6446 -0.2439 ];path(3,4,:)=[0 0];
%serial number of traffic light in each intersection
light(1,1)=1;light(1,2)=2;light(1,3)=3;light(2,1)=4;light(2,2)=5;light(2,3)=6;
%initial traffic light color
%light_color=[0,1,1,1];
num_pololu =3;%number of cars in system
N=zeros(num_pololu,1);%mark the serial number of next traffic light in path for every car.
current=zeros(num_pololu,1);% record the serial number of the current traffic light
TT=zeros(num_pololu,1);
pathnum=zeros(num_pololu,1);
pathnum(1)=1;
pathnum(2)=1;
pathnum(3)=1;
%pathnum(4)=2;
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

end
while(1)
   pause(0.04);
   mydata=load('light_color');
   light_color=mydata.light_color;
   [xc,yc,thetac]=read_pos(opti,frame);
   
   
   %************************************************************************************
   %carID=i; pathnum is path number
   for number_car=1:1:num_pololu
   CARID=num2str(number_car);
   %leaderPos=path(pathnum,N(str2double(CARID)));  
  if(path(pathnum(str2double(CARID)),N(str2double(CARID)),1)~=0&&path(pathnum(str2double(CARID)),N(str2double(CARID)),2)~=0)
    
    
    % command calculation()
    current_pos = [xc(str2double(CARID)), yc(str2double(CARID))];
    current_dir = thetac(str2double(CARID));
 
    [heading_angle,dist] = cal_command(current_pos,current_dir,[path(pathnum(str2double(CARID)),N(str2double(CARID)),1),path(pathnum(str2double(CARID)),N(str2double(CARID)),2)]);  
   
    % execute command()
    %check position, unit: m , degree
    %light_color(light(num,N))
    if(detect_collision()==1)
        if TT(str2double(CARID))==1
            %timerBusy(str2double(CARID))=0;
            TT(str2double(CARID))=0;
            fprintf(serialPC, '%c%c\n',strcat(CARID,'f'),'sync');
            fprintf(serialPC, '%c%c\n',strcat(CARID,'f'),'sync');
            fprintf(serialPC, '%c%c\n',strcat(CARID,'f'),'sync');
            fprintf(serialPC, '%c%c\n',strcat(CARID,'f'),'sync');
            fprintf(serialPC, '%c%c\n',strcat(CARID,'f'),'sync');
            fprintf(serialPC, '%c%c\n',strcat(CARID,'f'),'sync');
            fprintf(serialPC, '%c%c\n',strcat(CARID,'f'),'sync');
        end
    if dist > 0.05 && reachTarget(str2double(CARID)) == 0
        current(str2double(CARID))=N(str2double(CARID));
        if abs(heading_angle) > 5  % degree
            if  timerBusy(str2double(CARID)) == 0         % check timer
                turnPololu(CARID,serialPC,heading_angle);
            end
        end
        
    else
        if(light_color(light(pathnum(str2double(CARID)),current((str2double(CARID)))))==1)   %red light
        fprintf(serialPC, '%c%c\n',strcat(CARID,'s'),'sync');
        fprintf(serialPC, '%c%c\n',strcat(CARID,'s'),'sync');
        fprintf(serialPC, '%c%c\n',strcat(CARID,'s'),'sync');
        fprintf(serialPC, '%c%c\n',strcat(CARID,'s'),'sync');
        reachTarget(str2double(CARID)) = 1;
        N(str2double(CARID))=current(str2double(CARID))+1;
        elseif light_color(light(pathnum(str2double(CARID)),current(str2double(CARID))))==0   %green light
            reachTarget(str2double(CARID))=0;
            N(str2double(CARID))=current(str2double(CARID))+1;
            fprintf(serialPC, '%c%c\n',strcat(CARID,'f'),'sync');
            fprintf(serialPC, '%c%c\n',strcat(CARID,'f'),'sync');
            fprintf(serialPC, '%c%c\n',strcat(CARID,'f'),'sync');
            fprintf(serialPC, '%c%c\n',strcat(CARID,'f'),'sync');
            fprintf(serialPC, '%c%c\n',strcat(CARID,'f'),'sync');
        end 
    end
    elseif detect_collision()==0
        TT(str2double(CARID))=1;
       % timerBusy(str2double(CARID)) = 1;
        fprintf(serialPC, '%c%c\n',strcat(CARID,'s'),'sync');
        fprintf(serialPC, '%c%c\n',strcat(CARID,'s'),'sync');
        fprintf(serialPC, '%c%c\n',strcat(CARID,'s'),'sync');
        fprintf(serialPC, '%c%c\n',strcat(CARID,'s'),'sync');
        
    end
  else
        fprintf(serialPC, '%c%c\n',strcat(CARID,'s'),'sync');
        fprintf(serialPC, '%c%c\n',strcat(CARID,'s'),'sync');
        fprintf(serialPC, '%c%c\n',strcat(CARID,'s'),'sync');
        fprintf(serialPC, '%c%c\n',strcat(CARID,'s'),'sync');
  end
% *************************************************************************************************
 
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
%**********************************************************************
%if result=1, there is no car before;else if result=0, car should stop to avoid collision.
%the serial number of current traffic light in path for every car.
% function result=detect_collision()
% result=1; 
%  for j=1:1:num_pololu
%     if(str2double(CARID)~=j)
%         if(norm([xc(str2double(CARID)), yc(str2double(CARID))]-[xc(j), yc(j)])<0.28)
%             if(pathnum(str2double(CARID))==pathnum(j))
%                 if(current(str2double(CARID))==current(j))
%                      y2=path(pathnum(str2double(CARID)),current(str2double(CARID)),2)-yc(str2double(CARID));
%                      y1=path(pathnum(str2double(CARID)),current(str2double(CARID)),2)-yc(j);
%                      x2=path(pathnum(str2double(CARID)),current(str2double(CARID)),1)-xc(str2double(CARID));
%                      x1=path(pathnum(str2double(CARID)),current(str2double(CARID)),1)-xc(j);
%             if(norm([0,0]-[x2,y2])>norm([0,0]-[x1,y1]))
%               result=0;
%                 break;
%            
%             end
%                 elseif((current(j)-current(str2double(CARID)))==1)
%                     result=0;
%                     break;
%                 end
%             end
%          
%             
%         end
%     end
%  end
% end

function result=detect_collision()
result=1; 
 for j=1:1:num_pololu
    if(str2double(CARID)~=j)
        if(norm([xc(str2double(CARID)), yc(str2double(CARID))]-[xc(j), yc(j)])<0.25)
            if(path(pathnum(str2double(CARID)),current(str2double(CARID)),1)==path(pathnum(j),current(j),1))&&(path(pathnum(str2double(CARID)),current(str2double(CARID)),2)==path(pathnum(j),current(j),2))
               
                     y2=path(pathnum(str2double(CARID)),current(str2double(CARID)),2)-yc(str2double(CARID));
                     y1=path(pathnum(str2double(CARID)),current(str2double(CARID)),2)-yc(j);
                     x2=path(pathnum(str2double(CARID)),current(str2double(CARID)),1)-xc(str2double(CARID));
                     x1=path(pathnum(str2double(CARID)),current(str2double(CARID)),1)-xc(j);
                     if(norm([0,0]-[x2,y2])>norm([0,0]-[x1,y1]))
                         result=0;
                         break;
           
                     end
            elseif (path(pathnum(str2double(CARID)),current(str2double(CARID)),1)==path(pathnum(j),max(1,current(j)-1),1))&&(path(pathnum(str2double(CARID)),current(str2double(CARID)),2)==path(pathnum(j),max(1,current(j)-1),2))
                %if (norm([xc(str2double(CARID)), yc(str2double(CARID))]-[xc(j), yc(j)])<0.2)
                    result=0;
                    break;
               % end
             end
         
            
        end
    end
 end
end
end