% clc;clear;
% % surf(peaks);
% % shading interp
% % set(gca,'position',[0 0 1 1]);
% % print -dtiff 'x.tif';
% i=1;
% while(1)
%     if rem(i,2)~=0 && i<6
%         img=imread('1b.jpg');
%         light_color=[1 1 0 1];
%         save('light_color');
%         figure(1);
%         clf(1);
%     elseif (rem(i,2)==0)&&i<6
%         img=imread('2b.jpg');
%          light_color=[0 1 1 1];
%         save('light_color');
%          figure(1);
%         clf(1);
%     elseif i>=6
%        break;
%     end
%     i=i+1;
% scrsz = get(0,'ScreenSize');
% imshow(img,'border','tight','initialmagnification','fit');
% %figure('units','normlized','position',[.1 .1 .4 .4])
%  set(gcf,'Position',scrsz);
% axis off; 
% pause(20);
% end


clc;clear;
% surf(peaks);
% shading interp
% set(gca,'position',[0 0 1 1]);
% print -dtiff 'x.tif';
i=2;
while(1)
    if rem(i,2)~=0
        img=imread('24_1.jpg');
        light_color=[0 0 0 0 0 0 0 0 0 0 ... 
                     0 0 0 0 0 ... 
                     1 0 0 0 0 1 0 0 0 0];
        save('light_color','light_color');
        figure(1);
        clf(1);
    elseif (rem(i,2)==0)
        img=imread('24_2.jpg');
         light_color=[1 0 0 0 0 1 0 0 0 0 ... 
                      1 0 0 0 0 ... 
                      0 0 0 0 0 0 0 0 0 0];
         save('light_color','light_color');
         figure(1);
        clf(1);
  
    end
    i=i+1;

image(img);
%figure('units','normlized','position',[.1 .1 .4 .4])
axis off;
pause(9);
end