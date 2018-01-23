function [headingAngle,dist] = cal_command(currentPosition,currentDirection,targetPosition)
% The zero degree is (x=1, y=0). positive degree is (y>0). 90 degree
% is(y =1 x=0). negtive degree is (y<0) -90 is (x = to =0, y=-1)
% alpha(-180, 180) is the angle formed by currentPostion and targetPosition. 

%cal_dist
dist = norm(targetPosition-currentPosition);

%cal rotation angle
vector = targetPosition-currentPosition;
headingAngle = atan2d(vector(2),vector(1)) - currentDirection;
if abs(headingAngle) > 180
    if headingAngle > 0
        headingAngle = headingAngle -360;
    else headingAngle = headingAngle +360;
    end
end


end
